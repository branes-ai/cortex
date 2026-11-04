// Observability ledger (issue #480).
//
// The ledger carries the unobservable basis through a running MSCKF with the
// filter's own Jacobians and books the camera's information along it to the
// stage whose defect put it there. These cases lock the instrument (its basis,
// its bookkeeping, its untapped-change detector) and the finding on the
// synthetic world: the spurious information is yaw, never global position, and
// it comes from S6 moving the linearization point, not from S2's Φ or S3's clone,
// and it accumulates over the run.

#include <branes/sdk/eval/synthetic_world.hpp>
#include <branes/sdk/msckf/state_helper.hpp>
#include <branes/sdk/msckf_backend.hpp>
#include <branes/tools/bench/s6_scene.hpp>
#include <branes/tools/observability_ledger.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <span>
#include <vector>

namespace {

namespace bn = branes::tools::bench;
namespace sdk = branes::sdk;
using branes::tools::LeakSource;
using Backend = sdk::MsckfBackend<double>;
using St = sdk::msckf::State<double>;
using Ledger = branes::tools::ObservabilityLedger<double>;

const sdk::eval::SyntheticData<double>& world() {
    static const auto w = sdk::eval::generate_world<double>(sdk::eval::SyntheticConfig<double>{});
    return w;
}

Backend make_backend() {
    Backend::CameraCalibration cal;
    cal.intrinsics = world().camera;
    cal.extrinsics.R_imu_cam = world().R_imu_cam;
    cal.extrinsics.p_imu_cam = world().p_imu_cam;
    Backend be(std::vector<Backend::CameraCalibration>{cal});
    be.initialize(sdk::VioConfig{});
    return be;
}

/// Run the world's first `frames` frames with `px` pixels of pseudo-normal
/// observation noise (the IMU is exact), the ledger tapped on the backend.
Ledger run_world(std::size_t frames, double px) {
    Ledger ledger;
    auto be = make_backend();
    be.set_stage_tap(&ledger);
    const auto& w = world();
    std::size_t k = 0;
    std::uint32_t draw = 0;
    for (std::size_t f = 0; f < std::min(frames, w.frames.size()); ++f) {
        for (; k < w.imu.size() && w.imu[k].timestamp_s <= w.frames[f].t; ++k)
            be.process_imu(w.imu[k]);
        auto obs = w.frames[f].obs;
        for (auto& o : obs) {
            o.u += px * bn::s6::pseudo_normal(draw++);
            o.v += px * bn::s6::pseudo_normal(draw++);
        }
        be.process_camera(w.frames[f].t, std::span<const sdk::FrontendObservation<double>>(obs));
    }
    be.set_stage_tap(nullptr);
    return ledger;
}

double max_abs(const sdk::msckf::DynMat<double>& m) {
    double x = 0.0;
    for (const double v : m.d)
        x = std::max(x, std::abs(v));
    return x;
}

/// A state with two clones at distinct poses, away from the origin.
St scene_state() {
    St s(0.1);
    s.R = branes::math::lie::SO3<double>::exp({{0.2, -0.1, 0.7}});
    s.p = {{1.0, -2.0, 0.5}};
    s.v = {{0.3, 0.4, -0.1}};
    sdk::msckf::StateHelper<double>::augment_clone(s);
    s.R = branes::math::lie::SO3<double>::exp({{0.25, -0.05, 0.9}});
    s.p = {{1.2, -1.7, 0.6}};
    sdk::msckf::StateHelper<double>::augment_clone(s);
    return s;
}

}  // namespace

TEST_CASE("Observability ledger: the basis is the S6 benches' unobservable basis", "[tools][observability]") {
    const St s = scene_state();
    const auto mine = branes::tools::unobservable_basis(s);
    const auto ref = bn::s6::unobservable_basis<double>(s);
    REQUIRE(mine.rows == ref.rows);
    REQUIRE(mine.cols == ref.cols);
    for (std::size_t i = 0; i < mine.d.size(); ++i)
        REQUIRE(std::abs(mine.d[i] - ref.d[i]) < 1e-15);
}

TEST_CASE("Observability ledger: a change no tap reported is booked as untapped", "[tools][observability]") {
    Ledger ledger;
    St s = scene_state();
    St a = s;
    sdk::msckf::StateHelper<double>::augment_clone(a);
    ledger.on_s3(s, 0.0, {}, a);
    REQUIRE(max_abs(ledger.defect(LeakSource::S3_augmentation)) == 0.0);  // a clone is an exact copy

    // Between the boundaries the state yaws and moves without a tap seeing it.
    St b = a;
    b.R = branes::math::lie::SO3<double>::exp({{0.0, 0.0, 0.3}}) * b.R;
    b.p = {{b.p[0] + 0.5, b.p[1], b.p[2]}};
    St c = b;
    sdk::msckf::StateHelper<double>::augment_clone(c);
    ledger.on_s3(b, 0.1, {}, c);
    REQUIRE(max_abs(ledger.defect(LeakSource::untapped)) > 0.1);
    REQUIRE(ledger.totals().closure_max < 1e-12);
    REQUIRE(ledger.totals().restarts == 0);
}

TEST_CASE("Observability ledger: on the synthetic world the leak is yaw, from S6 moving the mean",
          "[tools][observability]") {
    const auto ledger = run_world(160, 0.5);
    REQUIRE(ledger.started());
    const auto& t = ledger.totals();
    REQUIRE(t.s6_applied > 100);
    REQUIRE(t.prep_mismatch == 0);
    REQUIRE(t.restarts == 0);

    // The bookkeeping closes: T = N(x̂) + Σ D to roundoff.
    REQUIRE(t.closure_max < 1e-9);
    // Every state change was tapped; S3 and S9 introduce no defect.
    REQUIRE(max_abs(ledger.defect(LeakSource::untapped)) == 0.0);
    REQUIRE(max_abs(ledger.defect(LeakSource::S3_augmentation)) == 0.0);
    REQUIRE(max_abs(ledger.defect(LeakSource::S9_marginalization)) == 0.0);

    // The shares add up to the total.
    double yaw = 0.0, pos = 0.0;
    for (const auto& s : t.share) {
        yaw += s.yaw;
        pos += s.pos;
    }
    REQUIRE(std::abs(yaw - t.info.yaw) <= 1e-9 * t.info.yaw);

    // Global translation keeps its basis under every stage: no position leak.
    REQUIRE(t.info.pos < 1e-12 * t.info.yaw);
    REQUIRE(std::abs(pos - t.info.pos) < 1e-12 * t.info.yaw);

    // The spurious yaw information outweighs the prior (1/σ₀² = 100 rad⁻²) …
    REQUIRE(t.info.yaw > 1.0 / (t.yaw_sigma_start * t.yaw_sigma_start));
    // … and S6's mean shift carries it; S2's first-order Φ is a rounding error beside it.
    const auto& shift = t.share[static_cast<std::size_t>(LeakSource::S6_mean_shift)];
    const auto& s2 = t.alone[static_cast<std::size_t>(LeakSource::S2_propagation)];
    const auto& s6 = t.alone[static_cast<std::size_t>(LeakSource::S6_mean_shift)];
    REQUIRE(shift.yaw > 0.95 * t.info.yaw);
    REQUIRE(s2.yaw < 1e-2 * s6.yaw);
    // S6's own projection annihilates N(x̂), as the S6 benches found (#457).
    REQUIRE(t.alone[static_cast<std::size_t>(LeakSource::S6_projection)].yaw < 1e-12 * t.info.yaw);

    // The filter's yaw σ fell, which a consistent filter cannot do.
    REQUIRE(t.yaw_sigma_min < t.yaw_sigma_start);
    REQUIRE(ledger.rows().size() > 50);
}

TEST_CASE("Observability ledger: the spurious yaw information accumulates over the run", "[tools][observability]") {
    const auto early = run_world(80, 0.5).totals();
    const auto late = run_world(250, 0.5).totals();
    REQUIRE(late.info.yaw > 10.0 * early.info.yaw);
    REQUIRE(late.yaw_sigma_min < early.yaw_sigma_min);
}
