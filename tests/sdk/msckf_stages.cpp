// VIO stages as explicit transformations (issue #452).
//
// Each stage in msckf/stages/ is callable in isolation as (state, inputs,
// params) → (state, diagnostics). These cases call every stage on its own,
// check its diagnostics against the #445 invariants where one applies, and
// lock the S5 → S6e composite to CameraUpdater::update bit for bit, so the
// backend (which runs the composite) and the probes (which run update()) cannot
// drift apart.

#include <branes/sdk/eval/invariants.hpp>
#include <branes/sdk/msckf.hpp>
#include <branes/sdk/msckf/stages.hpp>

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace {

namespace ms = branes::sdk::msckf;
namespace st = branes::sdk::msckf::stages;
namespace inv = branes::sdk::eval::inv;
using T = double;
using Vec3 = ms::CameraUpdater<T>::Vec3;
using Vec2 = ms::CameraUpdater<T>::Vec2;
using SO3 = ms::CameraUpdater<T>::SO3;

// Clones along x looking down +z, with propagation between augmentations so P
// is full rank (identity extrinsics: camera == IMU).
template <class Cov = ms::FullCovariance<T>>
ms::State<T, Cov> window(std::size_t n) {
    ms::State<T, Cov> s(0.3);
    const ms::Propagator<T> prop;
    for (std::size_t i = 0; i < n; ++i) {
        s.p = Vec3{{0.5 * static_cast<T>(i), 0.0, 0.0}};
        st::s3_augmentation::apply(s, static_cast<double>(i));
        for (int k = 0; k < 10; ++k)
            st::s2_propagation::apply(s, prop, Vec3{{0.01, -0.005, 0.008}}, Vec3{{0.05, 0.02, 9.81}}, 0.01);
    }
    return s;
}

template <class Cov>
ms::FeatureTrack<T> track_of(const ms::State<T, Cov>& s, const Vec3& f) {
    ms::FeatureTrack<T> tr;
    for (std::size_t i = 0; i < s.clones.size(); ++i) {
        const auto& cl = s.clones[i];
        const Vec3 pc = cl.R.inverse() * (f - cl.p);
        tr.observations.push_back({i, 0, Vec2{{pc[0] / pc[2] + 0.001 * static_cast<T>(i), pc[1] / pc[2]}}});
    }
    return tr;
}

ms::CameraUpdater<T> updater() {
    return ms::CameraUpdater<T>(std::vector<ms::CameraExtrinsics<T>>(1));
}

template <class Cov>
bool bit_equal(const ms::State<T, Cov>& a, const ms::State<T, Cov>& b) {
    if (a.covariance().d != b.covariance().d || a.clones.size() != b.clones.size())
        return false;
    for (std::size_t i = 0; i < 4; ++i)
        if (a.R.quaternion()[i] != b.R.quaternion()[i])
            return false;
    for (std::size_t i = 0; i < 3; ++i)
        if (a.p[i] != b.p[i] || a.v[i] != b.v[i] || a.bg[i] != b.bg[i] || a.ba[i] != b.ba[i])
            return false;
    for (std::size_t c = 0; c < a.clones.size(); ++c)
        for (std::size_t i = 0; i < 3; ++i)
            if (a.clones[c].p[i] != b.clones[c].p[i])
                return false;
    return true;
}

}  // namespace

TEST_CASE("S0_sensor_model normalizes a pixel and rejects a ray behind the camera", "[sdk][stages]") {
    // A unit pinhole: unproject(u, v) is the bearing through (u, v, 1).
    struct UnitPinhole {
        std::array<T, 3> unproject(const std::array<T, 2>& uv) const {
            return {uv[0], uv[1], z};
        }
        T z = 1.0;
    };
    const auto ok = st::s0_sensor_model::apply<T>(UnitPinhole{}, 0.2, -0.4);
    REQUIRE(ok.valid);
    REQUIRE(ok.xy[0] == 0.2);
    REQUIRE(ok.xy[1] == -0.4);
    REQUIRE_FALSE(st::s0_sensor_model::apply<T>(UnitPinhole{-1.0}, 0.2, -0.4).valid);
}

TEST_CASE("S2_propagation and S3_augmentation: dimensions and covariance invariants", "[sdk][stages]") {
    ms::State<T> s(0.3);
    const ms::Propagator<T> prop;
    const auto d2 = st::s2_propagation::apply(s, prop, Vec3{{0.01, 0.0, 0.0}}, Vec3{{0.0, 0.0, 9.81}}, 0.005);
    REQUIRE(d2.applied);
    REQUIRE(d2.dim == 15);
    REQUIRE_FALSE(st::s2_propagation::apply(s, prop, Vec3{}, Vec3{}, 0.0).applied);  // Δt ≤ 0: no-op
    REQUIRE(inv::check_psd(s.covariance(), inv::Stage::S2_propagation).pass);
    REQUIRE(inv::check_symmetric(s.covariance(), inv::Stage::S2_propagation).pass);

    const auto d3 = st::s3_augmentation::apply(s, 1.25);
    REQUIRE(d3.dim_before == 15);
    REQUIRE(d3.dim_after == 21);
    REQUIRE(d3.clones == 1);
    REQUIRE(s.clones.back().timestamp == 1.25);
    REQUIRE(inv::check_psd(s.covariance(), inv::Stage::S3_augmentation).pass);
}

TEST_CASE("S4_frontend ingests observations and releases ended tracks in id order", "[sdk][stages]") {
    st::s4_frontend::TrackTable<T> tracks;
    auto normalize = [](std::uint32_t cam, T u, T v, Vec2& xy) {
        if (cam != 0)
            return false;
        xy = Vec2{{u, v}};
        return true;
    };
    using Obs = branes::sdk::FrontendObservation<T>;
    const std::vector<Obs> f0{{7, 0, 0.1, 0.1}, {3, 0, 0.2, 0.2}, {9, 0, 0.3, 0.3}};
    const auto d0 = st::s4_frontend::apply<T>(tracks, 0.0, f0, normalize);
    REQUIRE(d0.ingested == 3);
    REQUIRE(d0.ended.empty());

    const std::vector<Obs> f1{{7, 0, 0.1, 0.1}, {5, 1, 0.0, 0.0}};  // 5 is on an unknown camera
    const auto d1 = st::s4_frontend::apply<T>(tracks, 1.0, f1, normalize);
    REQUIRE(d1.ingested == 1);
    REQUIRE(d1.rejected == 1);
    REQUIRE(d1.ended == std::vector<std::uint64_t>{3, 9});

    // assemble resolves clone times to the current clone indices.
    auto s = window(2);  // clones at t = 0, 1
    const auto tr = st::s4_frontend::assemble(tracks.at(7), s);
    REQUIRE(tr.observations.size() == 2);
    REQUIRE(tr.observations[1].clone_index == 1);
}

TEST_CASE("S5 and S6a-S6e run one feature track step by step", "[sdk][stages]") {
    auto s = window(4);
    const auto upd = updater();
    const auto track = track_of(s, Vec3{{0.7, 0.2, 5.0}});

    const auto tri = st::s5_triangulation::apply(s, upd, track);
    REQUIRE(tri.ok);
    REQUIRE(tri.p_f[2] > 4.5);

    const auto jac = st::s6a_jacobians::apply(s, upd, track, tri.p_f);
    REQUIRE(jac.ok);
    REQUIRE(jac.system.rows == 8);
    REQUIRE(jac.system.cols == s.dim());

    const auto proj = st::s6b_nullspace_projection::apply(jac.system);
    REQUIRE(proj.ok);
    REQUIRE(proj.projected.H.rows == 5);  // 2m − 3

    const auto comp = st::s6c_compression::apply(proj.projected);
    REQUIRE(comp.rows_in == comp.rows_out);
    REQUIRE_FALSE(comp.compressed_applied);
    REQUIRE(comp.compressed.H.d == proj.projected.H.d);

    const auto gate = st::s6d_gating::apply(s, upd, comp.compressed);
    REQUIRE(gate.nis.valid);
    REQUIRE(gate.nis.dof == 5);

    const auto p_before = s.covariance();
    const auto upd_e = st::s6e_ekf_update::apply(s, upd, comp.compressed);
    REQUIRE(upd_e.dx.size() == s.dim());
    REQUIRE(inv::check_finite<T>(std::span<const T>(upd_e.dx), inv::Stage::S6e_ekf_update).pass);
    REQUIRE(inv::check_loewner_le(s.covariance(), p_before, inv::Stage::S6e_ekf_update).pass);
    REQUIRE(inv::check_psd(s.covariance(), inv::Stage::S6e_ekf_update).pass);
}

TEMPLATE_TEST_CASE("S6 composite is bit-identical to CameraUpdater::update",
                   "[sdk][stages]",
                   ms::FullCovariance<T>,
                   ms::SqrtCovariance<T>) {
    const auto upd = updater();
    for (const Vec3& f : {Vec3{{0.7, 0.2, 5.0}}, Vec3{{-1.0, 0.5, 3.0}}, Vec3{{0.1, -0.3, 8.0}}}) {
        auto a = window<TestType>(5);
        auto b = window<TestType>(5);
        auto track = track_of(a, f);
        if (f[2] > 7.0)
            track.observations.resize(2);  // m = 2: a single projected row
        ms::NisSample<T> nis_a;
        const bool applied_a = upd.update(a, track, &nis_a);
        const auto d = st::s6_msckf_update::apply(b, upd, track);
        REQUIRE(applied_a == d.accepted());
        REQUIRE(nis_a.value == d.nis.value);
        REQUIRE(nis_a.dof == d.nis.dof);
        REQUIRE(nis_a.innov_sum == d.nis.innov_sum);
        REQUIRE(bit_equal(a, b));
    }
}

TEST_CASE("S6 composite reports where a track stopped", "[sdk][stages]") {
    auto s = window(3);
    const auto upd = updater();
    ms::FeatureTrack<T> short_track;
    short_track.observations.push_back({0, 0, Vec2{{0.0, 0.0}}});
    REQUIRE(st::s6_msckf_update::apply(s, upd, short_track).outcome == st::s6_msckf_update::Outcome::Rejected);
    // Contradictory observations far outside the χ² gate.
    auto track = track_of(s, Vec3{{0.7, 0.2, 5.0}});
    track.observations[1].xy[0] += 0.5;
    const auto d = st::s6_msckf_update::apply(s, upd, track);
    REQUIRE_FALSE(d.accepted());
}

TEST_CASE("S9_marginalization keeps the principal submatrix; S10 adds calibration states", "[sdk][stages]") {
    auto s = window(3);
    const auto before = s.covariance();
    const std::size_t off = s.clone_offset(0);
    std::vector<std::size_t> keep;
    for (std::size_t i = 0; i < before.rows; ++i)
        if (i < off || i >= off + 6)
            keep.push_back(i);
    const auto d9 = st::s9_marginalization::apply(s, 0);
    REQUIRE(d9.dim_after + 6 == d9.dim_before);
    REQUIRE(d9.removed_time == 0.0);
    for (const auto& r : inv::check_principal_submatrix<T>(before, keep, s.covariance()))
        REQUIRE(r.pass);

    ms::State<T> fresh(0.3);
    const auto d10 = st::s10_online_calibration::apply(
        fresh, std::vector<ms::State<T>::CalibState>{ms::State<T>::CalibState{}}, 0.01, 0.005);
    REQUIRE(d10.cameras == 1);
    REQUIRE(d10.dim_after == 21);
    REQUIRE(inv::check_spd(fresh.covariance(), inv::Stage::S10_online_calibration).pass);
}

TEST_CASE("S9 track bookkeeping: touching tracks in id order, purge drops emptied tracks", "[sdk][stages]") {
    st::s4_frontend::TrackTable<T> tracks;
    tracks[4] = {{0.0, 0, {}}, {1.0, 0, {}}};
    tracks[2] = {{0.0, 0, {}}};
    tracks[8] = {{1.0, 0, {}}};
    REQUIRE(st::s9_marginalization::tracks_touching(tracks, 0.0) == std::vector<std::uint64_t>{2, 4});
    st::s9_marginalization::purge(tracks, 0.0);
    REQUIRE(tracks.size() == 2);
    REQUIRE(tracks.at(4).size() == 1);
    REQUIRE(tracks.count(2) == 0);
}

// ── S6b invariants ──────────────────────────────────────────────────────────
// For ANY orthonormal basis of the left null space of H_f (Householder or
// Givens): H0'H0 = Hx'Hx - (Hx'Hf) G (Hf'Hx) and H0'r0 = Hx'r - (Hx'Hf) G (Hf'r),
// with G = (Hf'Hf)^-1. Plain double, independent of the code under test.
namespace {

using Vd = std::vector<double>;

Vd mm(const Vd& A, std::size_t ar, std::size_t ac, const Vd& B, std::size_t bc) {
    Vd C(ar * bc, 0.0);
    for (std::size_t i = 0; i < ar; ++i)
        for (std::size_t k = 0; k < ac; ++k)
            for (std::size_t j = 0; j < bc; ++j)
                C[i * bc + j] += A[i * ac + k] * B[k * bc + j];
    return C;
}
Vd tp(const Vd& A, std::size_t r, std::size_t c) {
    Vd R(c * r);
    for (std::size_t i = 0; i < r; ++i)
        for (std::size_t j = 0; j < c; ++j)
            R[j * r + i] = A[i * c + j];
    return R;
}
Vd inv3(const Vd& M) {
    const double a = M[0], b = M[1], c = M[2], d = M[3], e = M[4], f = M[5], g = M[6], h = M[7], i = M[8];
    const double det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    return {(e * i - f * h) / det,
            (c * h - b * i) / det,
            (b * f - c * e) / det,
            (f * g - d * i) / det,
            (a * i - c * g) / det,
            (c * d - a * f) / det,
            (d * h - e * g) / det,
            (b * g - a * h) / det,
            (a * e - b * d) / det};
}
double max_abs(const Vd& a) {
    double m = 0;
    for (double x : a)
        m = std::max(m, std::abs(x));
    return m;
}
double rel_diff(const Vd& a, const Vd& b) {
    double d = 0;
    for (std::size_t i = 0; i < a.size(); ++i)
        d = std::max(d, std::abs(a[i] - b[i]));
    return d / std::max(max_abs(b), 1e-300);
}

struct Info {
    Vd hth, htr;
};

// What the marginalized information must be, from the S6a system alone.
Info expected_info(const ms::MeasurementSystem<T>& s) {
    const std::size_t m = s.rows, n = s.cols;
    const Vd Hf(s.Hf.begin(), s.Hf.end()), Hx(s.Hx.begin(), s.Hx.end()), r(s.r.begin(), s.r.end());
    const Vd HxT = tp(Hx, m, n), HfT = tp(Hf, m, 3);
    const Vd G = inv3(mm(HfT, 3, m, Hf, 3));
    const Vd HxT_Hf = mm(HxT, n, m, Hf, 3);  // n x 3
    const Vd Hf_T_Hx = tp(HxT_Hf, n, 3);     // 3 x n
    Vd hth = mm(HxT, n, m, Hx, n);
    const Vd corr = mm(mm(HxT_Hf, n, 3, G, 3), n, 3, Hf_T_Hx, n);
    for (std::size_t i = 0; i < hth.size(); ++i)
        hth[i] -= corr[i];
    Vd htr = mm(HxT, n, m, r, 1);
    const Vd corr_r = mm(mm(HxT_Hf, n, 3, G, 3), n, 3, mm(HfT, 3, m, r, 1), 1);
    for (std::size_t i = 0; i < htr.size(); ++i)
        htr[i] -= corr_r[i];
    return {hth, htr};
}

Info actual_info(const ms::ProjectedMeasurement<T>& p) {
    const Vd Ht = tp(p.H.d, p.H.rows, p.H.cols);
    return {mm(Ht, p.H.cols, p.H.rows, p.H.d, p.H.cols), mm(Ht, p.H.cols, p.H.rows, Vd(p.r.begin(), p.r.end()), 1)};
}

ms::MeasurementSystem<T> system_for(std::size_t m) {
    auto s = window(m);
    const auto upd = updater();
    const auto track = track_of(s, Vec3{{0.7, 0.2, 5.0}});
    const auto tri = st::s5_triangulation::apply(s, upd, track);
    REQUIRE(tri.ok);
    const auto jac = st::s6a_jacobians::apply(s, upd, track, tri.p_f);
    REQUIRE(jac.ok);
    return jac.system;
}

}  // namespace

TEST_CASE("S6b invariant: rows = 2m-3 and H0'H0, H0'r0 equal the feature-marginalized information",
          "[sdk][stages][s6b]") {
    for (std::size_t m : {3u, 4u, 5u}) {
        const auto sys = system_for(m);
        const auto want = expected_info(sys);
        for (auto method :
             {st::s6b_nullspace_projection::Method::Householder, st::s6b_nullspace_projection::Method::Givens}) {
            const auto out = st::s6b_nullspace_projection::apply(sys, method);
            REQUIRE(out.ok);
            REQUIRE(inv::check_dimension(out.projected.H.rows, 2 * m - 3, inv::Stage::S6b_nullspace_projection).pass);
            REQUIRE(out.projected.r.size() == 2 * m - 3);
            const auto got = actual_info(out.projected);
            CHECK(rel_diff(got.hth, want.hth) < 1e-8);
            CHECK(rel_diff(got.htr, want.htr) < 1e-8);
        }
    }
}

TEST_CASE("S6b invariant: Householder and Givens agree on H'H and H'r", "[sdk][stages][s6b]") {
    const auto sys = system_for(5);
    const auto hh = st::s6b_nullspace_projection::apply(sys);
    const auto gv = st::s6b_nullspace_projection::apply_givens(sys);
    REQUIRE(hh.ok);
    REQUIRE(gv.ok);
    const auto res = inv::check_compression<T>(hh.projected.H,
                                               std::span<const T>(hh.projected.r),
                                               gv.projected.H,
                                               std::span<const T>(gv.projected.r),
                                               inv::Stage::S6b_nullspace_projection);
    REQUIRE(res.size() >= 2);
    CHECK(res[0].pass);  // normal matrix
    CHECK(res[1].pass);  // normal rhs
}

TEST_CASE("S6b: a single observation (2 rows) leaves nothing to project", "[sdk][stages][s6b]") {
    ms::MeasurementSystem<T> sys;
    sys.rows = 2;
    sys.cols = 3;
    sys.Hf = {1, 0, 0, 0, 1, 0};
    sys.Hx = {1, 0, 0, 0, 1, 0};
    sys.r = {0.1, 0.2};
    CHECK_FALSE(st::s6b_nullspace_projection::apply(sys).ok);
    CHECK_FALSE(st::s6b_nullspace_projection::apply_givens(sys).ok);
}
