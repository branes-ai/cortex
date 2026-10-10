// SPDX-License-Identifier: MIT
//
// branes/tools/bench/stage_recorder.hpp — capture a running filter's stage
// boundaries as replayable bench fixtures (issue #446, epic #444 §C).
//
// `StageRecorder<T>` is a `sdk::msckf::StageTap` (stage_tap.hpp): set it on an
// `MsckfBackend<T>` with `set_stage_tap(&recorder)` and every selected boundary
// becomes a `captured` fixture of its stage bench — the stage's input, and the
// output the run produced, in the run's arithmetic `T`:
//
//   tap boundary   fixtures (bench)
//   S0             S0_sensor_model    — one per frame and camera (its pixels)
//   S2             S2_propagation     — one per IMU step (and the hold to a frame)
//   S3             S3_augmentation
//   S6             S5_triangulation, S6a_jacobians, S6b_nullspace_projection,
//                  S6c_compression, S6d_gating, S6e_ekf_update — one feature
//                  track through as many sub-steps as it reached
//   S9             S9_marginalization
//
// Expected outputs: where the run exposes a stage's output (the state after S2,
// S3, S9 and S6e; the S0 bearings; the S6d gate decision and NIS) the fixture
// records the run's own value. The S6 intermediates the run does not expose
// (p_f, H_x/H_f/r, the projected system) are recomputed from the captured
// state with the same stage functions, and the recomputed chain is checked
// against the run: its outcome, NIS and state after must equal the run's bit
// for bit. A difference is counted (`stats().chain_mismatches`), never hidden.
//
// Selection (`CaptureOptions`): by stage, by frame range, and by trigger —
//   • `nis_per_dof_above`: an S6 update whose NIS/dof exceeds the threshold;
//   • `first_violation`:   the first boundary whose fixtures fail their bench's
//                          invariants (each candidate boundary is run through
//                          its benches in T; slow, and only when asked).
// With no trigger set, every selected boundary in the frame range is captured.
// With a trigger set, a boundary is captured only when a trigger fires on it.
//
// Frames: S2 steps belong to the frame they propagate to; everything else to
// the frame being processed. Frame 0 is the first frame after initialization.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_BENCH_STAGE_RECORDER_HPP
#define BRANES_TOOLS_BENCH_STAGE_RECORDER_HPP

#include <branes/sdk/msckf/stage_tap.hpp>
#include <branes/tools/bench/s0_sensor_model_bench.hpp>
#include <branes/tools/bench/s2_propagation_bench.hpp>
#include <branes/tools/bench/s3_augmentation_bench.hpp>
#include <branes/tools/bench/s5_triangulation_bench.hpp>
#include <branes/tools/bench/s6a_jacobians_bench.hpp>
#include <branes/tools/bench/s6b_nullspace_projection_bench.hpp>
#include <branes/tools/bench/s6c_compression_bench.hpp>
#include <branes/tools/bench/s6d_gating_bench.hpp>
#include <branes/tools/bench/s6e_ekf_update_bench.hpp>
#include <branes/tools/bench/s9_marginalization_bench.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace branes::tools::bench {

/// The fixture stages a recorder can produce, in pipeline order.
[[nodiscard]] inline const std::vector<std::string>& capturable_stages() {
    static const std::vector<std::string> s{"S0_sensor_model",
                                            "S2_propagation",
                                            "S3_augmentation",
                                            "S5_triangulation",
                                            "S6a_jacobians",
                                            "S6b_nullspace_projection",
                                            "S6c_compression",
                                            "S6d_gating",
                                            "S6e_ekf_update",
                                            "S9_marginalization"};
    return s;
}

struct CaptureOptions {
    /// Fixture stages to capture (names from `capturable_stages()`); empty: all.
    std::vector<std::string> stages;
    std::uint64_t first_frame = 0;                                         ///< inclusive
    std::uint64_t last_frame = std::numeric_limits<std::uint64_t>::max();  ///< inclusive
    std::optional<double> nis_per_dof_above;  ///< trigger: an S6 update's NIS / dof above this
    bool first_violation = false;             ///< trigger: the first boundary failing its bench invariants
    std::size_t max_fixtures = 100000;        ///< stop capturing past this many fixtures
    std::string source = "run";               ///< recorded in each fixture's `source`, with frame and time
};

/// The first boundary that failed its bench (live-assertion mode, #447): where,
/// which invariant, its value and bound.
struct Violation {
    std::string stage;  ///< e.g. "S6a_jacobians"
    std::uint64_t frame = 0;
    double t = 0.0;
    std::string fixture;  ///< the boundary's fixture name
    std::string invariant;
    double value = 0.0;
    double threshold = 0.0;
    double threshold_hi = 0.0;  ///< a band's upper edge
};

struct CaptureStats {
    std::size_t fixtures = 0;          ///< fixtures handed to the sink
    std::size_t boundaries = 0;        ///< boundaries captured
    std::size_t chain_mismatches = 0;  ///< S6 boundaries examined whose recomputed chain differs from the run
    std::size_t skipped = 0;           ///< selected boundaries the recorder cannot express (see `on_s6`)
    std::optional<std::uint64_t> first_violation_frame;  ///< the frame the first-violation trigger fired on
    std::optional<Violation> first_violation;            ///< …and what failed there
};

template <math::Scalar T>
class StageRecorder final : public sdk::msckf::StageTap<T, sdk::msckf::FullCovariance<T>> {
public:
    using Base = sdk::msckf::StageTap<T, sdk::msckf::FullCovariance<T>>;
    using St = typename Base::St;
    using Vec3 = typename Base::Vec3;
    using Intrinsics = typename S0SensorModelBench::template Intrinsics<T>;
    using Sink = std::function<void(const NamedFixture&)>;

    /// `intrinsics[c]` is camera c's calibration, needed only for S0 fixtures
    /// (the backend's camera model does not expose its parameters). Without it
    /// S0 is not captured. Fixtures go to `sink` (default: `fixtures()`).
    explicit StageRecorder(CaptureOptions options, std::vector<Intrinsics> intrinsics = {}, Sink sink = {})
        : opt_(std::move(options)), intr_(std::move(intrinsics)), sink_(std::move(sink)) {
        for (const auto& s : opt_.stages)
            if (std::find(capturable_stages().begin(), capturable_stages().end(), s) == capturable_stages().end())
                throw std::invalid_argument("stage recorder: unknown stage '" + s + "'");
        if (opt_.first_frame > opt_.last_frame)
            throw std::invalid_argument("stage recorder: first_frame > last_frame");
        if (opt_.nis_per_dof_above && !(*opt_.nis_per_dof_above >= 0.0))
            throw std::invalid_argument("stage recorder: the NIS trigger needs a threshold >= 0");
    }

    /// S0's intrinsics of a camera model and its image size.
    [[nodiscard]] static Intrinsics intrinsics_of(const math::cameras::PinholeRadtanCamera<T>& cam, T width, T height) {
        const auto& d = cam.radtan();
        return Intrinsics{cam.fx(), cam.fy(), cam.cx(), cam.cy(), d.k1, d.k2, d.p1, d.p2, d.k3, width, height};
    }

    /// A sink that writes each fixture to `dir/<stage>/<name>.json`.
    [[nodiscard]] static Sink write_to(std::filesystem::path dir) {
        return [dir = std::move(dir)](const NamedFixture& nf) {
            save(nf.fixture, dir / nf.fixture.stage / (nf.name + ".json"));
        };
    }

    [[nodiscard]] const std::vector<NamedFixture>& fixtures() const noexcept {
        return kept_;
    }
    [[nodiscard]] const CaptureStats& stats() const noexcept {
        return stats_;
    }

    /// Flush the pending S0 frame (call once the run has ended).
    void finish() {
        flush_s0();
    }

    // ── StageTap ────────────────────────────────────────────────────────────
    void on_frame(double t) override {
        flush_s0();
        frame_ = next_frame_++;
        frame_t_ = t;
        s2_step_ = 0;
    }

    [[nodiscard]] bool wants(sdk::msckf::TapStage stage) const override {
        using TS = sdk::msckf::TapStage;
        if (done())
            return false;
        const std::uint64_t f = stage == TS::S2_propagation ? next_frame_ : frame_;
        if (f < opt_.first_frame || f > opt_.last_frame)
            return false;
        // With a trigger set, only a boundary it can fire on is worth copying.
        const bool triggered = opt_.nis_per_dof_above || opt_.first_violation;
        const bool can_fire =
            !triggered || (opt_.first_violation && armed_) || (opt_.nis_per_dof_above && stage == TS::S6_msckf_update);
        if (!can_fire)
            return false;
        switch (stage) {
        case TS::S0_sensor_model:
            return !intr_.empty() && selected("S0_sensor_model");
        case TS::S2_propagation:
            return selected("S2_propagation");
        case TS::S3_augmentation:
            return selected("S3_augmentation");
        case TS::S6_msckf_update:
            for (const char* s : {"S5_triangulation",
                                  "S6a_jacobians",
                                  "S6b_nullspace_projection",
                                  "S6c_compression",
                                  "S6d_gating",
                                  "S6e_ekf_update"})
                if (selected(s))
                    return true;
            return false;
        case TS::S9_marginalization:
            return selected("S9_marginalization");
        }
        return false;
    }

    void on_s0(std::uint32_t cam, T u, T v, const sdk::msckf::stages::s0_sensor_model::Result<T>& out) override {
        // Before the first frame the backend is still initializing (its
        // dynamic-init buffer normalizes pixels too): no frame to file them under.
        if (cam >= intr_.size() || next_frame_ == 0)
            return;
        auto& fr = s0_[cam];
        fr.in.intr = intr_[cam];
        fr.in.pixels.push_back({u, v});
        fr.out.valid.push_back(out.valid ? 1 : 0);
        fr.out.normalized.push_back({out.xy[0], out.xy[1]});
        fr.out.reprojected.push_back(out.valid ? intr_[cam].camera().project({out.xy[0], out.xy[1], T{1}})
                                               : std::array<T, 2>{});
    }

    void on_s2(const St& before,
               const sdk::msckf::Propagator<T>& prop,
               const Vec3& gyro,
               const Vec3& accel,
               T dt,
               const St& after) override {
        using B = S2PropagationBench;
        typename B::template Input<T> in;
        in.state = before;
        in.samples.push_back({dt, gyro, accel});
        in.noise = prop.noise();
        in.gravity = prop.gravity();
        Group g(next_frame_, after.timestamp);
        add<B>(g, "s" + std::to_string(s2_step_++), in, B::encode_output(typename B::template Output<T>{after, {}}));
        commit(std::move(g), false);
    }

    void on_s3(const St& before,
               double t,
               const sdk::msckf::stages::s3_augmentation::Diagnostics& diag,
               const St& after) override {
        using B = S3AugmentationBench;
        typename B::template Input<T> in;
        in.state = before;
        in.t = t;
        Group g(frame_, t);
        add<B>(g, "", in, B::encode_output(typename B::template Output<T>{after, diag}));
        commit(std::move(g), false);
    }

    void on_s9(const St& before,
               std::size_t index,
               const sdk::msckf::stages::s9_marginalization::Diagnostics& diag,
               const St& after) override {
        using B = S9MarginalizationBench;
        typename B::template Input<T> in;
        in.state = before;
        in.clone_index = index;
        Group g(frame_, frame_t_);
        add<B>(
            g, "c" + std::to_string(index), in, B::encode_output(typename B::template Output<T>{after, diag, index}));
        commit(std::move(g), false);
    }

    /// One feature track through S5 → S6e. The S5 / S6a benches model one
    /// camera, so a multi-camera updater's tracks are skipped (counted).
    void on_s6(const St& before,
               const sdk::msckf::CameraUpdater<T>& upd,
               std::uint64_t feature_id,
               const sdk::msckf::FeatureTrack<T>& track,
               const sdk::msckf::stages::s6_msckf_update::Diagnostics<T>& diag,
               const St& after) override {
        namespace st = sdk::msckf::stages;
        using Outcome = st::s6_msckf_update::Outcome;
        if (upd.cameras().size() != 1) {
            ++stats_.skipped;
            return;
        }
        const bool nis_fired =
            opt_.nis_per_dof_above && diag.nis.valid && diag.nis.dof > 0 &&
            static_cast<double>(diag.nis.value) / static_cast<double>(diag.nis.dof) > *opt_.nis_per_dof_above;
        // A trigger that cannot fire here: skip building the fixtures.
        if ((opt_.nis_per_dof_above || opt_.first_violation) && !nis_fired && !(opt_.first_violation && armed_))
            return;
        const auto& opts = upd.options();
        const std::string tag = "feat" + std::to_string(feature_id);
        Group g(frame_, frame_t_);
        g.feature = feature_id;

        // Recompute the chain the backend ran, keeping each boundary.
        Outcome outcome = Outcome::Rejected;
        St s = before;
        st::s6_msckf_update::Diagnostics<T> mine;
        {
            typename S5TriangulationBench::template Input<T> in{before, upd.cameras()[0], opts, track};
            add<S5TriangulationBench>(g, tag, in, json());
        }
        const auto tri = st::s5_triangulation::apply(before, upd, track);
        if (tri.ok) {
            outcome = Outcome::BehindCamera;
            typename S6aJacobiansBench::template Input<T> in{before, upd.cameras()[0], opts, track, tri.p_f};
            add<S6aJacobiansBench>(g, tag, in, json());
            const auto jac = st::s6a_jacobians::apply(before, upd, track, tri.p_f);
            if (jac.ok) {
                outcome = Outcome::NoNullspace;
                const auto basis = s6::unobservable_basis<T>(before);
                add<S6bNullspaceProjectionBench>(
                    g, tag, typename S6bNullspaceProjectionBench::template Input<T>{jac.system, basis}, json());
                auto proj = st::s6b_nullspace_projection::apply(jac.system);
                if (proj.ok) {
                    add<S6cCompressionBench>(
                        g, tag, typename S6cCompressionBench::template Input<T>{proj.projected}, json());
                    const auto comp = st::s6c_compression::apply(std::move(proj.projected));
                    mine.rows = comp.rows_out;
                    const auto gate = st::s6d_gating::apply(before, upd, comp.compressed);
                    mine.nis = gate.nis;
                    outcome = gate.accepted ? Outcome::Applied : Outcome::Gated;
                    // S6d: the run's own decision and NIS.
                    const std::size_t k = comp.compressed.H.rows;
                    const double thr =
                        opts.enable_gating ? static_cast<double>(opts.chi2_per_dof) * static_cast<double>(k) : 0.0;
                    add<S6dGatingBench>(
                        g,
                        tag,
                        typename S6dGatingBench::template Input<T>{before, opts, comp.compressed},
                        S6dGatingBench::encode_output(typename S6dGatingBench::template Output<T>{
                            diag.accepted() ? 1 : 0, diag.nis.value, diag.nis.dof, diag.nis.valid ? 1 : 0, thr}));
                    if (gate.accepted) {
                        // S6e: the recomputed δx, and the run's own covariance after.
                        const auto dx = st::s6e_ekf_update::apply(s, upd, comp.compressed).dx;
                        typename S6eEkfUpdateBench::template Input<T> in6{before, opts, comp.compressed, basis, {}};
                        add<S6eEkfUpdateBench>(
                            g,
                            tag,
                            in6,
                            S6eEkfUpdateBench::encode_output(
                                typename S6eEkfUpdateBench::template Output<T>{dx, after.cov.covariance()}));
                    }
                }
            }
        }
        mine.outcome = outcome;
        const bool same = mine.outcome == diag.outcome && mine.rows == diag.rows &&
                          same_bits(mine.nis.value, diag.nis.value) && mine.nis.dof == diag.nis.dof &&
                          mine.nis.valid == diag.nis.valid && pack(s) == pack(after);
        if (!same)
            ++stats_.chain_mismatches;
        commit(std::move(g), nis_fired);
    }

private:
    /// The fixtures of one boundary, captured (or not) together.
    struct Group {
        Group(std::uint64_t f, double tt) : frame(f), t(tt) {}
        std::uint64_t frame;
        double t;
        std::optional<std::uint64_t> feature;
        std::vector<NamedFixture> fixtures;
        std::vector<std::optional<inv::InvariantResult> (*)(const NamedFixture&)>
            violates;  ///< per fixture: its first failure in T
    };

    /// Add a fixture of bench B. `expected` null: the bench's own output on
    /// the input (an intermediate the run does not expose).
    template <class B>
    void add(Group& g, const std::string& tag, const typename B::template Input<T>& in, json expected) const {
        if (!selected(B::kStage))
            return;
        if (expected.is_null())
            expected = B::encode_output(B::template run<T>(in, kShipped));
        std::string where = opt_.source + ", frame " + std::to_string(g.frame) + ", t=" + std::to_string(g.t) + " s";
        if (g.feature)
            where += ", feature " + std::to_string(*g.feature);
        NamedFixture nf;
        nf.name = frame_name(g.frame) + "_" + std::string(B::kStage) + (tag.empty() ? "" : "_" + tag);
        nf.fixture = capture(std::string(B::kStage), std::string(type_name<T>()), where, B::encode_input(in), expected);
        g.fixtures.push_back(std::move(nf));
        g.violates.push_back(&fails<B>);
    }

    /// A fixture's first failure in its bench (invariants and replay) in T.
    template <class B>
    [[nodiscard]] static std::optional<inv::InvariantResult> fails(const NamedFixture& nf) {
        return run_fixture<B>(nf, kShipped, TypeList<T>{}).types.front().report.first_failure();
    }

    [[nodiscard]] static std::string frame_name(std::uint64_t f) {
        std::string n = std::to_string(f);
        return "f" + std::string(n.size() < 6 ? 6 - n.size() : 0, '0') + n;
    }

    [[nodiscard]] static bool same_bits(T a, T b) {
        const double x = static_cast<double>(a), y = static_cast<double>(b);
        return (std::isnan(x) && std::isnan(y)) || x == y;
    }

    [[nodiscard]] bool selected(std::string_view stage) const {
        return opt_.stages.empty() || std::find(opt_.stages.begin(), opt_.stages.end(), stage) != opt_.stages.end();
    }
    [[nodiscard]] bool done() const {
        return stats_.fixtures >= opt_.max_fixtures;
    }

    void commit(Group g, bool nis_fired) {
        if (g.fixtures.empty() || done())
            return;
        bool take = !opt_.nis_per_dof_above && !opt_.first_violation;
        take = take || nis_fired;
        if (!take && opt_.first_violation && armed_) {
            for (std::size_t i = 0; i < g.fixtures.size(); ++i)
                if (const auto bad = g.violates[i](g.fixtures[i])) {
                    take = true;
                    armed_ = false;
                    stats_.first_violation_frame = g.frame;
                    stats_.first_violation = Violation{g.fixtures[i].fixture.stage,
                                                       g.frame,
                                                       g.t,
                                                       g.fixtures[i].name,
                                                       std::string(bad->name),
                                                       bad->value,
                                                       bad->threshold,
                                                       bad->threshold_hi};
                    break;
                }
        }
        if (!take)
            return;
        ++stats_.boundaries;
        for (auto& nf : g.fixtures) {
            if (done())
                break;
            ++stats_.fixtures;
            if (sink_)
                sink_(nf);
            else
                kept_.push_back(std::move(nf));
        }
    }

    void flush_s0() {
        for (auto& [cam, fr] : s0_) {
            using B = S0SensorModelBench;
            Group g(frame_, frame_t_);
            add<B>(g, "cam" + std::to_string(cam), fr.in, B::encode_output(fr.out));
            commit(std::move(g), false);
        }
        s0_.clear();
    }

    struct S0Frame {
        typename S0SensorModelBench::template Input<T> in;
        typename S0SensorModelBench::template Output<T> out;
    };

    CaptureOptions opt_;
    std::vector<Intrinsics> intr_;
    Sink sink_;
    std::vector<NamedFixture> kept_;
    CaptureStats stats_;
    bool armed_ = true;  ///< the first-violation trigger has not fired yet
    std::uint64_t next_frame_ = 0;
    std::uint64_t frame_ = 0;
    double frame_t_ = 0.0;
    std::size_t s2_step_ = 0;
    std::map<std::uint32_t, S0Frame> s0_;
};

/// Several taps on one backend: each boundary goes to every tap that wants it.
template <math::Scalar T, class Cov = sdk::msckf::FullCovariance<T>>
class StageTapFanout final : public sdk::msckf::StageTap<T, Cov> {
public:
    using Base = sdk::msckf::StageTap<T, Cov>;
    using St = typename Base::St;
    using Vec3 = typename Base::Vec3;

    void add(Base* tap) {
        if (tap)
            taps_.push_back(tap);
    }
    [[nodiscard]] bool empty() const noexcept {
        return taps_.empty();
    }

    void on_frame(double t) override {
        for (auto* x : taps_)
            x->on_frame(t);
    }
    [[nodiscard]] bool wants(sdk::msckf::TapStage stage) const override {
        return std::any_of(taps_.begin(), taps_.end(), [&](const Base* x) { return x->wants(stage); });
    }
    void on_s0(std::uint32_t cam, T u, T v, const sdk::msckf::stages::s0_sensor_model::Result<T>& out) override {
        each(sdk::msckf::TapStage::S0_sensor_model, [&](Base* x) { x->on_s0(cam, u, v, out); });
    }
    void on_s2(const St& before,
               const sdk::msckf::Propagator<T>& p,
               const Vec3& g,
               const Vec3& a,
               T dt,
               const St& after) override {
        each(sdk::msckf::TapStage::S2_propagation, [&](Base* x) { x->on_s2(before, p, g, a, dt, after); });
    }
    void on_s3(const St& before,
               double t,
               const sdk::msckf::stages::s3_augmentation::Diagnostics& d,
               const St& after) override {
        each(sdk::msckf::TapStage::S3_augmentation, [&](Base* x) { x->on_s3(before, t, d, after); });
    }
    void on_s6(const St& before,
               const sdk::msckf::CameraUpdater<T>& upd,
               std::uint64_t feature,
               const sdk::msckf::FeatureTrack<T>& track,
               const sdk::msckf::stages::s6_msckf_update::Diagnostics<T>& d,
               const St& after) override {
        each(sdk::msckf::TapStage::S6_msckf_update, [&](Base* x) { x->on_s6(before, upd, feature, track, d, after); });
    }
    void on_s9(const St& before,
               std::size_t index,
               const sdk::msckf::stages::s9_marginalization::Diagnostics& d,
               const St& after) override {
        each(sdk::msckf::TapStage::S9_marginalization, [&](Base* x) { x->on_s9(before, index, d, after); });
    }

private:
    template <class F>
    void each(sdk::msckf::TapStage stage, F&& f) {
        for (auto* x : taps_)
            if (x->wants(stage))
                f(x);
    }
    std::vector<Base*> taps_;
};

}  // namespace branes::tools::bench

#endif  // BRANES_TOOLS_BENCH_STAGE_RECORDER_HPP
