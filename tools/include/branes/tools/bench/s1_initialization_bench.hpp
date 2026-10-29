// SPDX-License-Identifier: MIT
//
// branes/tools/bench/s1_initialization_bench.hpp — the S1_initialization stage
// bench (issue #454, epic #444).
//
// Stage under test: an ImuInitializer resolution followed by the S1 seeding
// transformation (stages::s1_initialization::seed_from_imu /
// seed_from_alignment) — init window → seeded filter state + diagnostics.
// Contract (#445 §B, S1):
//   • the initializer resolves (the variant has the data it needs);
//   • the seeded attitude is in SO(3);
//   • gravity points down with the configured magnitude (#262);
//   • the biases are bounded; P₀ is symmetric positive-definite;
//   • the measured gravity-magnitude residual is within the gate.
//
// Every fixture carries a static IMU window, a dynamic visual-inertial keyframe
// window and an SfM window of image observations, so all variants run on
// identical inputs:
//   shipped        try_static → seed_from_imu (the backend's default path)
//   gravity_align  try_gravity_align → seed_from_imu (the moving-start fallback)
//   dynamic        try_dynamic → seed_from_alignment (VI alignment: yaw-free
//                  attitude, velocity, scale), from ready-made keyframes
//   sfm_two_view_pnp
//                  the backend's dynamic init from images (#472):
//                  build_init_window (two-view bootstrap on the largest-parallax
//                  frame, PnP for the rest) → try_dynamic → seed_from_alignment
//   sfm_adjacent_bootstrap
//                  the same with the bootstrap on frame 1 (the pre-#247 choice)
//
// Fixtures:
//   known_answer   a noiseless, tilted, gyro-biased static window: the expected
//                  seed (attitude, biases, P₀, diagnostics) in closed form
//   ground_truth   the synthetic world's stationary warm-up IMU (with its true
//                  biases) + keyframes from a known trajectory; truth = attitude,
//                  gyro bias, metric scale and gravity
//   captured       a noisy window through the shipped variant, recorded
//
// The SfM window contract (#472): the window builds, its keyframe attitudes are
// in SO(3), the PnP reprojection residual is inside the PnP gate, and, against
// truth, the keyframe trajectory matches up to scale and gauge.
//
// Sweep: window length × IMU noise × translational excitation → static
// roll/pitch error and dynamic scale error; observation noise × SfM window
// length × camera speed (parallax) → where the SfM window builds and resolves.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_BENCH_S1_INITIALIZATION_BENCH_HPP
#define BRANES_TOOLS_BENCH_S1_INITIALIZATION_BENCH_HPP

#include <branes/sdk/eval/initialization_probe.hpp>  // make_dynamic_keyframes (synthetic VI keyframes)
#include <branes/sdk/eval/invariants.hpp>
#include <branes/sdk/eval/synthetic_world.hpp>
#include <branes/sdk/eval/trajectory_metrics.hpp>  // horn_rotation
#include <branes/sdk/imu_init.hpp>
#include <branes/sdk/msckf/stages/s1_initialization.hpp>
#include <branes/sdk/sfm/init_window.hpp>
#include <branes/tools/bench/bench.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <random>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace branes::tools::bench {

struct S1InitializationBench {
    static constexpr std::string_view kStage = "S1_initialization";
    static constexpr inv::Stage kInvStage = inv::Stage::S1_initialization;
    static constexpr double kInitialSigma = 0.1;  ///< the backend's fresh-state σ (MsckfBackendT::kInitialSigma)

    template <class T>
    using Vec3 = math::lie::detail::Vec<T, 3>;
    template <class T>
    using Keyframe = sdk::DynInitKeyframe<T>;
    template <class T>
    using Frame = sdk::sfm::InitFrame<T>;

    template <class T>
    struct Input {
        std::vector<Vec3<T>> gyro, accel;              ///< the static init window
        std::vector<Keyframe<T>> keyframes;            ///< the dynamic init window (VI keyframes)
        std::vector<Frame<T>> frames;                  ///< the SfM window (image observations + preintegration)
        sdk::msckf::CameraExtrinsics<T> extrinsics{};  ///< camera 0 → IMU, for the SfM window
        double t = 0.0;                                ///< time the init completes
        T gravity_magnitude = T{981} / T{100};
    };
    template <class T>
    struct Output {
        int success = 0;
        int method = 0;  ///< sdk::InitMethod
        sdk::msckf::State<T> state{T(kInitialSigma)};
        sdk::InitDiagnostics<T> diag{};
        Vec3<T> gravity_world{};  ///< the initializer's gravity estimate (world frame)
        T scale = T{1};
        int sfm = 0;                             ///< the SfM front end ran (sfm_* variants)
        sdk::sfm::InitWindowResult<T> window{};  ///< …and what it built
    };

    /// The SfM variants run hundreds of RANSAC eigensolves per window: by
    /// default only in native types, never in an emulated posit (~10⁴× slower).
    [[nodiscard]] static bool runs_by_default(std::string_view variant, std::string_view type) {
        return !variant.starts_with("sfm_") || (type != "posit32" && type != "posit16");
    }

    [[nodiscard]] static std::vector<Variant> variants() {
        return {{"shipped", "try_static -> seed_from_imu (backend default)"},
                {"gravity_align", "try_gravity_align -> seed_from_imu (moving-start fallback)"},
                {"dynamic", "try_dynamic -> seed_from_alignment (visual-inertial alignment)"},
                {"sfm_two_view_pnp",
                 "build_init_window (largest-parallax two-view + PnP) -> try_dynamic -> seed_from_alignment"},
                {"sfm_adjacent_bootstrap", "the same, with the two-view bootstrap on frame 1 (pre-#247)"},
                {"sfm_keyframes_4hz",
                 "the same SfM, keyframes >= 0.25 s apart for the alignment (preintegration composed)"}};
    }

    // ── Codec ───────────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static json pack_vecs(const std::vector<Vec3<T>>& v) {
        json a = json::array();
        for (const auto& x : v)
            a.push_back(pack(x));
        return a;
    }
    template <class T>
    [[nodiscard]] static std::vector<Vec3<T>> unpack_vecs(const json& j) {
        std::vector<Vec3<T>> v;
        for (const auto& x : j)
            v.push_back(unpack_fixed<T, 3>(x));
        return v;
    }
    template <class T>
    [[nodiscard]] static json encode_input(const Input<T>& in) {
        json kfs = json::array();
        for (const auto& k : in.keyframes)
            kfs.push_back({{"R_world_imu", pack(k.R_world_imu)},
                           {"p_world_imu", pack(k.p_world_imu)},
                           {"dR", pack(k.dR)},
                           {"dv", pack(k.dv)},
                           {"dp", pack(k.dp)},
                           {"dR_dbg", pack_vec<T>(std::span<const T>(k.dR_dbg.e))},
                           {"dt", pack(k.dt)}});
        json j{{"gyro", pack_vecs(in.gyro)},
               {"accel", pack_vecs(in.accel)},
               {"keyframes", kfs},
               {"t", pack_num(in.t)},
               {"gravity_magnitude", pack(in.gravity_magnitude)}};
        if (!in.frames.empty()) {
            json fr = json::array();
            for (const auto& f : in.frames) {
                json obs = json::array();
                for (const auto& o : f.obs)
                    obs.push_back(pack(o));
                fr.push_back({{"timestamp", pack_num(f.timestamp)},
                              {"ids", f.ids},
                              {"obs", obs},
                              {"dR", pack(f.dR)},
                              {"dv", pack(f.dv)},
                              {"dp", pack(f.dp)},
                              {"dR_dbg", pack_vec<T>(std::span<const T>(f.dR_dbg.e))},
                              {"dt", pack(f.dt)}});
            }
            j["frames"] = fr;
            j["extrinsics"] = {{"R_imu_cam", pack(in.extrinsics.R_imu_cam)},
                               {"p_imu_cam", pack(in.extrinsics.p_imu_cam)}};
        }
        return j;
    }
    template <class T>
    [[nodiscard]] static Input<T> decode_input(const json& j) {
        Input<T> in;
        in.gyro = unpack_vecs<T>(j.at("gyro"));
        in.accel = unpack_vecs<T>(j.at("accel"));
        for (const auto& k : j.at("keyframes")) {
            Keyframe<T> kf;
            kf.R_world_imu = unpack_so3<T>(k.at("R_world_imu"));
            kf.p_world_imu = unpack_fixed<T, 3>(k.at("p_world_imu"));
            kf.dR = unpack_so3<T>(k.at("dR"));
            kf.dv = unpack_fixed<T, 3>(k.at("dv"));
            kf.dp = unpack_fixed<T, 3>(k.at("dp"));
            const auto m = unpack_vec<T>(k.at("dR_dbg"));
            if (m.size() != 9)
                throw std::invalid_argument("s1 bench: dR_dbg must have 9 entries");
            for (std::size_t i = 0; i < 9; ++i)
                kf.dR_dbg.e[i] = m[i];
            kf.dt = unpack_scalar<T>(k.at("dt"));
            in.keyframes.push_back(kf);
        }
        if (j.contains("frames")) {  // fixtures from before #472 have no SfM window
            for (const auto& fj : j.at("frames")) {
                Frame<T> f;
                f.timestamp = unpack_num(fj.at("timestamp"));
                f.ids = fj.at("ids").get<std::vector<std::uint64_t>>();
                for (const auto& o : fj.at("obs"))
                    f.obs.push_back(unpack_fixed<T, 2>(o));
                if (f.ids.size() != f.obs.size())
                    throw std::invalid_argument("s1 bench: a frame's ids and obs differ in length");
                f.dR = unpack_so3<T>(fj.at("dR"));
                f.dv = unpack_fixed<T, 3>(fj.at("dv"));
                f.dp = unpack_fixed<T, 3>(fj.at("dp"));
                const auto m = unpack_vec<T>(fj.at("dR_dbg"));
                if (m.size() != 9)
                    throw std::invalid_argument("s1 bench: a frame's dR_dbg must have 9 entries");
                for (std::size_t i = 0; i < 9; ++i)
                    f.dR_dbg.e[i] = m[i];
                f.dt = unpack_scalar<T>(fj.at("dt"));
                in.frames.push_back(std::move(f));
            }
            const auto& e = j.at("extrinsics");
            in.extrinsics.R_imu_cam = unpack_so3<T>(e.at("R_imu_cam"));
            in.extrinsics.p_imu_cam = unpack_fixed<T, 3>(e.at("p_imu_cam"));
        }
        in.t = unpack_num(j.at("t"));
        in.gravity_magnitude = unpack_scalar<T>(j.at("gravity_magnitude"));
        return in;
    }
    template <class T>
    [[nodiscard]] static json encode_output(const Output<T>& o) {
        const auto& d = o.diag;
        json j{{"success", o.success},
               {"method", o.method},
               {"state", pack(o.state)},
               {"gravity_world", pack(o.gravity_world)},
               {"scale", pack(o.scale)},
               {"diag",
                {{"t_s", pack_num(d.t_s)},
                 {"up_body", pack(d.up_body)},
                 {"gravity_residual", pack_num(d.gravity_residual)},
                 {"gyro_bias", pack(d.gyro_bias)},
                 {"accel_bias", pack(d.accel_bias)},
                 {"dyn_scale", pack_num(d.dyn_scale)},
                 {"dyn_seed_speed", pack_num(d.dyn_seed_speed)},
                 {"dyn_keyframes", d.dyn_keyframes},
                 {"dyn_tilt_vs_accel_deg", pack_num(d.dyn_tilt_vs_accel_deg)}}}};
        if (o.sfm) {
            const auto& w = o.window;
            json kfs = json::array();
            for (const auto& k : w.keyframes)
                kfs.push_back({{"R_world_imu", pack(k.R_world_imu)}, {"p_world_imu", pack(k.p_world_imu)}});
            j["window"] = {{"success", w.success ? 1 : 0},
                           {"status", static_cast<int>(w.status)},
                           {"bootstrap_frame", w.bootstrap_frame},
                           {"bootstrap_parallax", pack(w.bootstrap_parallax)},
                           {"bootstrap_correspondences", w.bootstrap_correspondences},
                           {"landmarks", w.landmarks},
                           {"failed_frame", w.failed_frame},
                           {"pnp_inliers", w.pnp_inliers},
                           {"keyframe_frames", w.keyframe_frames},
                           {"pnp_rms", pack_vec<T>(std::span<const T>(w.pnp_rms))},
                           {"keyframes", kfs}};
        }
        return j;
    }
    template <class T>
    [[nodiscard]] static Output<T> decode_output(const json& j) {
        Output<T> o;
        o.success = j.at("success").get<int>();
        o.method = j.at("method").get<int>();
        o.state = unpack_state<T>(j.at("state"));
        o.gravity_world = unpack_fixed<T, 3>(j.at("gravity_world"));
        o.scale = unpack_scalar<T>(j.at("scale"));
        const auto& d = j.at("diag");
        o.diag.method = static_cast<sdk::InitMethod>(o.method);
        o.diag.t_s = unpack_num(d.at("t_s"));
        o.diag.up_body = unpack_fixed<T, 3>(d.at("up_body"));
        o.diag.gravity_residual = unpack_num(d.at("gravity_residual"));
        o.diag.gyro_bias = unpack_fixed<T, 3>(d.at("gyro_bias"));
        o.diag.accel_bias = unpack_fixed<T, 3>(d.at("accel_bias"));
        o.diag.dyn_scale = unpack_num(d.at("dyn_scale"));
        o.diag.dyn_seed_speed = unpack_num(d.at("dyn_seed_speed"));
        o.diag.dyn_keyframes = d.at("dyn_keyframes").get<int>();
        o.diag.dyn_tilt_vs_accel_deg = unpack_num(d.at("dyn_tilt_vs_accel_deg"));
        if (j.contains("window")) {
            const auto& w = j.at("window");
            o.sfm = 1;
            o.window.success = w.at("success").get<int>() != 0;
            o.window.status = static_cast<sdk::sfm::InitWindowStatus>(w.at("status").get<int>());
            o.window.bootstrap_frame = w.at("bootstrap_frame").get<std::size_t>();
            o.window.bootstrap_parallax = unpack_scalar<T>(w.at("bootstrap_parallax"));
            o.window.bootstrap_correspondences = w.at("bootstrap_correspondences").get<std::size_t>();
            o.window.landmarks = w.at("landmarks").get<std::size_t>();
            o.window.failed_frame = w.at("failed_frame").get<std::size_t>();
            o.window.pnp_inliers = w.at("pnp_inliers").get<std::vector<std::size_t>>();
            o.window.keyframe_frames = w.at("keyframe_frames").get<std::vector<std::size_t>>();
            o.window.pnp_rms = unpack_vec<T>(w.at("pnp_rms"));
            for (const auto& k : w.at("keyframes")) {
                Keyframe<T> kf;
                kf.R_world_imu = unpack_so3<T>(k.at("R_world_imu"));
                kf.p_world_imu = unpack_fixed<T, 3>(k.at("p_world_imu"));
                o.window.keyframes.push_back(kf);
            }
        }
        return o;
    }

    // ── The stage ───────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static Output<T> run(const Input<T>& in, std::string_view variant) {
        namespace s1 = sdk::msckf::stages::s1_initialization;
        sdk::ImuInitConfig<T> cfg;
        cfg.gravity_magnitude = in.gravity_magnitude;
        const sdk::ImuInitializer<T> init(cfg);
        const std::span<const Vec3<T>> g{in.gyro}, a{in.accel};
        Output<T> out;
        if (variant.starts_with("sfm_")) {
            sdk::sfm::InitWindowOptions<T> opt;
            if (variant == "sfm_adjacent_bootstrap")
                opt.bootstrap = sdk::sfm::InitBootstrap::Adjacent;
            else if (variant == "sfm_keyframes_4hz")
                opt.min_keyframe_dt = 0.25;
            out.sfm = 1;
            out.window = sdk::sfm::build_init_window<T>(std::span<const Frame<T>>{in.frames}, in.extrinsics, opt);
            if (!out.window.success)
                return out;
            const auto r = init.try_dynamic(std::span<const Keyframe<T>>{out.window.keyframes});
            if (!r.success)
                return out;
            s1::seed_from_alignment(out.state, r, out.window, in.t, a, in.gravity_magnitude, out.diag);
            out.gravity_world = r.gravity_world;
            out.scale = r.scale;
        } else if (variant == "dynamic") {
            const auto r = init.try_dynamic(std::span<const Keyframe<T>>{in.keyframes});
            if (!r.success)
                return out;
            sdk::sfm::InitWindowResult<T> win;
            win.success = true;
            win.keyframes = in.keyframes;
            s1::seed_from_alignment(out.state, r, win, in.t, a, in.gravity_magnitude, out.diag);
            out.gravity_world = r.gravity_world;
            out.scale = r.scale;
        } else {
            const bool align = variant == "gravity_align";
            const auto r = align ? init.try_gravity_align(g, a) : init.try_static(g, a);
            if (!r.success)
                return out;
            s1::seed_from_imu(out.state,
                              r,
                              align ? sdk::InitMethod::GravityAlign : sdk::InitMethod::Static,
                              in.t,
                              a,
                              in.gravity_magnitude,
                              out.diag);
            out.gravity_world = gravity_from_accel(out.state.R, in.accel);
            out.scale = r.scale;
        }
        out.success = 1;
        out.method = static_cast<int>(out.diag.method);
        return out;
    }

    template <class T>
    [[nodiscard]] static std::vector<double> flatten(const Output<T>& o) {
        std::vector<double> v{static_cast<double>(o.success), static_cast<double>(o.method)};
        auto put3 = [&](const auto& x) {
            for (std::size_t i = 0; i < 3; ++i)
                v.push_back(static_cast<double>(x[i]));
        };
        const auto& s = o.state;
        for (std::size_t i = 0; i < 4; ++i)
            v.push_back(static_cast<double>(s.R.quaternion()[i]));
        put3(s.p);
        put3(s.v);
        put3(s.bg);
        put3(s.ba);
        v.push_back(s.timestamp);
        for (const T& x : s.cov.P.d)
            v.push_back(static_cast<double>(x));
        put3(o.gravity_world);
        v.push_back(static_cast<double>(o.scale));
        const auto& d = o.diag;
        v.push_back(d.t_s);
        put3(d.up_body);
        v.push_back(d.gravity_residual);
        put3(d.gyro_bias);
        put3(d.accel_bias);
        v.insert(v.end(),
                 {d.dyn_scale, d.dyn_seed_speed, static_cast<double>(d.dyn_keyframes), d.dyn_tilt_vs_accel_deg});
        if (o.sfm) {  // the SfM block only for the sfm_* variants, so the others keep their shape
            const auto& w = o.window;
            v.insert(v.end(),
                     {w.success ? 1.0 : 0.0,
                      static_cast<double>(w.status),
                      static_cast<double>(w.bootstrap_frame),
                      static_cast<double>(w.bootstrap_parallax),
                      static_cast<double>(w.bootstrap_correspondences),
                      static_cast<double>(w.landmarks),
                      static_cast<double>(w.failed_frame)});
            for (const auto& k : w.keyframes) {
                for (std::size_t i = 0; i < 4; ++i)
                    v.push_back(static_cast<double>(k.R_world_imu.quaternion()[i]));
                put3(k.p_world_imu);
            }
            for (const std::size_t n : w.pnp_inliers)
                v.push_back(static_cast<double>(n));
            for (const std::size_t f : w.keyframe_frames)
                v.push_back(static_cast<double>(f));
            for (const T& e : w.pnp_rms)
                v.push_back(static_cast<double>(e));
        }
        return v;
    }

    // ── The contract ────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static std::vector<inv::InvariantResult>
    invariants(const Input<T>& in, const Output<T>& o, const Fixture& f) {
        std::vector<inv::InvariantResult> r;
        if (o.sfm)
            for (auto& x : window_checks<T>(o.window))
                r.push_back(std::move(x));
        // Every variant has the data it needs in every fixture, so it must resolve.
        r.push_back(inv::check_scalar(o.success, 1.0, inv::Bound::Lower, kInvStage, "init.resolved", "flag"));
        if (o.sfm && o.window.success && f.truth.is_object() && f.truth.contains("sfm"))
            for (auto& x : sfm_truth_checks<T>(in, o, f.truth.at("sfm")))
                r.push_back(std::move(x));
        if (!o.success)
            return r;
        for (auto& x : inv::check_so3<T>(
                 o.state.R, kInvStage, "attitude.orthogonality", "attitude.determinant", safety_vs_double<T>()))
            r.push_back(x);
        const T g0 = in.gravity_magnitude;
        for (auto& x : inv::check_gravity<T>(o.gravity_world, g0, g0 * T(0.05), T(2.0 * kDeg)))
            r.push_back(x);
        r.push_back(inv::check_bounded<T>(std::span<const T>(o.state.bg.e), T(0.1), kInvStage, "bias.gyro", "rad/s"));
        r.push_back(inv::check_bounded<T>(std::span<const T>(o.state.ba.e), T(1.0), kInvStage, "bias.accel", "m/s^2"));
        r.push_back(inv::check_spd(o.state.covariance(), kInvStage, "P0.spd"));
        r.push_back(inv::check_scalar(o.diag.gravity_residual,
                                      0.05,
                                      inv::Bound::Upper,
                                      kInvStage,
                                      "init.gravity_magnitude_residual",
                                      "fraction"));
        if (f.kind == FixtureKind::GroundTruth && !o.sfm)
            for (auto& x : truth_checks<T>(o, f.truth))
                r.push_back(x);
        return r;
    }

    // ── Built-in fixtures ───────────────────────────────────────────────────
    [[nodiscard]] static std::vector<NamedFixture> builtin_fixtures() {
        return {{"known_answer_tilted_static", known_answer()},
                {"ground_truth_synthetic_warmup", ground_truth()},
                {"captured_noisy_window", captured()}};
    }

    /// Noiseless static window: tilt 8° about x, gyro bias (0.004, −0.002,
    /// 0.003). The expected static seed is closed-form: attitude = the tilt
    /// (zero yaw), gyro bias = the bias, accel bias = 0, P₀ = σ²·I.
    [[nodiscard]] static Fixture known_answer() {
        const double tilt = 8.0 * kDeg;
        const Vec3<double> bg{{0.004, -0.002, 0.003}};
        const auto R_true = math::lie::SO3<double>::exp({{tilt, 0.0, 0.0}});
        Input<double> in = static_window(R_true, bg, {}, 0.0, 0.0, 50, 0);
        in.keyframes = dynamic_keyframes(1.5, 0.0);
        json truth;  // the SfM window's: exact observations and preintegration
        // Exact data: the attitudes and the track to rounding. The scale is 0.8 %
        // off even so: the window adds the metric camera→IMU lever arm to
        // up-to-scale camera positions (#499), hence 2 %.
        add_window(in, truth, sfm_window(1.5, 0.5, 10, 0.0, 0.0, 0), 1e-6, 1e-6, 2.0);
        in.t = 2.5;

        Output<double> want;
        want.success = 1;
        want.method = static_cast<int>(sdk::InitMethod::Static);
        want.state.R = R_true;
        want.state.bg = bg;
        want.state.timestamp = in.t;
        want.gravity_world = {{0.0, 0.0, -in.gravity_magnitude}};
        want.diag.method = sdk::InitMethod::Static;
        want.diag.t_s = in.t;
        want.diag.up_body = R_true.inverse() * Vec3<double>{{0.0, 0.0, 1.0}};
        want.diag.gyro_bias = bg;

        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::KnownAnswer;
        f.variant = "shipped";  // the expected seed is the static path's
        f.source = "s1_initialization_bench: noiseless 8 deg tilt, biased gyro, 50 samples";
        f.description = "closed-form static seed";
        f.input = encode_input(in);
        f.expected = encode_output(want);
        f.truth = truth;
        return f;
    }

    /// The synthetic world's stationary warm-up (true attitude and IMU biases)
    /// plus dynamic keyframes from a known trajectory (scale 2, ‖a‖ = 1.5 m/s²).
    [[nodiscard]] static Fixture ground_truth() {
        const sdk::eval::SyntheticConfig<double> cfg;
        const auto world = sdk::eval::generate_world<double>(cfg);
        Input<double> in;
        for (std::size_t k = 100; k < 150; ++k) {  // 0.25 s inside the 1.5 s warm-up
            const auto& m = world.imu[k];
            in.gyro.push_back({{m.angular_velocity[0], m.angular_velocity[1], m.angular_velocity[2]}});
            in.accel.push_back({{m.linear_acceleration[0], m.linear_acceleration[1], m.linear_acceleration[2]}});
        }
        in.keyframes = dynamic_keyframes(1.5, 0.0);
        in.t = world.imu[149].timestamp_s;
        in.gravity_magnitude = cfg.gravity;
        // The warm-up attitude is the trajectory's start pose (motion begins at
        // the end of the warm-up, continuously), i.e. the first ground-truth pose.
        json truth{{"R_world_imu", pack(world.gt.front().R)},
                   {"gyro_bias", pack(world.gyro_bias)},
                   {"scale", 2.0},
                   {"gravity_world", pack(Vec3<double>{{0.0, 0.0, -cfg.gravity}})}};
        // An SfM window of 20 frames with 0.1 px of observation noise: it builds
        // and matches the truth in attitude and track, but the alignment's
        // metric scale is far off (#501), so its 5 % fails.
        add_window(in, truth, sfm_window(1.5, 0.5, 20, 0.1, 0.002, 0x5F1), 0.5, 3.0, 5.0);
        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::GroundTruth;
        f.source = "synthetic_world (default config) IMU samples 100-149 (warm-up); dynamic keyframes scale 2; "
                   "SfM window: 20 frames at 20 Hz, 0.1 px noise (seed 0x5F1)";
        f.seed = cfg.seed;
        f.description = "truth: attitude, gyro bias, metric scale, gravity; SfM camera track and attitudes";
        f.input = encode_input(in);
        f.truth = truth;
        return f;
    }

    /// A noisy, tilted static window and noisy keyframes through the shipped path.
    [[nodiscard]] static Fixture captured() {
        const auto R = math::lie::SO3<double>::exp({{0.12, -0.05, 0.3}});
        Input<double> in = static_window(R, {{0.003, 0.001, -0.002}}, {{0.02, -0.01, 0.0}}, 0.05, 0.002, 60, 0xA17);
        in.keyframes = dynamic_keyframes(1.5, 0.01);
        const auto w = sfm_window(1.5, 0.5, 10, 1.0, 0.005, 0xA18);
        in.frames = w.frames;
        in.extrinsics = w.extrinsics;
        in.t = 3.0;
        return capture(std::string(kStage),
                       "double",
                       "s1_initialization_bench: noisy static window (seed 0xA17) + noisy keyframes + SfM window "
                       "(1 px, seed 0xA18)",
                       encode_input(in),
                       encode_output(run<double>(in, kShipped)));
    }

    // ── Precision characterization (#450) ───────────────────────────────────
    /// The initializer's verdict, the state it builds, and its gravity and
    /// scale estimates. No ladder: the fixtures (static, noisy, EuRoC MH_05
    /// at rest and moving) span the conditions S1 meets.
    template <class T>
    [[nodiscard]] static std::vector<OutputGroup> precision_groups(const Output<T>& out) {
        auto g = state_groups(out.state);
        g.insert(g.begin(), {"verdict", {static_cast<double>(out.success), static_cast<double>(out.method)}});
        g.push_back({"gravity",
                     {static_cast<double>(out.gravity_world[0]),
                      static_cast<double>(out.gravity_world[1]),
                      static_cast<double>(out.gravity_world[2])}});
        g.push_back({"scale", {static_cast<double>(out.scale)}});
        return g;
    }

    // ── Characterization sweep ──────────────────────────────────────────────
    [[nodiscard]] static Sweep default_sweep() {
        Sweep sw;
        sw.axis("window", {50, 200})
            .axis("imu_noise", {0.01, 0.05})
            .axis("excitation", {0.4, 1.5})
            .axis("obs_noise_px", {0.0, 0.5, 2.0})
            .axis("frames", {5, 10, 20})
            .axis("speed", {0.0, 0.3, 1.0});
        return sw;
    }
    [[nodiscard]] static std::vector<std::string> sweep_columns() {
        return {"static_success",
                "static_rollpitch_err_deg",
                "dynamic_success",
                "dynamic_scale_err_pct",
                "sfm_window_built",
                "sfm_status",
                "sfm_bootstrap_parallax",
                "sfm_resolved",
                "sfm_rotation_err_deg",
                "sfm_position_err_pct",
                "sfm_scale_err_pct",
                "adjacent_window_built",
                "adjacent_resolved",
                "kf4hz_resolved",
                "kf4hz_scale_err_pct"};
    }
    template <class T>
    [[nodiscard]] static std::vector<double> sweep_point(const Point& p) {
        const auto get = [&](const char* k, double dflt) {
            const auto it = p.find(k);
            return it == p.end() ? dflt : it->second;
        };
        const double w = p.at("window"), noise = p.at("imu_noise"), exc = p.at("excitation");
        const double obs = get("obs_noise_px", 0.5), frames = get("frames", 10.0), speed = get("speed", 0.5);
        if (!std::isfinite(w) || w < 2.0 || w > 10000.0 || w != std::floor(w))
            throw std::invalid_argument("s1 bench sweep: window must be an integer in [2, 10000]");
        if (!std::isfinite(noise) || noise < 0.0 || noise > 1.0)
            throw std::invalid_argument("s1 bench sweep: imu_noise must be in [0, 1]");
        if (!std::isfinite(exc) || exc <= 0.0 || exc > 20.0)
            throw std::invalid_argument("s1 bench sweep: excitation must be in (0, 20]");
        if (!std::isfinite(obs) || obs < 0.0 || obs > 50.0)
            throw std::invalid_argument("s1 bench sweep: obs_noise_px must be in [0, 50]");
        if (!std::isfinite(frames) || frames < 3.0 || frames > 200.0 || frames != std::floor(frames))
            throw std::invalid_argument("s1 bench sweep: frames must be an integer in [3, 200]");
        if (!std::isfinite(speed) || speed < 0.0 || speed > 20.0)
            throw std::invalid_argument("s1 bench sweep: speed must be in [0, 20]");
        const auto R_true = math::lie::SO3<double>::exp({{8.0 * kDeg, 0.0, 0.0}});
        Input<double> in0 = static_window(R_true, {}, {}, noise, noise / 25.0, static_cast<std::size_t>(w), 0xA17);
        in0.keyframes = dynamic_keyframes(exc, noise);
        json truth;
        add_window(
            in0, truth, sfm_window(exc, speed, static_cast<std::size_t>(frames), obs, noise / 25.0, 0x5F1), 1, 1, 1);
        const auto in = decode_input<T>(encode_input(in0));
        const auto st = run<T>(in, kShipped);
        const auto dy = run<T>(in, "dynamic");
        const double rp = st.success ? up_angle_deg(st.state.R, R_true) : std::nan("");
        const double se = dy.success ? std::abs(static_cast<double>(dy.scale) - 2.0) / 2.0 * 100.0 : std::nan("");
        // The SfM front end only in native arithmetic: its RANSAC is far too
        // costly per point in an emulated type (runs_by_default).
        constexpr bool native = std::numeric_limits<T>::is_iec559;
        Output<T> sf, ad, k4;
        if constexpr (native) {
            sf = run<T>(in, "sfm_two_view_pnp");
            ad = run<T>(in, "sfm_adjacent_bootstrap");
            k4 = run<T>(in, "sfm_keyframes_4hz");
        }
        double rot = std::nan(""), pos = std::nan(""), sc = std::nan("");
        if (sf.window.success)
            for (const auto& x : sfm_truth_checks<T>(in, sf, truth.at("sfm"))) {
                if (x.name == "truth.sfm_rotation_error")
                    rot = x.value;
                else if (x.name == "truth.sfm_position_error")
                    pos = x.value;
                else if (x.name == "truth.sfm_scale_error")
                    sc = x.value;
            }
        const double na = std::nan("");
        if (!native)
            return {static_cast<double>(st.success),
                    rp,
                    static_cast<double>(dy.success),
                    se,
                    na,
                    na,
                    na,
                    na,
                    na,
                    na,
                    na,
                    na,
                    na,
                    na,
                    na};
        double k4s = na;
        if (k4.success)
            for (const auto& x : sfm_truth_checks<T>(in, k4, truth.at("sfm")))
                if (x.name == "truth.sfm_scale_error")
                    k4s = x.value;
        return {static_cast<double>(st.success),
                rp,
                static_cast<double>(dy.success),
                se,
                sf.window.success ? 1.0 : 0.0,
                static_cast<double>(sf.window.status),
                static_cast<double>(sf.window.bootstrap_parallax),
                static_cast<double>(sf.success),
                rot,
                pos,
                sc,
                ad.window.success ? 1.0 : 0.0,
                static_cast<double>(ad.success),
                static_cast<double>(k4.success),
                k4s};
    }

private:
    static constexpr double kDeg = std::numbers::pi / 180.0;

    /// −(R · mean specific force): the gravity a static seed implies, world frame.
    template <class T>
    [[nodiscard]] static Vec3<T> gravity_from_accel(const math::lie::SO3<T>& R, const std::vector<Vec3<T>>& accel) {
        Vec3<T> m{};
        for (const auto& a : accel)
            m = m + a;
        if (!accel.empty())
            m = m * (T{1} / static_cast<T>(accel.size()));
        return -(R * m);
    }

    /// Roll/pitch error: the angle between the estimated and true body "up".
    template <class T>
    [[nodiscard]] static double up_angle_deg(const math::lie::SO3<T>& R, const math::lie::SO3<double>& R_true) {
        const auto ue = R.inverse() * Vec3<T>{{T{0}, T{0}, T{1}}};
        const auto ut = R_true.inverse() * Vec3<double>{{0.0, 0.0, 1.0}};
        const double ex = static_cast<double>(ue[0]), ey = static_cast<double>(ue[1]), ez = static_cast<double>(ue[2]);
        const double cx = ey * ut[2] - ez * ut[1], cy = ez * ut[0] - ex * ut[2], cz = ex * ut[1] - ey * ut[0];
        return std::atan2(std::sqrt(cx * cx + cy * cy + cz * cz), ex * ut[0] + ey * ut[1] + ez * ut[2]) / kDeg;
    }

    /// A static window at attitude R: specific force Rᵀ(0,0,g) + accel bias,
    /// gyro = gyro bias, each with optional Gaussian noise (generated in double).
    [[nodiscard]] static Input<double> static_window(const math::lie::SO3<double>& R,
                                                     const Vec3<double>& bg,
                                                     const Vec3<double>& ba,
                                                     double accel_noise,
                                                     double gyro_noise,
                                                     std::size_t n,
                                                     std::uint64_t seed) {
        Input<double> in;
        const Vec3<double> sf = R.inverse() * Vec3<double>{{0.0, 0.0, in.gravity_magnitude}};
        std::mt19937_64 rng(seed);
        std::normal_distribution<double> na(0.0, accel_noise > 0 ? accel_noise : 1.0);
        std::normal_distribution<double> ng(0.0, gyro_noise > 0 ? gyro_noise : 1.0);
        for (std::size_t k = 0; k < n; ++k) {
            Vec3<double> a = sf + ba, g = bg;
            for (std::size_t c = 0; c < 3; ++c) {
                if (accel_noise > 0)
                    a[c] += na(rng);
                if (gyro_noise > 0)
                    g[c] += ng(rng);
            }
            in.accel.push_back(a);
            in.gyro.push_back(g);
        }
        return in;
    }

    /// A synthetic SfM init window, as the backend buffers it: `frames` camera
    /// frames at 20 Hz (EuRoC's rate) of a body that starts at the world origin
    /// with velocity `speed`·(0.6, −0.8, 0) m/s, accelerates at `excitation`
    /// m/s² and turns at a constant rate; 80 landmarks 4–8 m in front of the
    /// first camera; normalized observations with `obs_noise_px` pixels of
    /// noise (EuRoC's focal length, 458 px) and 200 Hz preintegration with
    /// `imu_noise`. The truth is the camera track (metric) and the IMU attitudes.
    struct SfmWindow {
        std::vector<Frame<double>> frames;
        sdk::msckf::CameraExtrinsics<double> extrinsics;
        json truth;
    };
    [[nodiscard]] static SfmWindow sfm_window(double excitation,
                                              double speed,
                                              std::size_t n_frames,
                                              double obs_noise_px,
                                              double imu_noise,
                                              std::uint64_t seed) {
        using SO3 = math::lie::SO3<double>;
        using V = Vec3<double>;
        const V g_world{{0.0, 0.0, -9.81}};
        const V omega{{0.15, -0.1, 0.4}};
        const double dn = std::sqrt(0.8 * 0.8 + 0.5 * 0.5 + 0.2 * 0.2);
        const V a_w{{0.8 / dn * excitation, 0.5 / dn * excitation, 0.2 / dn * excitation}};
        const V v0{{0.6 * speed, -0.8 * speed, 0.0}};
        const double frame_dt = 0.05, imu_dt = 0.005, focal = 458.0;
        const auto sub = static_cast<std::size_t>(std::lround(frame_dt / imu_dt));
        auto R_at = [&](double t) { return SO3::exp(omega * t); };
        auto p_at = [&](double t) { return v0 * t + a_w * (0.5 * t * t); };

        SfmWindow w;
        w.extrinsics.R_imu_cam = SO3::exp(V{{0.05, -0.03, 0.02}});
        w.extrinsics.p_imu_cam = V{{0.05, -0.02, 0.01}};
        const auto cam_pose = [&](double t, SO3& R_w_cam, V& p_w_cam) {
            const SO3 R = R_at(t);
            R_w_cam = R * w.extrinsics.R_imu_cam;
            p_w_cam = p_at(t) + R * w.extrinsics.p_imu_cam;
        };
        // Landmarks on a jittered grid in the first camera's view.
        SO3 Rc0;
        V pc0;
        cam_pose(0.0, Rc0, pc0);
        std::vector<V> X;
        for (int i = 0; i < 10; ++i)
            for (int j = 0; j < 8; ++j) {
                const double h = std::sin(i * 12.9898 + j * 78.233) * 43758.5453;
                const double z = 4.0 + 4.0 * (h - std::floor(h));
                X.push_back(Rc0 * V{{(-3.0 + 6.0 * i / 9.0) * z / 6.0, (-2.4 + 4.8 * j / 7.0) * z / 6.0, z}} + pc0);
            }

        std::mt19937_64 rng(seed);
        std::normal_distribution<double> n01(0.0, 1.0);
        json cam_p = json::array(), R_imu = json::array();
        for (std::size_t f = 0; f < n_frames; ++f) {
            const double t = static_cast<double>(f) * frame_dt;
            Frame<double> fr;
            fr.timestamp = t;
            SO3 Rc;
            V pc;
            cam_pose(t, Rc, pc);
            const SO3 Rct = Rc.inverse();
            for (std::size_t m = 0; m < X.size(); ++m) {
                const V xc = Rct * (X[m] - pc);
                if (!(xc[2] > 0.2))
                    continue;
                const double u = xc[0] / xc[2], v = xc[1] / xc[2];
                if (std::abs(u) > 0.8 || std::abs(v) > 0.6)
                    continue;
                fr.ids.push_back(static_cast<std::uint64_t>(m));
                fr.obs.push_back({{u + obs_noise_px / focal * n01(rng), v + obs_noise_px / focal * n01(rng)}});
            }
            if (f > 0) {
                sdk::ImuPreintegrator<double> pre{};
                for (std::size_t k = 0; k < sub; ++k) {
                    const double tk = t - frame_dt + static_cast<double>(k) * imu_dt;
                    V sf = R_at(tk).inverse() * (a_w - g_world);  // specific force
                    V wv = omega;
                    for (std::size_t c = 0; c < 3; ++c) {
                        sf[c] += imu_noise * n01(rng);
                        wv[c] += imu_noise * n01(rng);
                    }
                    pre.integrate(wv, sf, imu_dt);
                }
                fr.dR = pre.delta_rotation();
                fr.dv = pre.delta_velocity();
                fr.dp = pre.delta_position();
                fr.dR_dbg = pre.d_rotation_d_gyro_bias();
                fr.dt = pre.delta_time();
            }
            w.frames.push_back(std::move(fr));
            cam_p.push_back(pack(pc));
            R_imu.push_back(pack(R_at(t)));
        }
        w.truth = {{"cam_positions", cam_p}, {"R_world_imu", R_imu}};
        return w;
    }

    /// Attach an SfM window, and its truth with tolerances, to a fixture input.
    static void
    add_window(Input<double>& in, json& truth, const SfmWindow& w, double rot_deg, double pos_pct, double scale_pct) {
        in.frames = w.frames;
        in.extrinsics = w.extrinsics;
        json t = w.truth;
        t["rotation_tol_deg"] = rot_deg;
        t["position_tol_pct"] = pos_pct;
        t["scale_tol_pct"] = scale_pct;
        if (!truth.is_object())
            truth = json::object();
        truth["sfm"] = t;
    }

    /// Six VI keyframes at 4 Hz from rest under constant acceleration
    /// `excitation` (m/s²), vision scale 2, IMU noise `imu_noise`.
    [[nodiscard]] static std::vector<Keyframe<double>> dynamic_keyframes(double excitation, double imu_noise) {
        const Vec3<double> dir{{0.8, 0.5, 0.2}};
        const double dn = std::sqrt(0.8 * 0.8 + 0.5 * 0.5 + 0.2 * 0.2);
        const Vec3<double> a_w{{dir[0] / dn * excitation, dir[1] / dn * excitation, dir[2] / dn * excitation}};
        return sdk::eval::init_detail::make_dynamic_keyframes<double>(a_w, 2.0, 6, imu_noise, 0xD33).kfs;
    }

    /// The SfM window's own contract: it builds, its keyframe attitudes are in
    /// SO(3) (the worst keyframe is reported), and every resection's
    /// reprojection RMS is inside the PnP inlier gate.
    template <class T>
    [[nodiscard]] static std::vector<inv::InvariantResult> window_checks(const sdk::sfm::InitWindowResult<T>& w) {
        std::vector<inv::InvariantResult> r;
        r.push_back(
            inv::check_scalar(w.success ? 1.0 : 0.0, 1.0, inv::Bound::Lower, kInvStage, "sfm.window_built", "flag"));
        if (!w.success)
            return r;
        std::vector<inv::InvariantResult> worst;
        for (const auto& k : w.keyframes) {
            auto c = inv::check_so3<T>(k.R_world_imu,
                                       kInvStage,
                                       "sfm.keyframe_orthogonality",
                                       "sfm.keyframe_determinant",
                                       safety_vs_double<T>());
            if (worst.empty())
                worst = std::move(c);
            else
                for (std::size_t i = 0; i < c.size(); ++i)
                    if ((worst[i].pass && !c[i].pass) || c[i].margin() > worst[i].margin())
                        worst[i] = c[i];
        }
        for (auto& x : worst)
            r.push_back(std::move(x));
        double rms = 0.0;
        for (const T& e : w.pnp_rms) {
            const double d = static_cast<double>(e);
            rms = std::isfinite(d) ? std::max(rms, d) : inv::detail::kInf;
        }
        r.push_back(inv::check_scalar(rms,
                                      static_cast<double>(sdk::sfm::PnpOptions<double>{}.reprojection_threshold),
                                      inv::Bound::Upper,
                                      kInvStage,
                                      "sfm.pnp_reprojection_rms",
                                      "normalized"));
        return r;
    }

    /// The SfM keyframes against truth, up to the gauge (world = camera 0) and
    /// the scale the window cannot see: the attitude of each keyframe relative
    /// to the first, and the camera positions after a similarity fit. Camera
    /// positions, not IMU positions: the window is up to scale but the
    /// camera→IMU lever arm is metric, so only the camera track is an exact
    /// similarity of the truth. When the alignment resolved, its metric scale
    /// against the true spread of the track.
    template <class T>
    [[nodiscard]] static std::vector<inv::InvariantResult>
    sfm_truth_checks(const Input<T>& in, const Output<T>& o, const json& truth) {
        using V = Vec3<double>;
        std::vector<inv::InvariantResult> r;
        const auto& kfs = o.window.keyframes;
        const auto& tp = truth.at("cam_positions");
        const auto& tR = truth.at("R_world_imu");
        const auto& at = o.window.keyframe_frames;  // the truth is per input frame
        if (kfs.empty() || at.size() != kfs.size() || tp.size() != tR.size())
            throw std::invalid_argument("s1 bench: SfM truth and window differ in length");
        for (const std::size_t f : at)
            if (f >= tp.size())
                throw std::invalid_argument("s1 bench: an SfM keyframe has no truth");
        const auto to_d = [](const auto& x) {
            return V{{static_cast<double>(x[0]), static_cast<double>(x[1]), static_cast<double>(x[2])}};
        };
        const auto so3_d = [](const auto& R) {
            const auto& q = R.quaternion();
            return math::lie::SO3<double>(math::lie::detail::Vec<double, 4>{{static_cast<double>(q[0]),
                                                                             static_cast<double>(q[1]),
                                                                             static_cast<double>(q[2]),
                                                                             static_cast<double>(q[3])}});
        };
        const V lever = to_d(in.extrinsics.p_imu_cam);
        std::vector<V> pe, pt;
        double rot = 0.0;
        const auto Re0 = so3_d(kfs.front().R_world_imu);
        const auto Rt0 = unpack_so3<double>(tR[at.front()]);
        for (std::size_t i = 0; i < kfs.size(); ++i) {
            const auto Re = so3_d(kfs[i].R_world_imu);
            pe.push_back(to_d(kfs[i].p_world_imu) + Re * lever);
            pt.push_back(unpack_fixed<double, 3>(tp[at[i]]));
            const auto d = (Re0.inverse() * Re).inverse() * (Rt0.inverse() * unpack_so3<double>(tR[at[i]]));
            const double a = math::lie::detail::norm(d.log()) / kDeg;
            rot = std::isfinite(a) ? std::max(rot, a) : inv::detail::kInf;
        }
        // Each tolerance is the fixture's (set for double) plus T's rounding: the
        // window chains two eigensolves, a triangulation and a resection, ~10⁴ ε.
        const double eps_t = static_cast<double>(std::numeric_limits<T>::epsilon());
        r.push_back(inv::check_scalar(rot,
                                      truth.at("rotation_tol_deg").get<double>() + 1e4 * eps_t / kDeg,
                                      inv::Bound::Upper,
                                      kInvStage,
                                      "truth.sfm_rotation_error",
                                      "deg"));
        // Similarity fit of the estimated camera track onto the true one.
        V me{}, mt{};
        for (std::size_t i = 0; i < pe.size(); ++i) {
            me = me + pe[i];
            mt = mt + pt[i];
        }
        me = me * (1.0 / static_cast<double>(pe.size()));
        mt = mt * (1.0 / static_cast<double>(pt.size()));
        std::vector<V> ce, ct;
        double se = 0.0, st = 0.0;
        for (std::size_t i = 0; i < pe.size(); ++i) {
            ce.push_back(pe[i] - me);
            ct.push_back(pt[i] - mt);
            se += math::lie::detail::dot(ce.back(), ce.back());
            st += math::lie::detail::dot(ct.back(), ct.back());
        }
        const auto Rf = sdk::eval::detail::horn_rotation<double>(ct, ce);
        double num = 0.0;
        for (std::size_t i = 0; i < ce.size(); ++i)
            num += math::lie::detail::dot(ce[i], Rf * ct[i]);
        const double sc = st > 0.0 ? num / st : 0.0;
        double res = 0.0;
        for (std::size_t i = 0; i < ce.size(); ++i) {
            const V e = ce[i] - (Rf * ct[i]) * sc;
            res += math::lie::detail::dot(e, e);
        }
        const double pos = se > 0.0 ? std::sqrt(res / se) * 100.0 : inv::detail::kInf;
        r.push_back(inv::check_scalar(std::isfinite(pos) ? pos : inv::detail::kInf,
                                      truth.at("position_tol_pct").get<double>() + 1e6 * eps_t,
                                      inv::Bound::Upper,
                                      kInvStage,
                                      "truth.sfm_position_error",
                                      "%"));
        if (o.success) {
            const double metric = static_cast<double>(o.scale) * std::sqrt(se), want = std::sqrt(st);
            const double err = want > 0.0 ? std::abs(metric - want) / want * 100.0 : inv::detail::kInf;
            r.push_back(inv::check_scalar(std::isfinite(err) ? err : inv::detail::kInf,
                                          truth.at("scale_tol_pct").get<double>() + 1e6 * eps_t,
                                          inv::Bound::Upper,
                                          kInvStage,
                                          "truth.sfm_scale_error",
                                          "%"));
        }
        return r;
    }

    template <class T>
    [[nodiscard]] static std::vector<inv::InvariantResult> truth_checks(const Output<T>& o, const json& truth) {
        std::vector<inv::InvariantResult> r;
        if (static_cast<sdk::InitMethod>(o.method) == sdk::InitMethod::Dynamic) {
            const double scale_true = truth.at("scale").get<double>();
            r.push_back(inv::check_scalar(std::abs(static_cast<double>(o.scale) - scale_true) / scale_true * 100.0,
                                          5.0,
                                          inv::Bound::Upper,
                                          kInvStage,
                                          "truth.scale_error",
                                          "%"));
            return r;
        }
        // Static paths: roll/pitch vs the true attitude (yaw is unobservable),
        // and the gyro bias vs the true bias. The accel bias is unobservable from
        // one static window and tilts "up" by ~atan(|ba|/g) — hence 0.5°.
        const auto R_true = unpack_so3<double>(truth.at("R_world_imu"));
        r.push_back(inv::check_scalar(
            up_angle_deg(o.state.R, R_true), 0.5, inv::Bound::Upper, kInvStage, "truth.roll_pitch_error", "deg"));
        const auto bg = unpack_fixed<double, 3>(truth.at("gyro_bias"));
        double worst = 0.0;
        for (std::size_t i = 0; i < 3; ++i) {
            const double d = std::abs(static_cast<double>(o.state.bg[i]) - bg[i]);
            worst = std::isfinite(d) ? std::max(worst, d) : inv::detail::kInf;
        }
        r.push_back(inv::check_scalar(worst,
                                      std::max(1e-6, tolerance_vs_double<T>(3, 0.01, 1024.0)),
                                      inv::Bound::Upper,
                                      kInvStage,
                                      "truth.gyro_bias_error",
                                      "rad/s"));
        return r;
    }
};

}  // namespace branes::tools::bench

#endif  // BRANES_TOOLS_BENCH_S1_INITIALIZATION_BENCH_HPP
