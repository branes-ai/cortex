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
// Every fixture carries both a static IMU window and a dynamic visual-inertial
// keyframe window, so all variants run on identical inputs:
//   shipped        try_static → seed_from_imu (the backend's default path)
//   gravity_align  try_gravity_align → seed_from_imu (the moving-start fallback)
//   dynamic        try_dynamic → seed_from_alignment (VI alignment: yaw-free
//                  attitude, velocity, scale)
//
// Fixtures:
//   known_answer   a noiseless, tilted, gyro-biased static window: the expected
//                  seed (attitude, biases, P₀, diagnostics) in closed form
//   ground_truth   the synthetic world's stationary warm-up IMU (with its true
//                  biases) + keyframes from a known trajectory; truth = attitude,
//                  gyro bias, metric scale and gravity
//   captured       a noisy window through the shipped variant, recorded
//
// Sweep: window length × IMU noise × translational excitation → static
// roll/pitch error and dynamic scale error.
//
// Not yet here: the SfM front-end variants (two-view vs PnP keyframe
// construction) need image-observation fixtures; follow-up of #454.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_BENCH_S1_INITIALIZATION_BENCH_HPP
#define BRANES_TOOLS_BENCH_S1_INITIALIZATION_BENCH_HPP

#include <branes/sdk/eval/initialization_probe.hpp>  // make_dynamic_keyframes (synthetic VI keyframes)
#include <branes/sdk/eval/invariants.hpp>
#include <branes/sdk/eval/synthetic_world.hpp>
#include <branes/sdk/imu_init.hpp>
#include <branes/sdk/msckf/stages/s1_initialization.hpp>
#include <branes/tools/bench/bench.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <random>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
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
    struct Input {
        std::vector<Vec3<T>> gyro, accel;    ///< the static init window
        std::vector<Keyframe<T>> keyframes;  ///< the dynamic init window (VI keyframes)
        double t = 0.0;                      ///< time the init completes
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
    };

    [[nodiscard]] static std::vector<Variant> variants() {
        return {{"shipped", "try_static -> seed_from_imu (backend default)"},
                {"gravity_align", "try_gravity_align -> seed_from_imu (moving-start fallback)"},
                {"dynamic", "try_dynamic -> seed_from_alignment (visual-inertial alignment)"}};
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
        return json{{"gyro", pack_vecs(in.gyro)},
                    {"accel", pack_vecs(in.accel)},
                    {"keyframes", kfs},
                    {"t", pack_num(in.t)},
                    {"gravity_magnitude", pack(in.gravity_magnitude)}};
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
        in.t = unpack_num(j.at("t"));
        in.gravity_magnitude = unpack_scalar<T>(j.at("gravity_magnitude"));
        return in;
    }
    template <class T>
    [[nodiscard]] static json encode_output(const Output<T>& o) {
        const auto& d = o.diag;
        return json{{"success", o.success},
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
        if (variant == "dynamic") {
            const auto r = init.try_dynamic(std::span<const Keyframe<T>>{in.keyframes});
            if (!r.success)
                return out;
            const sdk::sfm::InitWindowResult<T> win{true, in.keyframes};
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
        return v;
    }

    // ── The contract ────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static std::vector<inv::InvariantResult>
    invariants(const Input<T>& in, const Output<T>& o, const Fixture& f) {
        std::vector<inv::InvariantResult> r;
        // Every variant has the data it needs in every fixture, so it must resolve.
        r.push_back(inv::check_scalar(o.success, 1.0, inv::Bound::Lower, kInvStage, "init.resolved", "flag"));
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
        if (f.kind == FixtureKind::GroundTruth)
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
        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::GroundTruth;
        f.source = "synthetic_world (default config) IMU samples 100-149 (warm-up); dynamic keyframes scale 2";
        f.seed = cfg.seed;
        f.description = "truth: attitude, gyro bias, metric scale, gravity";
        f.input = encode_input(in);
        f.truth = truth;
        return f;
    }

    /// A noisy, tilted static window and noisy keyframes through the shipped path.
    [[nodiscard]] static Fixture captured() {
        const auto R = math::lie::SO3<double>::exp({{0.12, -0.05, 0.3}});
        Input<double> in = static_window(R, {{0.003, 0.001, -0.002}}, {{0.02, -0.01, 0.0}}, 0.05, 0.002, 60, 0xA17);
        in.keyframes = dynamic_keyframes(1.5, 0.01);
        in.t = 3.0;
        return capture(std::string(kStage),
                       "double",
                       "s1_initialization_bench: noisy static window (seed 0xA17) + noisy keyframes",
                       encode_input(in),
                       encode_output(run<double>(in, kShipped)));
    }

    // ── Characterization sweep ──────────────────────────────────────────────
    [[nodiscard]] static Sweep default_sweep() {
        Sweep sw;
        sw.axis("window", {25, 50, 200}).axis("imu_noise", {0.01, 0.05, 0.2}).axis("excitation", {0.15, 0.4, 1.5});
        return sw;
    }
    [[nodiscard]] static std::vector<std::string> sweep_columns() {
        return {"static_success", "static_rollpitch_err_deg", "dynamic_success", "dynamic_scale_err_pct"};
    }
    template <class T>
    [[nodiscard]] static std::vector<double> sweep_point(const Point& p) {
        const double w = p.at("window"), noise = p.at("imu_noise"), exc = p.at("excitation");
        if (!std::isfinite(w) || w < 2.0 || w > 10000.0 || w != std::floor(w))
            throw std::invalid_argument("s1 bench sweep: window must be an integer in [2, 10000]");
        if (!std::isfinite(noise) || noise < 0.0 || noise > 1.0)
            throw std::invalid_argument("s1 bench sweep: imu_noise must be in [0, 1]");
        if (!std::isfinite(exc) || exc <= 0.0 || exc > 20.0)
            throw std::invalid_argument("s1 bench sweep: excitation must be in (0, 20]");
        const auto R_true = math::lie::SO3<double>::exp({{8.0 * kDeg, 0.0, 0.0}});
        Input<double> in0 = static_window(R_true, {}, {}, noise, noise / 25.0, static_cast<std::size_t>(w), 0xA17);
        in0.keyframes = dynamic_keyframes(exc, noise);
        const auto in = decode_input<T>(encode_input(in0));
        const auto st = run<T>(in, kShipped);
        const auto dy = run<T>(in, "dynamic");
        const double rp = st.success ? up_angle_deg(st.state.R, R_true) : std::nan("");
        const double se = dy.success ? std::abs(static_cast<double>(dy.scale) - 2.0) / 2.0 * 100.0 : std::nan("");
        return {static_cast<double>(st.success), rp, static_cast<double>(dy.success), se};
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

    /// Six VI keyframes at 4 Hz from rest under constant acceleration
    /// `excitation` (m/s²), vision scale 2, IMU noise `imu_noise`.
    [[nodiscard]] static std::vector<Keyframe<double>> dynamic_keyframes(double excitation, double imu_noise) {
        const Vec3<double> dir{{0.8, 0.5, 0.2}};
        const double dn = std::sqrt(0.8 * 0.8 + 0.5 * 0.5 + 0.2 * 0.2);
        const Vec3<double> a_w{{dir[0] / dn * excitation, dir[1] / dn * excitation, dir[2] / dn * excitation}};
        return sdk::eval::init_detail::make_dynamic_keyframes<double>(a_w, 2.0, 6, imu_noise, 0xD33).kfs;
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
