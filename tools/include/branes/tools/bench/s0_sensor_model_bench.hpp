// SPDX-License-Identifier: MIT
//
// branes/tools/bench/s0_sensor_model_bench.hpp — the S0_sensor_model stage
// bench (issue #454, epic #444).
//
// Stage under test: stages::s0_sensor_model::apply(camera, u, v) → normalized
// image point (x/z, y/z), through the EuRoC pinhole-radtan model. The bench also
// re-projects each normalized point, so the camera model is exercised in both
// directions. Contract (#445 §B, inputs + S0):
//   • intrinsics positive, principal point inside the image;
//   • no in-image pixel of a forward camera is rejected (cheirality);
//   • unproject → project round-trips to the pixel (px);
//   • the analytic projection Jacobian matches central finite differences.
//
// Fixtures:
//   known_answer   a distortion-free camera: normalized = ((u−cx)/fx, (v−cy)/fy)
//                  in closed form, reprojection = the pixel
//   ground_truth   the synthetic world's exact projections of its landmarks;
//                  truth = the landmarks' camera-frame normalized coordinates
//   captured       one synthetic frame's observations through the stage
//
// Variants: "shipped" (Newton undistortion inside the model) and "fixed_point"
// (the classic fixed-point iteration n ← n − (distort(n) − n_d)).
//
// Sweep: distortion strength × radius from the principal point → round-trip
// residual, Jacobian error, and the pixel cost of calibration errors (1° of
// extrinsic rotation; 1 ms of time offset at 1 rad/s).
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_BENCH_S0_SENSOR_MODEL_BENCH_HPP
#define BRANES_TOOLS_BENCH_S0_SENSOR_MODEL_BENCH_HPP

#include <branes/math/cameras/pinhole_radtan.hpp>
#include <branes/sdk/eval/invariants.hpp>
#include <branes/sdk/eval/synthetic_world.hpp>
#include <branes/sdk/msckf/stages/s0_sensor_model.hpp>
#include <branes/tools/bench/bench.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace branes::tools::bench {

struct S0SensorModelBench {
    static constexpr std::string_view kStage = "S0_sensor_model";
    static constexpr inv::Stage kInvStage = inv::Stage::S0_sensor_model;

    /// Pinhole intrinsics + radtan distortion, and the image size.
    template <class T>
    struct Intrinsics {
        T fx{1}, fy{1}, cx{0}, cy{0}, k1{0}, k2{0}, p1{0}, p2{0}, k3{0};
        T width{0}, height{0};
        [[nodiscard]] math::cameras::PinholeRadtanCamera<T> camera() const {
            return {fx, fy, cx, cy, k1, k2, p1, p2, k3};
        }
    };

    template <class T>
    struct Input {
        Intrinsics<T> intr;
        std::vector<std::array<T, 2>> pixels;
    };
    template <class T>
    struct Output {
        std::vector<int> valid;                     ///< 1 if S0 accepted the pixel
        std::vector<std::array<T, 2>> normalized;   ///< (x/z, y/z)
        std::vector<std::array<T, 2>> reprojected;  ///< project(normalized, 1)
    };

    [[nodiscard]] static std::vector<Variant> variants() {
        return {{"shipped", "stages::s0_sensor_model::apply (Newton undistortion in the camera model)"},
                {"fixed_point", "fixed-point undistortion n <- n - (distort(n) - n_d)"}};
    }

    // ── Codec ───────────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static json pack_points(const std::vector<std::array<T, 2>>& pts) {
        json a = json::array();
        for (const auto& p : pts)
            a.push_back(json::array({pack(p[0]), pack(p[1])}));
        return a;
    }
    template <class T>
    [[nodiscard]] static std::vector<std::array<T, 2>> unpack_points(const json& j) {
        std::vector<std::array<T, 2>> pts;
        for (const auto& p : j)
            pts.push_back({unpack_scalar<T>(p.at(0)), unpack_scalar<T>(p.at(1))});
        return pts;
    }
    template <class T>
    [[nodiscard]] static json encode_input(const Input<T>& in) {
        const auto& c = in.intr;
        return json{{"intrinsics",
                     {{"fx", pack(c.fx)},
                      {"fy", pack(c.fy)},
                      {"cx", pack(c.cx)},
                      {"cy", pack(c.cy)},
                      {"k1", pack(c.k1)},
                      {"k2", pack(c.k2)},
                      {"p1", pack(c.p1)},
                      {"p2", pack(c.p2)},
                      {"k3", pack(c.k3)},
                      {"width", pack(c.width)},
                      {"height", pack(c.height)}}},
                    {"pixels", pack_points(in.pixels)}};
    }
    template <class T>
    [[nodiscard]] static Input<T> decode_input(const json& j) {
        const auto& c = j.at("intrinsics");
        Input<T> in;
        auto get = [&](const char* k) { return unpack_scalar<T>(c.at(k)); };
        in.intr = {get("fx"),
                   get("fy"),
                   get("cx"),
                   get("cy"),
                   get("k1"),
                   get("k2"),
                   get("p1"),
                   get("p2"),
                   get("k3"),
                   get("width"),
                   get("height")};
        in.pixels = unpack_points<T>(j.at("pixels"));
        return in;
    }
    template <class T>
    [[nodiscard]] static json encode_output(const Output<T>& out) {
        return json{{"valid", out.valid},
                    {"normalized", pack_points(out.normalized)},
                    {"reprojected", pack_points(out.reprojected)}};
    }
    template <class T>
    [[nodiscard]] static Output<T> decode_output(const json& j) {
        Output<T> out{j.at("valid").get<std::vector<int>>(),
                      unpack_points<T>(j.at("normalized")),
                      unpack_points<T>(j.at("reprojected"))};
        // The invariants index all three per pixel: a ragged output is a malformed fixture.
        if (out.normalized.size() != out.valid.size() || out.reprojected.size() != out.valid.size())
            throw std::invalid_argument("s0 bench: valid/normalized/reprojected lengths differ");
        return out;
    }

    // ── The stage ───────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static Output<T> run(const Input<T>& in, std::string_view variant) {
        const auto cam = in.intr.camera();
        Output<T> out;
        for (const auto& px : in.pixels) {
            std::array<T, 2> xy{};
            bool ok = false;
            if (variant == "fixed_point") {
                // Same validity rules as the shipped path: the iteration must
                // converge, and the ray (x, y, 1) is in front of the camera.
                const auto fp = fixed_point_undistort(in.intr, px);
                xy = fp.xy;
                ok = fp.converged;
            } else {
                const auto r = sdk::msckf::stages::s0_sensor_model::apply(cam, px[0], px[1]);
                ok = r.valid;
                xy = {r.xy[0], r.xy[1]};
            }
            out.valid.push_back(ok ? 1 : 0);
            out.normalized.push_back(xy);
            out.reprojected.push_back(ok ? cam.project({xy[0], xy[1], T{1}}) : std::array<T, 2>{});
        }
        return out;
    }

    template <class T>
    [[nodiscard]] static std::vector<double> flatten(const Output<T>& out) {
        std::vector<double> v;
        for (const int x : out.valid)
            v.push_back(static_cast<double>(x));
        for (const auto& p : out.normalized)
            for (const T& x : p)
                v.push_back(static_cast<double>(x));
        for (const auto& p : out.reprojected)
            for (const T& x : p)
                v.push_back(static_cast<double>(x));
        return v;
    }

    // ── The contract ────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static std::vector<inv::InvariantResult>
    invariants(const Input<T>& in, const Output<T>& out, const Fixture& f) {
        std::vector<inv::InvariantResult> r;
        const auto& c = in.intr;
        for (auto& x : inv::check_intrinsics<T>(c.fx, c.fy, c.cx, c.cy, c.width, c.height, kInvStage))
            r.push_back(x);

        // Cheirality: a forward pinhole accepts every in-image pixel.
        std::size_t rejected = 0;
        for (const int ok : out.valid)
            rejected += ok ? 0 : 1;
        r.push_back(inv::check_scalar(
            static_cast<double>(rejected), 0.0, inv::Bound::Upper, kInvStage, "cheirality.in_image_rejected", "count"));

        // Round trip pixel → normalized → pixel, in px. The undistortion is an
        // iterative solve that stops at ~100·ε_T, so its own safety applies.
        std::vector<T> a, b;
        for (std::size_t i = 0; i < in.pixels.size(); ++i)
            if (out.valid[i]) {
                a.insert(a.end(), {in.pixels[i][0], in.pixels[i][1]});
                b.insert(b.end(), {out.reprojected[i][0], out.reprojected[i][1]});
            }
        r.push_back(inv::check_round_trip<T>(a, b, kInvStage, "roundtrip.pixel", "px", 256.0));

        // Analytic projection Jacobian vs central FD, at up to 8 points (the
        // tightest one is reported), each placed at depth 3 m along its ray.
        r.push_back(jacobian_check<T>(in, out));

        if (f.kind == FixtureKind::GroundTruth)
            r.push_back(truth_check<T>(out, f.truth));
        return r;
    }

    // ── Built-in fixtures ───────────────────────────────────────────────────
    [[nodiscard]] static std::vector<NamedFixture> builtin_fixtures() {
        return {{"known_answer_pinhole_grid", known_answer()},
                {"ground_truth_synthetic_frame", ground_truth()},
                {"captured_synthetic_frame", captured()}};
    }

    /// EuRoC cam0 intrinsics (the synthetic world's camera), 752×480.
    [[nodiscard]] static Intrinsics<double> euroc(double distortion_scale = 1.0) {
        return {458.654,
                457.296,
                367.215,
                248.375,
                -0.28340811 * distortion_scale,
                0.07395907 * distortion_scale,
                0.00019359 * distortion_scale,
                1.76187114e-05 * distortion_scale,
                0.0,
                752.0,
                480.0};
    }

    /// Distortion-free camera on an 11×7 pixel grid: the expected normalized
    /// point is ((u−cx)/fx, (v−cy)/fy) and the reprojection is the pixel itself.
    [[nodiscard]] static Fixture known_answer() {
        Input<double> in;
        in.intr = euroc(0.0);
        for (int r = 0; r < 7; ++r)
            for (int c = 0; c < 11; ++c)
                in.pixels.push_back({5.0 + 74.0 * c, 5.0 + 78.0 * r});
        Output<double> want;
        for (const auto& px : in.pixels) {
            want.valid.push_back(1);
            want.normalized.push_back({(px[0] - in.intr.cx) / in.intr.fx, (px[1] - in.intr.cy) / in.intr.fy});
            want.reprojected.push_back(px);
        }
        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::KnownAnswer;
        f.source = "s0_sensor_model_bench: EuRoC pinhole without distortion, 11x7 grid";
        f.description = "closed-form normalization; valid for every variant";
        f.input = encode_input(in);
        f.expected = encode_output(want);
        return f;
    }

    /// The synthetic world's camera (sdk/eval/synthetic_world.hpp). It matches
    /// EuRoC cam0 except p2, which is 1.76187114e-2 there vs 1.76187114e-5 in the
    /// EuRoC calibration; the ground-truth fixture caught the difference.
    [[nodiscard]] static Intrinsics<double> synthetic_world_camera() {
        auto c = euroc();
        c.p2 = 176187114.0 / 10000000000.0;
        return c;
    }

    /// The synthetic world's frame `frame`: its exact pixel observations, with
    /// truth = each landmark's camera-frame (x/z, y/z) from the true pose.
    [[nodiscard]] static Fixture ground_truth(std::size_t frame = 40) {
        const auto world = sdk::eval::generate_world<double>(sdk::eval::SyntheticConfig<double>{});
        const auto& fr = world.frames.at(frame);
        const auto& gt = world.gt.at(frame);
        Input<double> in;
        in.intr = synthetic_world_camera();
        json truth = json::array();
        for (const auto& o : fr.obs) {
            in.pixels.push_back({o.u, o.v});
            const auto& L = world.landmarks[o.feature_id];
            const auto in_imu = gt.R.inverse() * (L - gt.p);
            const auto in_cam = world.R_imu_cam.inverse() * (in_imu - world.p_imu_cam);
            truth.push_back(json::array({in_cam[0] / in_cam[2], in_cam[1] / in_cam[2]}));
        }
        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::GroundTruth;
        f.source = "synthetic_world (default config), frame " + std::to_string(frame);
        f.seed = sdk::eval::SyntheticConfig<double>{}.seed;
        f.description = "exact projections; truth = camera-frame normalized landmark coordinates";
        f.input = encode_input(in);
        f.truth = truth;
        return f;
    }

    /// A synthetic frame's observations through the shipped stage, recorded.
    [[nodiscard]] static Fixture captured(std::size_t frame = 80) {
        const auto world = sdk::eval::generate_world<double>(sdk::eval::SyntheticConfig<double>{});
        Input<double> in;
        in.intr = synthetic_world_camera();
        for (const auto& o : world.frames.at(frame).obs)
            in.pixels.push_back({o.u, o.v});
        return capture(std::string(kStage),
                       "double",
                       "synthetic_world (default config), frame " + std::to_string(frame) + " observations",
                       encode_input(in),
                       encode_output(run<double>(in, kShipped)));
    }

    // ── Characterization sweep ──────────────────────────────────────────────
    [[nodiscard]] static Sweep default_sweep() {
        Sweep sw;
        sw.axis("distortion_scale", {0.0, 0.5, 1.0, 2.0}).axis("radius_frac", {0.1, 0.4, 0.7, 1.0});
        return sw;
    }
    [[nodiscard]] static std::vector<std::string> sweep_columns() {
        return {"roundtrip_px", "jacobian_fd_err", "ext_rot_px_per_deg", "time_offset_px_per_ms"};
    }
    /// 16 pixels on a circle of radius `radius_frac` × the image half-diagonal,
    /// through a camera with EuRoC distortion × `distortion_scale`.
    template <class T>
    [[nodiscard]] static std::vector<double> sweep_point(const Point& p) {
        const double ds = p.at("distortion_scale"), rf = p.at("radius_frac");
        if (!std::isfinite(ds) || ds < 0.0 || ds > 4.0)
            throw std::invalid_argument("s0 bench sweep: distortion_scale must be in [0, 4]");
        if (!std::isfinite(rf) || rf <= 0.0 || rf > 1.0)
            throw std::invalid_argument("s0 bench sweep: radius_frac must be in (0, 1]");
        Input<double> in0;
        in0.intr = euroc(ds);
        const double half_diag = 0.5 * std::sqrt(752.0 * 752.0 + 480.0 * 480.0);
        for (int k = 0; k < 16; ++k) {
            const double a = 2.0 * std::numbers::pi * k / 16.0;
            const double u = std::clamp(in0.intr.cx + rf * half_diag * std::cos(a), 0.0, 751.0);
            const double v = std::clamp(in0.intr.cy + rf * half_diag * std::sin(a), 0.0, 479.0);
            in0.pixels.push_back({u, v});
        }
        const auto in = decode_input<T>(encode_input(in0));
        const auto out = run<T>(in, kShipped);
        const auto cam = in.intr.camera();
        double rt = 0.0, rot = 0.0, toff = 0.0;
        for (std::size_t i = 0; i < in.pixels.size(); ++i) {
            for (std::size_t c = 0; c < 2; ++c)
                rt = std::max(rt, std::abs(static_cast<double>(out.reprojected[i][c] - in.pixels[i][c])));
            // Calibration error as a pixel shift: rotate the ray by 1° (extrinsic
            // rotation error) and by 1 mrad (1 ms of offset at 1 rad/s) about y.
            const auto& n = out.normalized[i];
            auto shifted = [&](double angle) {
                const T ca = T(std::cos(angle)), sa = T(std::sin(angle));
                const std::array<T, 3> ray{ca * n[0] + sa, n[1], -sa * n[0] + ca};
                const auto px = cam.project(ray);
                const T du = px[0] - in.pixels[i][0], dv = px[1] - in.pixels[i][1];
                return std::sqrt(static_cast<double>(du * du + dv * dv));
            };
            rot = std::max(rot, shifted(std::numbers::pi / 180.0));
            toff = std::max(toff, shifted(1e-3));
        }
        return {rt, jacobian_check<T>(in, out).value, rot, toff};
    }

private:
    template <class T>
    struct FixedPoint {
        std::array<T, 2> xy{};
        bool converged = false;
    };

    /// n ← n − (distort(n) − n_d) until the step drops below 100·ε_T. A run
    /// that exhausts its iterations without the step settling is accepted only
    /// if the final residual |distort(n) − n_d| is within 1000·ε_T (roundoff
    /// limit cycles in narrow types); otherwise the pixel is reported invalid.
    template <class T>
    [[nodiscard]] static FixedPoint<T> fixed_point_undistort(const Intrinsics<T>& c, const std::array<T, 2>& px) {
        using std::abs;
        using std::isfinite;
        const auto cam = c.camera();
        const std::array<T, 2> nd{(px[0] - c.cx) / c.fx, (px[1] - c.cy) / c.fy};
        FixedPoint<T> out{nd, false};
        const T eps = T{100} * std::numeric_limits<T>::epsilon();
        for (int it = 0; it < 200 && !out.converged; ++it) {
            const auto d = cam.distort(out.xy);
            const T dx = d[0] - nd[0], dy = d[1] - nd[1];
            out.xy = {out.xy[0] - dx, out.xy[1] - dy};
            out.converged = abs(dx) + abs(dy) < eps;
        }
        if (!out.converged) {
            const auto d = cam.distort(out.xy);
            out.converged = abs(d[0] - nd[0]) + abs(d[1] - nd[1]) < T{10} * eps;
        }
        out.converged = out.converged && isfinite(out.xy[0]) && isfinite(out.xy[1]);
        return out;
    }

    template <class T>
    [[nodiscard]] static inv::InvariantResult jacobian_check(const Input<T>& in, const Output<T>& out) {
        const auto cam = in.intr.camera();
        inv::InvariantResult worst =
            inv::check_scalar(0.0, 0.0, inv::Bound::Upper, kInvStage, "jacobian.project_vs_fd", "px/m");
        std::size_t used = 0;
        const std::size_t stride = std::max<std::size_t>(1, in.pixels.size() / 8);
        for (std::size_t i = 0; i < in.pixels.size() && used < 8; i += stride) {
            if (!out.valid[i])
                continue;
            const T z{3};
            const std::array<T, 3> P{out.normalized[i][0] * z, out.normalized[i][1] * z, z};
            const auto J = cam.project_jacobian(P);
            auto f = [&](std::span<const T> x) {
                const auto px = cam.project({x[0], x[1], x[2]});
                return std::vector<T>{px[0], px[1]};
            };
            auto res = inv::check_jacobian_fd<T>(
                f, std::span<const T>(P), std::span<const T>(J), kInvStage, "jacobian.project_vs_fd");
            res.unit = "px/m";
            if (used == 0 || res.margin() > worst.margin())
                worst = res;
            ++used;
        }
        return worst;
    }

    template <class T>
    [[nodiscard]] static inv::InvariantResult truth_check(const Output<T>& out, const json& truth) {
        double worst = truth.size() == out.normalized.size() ? 0.0 : inv::detail::kInf;
        for (std::size_t i = 0; i < std::min(truth.size(), out.normalized.size()); ++i)
            for (std::size_t c = 0; c < 2; ++c) {
                const double d = std::abs(static_cast<double>(out.normalized[i][c]) - unpack_num(truth[i][c]));
                worst = std::isfinite(d) ? std::max(worst, d) : inv::detail::kInf;
            }
        return inv::check_scalar(worst,
                                 tolerance_vs_double<T>(2, 1.0, 256.0),
                                 inv::Bound::Upper,
                                 kInvStage,
                                 "truth.normalized_point",
                                 "normalized");
    }
};

}  // namespace branes::tools::bench

#endif  // BRANES_TOOLS_BENCH_S0_SENSOR_MODEL_BENCH_HPP
