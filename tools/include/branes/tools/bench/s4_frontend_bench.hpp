// SPDX-License-Identifier: MIT
//
// branes/tools/bench/s4_frontend_bench.hpp — the S4_frontend stage bench
// (issue #456, epic #444).
//
// Stage under test: stages::s4_frontend::track(state, image, params) — the
// image-domain front end VioEstimator runs per frame (pyramidal KLT, the optional
// forward-backward gate, FAST replenishment) — over a short frame sequence, with
// its observations then fed through the backend hand-off (s4_frontend::apply and
// the S9 window purge) to check the bookkeeping. Contract (#445 §B, S4):
//   • every observation lies inside the image;
//   • ids are unique within a frame, and an id that ended never comes back;
//   • the KLT forward–backward residual (re-measured here, independently of the
//     optional gate) is bounded — enforced when the gate is on, reported when
//     it is off (the shipped default keeps drifted tracks by design);
//   • reported: the share of new detections lost on their first tracked frame
//     (KLT's coarsest-level border vs FAST's detection border);
//   • through the backend hand-off, no track is longer than the clone window.
//
// The front end is not generic in T (KLT is double/float internally): the bench
// runs in double only (SupportedTypes), per #456 "multi-type green where the
// stage is generic"; whether it becomes generic is #449's decision.
//
// Fixtures (frames stored losslessly, base64):
//   known_answer   an unchanged scene: every track stays exactly where it was,
//                  nothing is lost or re-detected after the first frame
//   ground_truth   a textured scene translating by known integer shifts; truth
//                  = the shifts (end-point error per track)
//   captured       a noisy translating sequence through the shipped front end
//
// Variants: "shipped" (the params as given: forward-backward gate off),
// "fb_gate_1px" (the gate at 1 px), "klt_window_7" (a 15×15 KLT window).
//
// Sweep: image noise × shift magnitude → survival, end-point RMS, FB median.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_BENCH_S4_FRONTEND_BENCH_HPP
#define BRANES_TOOLS_BENCH_S4_FRONTEND_BENCH_HPP

#include <branes/cv/klt.hpp>
#include <branes/cv/pyramid.hpp>
#include <branes/sdk/eval/invariants.hpp>
#include <branes/sdk/msckf/stages/s4_frontend.hpp>
#include <branes/sdk/msckf/stages/s9_marginalization.hpp>
#include <branes/tools/bench/bench.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace branes::tools::bench {

struct S4FrontendBench {
    static constexpr std::string_view kStage = "S4_frontend";
    static constexpr inv::Stage kInvStage = inv::Stage::S4_frontend;
    using SupportedTypes = TypeList<double>;  // the KLT front end is not generic in T (#449)

    using Img = cv::OwnedImage<std::uint8_t>;

    template <class T>
    struct Input {
        std::vector<Img> frames;
        sdk::FrontendParams params{};
        std::size_t window = 11;  ///< the backend's clone window, for the bookkeeping check
    };
    struct Obs {
        std::uint64_t id = 0;
        double u = 0.0, v = 0.0;
    };
    template <class T>
    struct Output {
        std::vector<std::vector<Obs>> frames;  ///< observations per frame
        std::vector<sdk::msckf::stages::s4_frontend::TrackDiagnostics> diag;
        double fb_gate = 0.0;             ///< the forward-backward gate the run used (0 = off)
        sdk::FrontendParams effective{};  ///< the params the run tracked with (not serialized)
    };

    [[nodiscard]] static std::vector<Variant> variants() {
        return {{"shipped", "stages::s4_frontend::track with the params as given (FB gate off)"},
                {"fb_gate_1px", "the forward-backward gate at 1 px"},
                {"klt_window_7", "a 15x15 KLT window (half 7)"}};
    }

    // ── Codec ───────────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static json encode_input(const Input<T>& in) {
        json frames = json::array();
        for (const auto& f : in.frames)
            frames.push_back(pack(f.view()));
        const auto& p = in.params;
        return json{{"frames", frames},
                    {"window", in.window},
                    {"params",
                     {{"pyramid_levels", p.pyramid_levels},
                      {"fast_threshold", p.fast_threshold},
                      {"target_features", p.target_features},
                      {"min_feature_distance", p.min_feature_distance},
                      {"fb_max_residual", p.fb_max_residual},
                      {"klt_window_half", p.klt.window_half},
                      {"klt_max_iters", p.klt.max_iters},
                      {"klt_residual_eps", p.klt.residual_eps},
                      {"klt_min_eigenvalue", p.klt.min_eigenvalue}}}};
    }
    template <class T>
    [[nodiscard]] static Input<T> decode_input(const json& j) {
        Input<T> in;
        for (const auto& f : j.at("frames"))
            in.frames.push_back(unpack_image(f));
        in.window = j.at("window").get<std::size_t>();
        const auto& p = j.at("params");
        in.params.pyramid_levels = p.at("pyramid_levels").get<int>();
        in.params.fast_threshold = p.at("fast_threshold").get<double>();
        in.params.target_features = p.at("target_features").get<std::size_t>();
        in.params.min_feature_distance = p.at("min_feature_distance").get<double>();
        in.params.fb_max_residual = p.at("fb_max_residual").get<double>();
        in.params.klt.window_half = p.at("klt_window_half").get<int>();
        in.params.klt.max_iters = p.at("klt_max_iters").get<int>();
        in.params.klt.residual_eps = p.at("klt_residual_eps").get<double>();
        in.params.klt.min_eigenvalue = p.at("klt_min_eigenvalue").get<double>();
        return in;
    }
    template <class T>
    [[nodiscard]] static json encode_output(const Output<T>& out) {
        json frames = json::array();
        for (const auto& f : out.frames) {
            json obs = json::array();
            for (const auto& o : f)
                obs.push_back(json::array({o.id, pack_num(o.u), pack_num(o.v)}));
            frames.push_back(obs);
        }
        json diag = json::array();
        for (const auto& d : out.diag)
            diag.push_back({d.tracks_in, d.klt_lost, d.fb_rejected, d.detected, d.tracks_out});
        return json{{"frames", frames}, {"diag", diag}, {"fb_gate", out.fb_gate}};
    }
    template <class T>
    [[nodiscard]] static Output<T> decode_output(const json& j) {
        Output<T> out;
        for (const auto& f : j.at("frames")) {
            std::vector<Obs> obs;
            for (const auto& o : f)
                obs.push_back({o.at(0).get<std::uint64_t>(), unpack_num(o.at(1)), unpack_num(o.at(2))});
            out.frames.push_back(std::move(obs));
        }
        for (const auto& d : j.at("diag")) {
            if (d.size() != 5)
                throw std::invalid_argument("s4 bench: a frame diagnostic must have 5 counts");
            out.diag.push_back({d.at(0).get<std::size_t>(),
                                d.at(1).get<std::size_t>(),
                                d.at(2).get<std::size_t>(),
                                d.at(3).get<std::size_t>(),
                                d.at(4).get<std::size_t>()});
        }
        if (out.diag.size() != out.frames.size())
            throw std::invalid_argument("s4 bench: frames and diagnostics differ in length");
        out.fb_gate = j.value("fb_gate", 0.0);
        return out;
    }

    // ── The stage ───────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static Output<T> run(const Input<T>& in, std::string_view variant) {
        auto params = in.params;
        if (variant == "fb_gate_1px")
            params.fb_max_residual = 1.0;
        else if (variant == "klt_window_7")
            params.klt.window_half = 7;
        sdk::msckf::stages::s4_frontend::FrontendState st;
        Output<T> out;
        out.fb_gate = params.fb_max_residual;
        out.effective = params;
        for (const auto& f : in.frames) {
            const auto tr = sdk::msckf::stages::s4_frontend::track<T>(st, f.view(), params);
            std::vector<Obs> obs;
            for (const auto& o : tr.observations)
                obs.push_back({o.feature_id, static_cast<double>(o.u), static_cast<double>(o.v)});
            out.frames.push_back(std::move(obs));
            out.diag.push_back(tr.diag);
        }
        return out;
    }

    template <class T>
    [[nodiscard]] static std::vector<double> flatten(const Output<T>& out) {
        std::vector<double> v{out.fb_gate};
        for (std::size_t k = 0; k < out.frames.size(); ++k) {
            const auto& d = out.diag[k];
            v.insert(v.end(),
                     {double(d.tracks_in),
                      double(d.klt_lost),
                      double(d.fb_rejected),
                      double(d.detected),
                      double(d.tracks_out)});
            for (const auto& o : out.frames[k])
                v.insert(v.end(), {static_cast<double>(o.id), o.u, o.v});
        }
        return v;
    }

    // ── The contract ────────────────────────────────────────────────────────
    template <class T>
    [[nodiscard]] static std::vector<inv::InvariantResult>
    invariants(const Input<T>& in, const Output<T>& out, const Fixture& f) {
        std::vector<inv::InvariantResult> r;
        if (in.frames.empty())
            return r;
        const double w = static_cast<double>(in.frames.front().width());
        const double h = static_cast<double>(in.frames.front().height());

        // Every observation inside the image.
        std::vector<std::array<double, 2>> pts;
        for (const auto& fr : out.frames)
            for (const auto& o : fr)
                pts.push_back({o.u, o.v});
        r.push_back(inv::check_points_in_image<double>(pts, w - 1.0, h - 1.0, 0.0, kInvStage, "tracks.in_image"));

        // Ids: unique within a frame; an id that ended never reappears.
        std::size_t dup = 0, revived = 0;
        std::set<std::uint64_t> ended, prev;
        for (const auto& fr : out.frames) {
            std::set<std::uint64_t> cur;
            for (const auto& o : fr) {
                dup += cur.insert(o.id).second ? 0 : 1;
                revived += ended.count(o.id);
            }
            for (const auto id : prev)
                if (!cur.count(id))
                    ended.insert(id);
            prev = cur;
        }
        r.push_back(inv::check_scalar(double(dup), 0.0, inv::Bound::Upper, kInvStage, "ids.unique_per_frame", "count"));
        r.push_back(
            inv::check_scalar(double(revived), 0.0, inv::Bound::Upper, kInvStage, "ids.never_revived", "count"));

        // Forward–backward residual, re-measured independently of the gate:
        // re-track each surviving observation from frame k+1 back to frame k.
        // Enforced when the run used the gate; with the gate off (the shipped
        // default) a front end keeps drifted tracks by design, so it is reported.
        for (auto& x : fb_check(in, out))
            r.push_back(x);

        // Track churn: tracks lost on their first tracked frame. KLT drops a
        // feature whose window does not fit at the coarsest pyramid level, while
        // FAST detects up to the image edge — so border detections die at once.
        r.push_back(churn_report(out));

        // Backend hand-off: ingest per frame (clone time = frame index), purge the
        // clone that leaves the window, and check no track outgrows the window.
        r.push_back(window_check(in, out));

        if (f.kind == FixtureKind::GroundTruth)
            r.push_back(truth_check(out, f.truth));
        return r;
    }

    // ── Built-in fixtures ───────────────────────────────────────────────────
    [[nodiscard]] static std::vector<NamedFixture> builtin_fixtures() {
        return {{"known_answer_static_scene", known_answer()},
                {"ground_truth_known_shifts", ground_truth()},
                {"captured_noisy_translation", captured()}};
    }

    /// A small textured scene (sinusoids + a grid of bright squares, the kind
    /// FAST and KLT work on) translated by (dx, dy), with deterministic, portable
    /// pseudo-noise of amplitude `noise` grey levels (a hash, not <random>, so
    /// the frames are identical on every standard library).
    [[nodiscard]] static Img
    render(int w, int h, double dx, double dy, double noise, std::uint32_t seed, int marker_margin = 0) {
        Img img(static_cast<std::size_t>(w), static_cast<std::size_t>(h));
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x) {
                const double px = x - dx, py = y - dy;
                double v = 100.0 + 40.0 * std::sin(0.21 * px + 0.10 * py) + 35.0 * std::cos(0.13 * px - 0.17 * py) +
                           25.0 * std::sin(0.07 * px) * std::cos(0.09 * py);
                const double mx = px - 32.0 * std::floor(px / 32.0), my = py - 32.0 * std::floor(py / 32.0);
                const bool interior =
                    x >= marker_margin && y >= marker_margin && x < w - marker_margin && y < h - marker_margin;
                if (mx < 6.0 && my < 6.0 && interior)
                    v = 245.0;
                if (noise > 0.0) {
                    std::uint32_t k = seed ^ (static_cast<std::uint32_t>(x) * 73856093u) ^
                                      (static_cast<std::uint32_t>(y) * 19349663u);
                    k ^= k >> 13;
                    k *= 0x5bd1e995u;
                    k ^= k >> 15;
                    v += noise * ((static_cast<double>(k & 0xFFFF) / 65535.0) * 2.0 - 1.0);
                }
                img(static_cast<std::size_t>(y), static_cast<std::size_t>(x)) =
                    static_cast<std::uint8_t>(std::clamp(v, 0.0, 255.0));
            }
        return img;
    }

    /// The same frame three times. Tracking an unchanged image is a fixed point
    /// of KLT (the image difference, hence every update, is exactly zero), so the
    /// expected output is: frame-0 detections, then every track exactly where it
    /// was with nothing lost or re-detected. The frame-0 detections come from the
    /// detector itself; the known answer is the tracking that follows.
    [[nodiscard]] static Fixture known_answer() {
        Input<double> in;
        for (int k = 0; k < 3; ++k)
            in.frames.push_back(render(160, 120, 0.0, 0.0, 0.0, 0, /*marker_margin=*/12));
        in.params.target_features = 40;
        in.params.pyramid_levels = 1;  // corners well inside every KLT window bound
        const auto first = run<double>(Input<double>{{in.frames.front()}, in.params, in.window}, kShipped);
        Output<double> want;
        want.frames = {first.frames[0], first.frames[0], first.frames[0]};
        const std::size_t n = first.frames[0].size();
        want.diag = {first.diag[0], {n, 0, 0, 0, n}, {n, 0, 0, 0, n}};
        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::KnownAnswer;
        f.variant = "shipped";
        f.source = "s4_frontend_bench: 160x120 textured scene, interior corners, unchanged over 3 frames, 1 level";
        f.description = "tracking an unchanged image returns every track exactly in place";
        f.input = encode_input(in);
        f.expected = encode_output(want);
        return f;
    }

    /// A textured scene translating by known integer shifts frame to frame.
    [[nodiscard]] static Fixture ground_truth() {
        // Integer shifts: the marker squares are binary, so a sub-pixel shift
        // moves their sampled corners by a whole pixel and the truth would be
        // the aliased motion, not the shift.
        const std::vector<std::array<double, 2>> shifts{{0.0, 0.0}, {2.0, -1.0}, {3.0, -2.0}, {5.0, -3.0}};
        Input<double> in;
        json truth = json::array();
        for (const auto& s : shifts) {
            in.frames.push_back(render(160, 120, s[0], s[1], 0.0, 0));
            truth.push_back(json::array({s[0], s[1]}));
        }
        in.params.target_features = 40;
        Fixture f;
        f.stage = std::string(kStage);
        f.kind = FixtureKind::GroundTruth;
        f.source = "s4_frontend_bench: 160x120 textured scene, known integer translations";
        f.description = "truth: the cumulative scene shift per frame";
        f.input = encode_input(in);
        f.truth = truth;
        return f;
    }

    /// A noisy translating sequence through the shipped front end, recorded.
    [[nodiscard]] static Fixture captured() {
        Input<double> in;
        for (int k = 0; k < 4; ++k)
            in.frames.push_back(render(160, 120, 1.5 * k, -0.5 * k, 4.0, 0xC0FFEEu + static_cast<std::uint32_t>(k)));
        in.params.target_features = 40;
        return capture(std::string(kStage),
                       "double",
                       "s4_frontend_bench: noisy (4 grey levels) translating sequence",
                       encode_input(in),
                       encode_output(run<double>(in, kShipped)));
    }

    // ── Characterization sweep ──────────────────────────────────────────────
    [[nodiscard]] static Sweep default_sweep() {
        Sweep sw;
        sw.axis("noise", {0.0, 4.0, 12.0, 25.0}).axis("shift_px", {0.5, 2.0, 5.0});
        return sw;
    }
    [[nodiscard]] static std::vector<std::string> sweep_columns() {
        return {"survival_pct", "endpoint_rms_px", "fb_median_px"};
    }
    template <class T>
    [[nodiscard]] static std::vector<double> sweep_point(const Point& p) {
        const double noise = p.at("noise"), shift = p.at("shift_px");
        if (!std::isfinite(noise) || noise < 0.0 || noise > 100.0)
            throw std::invalid_argument("s4 bench sweep: noise must be in [0, 100]");
        if (!std::isfinite(shift) || shift < 0.0 || shift > 20.0)
            throw std::invalid_argument("s4 bench sweep: shift_px must be in [0, 20]");
        Input<T> in;
        in.frames = {render(160, 120, 0.0, 0.0, noise, 1u), render(160, 120, shift, -0.5 * shift, noise, 2u)};
        in.params.target_features = 40;
        const auto out = run<T>(in, kShipped);
        const auto& f0 = out.frames[0];
        const auto& f1 = out.frames[1];
        double se = 0.0;
        std::size_t matched = 0;
        for (const auto& o1 : f1)
            for (const auto& o0 : f0)
                if (o0.id == o1.id) {
                    const double ex = o1.u - (o0.u + shift), ey = o1.v - (o0.v - 0.5 * shift);
                    se += ex * ex + ey * ey;
                    ++matched;
                }
        const auto fb = fb_residuals(in, out);
        auto med = fb;
        std::sort(med.begin(), med.end());
        return {f0.empty() ? 0.0 : 100.0 * double(matched) / double(f0.size()),
                matched ? std::sqrt(se / double(matched)) : std::nan(""),
                med.empty() ? std::nan("") : med[med.size() / 2]};
    }

private:
    /// Forward–backward residual of every track continuing from frame k to k+1:
    /// KLT from frame k+1 back to frame k, distance to the frame-k position (px).
    template <class T>
    [[nodiscard]] static std::vector<double> fb_residuals(const Input<T>& in, const Output<T>& out) {
        std::vector<double> res;
        for (std::size_t k = 0; k + 1 < out.frames.size(); ++k) {
            // Re-track with the parameters the run used (the variant's), so the
            // residual measures that run's tracking.
            const auto& fe = out.effective;
            const cv::Pyramid<std::uint8_t> p0(in.frames[k].view(), std::max(1, fe.pyramid_levels));
            const cv::Pyramid<std::uint8_t> p1(in.frames[k + 1].view(), std::max(1, fe.pyramid_levels));
            std::vector<cv::KeyPoint> pts;
            std::vector<std::array<double, 2>> origin;
            for (const auto& o1 : out.frames[k + 1])
                for (const auto& o0 : out.frames[k])
                    if (o0.id == o1.id) {
                        pts.push_back(cv::KeyPoint{static_cast<float>(o1.u), static_cast<float>(o1.v), 0.0f});
                        origin.push_back({o0.u, o0.v});
                    }
            const auto back = cv::track_klt_pyramidal(p1, p0, pts, fe.klt);
            for (std::size_t i = 0; i < back.size(); ++i) {
                if (back[i].status != cv::TrackStatus::Tracked) {
                    res.push_back(inv::detail::kInf);  // can't round-trip at all
                    continue;
                }
                const double dx = back[i].x - origin[i][0], dy = back[i].y - origin[i][1];
                res.push_back(std::sqrt(dx * dx + dy * dy));
            }
        }
        return res;
    }

    template <class T>
    [[nodiscard]] static std::vector<inv::InvariantResult> fb_check(const Input<T>& in, const Output<T>& out) {
        const auto res = fb_residuals(in, out);
        const double bound = out.fb_gate > 0.0 ? out.fb_gate : 1.0;  // 1 px: the natural gate when none is set
        std::size_t unreturnable = 0, over = 0;
        double worst = 0.0;
        for (const double x : res) {
            if (!std::isfinite(x)) {
                ++unreturnable;
                continue;
            }
            worst = std::max(worst, x);
            over += x > bound ? 1 : 0;
        }
        auto a = inv::check_scalar(worst, bound, inv::Bound::Upper, kInvStage, "klt.forward_backward_max", "px");
        auto b = inv::check_scalar(
            double(unreturnable + over), 0.0, inv::Bound::Upper, kInvStage, "klt.forward_backward_failures", "count");
        if (!(out.fb_gate > 0.0)) {
            a.bound = b.bound = inv::Bound::Report;  // gate off by design: measured, not enforced
            a.pass = b.pass = true;
        }
        return {a, b};
    }

    template <class T>
    [[nodiscard]] static inv::InvariantResult churn_report(const Output<T>& out) {
        std::size_t born = 0, died_young = 0;
        for (std::size_t k = 0; k + 1 < out.frames.size(); ++k) {
            const auto& d = out.diag[k];
            born += d.detected;
            // Of this frame's new detections (the last `detected` observations),
            // how many are gone in the next frame?
            const auto& fr = out.frames[k];
            for (std::size_t i = fr.size() - std::min(fr.size(), d.detected); i < fr.size(); ++i) {
                bool alive = false;
                for (const auto& o : out.frames[k + 1])
                    alive = alive || o.id == fr[i].id;
                died_young += alive ? 0 : 1;
            }
        }
        auto r = inv::check_scalar(born ? 100.0 * double(died_young) / double(born) : 0.0,
                                   0.0,
                                   inv::Bound::Report,
                                   kInvStage,
                                   "tracks.lost_on_first_frame",
                                   "%");
        return r;
    }

    template <class T>
    [[nodiscard]] static inv::InvariantResult window_check(const Input<T>& in, const Output<T>& out) {
        namespace st = sdk::msckf::stages;
        st::s4_frontend::TrackTable<double> tracks;
        std::vector<std::size_t> worst_lengths;
        auto identity = [](std::uint32_t, double u, double v, math::lie::detail::Vec<double, 2>& xy) {
            xy = {{u, v}};
            return true;
        };
        for (std::size_t k = 0; k < out.frames.size(); ++k) {
            if (k >= in.window)
                st::s9_marginalization::purge(tracks, static_cast<double>(k - in.window));
            std::vector<sdk::FrontendObservation<double>> obs;
            for (const auto& o : out.frames[k])
                obs.push_back({o.id, 0, o.u, o.v});
            const auto d = st::s4_frontend::apply<double>(tracks, static_cast<double>(k), obs, identity);
            for (const auto id : d.ended)
                tracks.erase(id);  // the backend updates with and drops ended tracks
            for (const auto& [id, recs] : tracks)
                worst_lengths.push_back(recs.size());
        }
        return inv::check_track_lengths(worst_lengths, in.window, kInvStage, "tracks.length_vs_clone_window");
    }

    template <class T>
    [[nodiscard]] static inv::InvariantResult truth_check(const Output<T>& out, const json& truth) {
        // End-point error: each continuing track must move by the frame-to-frame shift.
        double worst = out.frames.size() == truth.size() ? 0.0 : inv::detail::kInf;
        std::size_t matched = 0;
        for (std::size_t k = 0; k + 1 < std::min(out.frames.size(), truth.size()); ++k) {
            const double sx = unpack_num(truth[k + 1][0]) - unpack_num(truth[k][0]);
            const double sy = unpack_num(truth[k + 1][1]) - unpack_num(truth[k][1]);
            for (const auto& o1 : out.frames[k + 1])
                for (const auto& o0 : out.frames[k])
                    if (o0.id == o1.id) {
                        const double e = std::hypot(o1.u - (o0.u + sx), o1.v - (o0.v + sy));
                        worst = std::isfinite(e) ? std::max(worst, e) : inv::detail::kInf;
                        ++matched;
                    }
        }
        // No continuing track means nothing was checked: that is a failure, not a pass.
        if (matched == 0)
            worst = inv::detail::kInf;
        // A clean translation must be tracked to a fraction of a pixel.
        return inv::check_scalar(worst, 0.25, inv::Bound::Upper, kInvStage, "truth.endpoint_error", "px");
    }
};

}  // namespace branes::tools::bench

#endif  // BRANES_TOOLS_BENCH_S4_FRONTEND_BENCH_HPP
