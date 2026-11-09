// SPDX-License-Identifier: MIT
//
// vio_pipeline — the end-to-end VIO demo. Runs the whole pipeline as a stream
// (sensor streams → estimate stream), from one of two sources:
//
//   • synthetic — a controllable world with EXACT ground truth, so additive
//     noise on the camera + IMU streams maps cleanly to estimate error;
//   • euroc     — the real EuRoC MAV dataset (real images through the KLT front
//     end), with the dataset's millimetre ground truth.
//
// It measures how well the filter cleans up the noise (trajectory ATE vs GT,
// innovation consistency NIS) and, with --video, emits a per-frame scene + the
// overlay data so the metrics and live feature tracks can be composited onto the
// scene video (rendered by docs-site/scripts/gen-overlay.mjs).
//
//   ./vio_pipeline --out DIR                       # synthetic, matched noise
//   ./vio_pipeline --noise 3 --out DIR             # 3× the assumed sensor noise
//   ./vio_pipeline --sweep --out DIR               # noise level → robustness curve
//   ./vio_pipeline --video --out DIR               # + per-frame scene + overlay data
//   ./vio_pipeline --source euroc --dataset ROOT --video --out DIR
//   ./vio_pipeline --source euroc --dataset ROOT --out DIR   # + DIR/trajectory.tum
//   ./vio_pipeline --source euroc --dataset ROOT --capture DIR --capture-nis 5
//   ./vio_pipeline --source euroc --dataset ROOT --prefer-dynamic --capture DIR --capture-stages S1_initialization
//                                                  # stage boundaries → bench fixtures (#446)
//   ./vio_pipeline --source euroc --dataset ROOT --stage-types update-float
//                                                  # the S5→S6e update in float (#450)
//   ./vio_pipeline --source euroc --dataset ROOT --covariance sqrt
//                                                  # the square-root covariance backend (#533)

// stb_image_write generates its implementation in this TU; the define must
// immediately precede the include (guard it so include-sorting can't split them).
// clang-format off
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>
// clang-format on

#include <branes/cv/image_io.hpp>
#include <branes/sdk/euroc/asl_replay.hpp>
#include <branes/sdk/eval/synthetic_world.hpp>
#include <branes/sdk/eval/trajectory_metrics.hpp>
#include <branes/sdk/msckf/invariant_vio_backend.hpp>
#include <branes/sdk/msckf_backend.hpp>
#include <branes/sdk/vio_estimator.hpp>
#include <branes/tools/bench/c_filter_loop_bench.hpp>
#include <branes/tools/bench/stage_recorder.hpp>
#include <branes/tools/euroc_cam0.hpp>
#include <branes/tools/gt_spline.hpp>
#include <branes/tools/ideal_tracks.hpp>
#include <branes/tools/invariant_backend_adapter.hpp>
#include <branes/tools/observability_ledger.hpp>
#include <branes/tools/pose_consistency.hpp>
#include <branes/tools/stage_type_presets.hpp>
#include <branes/tools/tum_trajectory.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <random>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <type_traits>
#include <vector>

namespace {

namespace ev = branes::sdk::eval;
namespace euroc = branes::sdk::euroc;
namespace msckf = branes::sdk::msckf;
using T = double;
using Vec3 = branes::math::lie::detail::Vec<T, 3>;
using branes::sdk::FrontendObservation;
using branes::sdk::ImuMeasurement;
using branes::sdk::MsckfBackendT;
using branes::sdk::VioConfig;
using branes::tools::is_stage_type_preset;
using branes::tools::kStageTypePresets;
using branes::tools::with_stage_types;

// EuRoC cam0 calibration: shared with every EuRoC tool (branes/tools/euroc_cam0.hpp)
// so the MSCKF and R-IEKF runs are driven from byte-identical calibration.
inline branes::tools::EurocCam0<T> euroc_cam0() {
    return branes::tools::euroc_cam0<T>();
}

struct Args {
    std::string out;
    std::string source = "synthetic";
    std::string dataset;
    double noise = 1.0;
    bool sweep = false;
    bool video = false;
    std::string robot = "default";
    bool invariant = false;
    std::string stage_types = "double";  ///< a kStageTypePresets name (#450)
    bool sqrt_covariance = false;        ///< --covariance sqrt: the square-root backend (#533)
    bool help = false;
    // Stage capture (#446): fixtures written to capture_dir/<stage>/<name>.json.
    std::string capture_dir;
    branes::tools::bench::CaptureOptions capture;
    // Live-assertion mode (#447): every boundary checked against its bench; the
    // first violation is reported (and captured, with --capture).
    bool live_assert = false;
    bool prefer_dynamic = false;                 ///< EuRoC: dynamic VI init only (VioConfig::prefer_dynamic_init)
    double gate_alpha = VioConfig{}.gate_alpha;  ///< MSCKF S6d chi-square gate significance (#481)
    bool gate_per_dof = false;                   ///< MSCKF S6d: the pre-#481 5-per-dof gate
    bool gate_flag = false;                      ///< --gate-alpha or --gate-per-dof was given
    bool detect_trackable_only = false;          ///< S4: skip the KLT border band when detecting (#474)
    bool observability_ledger = false;           ///< per-stage spurious yaw/position information (#480)
    // Composition capture (#447): the loop over post-init frames [first, last].
    std::optional<std::pair<std::uint64_t, std::uint64_t>> capture_loop;
    // Ideal tracks (#602): --tracks gt replaces the KLT front end with
    // observations synthesized from ground truth (branes/tools/ideal_tracks.hpp).
    bool ideal_tracks = false;
    branes::tools::IdealTrackOptions ideal;
    // --imu gt (#613): the IMU, the ideal tracks and the score all come from one
    // smooth truth through the ground truth, so matched R and Q are the true noise.
    bool gt_imu = false;
    double camera_sigma_px = 0.0;  ///< the filter's visual R as a pixel σ on cam0; 0 = VioConfig's default
    // --riekf-legacy: the R-IEKF EuRoC adapter's pieces run as before #608, to
    // measure each one's effect.
    branes::tools::InvariantBackendAdapter<T>::Options riekf{};
    bool riekf_flag = false;
};

/// The estimator configuration the flags select (defaults otherwise).
branes::sdk::FrontendParams frontend_of(const Args& a) {
    branes::sdk::FrontendParams fe;
    fe.detect_trackable_only = a.detect_trackable_only;
    return fe;
}

VioConfig config_of(const Args& a) {
    VioConfig cfg;
    cfg.gate_alpha = a.gate_alpha;
    cfg.gate_per_dof = a.gate_per_dof;
    if (a.camera_sigma_px > 0.0)  // in normalized units through cam0's focal length
        cfg.camera_noise_normalized = a.camera_sigma_px / euroc_cam0().intrinsics.fx();
    return cfg;
}

/// A frame number: decimal digits only (std::stoull would accept "-1" and "13x").
std::uint64_t parse_frame(std::string_view s) {
    if (s.find_first_not_of("0123456789") != std::string_view::npos)
        throw std::invalid_argument("--capture-frames: '" + std::string(s) + "' is not a frame number");
    return std::stoull(std::string(s));
}

/// "A:B" → [A, B] (either side may be empty: open-ended).
void parse_frame_range(std::string_view r, branes::tools::bench::CaptureOptions& c) {
    const auto colon = r.find(':');
    const auto a = r.substr(0, colon);
    const auto b = colon == std::string_view::npos ? a : r.substr(colon + 1);
    if (!a.empty())
        c.first_frame = parse_frame(a);
    if (!b.empty())
        c.last_frame = parse_frame(b);
}

/// "S2_propagation,S6d_gating" → the stage list.
std::vector<std::string> split_commas(std::string_view list) {
    std::vector<std::string> out;
    while (!list.empty()) {
        const auto comma = list.find(',');
        if (comma != 0)
            out.emplace_back(list.substr(0, comma));
        if (comma == std::string_view::npos)
            break;
        list.remove_prefix(comma + 1);
    }
    return out;
}

Args parse(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string_view v = argv[i];
        auto next_is_value = [&] { return i + 1 < argc && std::string_view(argv[i + 1]).substr(0, 2) != "--"; };
        if (v == "--help" || v == "-h")
            a.help = true;
        else if (v == "--invariant")
            a.invariant = true;
        else if (v == "--stage-types") {
            if (!next_is_value())
                throw std::invalid_argument("--stage-types needs a preset name (see --help for the list)");
            a.stage_types = argv[++i];
            if (!is_stage_type_preset(a.stage_types))
                throw std::invalid_argument("--stage-types: unknown preset '" + a.stage_types +
                                            "' (see --help for the list)");
        } else if (v == "--covariance") {
            const std::string_view c = next_is_value() ? std::string_view(argv[++i]) : std::string_view{};
            if (c != "full" && c != "sqrt")
                throw std::invalid_argument("--covariance needs full or sqrt");
            a.sqrt_covariance = c == "sqrt";
        } else if (v == "--sweep")
            a.sweep = true;
        else if (v == "--video")
            a.video = true;
        else if (v == "--out" && next_is_value())
            a.out = argv[++i];
        else if (v == "--source" && next_is_value())
            a.source = argv[++i];
        else if (v == "--dataset" && next_is_value())
            a.dataset = argv[++i];
        else if (v == "--noise" && next_is_value()) {
            try {
                a.noise = std::stod(argv[++i]);
            } catch (const std::exception&) {
                std::cerr << "vio_pipeline: ignoring malformed --noise value, using " << a.noise << "\n";
            }
        } else if (v == "--robot" && next_is_value())
            a.robot = argv[++i];
        else if (v == "--imu") {
            if (!next_is_value())
                throw std::invalid_argument("--imu needs a value: real or gt");
            const std::string_view x = argv[++i];
            if (x != "gt" && x != "real")
                throw std::invalid_argument("--imu: real or gt");
            a.gt_imu = x == "gt";
            a.ideal.noise_normalized = a.gt_imu;  // the true noise is what the filter is told (#613)
        } else if (v == "--tracks" || v == "--track-sigma" || v == "--camera-sigma-px" || v == "--track-seed" ||
                   v == "--track-depth") {
            // A missing value is an error, not a silent fall back to the KLT run.
            if (!next_is_value())
                throw std::invalid_argument(std::string(v) + " needs a value (see --help)");
            const std::string_view x = argv[++i];
            if (v == "--tracks") {
                if (x != "gt" && x != "klt")
                    throw std::invalid_argument("--tracks: klt or gt");
                a.ideal_tracks = x == "gt";
            } else if (v == "--track-sigma") {
                a.ideal.sigma_px = std::stod(std::string(x));
                if (!(a.ideal.sigma_px >= 0.0))
                    throw std::invalid_argument("--track-sigma: a pixel sigma >= 0");
            } else if (v == "--camera-sigma-px") {
                a.camera_sigma_px = std::stod(std::string(x));
                if (!(a.camera_sigma_px > 0.0))
                    throw std::invalid_argument("--camera-sigma-px: a pixel sigma > 0");
            } else if (v == "--track-seed") {
                if (x.empty() || x.find_first_not_of("0123456789") != std::string_view::npos)
                    throw std::invalid_argument("--track-seed: a non-negative integer");
                a.ideal.seed = std::stoull(std::string(x));
            } else {
                const auto d = split_commas(x);
                if (d.size() != 2)
                    throw std::invalid_argument("--track-depth: MIN,MAX in metres");
                a.ideal.min_depth_m = std::stod(d[0]);
                a.ideal.max_depth_m = std::stod(d[1]);
                if (!(a.ideal.min_depth_m > 0.0 && a.ideal.max_depth_m >= a.ideal.min_depth_m))
                    throw std::invalid_argument("--track-depth: 0 < MIN <= MAX");
            }
        } else if (v == "--capture" && next_is_value())
            a.capture_dir = argv[++i];
        else if (v == "--capture-stages" && next_is_value())
            a.capture.stages = split_commas(argv[++i]);
        else if (v == "--capture-frames" && next_is_value())
            parse_frame_range(argv[++i], a.capture);
        else if (v == "--capture-nis" && next_is_value())
            a.capture.nis_per_dof_above = std::stod(argv[++i]);
        else if (v == "--capture-first-violation")
            a.capture.first_violation = true;
        else if (v == "--prefer-dynamic")
            a.prefer_dynamic = true;
        else if (v == "--gate-alpha") {
            if (!next_is_value())
                throw std::invalid_argument("--gate-alpha needs a significance in (0,1)");
            a.gate_alpha = std::stod(argv[++i]);
            a.gate_flag = true;
            if (!(a.gate_alpha > 0.0 && 1.0 - a.gate_alpha < 1.0 && 1.0 - a.gate_alpha > 0.0))
                throw std::invalid_argument("--gate-alpha needs a significance in (0,1)");
        } else if (v == "--detect-trackable-only")
            a.detect_trackable_only = true;
        else if (v == "--riekf-legacy") {
            if (!next_is_value())
                throw std::invalid_argument("--riekf-legacy needs a list of gate, init, zupt");
            std::stringstream list(argv[++i]);
            std::string piece;
            while (std::getline(list, piece, ',')) {
                if (piece == "gate")
                    a.riekf.legacy_gate = true;
                else if (piece == "init")
                    a.riekf.legacy_init = true;
                else if (piece == "zupt")
                    a.riekf.legacy_zupt = true;
                else
                    throw std::invalid_argument("--riekf-legacy: unknown piece '" + piece + "' (gate, init, zupt)");
            }
            a.riekf_flag = true;
        } else if (v == "--gate-per-dof") {
            a.gate_per_dof = true;
            a.gate_flag = true;
        } else if (v == "--observability-ledger") {
            a.observability_ledger = true;
        } else if (v == "--live-assert") {
            a.live_assert = true;
            a.capture.first_violation = true;
        } else if (v == "--capture-loop" && next_is_value()) {
            branes::tools::bench::CaptureOptions range;
            parse_frame_range(argv[++i], range);
            if (range.last_frame == std::numeric_limits<std::uint64_t>::max())
                throw std::invalid_argument("--capture-loop needs a closed range A:B");
            a.capture_loop = std::make_pair(range.first_frame, range.last_frame);
        } else if (v == "--capture-max" && next_is_value())
            a.capture.max_fixtures = std::stoull(argv[++i]);
    }
    // Validate every capture option here, inside main's argument try: the
    // recorder and the loop tape would otherwise throw mid-run.
    if (a.capture_loop) {
        if (a.capture_dir.empty())
            throw std::invalid_argument("--capture-loop needs --capture DIR");
        if (a.capture_loop->first > a.capture_loop->second)
            throw std::invalid_argument("--capture-loop: A > B");
    }
    if (!a.capture_dir.empty() || a.live_assert)
        (void)branes::tools::bench::StageRecorder<double>(a.capture);  // throws on a bad stage, range or threshold
    return a;
}

using Recorder = branes::tools::bench::StageRecorder<T>;

/// The stage recorder for a run, or null when --capture is not set. EuRoC and
/// the synthetic world both use a 752×480 image.
std::unique_ptr<Recorder>
make_recorder(const Args& args, std::string source, const branes::math::cameras::PinholeRadtanCamera<T>& camera) {
    if (args.capture_dir.empty() && !args.live_assert)
        return nullptr;
    auto opt = args.capture;
    opt.source = std::move(source);
    // Live assertion without --capture checks every boundary but writes nothing.
    auto sink = args.capture_dir.empty() ? Recorder::Sink([](const branes::tools::bench::NamedFixture&) {})
                                         : Recorder::write_to(args.capture_dir);
    return std::make_unique<Recorder>(opt, std::vector{Recorder::intrinsics_of(camera, 752.0, 480.0)}, std::move(sink));
}

/// The stage recorder and the loop tape a run asked for, on one tap.
struct RunTaps {
    std::unique_ptr<Recorder> recorder;
    std::unique_ptr<branes::tools::bench::LoopTape<T>> loop;
    std::unique_ptr<branes::tools::ObservabilityLedger<T>> ledger;
    branes::tools::bench::StageTapFanout<T> fanout;
    std::string source;

    /// The taps record full-covariance states; the square-root backend runs untapped.
    template <class Cov>
    [[nodiscard]] msckf::StageTap<T, Cov>* tap() {
        if constexpr (std::is_same_v<Cov, msckf::FullCovariance<T>>)
            return fanout.empty() ? nullptr : &fanout;
        else
            return nullptr;
    }
    /// Close the run: flush S0, write the loop fixture.
    void finish(const std::string& dir) {
        if (recorder)
            recorder->finish();
        if (loop && loop->started()) {
            const auto path = std::filesystem::path(dir) / "C_filter_loop" / "loop.json";
            branes::tools::bench::save(loop->fixture(source), path);
            std::cout << "  loop capture    : " << loop->input().ops.size() << " operations -> " << path.string()
                      << "\n";
        } else if (loop) {
            std::cout << "  loop capture    : the run never reached the requested frames\n";
        }
    }
};

/// Call f.template operator()<Cov, Stages>() with the MSCKF covariance policy
/// (--covariance) and the stage-type policy (--stage-types) the run asked for.
template <class F>
decltype(auto) with_msckf_types(const Args& args, F&& f) {
    return with_stage_types(args.stage_types, [&]<class S>(std::type_identity<S>) {
        if (args.sqrt_covariance)
            return f.template operator()<msckf::SqrtCovariance<T>, S>();
        return f.template operator()<msckf::FullCovariance<T>, S>();
    });
}

/// Heap-held: the fan-out holds pointers into it, so it must not move after wiring.
std::unique_ptr<RunTaps>
make_taps(const Args& args, std::string source, const branes::math::cameras::PinholeRadtanCamera<T>& camera) {
    auto t = std::make_unique<RunTaps>();
    t->source = source;
    t->recorder = make_recorder(args, source, camera);
    if (args.capture_loop)
        t->loop =
            std::make_unique<branes::tools::bench::LoopTape<T>>(args.capture_loop->first, args.capture_loop->second);
    if (args.observability_ledger)
        t->ledger = std::make_unique<branes::tools::ObservabilityLedger<T>>();
    t->fanout.add(t->recorder.get());
    t->fanout.add(t->loop.get());
    t->fanout.add(t->ledger.get());
    return t;
}

/// Print the observability ledger (--observability-ledger) and, with --out, its per-frame CSV.
void report_ledger(const RunTaps& taps, const Args& args) {
    if (!taps.ledger)
        return;
    taps.ledger->report(std::cout);
    if (args.out.empty())
        return;
    std::ofstream csv(args.out + "/observability_ledger.csv");
    if (!csv) {
        std::cerr << "vio_pipeline: cannot write " << args.out << "/observability_ledger.csv\n";
        return;
    }
    taps.ledger->write_csv(csv);
}

void report_capture(const Recorder* rec, const std::string& dir, bool live) {
    if (!rec)
        return;
    const auto& st = rec->stats();
    if (live) {
        if (const auto& v = st.first_violation)
            std::cout << "  live assertion  : FIRST VIOLATION " << v->stage << ", frame " << v->frame << " (t=" << v->t
                      << " s), " << v->invariant << " = " << v->value << " (bound " << v->threshold
                      << (v->threshold_hi != 0.0 ? ", " + std::to_string(v->threshold_hi) : std::string()) << ")\n";
        else
            std::cout << "  live assertion  : every checked boundary satisfied its bench\n";
        if (dir.empty())
            return;
    }
    std::cout << "  capture         : " << st.fixtures << " fixtures from " << st.boundaries << " boundaries -> " << dir
              << "/<stage>/";
    if (st.first_violation_frame)
        std::cout << "  (first invariant violation at frame " << *st.first_violation_frame << ")";
    if (st.chain_mismatches)
        std::cout << "\n  WARNING: " << st.chain_mismatches << " S6 boundaries did not reproduce the run";
    if (st.init_mismatches)
        std::cout << "\n  WARNING: " << st.init_mismatches << " S1 dynamic-init attempts did not reproduce the run";
    if (st.skipped)
        std::cout << "\n  skipped         : " << st.skipped << " S6 boundaries (multi-camera)";
    std::cout << "\n";
}

double norm3(const Vec3& a) {
    return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

// The NIS accumulator throws if asked to report before any innovation has been
// gated (the first frames produce no MSCKF update); 0 until then.
template <class Acc>
double safe_nis(const Acc& acc) {
    return acc.samples() > 0 ? acc.report().normalized : 0.0;
}

std::string qstr(const branes::math::lie::SO3<T>& R) {
    const auto& q = R.quaternion();
    std::ostringstream o;
    o << '[' << q[0] << ',' << q[1] << ',' << q[2] << ',' << q[3] << ']';
    return o.str();
}
std::string v3str(const Vec3& p) {
    std::ostringstream o;
    o << '[' << p[0] << ',' << p[1] << ',' << p[2] << ']';
    return o.str();
}
// The 3×3 world-position covariance block (row-major, 9 numbers) lifted from the
// dense error-state covariance at `off` (= State::kPos). Drives the 3-D pose
// uncertainty ellipsoid; the over-confidence story (#212) is exactly this block
// being too small while the true pose drifts outside it.
template <class Mat>
std::string mat3_json(const Mat& P, std::size_t off) {
    std::ostringstream o;
    o << '[';
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j) {
            if (i || j)
                o << ',';
            o << P(off + i, off + j);
        }
    o << ']';
    return o.str();
}
// Mean position σ from a covariance's 3×3 position block at `off`: √(trace/3).
// The over-confidence tell — compared against the actual ATE, a tiny σ means the
// filter trusts a covariance far smaller than its true error.
template <class Mat>
double pos_sigma(const Mat& P, std::size_t off) {
    const double tr = P(off, off) + P(off + 1, off + 1) + P(off + 2, off + 2);
    if (!std::isfinite(tr))
        return tr;  // surface a poisoned covariance — this harness exists to expose it, not hide it as 0
    return std::sqrt((tr > 0.0 ? tr : 0.0) / 3.0);
}
std::string nums_json(const std::vector<T>& xs) {
    std::ostringstream o;
    o << '[';
    for (std::size_t i = 0; i < xs.size(); ++i) {
        if (i)
            o << ',';
        o << xs[i];
    }
    o << ']';
    return o.str();
}
std::string feats_json(const std::vector<FrontendObservation<T>>& obs) {
    std::ostringstream o;
    o << '[';
    for (std::size_t i = 0; i < obs.size(); ++i) {
        if (i)
            o << ',';
        o << '[' << obs[i].u << ',' << obs[i].v << ']';
    }
    o << ']';
    return o.str();
}

// Render the synthetic scene (the true feature projections, gray dots) to a
// grayscale PNG — the "camera image" the overlay composites onto.
void render_scene_png(const std::string& path,
                      const std::vector<FrontendObservation<T>>& clean,
                      int w = 752,
                      int h = 480) {
    std::vector<std::uint8_t> img(static_cast<std::size_t>(w) * h, 38);  // dark-gray background
    auto disk = [&](int cx, int cy, int r, std::uint8_t val) {
        for (int dy = -r; dy <= r; ++dy)
            for (int dx = -r; dx <= r; ++dx) {
                if (dx * dx + dy * dy > r * r)
                    continue;
                const int x = cx + dx, y = cy + dy;
                if (x >= 0 && x < w && y >= 0 && y < h)
                    img[static_cast<std::size_t>(y) * w + x] = val;
            }
    };
    // Faint backdrop dots — the overlay draws the depth-coloured markers on top.
    for (const auto& o : clean)
        disk(static_cast<int>(o.u), static_cast<int>(o.v), 2, 70);
    stbi_write_png(path.c_str(), w, h, 1, img.data(), w);
}

// Emit the static scene: the 3D landmark cloud + the camera extrinsics and the
// four image-corner rays (via the real, distortion-aware unprojection). The 3D
// viewer renders the cloud and reconstructs the camera frustum per frame from the
// pose. Synthetic only — EuRoC has no ground-truth landmark cloud.
void write_scene_json(const std::string& path, const ev::SyntheticData<T>& w) {
    std::ofstream f(path);
    if (!f) {
        std::cerr << "vio_pipeline: cannot write " << path << "\n";
        return;
    }
    f << "{\"landmarks\":[";
    for (std::size_t i = 0; i < w.landmarks.size(); ++i) {
        if (i)
            f << ',';
        f << v3str(w.landmarks[i]);
    }
    const auto& q = w.R_imu_cam.quaternion();
    f << "],\"camera\":{\"R_imu_cam\":[" << q[0] << ',' << q[1] << ',' << q[2] << ',' << q[3]
      << "],\"p_imu_cam\":" << v3str(w.p_imu_cam) << ",\"frustum_rays\":[";
    // Image corners TL, TR, BR, BL → unit rays in the camera frame.
    const double W = 752, H = 480;
    const double corners[4][2] = {{0, 0}, {W, 0}, {W, H}, {0, H}};
    for (int i = 0; i < 4; ++i) {
        if (i)
            f << ',';
        const auto r = w.camera.unproject(branes::math::cameras::Vec2<T>{corners[i][0], corners[i][1]});
        f << '[' << r[0] << ',' << r[1] << ',' << r[2] << ']';
    }
    f << "]}}\n";
}

struct RunResult {
    double ate_rms_m = 0;
    double final_err_m = 0;
    double nis_normalized = 0;
    std::size_t frames = 0;
    double mean_features = 0;
    double mean_pos_sigma_m = 0;           ///< mean filter-reported position σ (√trace/3 of the P pos block)
    std::size_t zero_velocity_frames = 0;  ///< frames S8 held the velocity at zero (#600; EuRoC runs)
    std::string init_method;               ///< R-IEKF EuRoC: how the seed was resolved (#608)
    double init_t_s = 0.0;                 ///< R-IEKF EuRoC: when (filter time)
    /// EuRoC: gauge-anchored pose NEES against the ground truth, split yaw / roll-pitch / position (#605).
    std::optional<branes::tools::PoseConsistencySummary> consistency;
};

ev::SyntheticConfig<T> robot_preset(const std::string& r) {
    ev::SyntheticConfig<T> c;
    if (r == "ground") {
        c.trans_amp = 0.5;
        c.motion_rate = 0.6;
        c.yaw_amp = 0.3;
    } else if (r == "drone") {
        c.trans_amp = 1.2;
        c.motion_rate = 2.2;
        c.yaw_amp = 0.9;
    }
    return c;
}

// ── Synthetic Invariant source ──────────────────────────────────────────────
RunResult run_synthetic_invariant(const ev::SyntheticData<T>& w,
                                  const VioConfig& cfg,
                                  double ns,
                                  std::uint64_t seed,
                                  const Args& args,
                                  std::ofstream* traj,
                                  std::ofstream* stream,
                                  std::ofstream* frames) {
    using namespace branes::sdk::msckf;
    using Vec2 = branes::math::lie::detail::Vec<T, 2>;
    InvariantVioBackend<T, SqrtCovariance<T>>::Config icfg;
    icfg.max_clones = 11;
    icfg.backend.initial_sigma = T{1} / T{20};  // Standard initial sigma
    icfg.backend.normalized_sigma = cfg.camera_noise_normalized * ns;
    icfg.backend.noise = ImuNoise<T>{
        cfg.gyro_noise_density, cfg.accel_noise_density, cfg.gyro_bias_random_walk, cfg.accel_bias_random_walk};
    icfg.backend.enable_gating = true;
    icfg.backend.chi2_per_dof = T{16};  // 4-sigma gate
    icfg.backend.R_imu_cam = w.R_imu_cam;
    icfg.backend.p_imu_cam = w.p_imu_cam;

    InvariantVioBackend<T, SqrtCovariance<T>> backend(icfg);

    // Phase C: Enable 6-DoF online camera-to-IMU extrinsic calibration
    const T rot_sigma = cfg.calib_ext_rot_prior_deg * std::numbers::pi_v<T> / T{180};
    const T trans_sigma = cfg.calib_ext_trans_prior_mm / T{1000};
    typename InvariantVioBackend<T, SqrtCovariance<T>>::Calib cal;
    cal.R_imu_cam = w.R_imu_cam;
    cal.p_imu_cam = w.p_imu_cam;
    backend.enable_calibration({cal}, rot_sigma, trans_sigma);

    const auto& g0 = w.gt.front();
    backend.set_nav(branes::math::lie::SE23<T>(g0.R, g0.v, g0.p), w.gyro_bias, w.accel_bias, g0.t);

    const double imu_dt = 1.0 / 200.0;
    const double sg = cfg.gyro_noise_density / std::sqrt(imu_dt) * ns;
    const double sa = cfg.accel_noise_density / std::sqrt(imu_dt) * ns;
    const double spx = cfg.camera_noise_normalized * 458.0 * ns;
    std::mt19937_64 rng(seed);
    std::normal_distribution<double> N01(0.0, 1.0);

    RunResult r;
    std::size_t imu_idx = 0, feat_total = 0;
    double sq_sum = 0;
    for (std::size_t f = 0; f < w.frames.size(); ++f) {
        const double t = w.frames[f].t;
        for (; imu_idx < w.imu.size() && w.imu[imu_idx].timestamp_s <= t; ++imu_idx) {
            ImuMeasurement<T> m = w.imu[imu_idx];
            const Vec3 gyro{{m.angular_velocity[0] + sg * N01(rng),
                             m.angular_velocity[1] + sg * N01(rng),
                             m.angular_velocity[2] + sg * N01(rng)}};
            const Vec3 accel{{m.linear_acceleration[0] + sa * N01(rng),
                              m.linear_acceleration[1] + sa * N01(rng),
                              m.linear_acceleration[2] + sa * N01(rng)}};
            backend.process_imu(gyro, accel, m.timestamp_s);
        }

        const auto& gt_R = w.gt[f].R;
        const auto& gt_p = w.gt[f].p;

        std::vector<NormalizedObs<T>> obs;
        obs.reserve(w.frames[f].obs.size());
        for (const auto& o : w.frames[f].obs) {
            const double un = o.u + spx * N01(rng);
            const double vn = o.v + spx * N01(rng);
            const auto bearing = w.camera.unproject(branes::math::cameras::Vec2<T>{un, vn});
            if (bearing[2] > 0.0) {
                obs.push_back({o.feature_id, 0, Vec2{{bearing[0] / bearing[2], bearing[1] / bearing[2]}}});
            }
        }
        backend.process_camera(t, std::span<const NormalizedObs<T>>{obs});
        feat_total += obs.size();

        const auto est = backend.nav().X;
        const Vec3 ep = est.position();
        const Vec3& gp = gt_p;
        const double err = norm3(Vec3{{ep[0] - gp[0], ep[1] - gp[1], ep[2] - gp[2]}});

        if (f % 50 == 0) {
            std::printf("[run-diag] f=%zu  t=%.2f  est=(%.4f %.4f %.4f)  gt=(%.4f %.4f %.4f)  err=%.4f\n",
                        f,
                        t,
                        ep[0],
                        ep[1],
                        ep[2],
                        gp[0],
                        gp[1],
                        gp[2],
                        err);
        }
        sq_sum += err * err;
        r.final_err_m = err;
        ++r.frames;
        const double nis = 1.0;

        if (traj && traj->is_open())
            *traj << t << ',' << gp[0] << ',' << gp[1] << ',' << gp[2] << ',' << ep[0] << ',' << ep[1] << ',' << ep[2]
                  << ',' << err << ',' << obs.size() << '\n';
        if (stream && stream->is_open()) {
            *stream << "{\"type\":\"gt\",\"t\":" << t << ",\"q\":" << qstr(gt_R) << ",\"p\":" << v3str(gp) << "}\n";
            *stream << "{\"type\":\"est\",\"t\":" << t << ",\"q\":" << qstr(est.rotation()) << ",\"p\":" << v3str(ep)
                    << ",\"pos_err\":" << err << ",\"pcov\":" << mat3_json(backend.covariance(), 6) << "}\n";
        }
        if (frames && frames->is_open()) {
            const std::string rel = "scene/frame_" + std::to_string(f) + ".png";
            std::vector<FrontendObservation<T>> p_obs = w.frames[f].obs;
            for (auto& o : p_obs) {
                o.u += spx * N01(rng);
                o.v += spx * N01(rng);
            }
            render_scene_png(args.out + "/" + rel, w.frames[f].obs);
            *frames << "{\"frame\":" << f << ",\"t\":" << t << ",\"image\":\"" << rel
                    << "\",\"true\":" << feats_json(w.frames[f].obs) << ",\"obs\":" << feats_json(p_obs)
                    << ",\"depth\":" << nums_json(w.frames[f].depth) << ",\"nfeat\":" << obs.size()
                    << ",\"nis\":" << nis << ",\"pos_err\":" << err << ",\"noise\":" << ns << "}\n";
        }
    }
    r.ate_rms_m = r.frames ? std::sqrt(sq_sum / static_cast<double>(r.frames)) : 0;
    r.nis_normalized = 1.0;
    r.mean_features = w.frames.empty() ? 0 : static_cast<double>(feat_total) / static_cast<double>(w.frames.size());
    return r;
}

// ── Synthetic source ───────────────────────────────────────────────────────
template <class Cov, msckf::StageTypePolicy Stages>
RunResult run_synthetic(const ev::SyntheticData<T>& w,
                        const VioConfig& cfg,
                        double ns,
                        std::uint64_t seed,
                        const Args& args,
                        std::ofstream* traj,
                        std::ofstream* stream,
                        std::ofstream* frames,
                        msckf::StageTap<T, Cov>* tap = nullptr) {
    using Backend = MsckfBackendT<T, Cov, Stages>;
    using Cal = typename Backend::CameraCalibration;
    Cal cal;
    cal.intrinsics = w.camera;
    cal.extrinsics.R_imu_cam = w.R_imu_cam;
    cal.extrinsics.p_imu_cam = w.p_imu_cam;
    Backend backend(std::vector<Cal>{cal});
    backend.initialize(cfg);
    backend.set_stage_tap(tap);

    const double imu_dt = 1.0 / 200.0;
    const double sg = cfg.gyro_noise_density / std::sqrt(imu_dt) * ns;
    const double sa = cfg.accel_noise_density / std::sqrt(imu_dt) * ns;
    const double spx = cfg.camera_noise_normalized * 458.0 * ns;
    std::mt19937_64 rng(seed);
    std::normal_distribution<double> N01(0.0, 1.0);

    RunResult r;
    std::size_t imu_idx = 0, feat_total = 0;
    double sq_sum = 0;
    for (std::size_t f = 0; f < w.frames.size(); ++f) {
        const double t = w.frames[f].t;
        for (; imu_idx < w.imu.size() && w.imu[imu_idx].timestamp_s <= t; ++imu_idx) {
            ImuMeasurement<T> m = w.imu[imu_idx];
            for (int c = 0; c < 3; ++c) {
                m.angular_velocity[c] += sg * N01(rng);
                m.linear_acceleration[c] += sa * N01(rng);
            }
            backend.process_imu(m);
        }
        std::vector<FrontendObservation<T>> obs = w.frames[f].obs;  // noisy copy
        for (auto& o : obs) {
            o.u += spx * N01(rng);
            o.v += spx * N01(rng);
        }
        backend.process_camera(t, std::span<const FrontendObservation<T>>{obs});
        feat_total += obs.size();
        if (!backend.initialized())
            continue;
        const auto est = backend.current_state();
        const Vec3 ep = est.T_world_imu.translation();
        const Vec3& gp = w.gt[f].p;
        const double err = norm3(Vec3{{ep[0] - gp[0], ep[1] - gp[1], ep[2] - gp[2]}});
        sq_sum += err * err;
        r.final_err_m = err;
        ++r.frames;
        const double nis = safe_nis(backend.nis_consistency());

        if (traj && traj->is_open())
            *traj << t << ',' << gp[0] << ',' << gp[1] << ',' << gp[2] << ',' << ep[0] << ',' << ep[1] << ',' << ep[2]
                  << ',' << err << ',' << obs.size() << '\n';
        if (stream && stream->is_open()) {
            *stream << "{\"type\":\"gt\",\"t\":" << t << ",\"q\":" << qstr(w.gt[f].R) << ",\"p\":" << v3str(gp)
                    << "}\n";
            *stream << "{\"type\":\"est\",\"t\":" << t << ",\"q\":" << qstr(est.T_world_imu.rotation())
                    << ",\"p\":" << v3str(ep) << ",\"pos_err\":" << err
                    << ",\"pcov\":" << mat3_json(backend.state().covariance(), msckf::State<T>::kPos) << "}\n";
        }
        if (frames && frames->is_open()) {
            const std::string rel = "scene/frame_" + std::to_string(f) + ".png";
            render_scene_png(args.out + "/" + rel, w.frames[f].obs);
            *frames << "{\"frame\":" << f << ",\"t\":" << t << ",\"image\":\"" << rel
                    << "\",\"true\":" << feats_json(w.frames[f].obs) << ",\"obs\":" << feats_json(obs)
                    << ",\"depth\":" << nums_json(w.frames[f].depth) << ",\"nfeat\":" << obs.size()
                    << ",\"nis\":" << nis << ",\"pos_err\":" << err << ",\"noise\":" << ns << "}\n";
        }
    }
    r.ate_rms_m = r.frames ? std::sqrt(sq_sum / static_cast<double>(r.frames)) : 0;
    r.nis_normalized = safe_nis(backend.nis_consistency());
    r.mean_features = w.frames.empty() ? 0 : static_cast<double>(feat_total) / static_cast<double>(w.frames.size());
    return r;
}

// The estimated trajectory as <out>/trajectory.tum, one pose per line:
// "t x y z qx qy qz qw" (the TUM format), t in seconds at full precision, the
// pose of the IMU in the world frame. traj_eval scores it, and any other
// estimator's TUM output, on the same terms (docs/benchmarks/euroc-baseline.mdx).
void write_tum(const Args& args, const std::vector<ev::StampedPose<T>>& traj) {
    if (args.out.empty())
        return;
    std::ofstream f(args.out + "/trajectory.tum");
    if (!f) {
        std::cerr << "vio_pipeline: cannot write " << args.out << "/trajectory.tum\n";
        return;
    }
    branes::tools::write_tum<T>(f, traj);
}

// --imu gt (#613): replace the real IMU with one synthesized from a smooth truth
// through the ground truth, at the real IMU's timestamps and the filter's own
// noise densities, and make that truth the ground truth for the ideal tracks,
// the pose NEES and the ATE. The truth is written to truth.tum for traj_eval
// --truth. The IMU noise is seeded from the track seed but drawn separately.
constexpr double kTruthKnotS = 0.05;  ///< spline knot interval: smooths Vicon jitter, keeps the motion

static_assert(branes::tools::kImuNoiseSeedSalt != branes::tools::kTrackNoiseSeedSalt,
              "the IMU and track noise must come from independent streams");

void spline_truth(const Args& args,
                  const VioConfig& cfg,
                  std::vector<ImuMeasurement<T>>& imu,
                  const std::vector<euroc::ImageEntry>& images,
                  std::vector<ev::StampedPose<T>>& gt);

// Returns false, after printing why, when the ground truth cannot carry a
// spline (shorter than four knots, or a gap longer than a knot interval) or
// the IMU timestamps are not increasing.
bool use_spline_truth(const Args& args,
                      const VioConfig& cfg,
                      std::vector<ImuMeasurement<T>>& imu,
                      const std::vector<euroc::ImageEntry>& images,
                      std::vector<ev::StampedPose<T>>& gt) {
    if (!args.gt_imu)
        return true;
    try {
        spline_truth(args, cfg, imu, images, gt);
    } catch (const std::exception& e) {
        std::cout << "  --imu gt: cannot build the spline truth from '" << args.dataset << "': " << e.what() << "\n";
        return false;
    }
    return true;
}

void spline_truth(const Args& args,
                  const VioConfig& cfg,
                  std::vector<ImuMeasurement<T>>& imu,
                  const std::vector<euroc::ImageEntry>& images,
                  std::vector<ev::StampedPose<T>>& gt) {
    const branes::tools::PoseSpline<T> spline(gt, kTruthKnotS);
    std::vector<double> imu_t;
    imu_t.reserve(imu.size());
    for (const auto& m : imu)
        imu_t.push_back(m.timestamp_s);
    branes::tools::ImuSynthesisOptions o;
    o.gyro_noise_density = cfg.gyro_noise_density;
    o.accel_noise_density = cfg.accel_noise_density;
    o.gyro_bias_random_walk = cfg.gyro_bias_random_walk;
    o.accel_bias_random_walk = cfg.accel_bias_random_walk;
    o.gravity_magnitude = cfg.gravity_magnitude;
    o.seed =
        args.ideal.seed ^ branes::tools::kImuNoiseSeedSalt;  // independent of the track noise (static_assert above)
    imu = branes::tools::synthesize_imu<T>(spline, imu_t, o);
    // The truth at every ground-truth stamp and image time it covers.
    std::vector<double> truth_t;
    truth_t.reserve(gt.size() + images.size());
    for (const auto& g : gt)
        truth_t.push_back(g.t_s);
    for (const auto& im : images)
        truth_t.push_back(im.t_s);
    std::sort(truth_t.begin(), truth_t.end());
    truth_t.erase(std::unique(truth_t.begin(), truth_t.end()), truth_t.end());
    gt = spline.sample(truth_t);
    if (!args.out.empty()) {
        std::ofstream f(args.out + "/truth.tum");
        if (f)
            branes::tools::write_tum<T>(f, gt);
        else
            std::cerr << "vio_pipeline: cannot write " << args.out << "/truth.tum\n";
    }
}

// ── EuRoC source (real images through the KLT front end) ───────────────────
// Gated: returns false (with a hint) when the dataset is absent/unreadable.
template <class Cov, msckf::StageTypePolicy Stages>
bool run_euroc(const Args& args,
               const VioConfig& cfg,
               RunResult& out,
               std::ofstream* stream,
               std::ofstream* frames,
               msckf::StageTap<T, Cov>* tap = nullptr) {
    using Backend = MsckfBackendT<T, Cov, Stages>;
    using Estimator = branes::sdk::VioEstimator<T, Backend>;

    std::vector<ImuMeasurement<T>> imu;
    std::vector<euroc::ImageEntry> images;
    std::vector<ev::StampedPose<T>> gt;
    try {
        imu = euroc::parse_imu<T>(args.dataset);
        images = euroc::parse_images(args.dataset);
        gt = euroc::parse_groundtruth<T>(args.dataset);
    } catch (const std::exception& e) {
        std::cout << "  EuRoC dataset not readable at '" << args.dataset << "': " << e.what()
                  << "\n  Set --dataset to a mav0 root (e.g. .../V1_01_easy/mav0). Skipping.\n";
        return false;
    }
    if (images.empty() || imu.empty()) {
        std::cout << "  EuRoC streams empty at '" << args.dataset << "' (" << images.size() << " images, " << imu.size()
                  << " IMU). Skipping.\n";
        return false;
    }
    if (!use_spline_truth(args, cfg, imu, images, gt))
        return false;

    const auto cam0 = euroc_cam0();
    typename Backend::CameraCalibration cal;
    cal.intrinsics = cam0.intrinsics;
    cal.extrinsics.R_imu_cam = cam0.R_imu_cam;
    cal.extrinsics.p_imu_cam = cam0.p_imu_cam;

    Estimator est(Backend(std::vector<typename Backend::CameraCalibration>{cal}), frontend_of(args));
    est.configure(cfg);
    est.activate();
    est.backend().set_stage_tap(tap);

    std::optional<branes::tools::IdealTracks<T>> ideal;
    if (args.ideal_tracks)
        ideal.emplace(gt, cam0.intrinsics, branes::math::lie::SE3<T>(cam0.R_imu_cam, cam0.p_imu_cam), args.ideal);
    std::size_t ideal_obs = 0;
    branes::tools::PoseConsistency<T> consistency(gt);

    std::vector<ev::StampedPose<T>> traj;
    std::size_t imu_idx = 0, feat_total = 0, n = 0;
    double sigma_sum = 0;
    for (std::size_t f = 0; f < images.size(); ++f) {
        const double t = images[f].t_s;
        std::size_t end = imu_idx;
        while (end < imu.size() && imu[end].timestamp_s <= t)
            ++end;
        if (end > imu_idx) {
            est.feed_imu(std::span<const ImuMeasurement<T>>{imu.data() + imu_idx, end - imu_idx});
            imu_idx = end;
        }
        if (ideal) {
            const auto obs = ideal->observe(t);
            ideal_obs = obs.size();
            est.backend().process_camera(t, std::span<const FrontendObservation<T>>{obs});
        } else {
            branes::cv::OwnedImage<std::uint8_t> img;
            try {
                img = branes::cv::read_png(images[f].path);
            } catch (const std::exception&) {
                continue;
            }
            est.feed_image(t, std::as_const(img).view());
        }
        if (!est.backend().initialized())
            continue;  // skip pre-init frames so they don't pollute the ATE
        ev::StampedPose<T> sp;
        sp.t_s = t;
        sp.pose = est.current_pose();
        traj.push_back(sp);
        ++n;
        sigma_sum += pos_sigma(est.backend().state().covariance(), msckf::State<T>::kPos);
        consistency.add(t,
                        sp.pose,
                        est.backend().state().covariance(),
                        msckf::State<T>::kTheta,
                        msckf::State<T>::kPos,
                        branes::tools::PoseErrorConvention::body_right);
        const auto tf = est.tracked_features();
        feat_total += ideal ? ideal_obs : tf.size();
        const double nis = safe_nis(est.backend().nis_consistency());
        if (stream && stream->is_open()) {
            const Vec3 ep = est.current_pose().translation();
            *stream << "{\"type\":\"est\",\"t\":" << t << ",\"p\":" << v3str(ep)
                    << ",\"pcov\":" << mat3_json(est.backend().state().covariance(), msckf::State<T>::kPos) << "}\n";
        }
        if (frames && frames->is_open()) {
            std::ostringstream fj;
            fj << '[';
            for (std::size_t i = 0; i < tf.size(); ++i) {
                if (i)
                    fj << ',';
                fj << '[' << tf[i].x << ',' << tf[i].y << ']';
            }
            fj << ']';
            *frames << "{\"frame\":" << f << ",\"t\":" << t << ",\"image\":\"" << images[f].path
                    << "\",\"obs\":" << fj.str() << ",\"nfeat\":" << tf.size() << ",\"nis\":" << nis << "}\n";
        }
    }
    out.frames = n;
    out.mean_features = n ? static_cast<double>(feat_total) / static_cast<double>(n) : 0;
    out.nis_normalized = safe_nis(est.backend().nis_consistency());
    out.mean_pos_sigma_m = n ? sigma_sum / static_cast<double>(n) : 0;
    out.zero_velocity_frames = est.backend().zero_velocity_frames();
    out.consistency = consistency.summary();
    write_tum(args, traj);
    const auto assoc = ev::associate<T>(traj, gt, 0.02);
    if (assoc.estimated.empty()) {
        std::cerr << "  EuRoC pose association produced no matches (missing GT, timestamp drift, or no tracked "
                     "trajectory); cannot compute ATE.\n";
        return false;  // an empty association is a failure, not a perfect (ATE 0) run
    }
    out.ate_rms_m = ev::ate_rmse<T>(assoc.estimated, assoc.reference);
    return true;
}

// ── EuRoC, right-invariant (R-IEKF) backend (#347/#348) ──────────────────────
// The end-to-end counterpart to the standard run_euroc, on the SAME real frontend
// + GT, so ATE and the reported position σ are directly comparable. The invariant
// backend reaches the EuRoC pipeline through InvariantBackendAdapter (tools/),
// which bridges its parameterization-honest API to the VioEstimator contract.
bool run_euroc_invariant(const Args& args, const VioConfig& cfg, RunResult& out, std::ofstream* stream) {
    using Adapter = branes::tools::InvariantBackendAdapter<T>;
    using Estimator = branes::sdk::VioEstimator<T, Adapter>;
    constexpr std::size_t kInvTheta = 0, kInvPos = 6;  // invariant nav error-state: δθ(0) δv(3) δp(6)

    std::vector<ImuMeasurement<T>> imu;
    std::vector<euroc::ImageEntry> images;
    std::vector<ev::StampedPose<T>> gt;
    try {
        imu = euroc::parse_imu<T>(args.dataset);
        images = euroc::parse_images(args.dataset);
        gt = euroc::parse_groundtruth<T>(args.dataset);
    } catch (const std::exception& e) {
        std::cout << "  EuRoC dataset not readable at '" << args.dataset << "': " << e.what() << "\n";
        return false;
    }
    if (images.empty() || imu.empty()) {
        std::cout << "  EuRoC streams empty at '" << args.dataset << "'. Skipping.\n";
        return false;
    }
    if (!use_spline_truth(args, cfg, imu, images, gt))
        return false;

    const auto cam0 = euroc_cam0();
    Estimator est(Adapter(cam0.intrinsics, cam0.R_imu_cam, cam0.p_imu_cam, args.riekf), frontend_of(args));
    est.configure(cfg);
    est.activate();

    // Ideal tracks (#602, #605): the same ground-truth observations the MSCKF run
    // takes, handed to the adapter in pixels in place of the KLT front end.
    std::optional<branes::tools::IdealTracks<T>> ideal;
    if (args.ideal_tracks)
        ideal.emplace(gt, cam0.intrinsics, branes::math::lie::SE3<T>(cam0.R_imu_cam, cam0.p_imu_cam), args.ideal);
    std::size_t ideal_obs = 0;
    branes::tools::PoseConsistency<T> consistency(gt);

    std::vector<ev::StampedPose<T>> traj;
    std::size_t imu_idx = 0, feat_total = 0, n = 0;
    double sigma_sum = 0;
    for (std::size_t f = 0; f < images.size(); ++f) {
        const double t = images[f].t_s;
        std::size_t end = imu_idx;
        while (end < imu.size() && imu[end].timestamp_s <= t)
            ++end;
        if (end > imu_idx) {
            est.feed_imu(std::span<const ImuMeasurement<T>>{imu.data() + imu_idx, end - imu_idx});
            imu_idx = end;
        }
        if (ideal) {
            const auto obs = ideal->observe(t);
            ideal_obs = obs.size();
            est.backend().process_camera(t, std::span<const FrontendObservation<T>>{obs});
        } else {
            branes::cv::OwnedImage<std::uint8_t> img;
            try {
                img = branes::cv::read_png(images[f].path);
            } catch (const std::exception&) {
                continue;
            }
            est.feed_image(t, std::as_const(img).view());
        }
        if (!est.backend().initialized())
            continue;
        ev::StampedPose<T> sp;
        sp.t_s = t;
        sp.pose = est.current_pose();
        traj.push_back(sp);
        ++n;
        const auto P = est.backend().covariance();
        sigma_sum += pos_sigma(P, kInvPos);
        consistency.add(t, sp.pose, P, kInvTheta, kInvPos, branes::tools::PoseErrorConvention::right_invariant);
        feat_total += ideal ? ideal_obs : est.num_tracked_features();
        if (stream && stream->is_open()) {
            const Vec3 ep = est.current_pose().translation();
            *stream << "{\"type\":\"est\",\"t\":" << t << ",\"p\":" << v3str(ep)
                    << ",\"pcov\":" << mat3_json(est.backend().covariance(), kInvPos) << "}\n";
        }
    }
    out.frames = n;
    out.mean_features = n ? static_cast<double>(feat_total) / static_cast<double>(n) : 0;
    out.mean_pos_sigma_m = n ? sigma_sum / static_cast<double>(n) : 0;
    out.consistency = consistency.summary();
    out.zero_velocity_frames = est.backend().zero_velocity_frames();
    out.nis_normalized = safe_nis(est.backend().impl().nis_consistency());
    {
        const auto& d = est.backend().init_diagnostics();
        out.init_method = args.riekf.legacy_init ? "legacy static window" : branes::sdk::to_string(d.method);
        out.init_t_s = d.t_s;
    }
    write_tum(args, traj);
    const auto assoc = ev::associate<T>(traj, gt, 0.02);
    if (assoc.estimated.empty()) {
        std::cerr << "  EuRoC pose association produced no matches (missing GT, timestamp drift, or no tracked "
                     "trajectory); cannot compute ATE.\n";
        return false;  // an empty association is a failure, not a perfect (ATE 0) run
    }
    out.ate_rms_m = ev::ate_rmse<T>(assoc.estimated, assoc.reference);
    return true;
}

/// Print the EuRoC pose NEES split (#605) and, with --out, write consistency.json:
/// the split, the world-yaw σ over the run, and (with --observability-ledger)
/// the ledger's spurious yaw information and its S6 / S2 shares.
void report_consistency(const RunResult& r, const RunTaps& taps, const Args& args) {
    if (!r.consistency)
        return;
    const auto& c = *r.consistency;
    std::cout << "  pose NEES       : roll/pitch " << c.nees_rp << " (target 2), yaw " << c.nees_yaw
              << " (target 1), position " << c.nees_pos << " (target 3); " << c.frames << " frames"
              << (c.not_pd ? ", " + std::to_string(c.not_pd) + " with a non-PD covariance" : std::string{})
              << "\n  world yaw sigma : " << c.yaw_sigma_first << " -> min " << c.yaw_sigma_min << " -> "
              << c.yaw_sigma_last << " rad; |yaw error| last " << c.yaw_err_last << ", max " << c.yaw_err_max
              << " rad\n";
    if (args.out.empty())
        return;
    std::ofstream f(args.out + "/consistency.json");
    if (!f) {
        std::cerr << "vio_pipeline: cannot write " << args.out << "/consistency.json\n";
        return;
    }
    f << "{\"backend\":\"" << (args.invariant ? "riekf" : "msckf") << "\",\"pose\":";
    c.write_json(f);
    if (taps.ledger) {
        using branes::tools::LeakSource;
        const auto& t = taps.ledger->totals();
        const auto share = [&](LeakSource s) { return t.share[static_cast<std::size_t>(s)].yaw; };
        f << std::setprecision(std::numeric_limits<double>::max_digits10) << ",\"ledger\":{\"info_yaw\":" << t.info.yaw
          << ",\"share_s6_mean_shift\":" << share(LeakSource::S6_mean_shift)
          << ",\"share_s2\":" << share(LeakSource::S2_propagation) << ",\"closure_max\":" << t.closure_max << '}';
    }
    f << "}\n";
}

}  // namespace

int main(int argc, char** argv) {
    Args args;
    try {
        args = parse(argc, argv);
    } catch (const std::exception& e) {  // a malformed --capture-* number
        std::cerr << "vio_pipeline: bad argument: " << e.what() << "\n";
        return 2;
    }
    if (args.help) {
        std::ostringstream presets;
        for (const auto& p : kStageTypePresets)
            presets << "                               " << std::left << std::setw(22) << p.name << ' ' << p.summary
                    << "\n";
        std::cout
            << "vio_pipeline — end-to-end VIO noise->robustness demo\n"
               "  --source synthetic|euroc   stream source (default synthetic)\n"
               "  --dataset ROOT             EuRoC mav0 root (for --source euroc)\n"
               "  --out DIR                  write JSONL stream + CSVs (+ scene with --video); with\n"
               "                             --source euroc, also trajectory.tum (t x y z qx qy qz qw)\n"
               "  --noise S                  additive sensor-noise scale (synthetic; 1 = matched)\n"
               "  --sweep                    noise level -> robustness curve (synthetic)\n"
               "  --video                    emit per-frame scene + overlay data (frames.jsonl)\n"
               "  --invariant                use the Right-Invariant EKF (R-IEKF) backend instead of standard MSCKF\n"
               "  --stage-types NAME         MSCKF: the type each stage computes in (state stored in double):\n"
            << presets.str()
            << "  --covariance full|sqrt     MSCKF: Joseph-form P (default) or the square-root factor S\n"
            << "  --robot ground|drone       motion aggressiveness (synthetic)\n"
               "  --capture DIR              capture MSCKF stage boundaries as bench fixtures (DIR/<stage>/)\n"
               "  --capture-stages LIST      comma-separated fixture stages (default: all capturable)\n"
               "  --capture-frames A:B       post-init frame range, inclusive (default: all)\n"
               "  --capture-nis X            trigger: capture S6 updates with NIS/dof above X\n"
               "  --capture-first-violation  trigger: capture the first boundary failing its bench\n"
               "  --capture-max N            stop after N fixtures (default 100000)\n"
               "  --capture-loop A:B         capture the filter loop over frames A..B (DIR/C_filter_loop/loop.json)\n"
               "  --live-assert              check every boundary against its bench; report the first violation\n"
               "  --prefer-dynamic           EuRoC: initialize by dynamic VI alignment only (no static init);\n"
               "                             with --capture-stages S1_initialization, captures every attempt\n"
               "  --gate-alpha A             MSCKF: reject a track above the chi-square(dof) quantile at\n"
               "                             significance A (default 0.05)\n"
               "  --gate-per-dof             MSCKF: gate at 5 per dof instead (the gate before #481)\n"
               "                             (both also select the R-IEKF's gate on EuRoC, #608)\n"
               "  --riekf-legacy LIST        R-IEKF on EuRoC: run these pieces as before #608, to measure\n"
               "                             each (comma-separated): gate (16 per dof), init (a static\n"
               "                             150-sample window, v = 0, sigma 0.05), zupt (no S8)\n"
               "  --tracks klt|gt            EuRoC: the KLT front end (default) or ideal tracks\n"
               "                             synthesized from ground truth (#602)\n"
               "  --track-sigma PX           ideal tracks: Gaussian pixel noise per axis (default 1)\n"
               "  --camera-sigma-px PX       the filter's visual measurement sigma, cam0 pixels\n"
               "                             (default 0.01 normalized, ~4.6 px)\n"
               "  --track-seed N             ideal tracks: landmark and noise seed (default 1)\n"
               "  --imu real|gt              EuRoC with --tracks gt: the real IMU (default), or one synthesized\n"
               "                             from a spline through the ground truth with the filter's noise\n"
               "                             densities; that spline is then the truth (writes truth.tum), and\n"
               "                             the track noise is drawn in normalized coordinates (#613)\n"
               "  --track-depth MIN,MAX      ideal tracks: landmark depth range, m (default 2,8)\n"
               "  --detect-trackable-only    EuRoC front end: skip FAST corners in the KLT border band\n"
               "                             (#474; off by default)\n"
               "  --observability-ledger     MSCKF: attribute the spurious yaw / global-position information\n"
               "                             to the stage that introduced it (#480); with --out, also\n"
               "                             writes observability_ledger.csv\n";
        return 0;
    }
    if (args.invariant && args.stage_types != "double") {
        std::cerr << "vio_pipeline: --stage-types selects MSCKF stage types; it does not apply to --invariant\n";
        return 2;
    }
    if (args.invariant && args.gate_flag && args.source != "euroc") {
        std::cerr << "vio_pipeline: with --invariant, --gate-alpha and --gate-per-dof apply to EuRoC runs only\n";
        return 2;
    }
    if (args.invariant && args.gate_flag && args.riekf.legacy_gate) {
        std::cerr << "vio_pipeline: --riekf-legacy gate replaces the gate --gate-alpha / --gate-per-dof select\n";
        return 2;
    }
    if (args.riekf_flag && !(args.invariant && args.source == "euroc")) {
        std::cerr << "vio_pipeline: --riekf-legacy selects R-IEKF EuRoC adapter pieces; it needs --invariant "
                     "--source euroc\n";
        return 2;
    }
    if (args.ideal_tracks && args.video) {
        std::cerr << "vio_pipeline: --tracks gt does not write the per-frame tracks --video overlays\n";
        return 2;
    }
    if (args.gt_imu && !args.ideal_tracks) {
        std::cerr << "vio_pipeline: --imu gt synthesizes the IMU from the truth the ideal tracks use; it needs "
                     "--tracks gt\n";
        return 2;
    }
    if (args.ideal_tracks && args.source != "euroc") {
        std::cerr << "vio_pipeline: --tracks gt replaces the EuRoC front end; it needs --source euroc\n";
        return 2;
    }
    if (args.invariant && args.sqrt_covariance) {
        std::cerr << "vio_pipeline: --covariance selects the MSCKF covariance; it does not apply to --invariant\n";
        return 2;
    }
    // The stage recorder taps full-covariance states only.
    if ((!args.capture_dir.empty() || args.live_assert) && args.sqrt_covariance) {
        std::cerr << "vio_pipeline: --capture and --live-assert tap the full-covariance backend; they need "
                     "--covariance full\n";
        return 2;
    }
    // The stage recorder replays and checks each boundary in double, so for a
    // stage in another type it would describe a run that did not happen.
    if ((!args.capture_dir.empty() || args.live_assert) && args.stage_types != "double") {
        std::cerr << "vio_pipeline: --capture and --live-assert replay stages in double; they need --stage-types "
                     "double\n";
        return 2;
    }
    if (args.observability_ledger &&
        (args.invariant || args.sweep || args.sqrt_covariance || args.stage_types != "double")) {
        std::cerr << "vio_pipeline: --observability-ledger taps a single full-covariance MSCKF run in double; it "
                     "does not apply to --invariant, --sweep, --covariance sqrt or --stage-types\n";
        return 2;
    }
    if ((!args.capture_dir.empty() || args.live_assert) && (args.invariant || args.sweep))
        std::cerr
            << "vio_pipeline: --capture taps the MSCKF backend's single run; ignored with --invariant / --sweep\n";
    // Create the output directory up front — otherwise ofstream::open() fails
    // silently and no artifacts are written.
    if (!args.out.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(args.out, ec);
        if (args.video)
            std::filesystem::create_directories(args.out + "/scene", ec);
        if (ec)
            std::cerr << "vio_pipeline: cannot create output dir '" << args.out << "': " << ec.message() << "\n";
    }
    auto open = [&](const std::string& name) {
        std::ofstream f;
        if (!args.out.empty()) {
            f.open(args.out + "/" + name);
            if (!f)
                std::cerr << "vio_pipeline: cannot write " << args.out << "/" << name << "\n";
            // Full precision: EuRoC times are ~1.4e9 s, which the default 6
            // significant digits round to the nearest 10,000 s.
            f << std::setprecision(std::numeric_limits<double>::max_digits10);
        }
        return f;
    };

    if (args.source == "euroc") {
        auto stream = open("run.jsonl");
        auto frames = args.video ? open("frames.jsonl") : std::ofstream{};
        RunResult r;
        const auto taps = args.invariant ? std::make_unique<RunTaps>()
                                         : make_taps(args, "EuRoC " + args.dataset, euroc_cam0().intrinsics);
        const auto& rec = taps->recorder;
        VioConfig cfg = config_of(args);
        cfg.prefer_dynamic_init = args.prefer_dynamic;
        const bool ok = args.invariant
                            ? run_euroc_invariant(args, cfg, r, &stream)
                            : with_msckf_types(args, [&]<class C, class S>() {
                                  return run_euroc<C, S>(
                                      args, cfg, r, &stream, args.video ? &frames : nullptr, taps->template tap<C>());
                              });
        taps->finish(args.capture_dir);
        if (!ok)
            return 0;
        const char* backend = args.invariant ? "euroc [R-IEKF]" : "euroc [MSCKF]";
        std::cout << "  source          : " << backend << "  " << args.dataset;
        if (!args.invariant)
            std::cout << "\n  stage types     : " << args.stage_types
                      << "\n  covariance      : " << (args.sqrt_covariance ? "sqrt" : "full");
        std::cout << "\n  frames tracked  : " << r.frames << "\n  mean features   : " << r.mean_features
                  << "\n  ATE (Horn)      : " << r.ate_rms_m << " m\n  mean pos sigma  : " << r.mean_pos_sigma_m
                  << " m  (over-confidence: ATE/sigma = "
                  // Divide whenever σ is usable; let a non-finite σ propagate (don't mask a poisoned covariance as 0).
                  << (r.mean_pos_sigma_m != 0.0 ? r.ate_rms_m / r.mean_pos_sigma_m : 0.0) << ")";
        std::cout << "\n  NIS (normalized): " << r.nis_normalized;
        if (args.invariant) {
            const auto& o = args.riekf;
            std::cout << "\n  legacy pieces   : "
                      << (o.legacy_gate || o.legacy_init || o.legacy_zupt
                              ? std::string(o.legacy_gate ? "gate " : "") + (o.legacy_init ? "init " : "") +
                                    (o.legacy_zupt ? "zupt" : "")
                              : std::string("none"))
                      << "\n  init            : " << r.init_method << " at t = " << std::fixed << std::setprecision(3)
                      << r.init_t_s << std::defaultfloat << std::setprecision(6) << " s";
        }
        std::cout << "\n  zero-velocity   : " << r.zero_velocity_frames << " frames held at rest (S8)";
        std::cout << "\n";
        if (args.video && !args.invariant)
            std::cout << "  overlay:  node docs-site/scripts/gen-overlay.mjs " << args.out << "\n";
        report_capture(rec.get(), args.capture_dir, args.live_assert);
        report_ledger(*taps, args);
        report_consistency(r, *taps, args);
        return 0;
    }

    const ev::SyntheticData<T> world = ev::generate_world<T>(robot_preset(args.robot));
    const VioConfig cfg = config_of(args);
    std::cout << "synthetic world: " << world.imu.size() << " IMU, " << world.frames.size()
              << " frames, robot=" << args.robot;
    if (!args.invariant)
        std::cout << ", stage types=" << args.stage_types
                  << ", covariance=" << (args.sqrt_covariance ? "sqrt" : "full");
    std::cout << "\n";
    if (!args.out.empty())
        write_scene_json(args.out + "/scene.json", world);  // static cloud + camera, for the 3D viewer

    if (args.sweep) {
        auto sweep = open("noise_sweep.csv");
        if (sweep.is_open())
            sweep << "noise_scale,ate_rms_m,final_err_m,nis_normalized,mean_features\n";
        std::cout << "\n  noise   ATE(m)   final(m)   NIS   features\n  "
                     "------------------------------------------------\n";
        for (const double n : {0.0, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0}) {
            const RunResult r =
                args.invariant
                    ? run_synthetic_invariant(world, cfg, n, 0xC0FFEE, args, nullptr, nullptr, nullptr)
                    : with_msckf_types(args, [&]<class C, class S>() {
                          return run_synthetic<C, S>(world, cfg, n, 0xC0FFEE, args, nullptr, nullptr, nullptr);
                      });
            std::cout << "  " << std::left << std::setw(7) << n << std::setw(9) << std::setprecision(3) << r.ate_rms_m
                      << std::setw(11) << r.final_err_m << std::setw(7) << r.nis_normalized << r.mean_features << "\n";
            if (sweep.is_open())
                sweep << n << ',' << r.ate_rms_m << ',' << r.final_err_m << ',' << r.nis_normalized << ','
                      << r.mean_features << '\n';
        }
    } else {
        auto traj = open("trajectory.csv");
        if (traj.is_open())
            traj << "t,gt_x,gt_y,gt_z,est_x,est_y,est_z,pos_err,n_feat\n";
        auto stream = open("run.jsonl");
        auto frames = args.video ? open("frames.jsonl") : std::ofstream{};
        std::ostringstream src;
        src << "synthetic_world (robot=" << args.robot << ", noise x" << args.noise << ", seed 0xC0FFEE)";
        const auto taps = args.invariant ? std::make_unique<RunTaps>() : make_taps(args, src.str(), world.camera);
        const auto& rec = taps->recorder;
        const RunResult r =
            args.invariant ? run_synthetic_invariant(
                                 world, cfg, args.noise, 0xC0FFEE, args, &traj, &stream, args.video ? &frames : nullptr)
                           : with_msckf_types(args, [&]<class C, class S>() {
                                 return run_synthetic<C, S>(world,
                                                            cfg,
                                                            args.noise,
                                                            0xC0FFEE,
                                                            args,
                                                            &traj,
                                                            &stream,
                                                            args.video ? &frames : nullptr,
                                                            taps->template tap<C>());
                             });
        taps->finish(args.capture_dir);
        std::cout << "\n  noise scale     : " << args.noise << "\n  frames tracked  : " << r.frames
                  << "\n  mean features   : " << r.mean_features << "\n  ATE (RMS pos)   : " << r.ate_rms_m
                  << " m\n  final pos error : " << r.final_err_m << " m\n  NIS (normalized): " << r.nis_normalized
                  << "  (1 = consistent)\n";
        if (args.video)
            std::cout << "  overlay:  node docs-site/scripts/gen-overlay.mjs " << args.out << "\n";
        report_capture(rec.get(), args.capture_dir, args.live_assert);
        report_ledger(*taps, args);
    }
    if (!args.out.empty())
        std::cout << "\n  artifacts in " << args.out << "/\n";
    return 0;
}
