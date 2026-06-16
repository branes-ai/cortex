// SPDX-License-Identifier: MIT
//
// s5_inspect — the S5 (feature triangulation) stage inspector (epic #371, issue
// #379). The real-data companion to the synthetic S5 contract probe
// (s5_triangulation.cpp); establishes the 3-D renderer tier, built on the S4
// inspector template (s4_inspect.cpp).
//
// It runs the REAL triangulation operator (CameraUpdater::triangulate, via
// branes/tools/s5_triangulation_inspect.hpp) on real EuRoC data: the shipped
// frontend (FAST + KLT) builds multi-frame feature tracks, EuRoC ground-truth
// poses supply the clone window, and the real triangulator turns each track into
// a 3-D landmark — with the position COVARIANCE, PARALLAX, and per-observation
// REPROJECTION RESIDUALS the production path hides.
//
// It emits three artifacts:
//   • scene.json   — the landmark cloud + per-landmark covariance + camera, for
//                    the 3-D viewer (docs-site/src/components/scene3d.js).
//   • run.jsonl    — the camera path through the clone window (the viewer's frames).
//   • frames.jsonl — per-frame reprojection residuals over the image, for the
//                    image-domain overlay (docs-site/scripts/gen-overlay.mjs, S5 mode).
//
// Usage:
//   s5_inspect --dataset <EuRoC mav0 dir> [--out <dir>] [--max-frames N]
//              [--min-obs N] [--px-noise PX] [--fast-threshold F] [--target-features N]
//
//   --dataset         sequence mav0 directory (cam0/, state_groundtruth_estimate0/). Required.
//   --out             output dir (default build/stage_probes/S5)
//   --max-frames      cap frames processed
//   --min-obs         min observations for a track to triangulate   (default 3)
//   --px-noise        per-observation pixel noise for covariance     (default 1.0)
//   --fast-threshold  FAST contrast threshold                        (default 20)
//   --target-features re-detect below this many tracks               (default 150)
//
// Then render:
//   node docs-site/scripts/gen-overlay.mjs <out>        # reprojection residuals
//   (3-D landmark cloud: load scene.json + run.jsonl in the Scene3D viewer)
//
// The camera is the EuRoC cam0 pinhole-radtan model + its cam0 extrinsic. EuRoC
// (~1.5 GB) is not vendored, so this is a developer tool; the inspector logic is
// gated by tests/tools/s5_triangulation_inspect.cpp.

#include <branes/cv/image_io.hpp>
#include <branes/math/cameras.hpp>
#include <branes/sdk/euroc/asl_replay.hpp>
#include <branes/sdk/vio_estimator.hpp>  // FrontendParams
#include <branes/tools/s4_frontend_inspect.hpp>
#include <branes/tools/s5_triangulation_inspect.hpp>
#include <branes/tools/vio_trace.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace bs = branes::sdk;
namespace cam = branes::math::cameras;
namespace cv = branes::cv;
namespace bt = branes::tools;
namespace tr = branes::tools::trace;
namespace lie = branes::math::lie;
using json = nlohmann::json;

namespace {

using SO3 = lie::SO3<double>;
using SE3 = lie::SE3<double>;
using Vec3 = lie::detail::Vec<double, 3>;

struct Args {
    std::string dataset;
    std::string out = "build/stage_probes/S5";
    std::uint64_t max_frames = std::numeric_limits<std::uint64_t>::max();
    std::uint32_t min_obs = 3;
    double px_noise = 1.0;
    bs::FrontendParams fe;
    bool help = false;
};

cam::PinholeRadtanCamera<double> euroc_cam0() {
    return {458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
}
Vec3 euroc_p_imu_cam() {
    return Vec3{{-0.0216401454975, -0.064676986768, 0.00981073058949}};
}
// EuRoC cam0 extrinsic rotation R_imu_cam (body ← camera), from the dataset's
// T_BS, converted to a quaternion (w,x,y,z) at construction.
SO3 euroc_R_imu_cam() {
    const double R[9] = {0.0148655429818,
                         -0.999880929698,
                         0.00414029679422,
                         0.999557249008,
                         0.0149672133247,
                         0.025715529948,
                         -0.0257744366974,
                         0.00375618835797,
                         0.999660727178};
    const double tr_ = R[0] + R[4] + R[8];
    double w, x, y, z;
    const double s = std::sqrt(std::max(0.0, 1.0 + tr_)) * 2.0;  // s = 4w
    w = 0.25 * s;
    x = (R[7] - R[5]) / s;
    y = (R[2] - R[6]) / s;
    z = (R[3] - R[1]) / s;
    const double n = std::sqrt(w * w + x * x + y * y + z * z);
    return SO3(SO3::Quaternion{{w / n, x / n, y / n, z / n}});
}

std::uint64_t to_u64(const std::string& raw) {
    if (!raw.empty() && (raw.front() == '-' || raw.front() == '+'))
        throw std::runtime_error("s5_inspect: expected a non-negative integer, got '" + raw + "'");
    std::size_t pos = 0;
    unsigned long long v = 0;
    try {
        v = std::stoull(raw, &pos);
    } catch (const std::exception&) {
        pos = 0;
    }
    if (raw.empty() || pos != raw.size())
        throw std::runtime_error("s5_inspect: invalid integer value '" + raw + "'");
    return v;
}
double to_f64(const std::string& raw) {
    std::size_t pos = 0;
    double v = 0;
    try {
        v = std::stod(raw, &pos);
    } catch (const std::exception&) {
        pos = 0;
    }
    if (raw.empty() || pos != raw.size())
        throw std::runtime_error("s5_inspect: invalid number value '" + raw + "'");
    return v;
}

Args parse(int argc, char** argv) {
    Args a;
    auto next = [&](int& i) -> std::string {
        if (i + 1 >= argc)
            throw std::runtime_error("s5_inspect: missing value after " + std::string(argv[i]));
        return argv[++i];
    };
    for (int i = 1; i < argc; ++i) {
        std::string_view v = argv[i];
        if (v == "--help" || v == "-h")
            a.help = true;
        else if (v == "--dataset")
            a.dataset = next(i);
        else if (v == "--out")
            a.out = next(i);
        else if (v == "--max-frames")
            a.max_frames = to_u64(next(i));
        else if (v == "--min-obs")
            a.min_obs = static_cast<std::uint32_t>(to_u64(next(i)));
        else if (v == "--px-noise")
            a.px_noise = to_f64(next(i));
        else if (v == "--fast-threshold")
            a.fe.fast_threshold = to_f64(next(i));
        else if (v == "--target-features")
            a.fe.target_features = static_cast<std::size_t>(to_u64(next(i)));
        else
            throw std::runtime_error("s5_inspect: unknown argument '" + std::string(v) + "'");
    }
    return a;
}

void usage() {
    std::cout << "s5_inspect — S5 triangulation inspector (EuRoC tracks + GT poses → 3-D landmarks)\n\n"
                 "  s5_inspect --dataset <EuRoC mav0 dir> [--out <dir>] [--max-frames N]\n"
                 "             [--min-obs N] [--px-noise PX] [--fast-threshold F] [--target-features N]\n\n"
                 "  Then: node docs-site/scripts/gen-overlay.mjs <out>   (reprojection residuals)\n";
}

// Nearest ground-truth pose to time `t_s` (GT is dense vs the camera rate).
const SE3* nearest_pose(const std::vector<bs::eval::StampedPose<double>>& gt, double t_s) {
    if (gt.empty())
        return nullptr;
    std::size_t best = 0;
    double bestd = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < gt.size(); ++i) {
        const double d = std::abs(gt[i].t_s - t_s);
        if (d < bestd) {
            bestd = d;
            best = i;
        }
    }
    return &gt[best].pose;
}

json v3(const Vec3& p) {
    return json::array({p[0], p[1], p[2]});
}

}  // namespace

int main(int argc, char** argv) {
    Args args;
    try {
        args = parse(argc, argv);
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << "\n";
        return 2;
    }
    if (args.help) {
        usage();
        return 0;
    }
    if (args.dataset.empty()) {
        std::cerr << "s5_inspect: --dataset <EuRoC mav0 dir> is required.\n\n";
        usage();
        return 2;
    }

    std::vector<bs::euroc::ImageEntry> images;
    std::vector<bs::eval::StampedPose<double>> gt;
    try {
        images = bs::euroc::parse_images(args.dataset);
        gt = bs::euroc::parse_groundtruth<double>(args.dataset);
    } catch (const std::exception& ex) {
        std::cerr << "s5_inspect: " << ex.what() << "\n";
        return 1;
    }
    if (images.empty()) {
        std::cerr << "s5_inspect: no images under " << args.dataset << "/cam0\n";
        return 1;
    }
    if (gt.empty()) {
        std::cerr << "s5_inspect: no ground-truth poses (state_groundtruth_estimate0/data.csv) — "
                     "S5 needs clone poses\n";
        return 1;
    }

    const auto camera = euroc_cam0();

    // ── Pass: run the real frontend; accumulate per-feature observations and the
    // per-frame clone pose. Pixel → normalized via the real camera unproject. ──
    struct FrameObs {
        std::uint64_t id;
        double u, v;  // pixel
    };
    bt::S4FrontendInspector frontend(args.fe);
    std::vector<SE3> clone_poses;                  // one per processed frame
    std::vector<double> clone_t;                   // frame timestamps
    std::vector<std::string> clone_image;          // frame image paths
    std::vector<std::vector<FrameObs>> per_frame;  // observations seen at each frame
    std::map<std::uint64_t, std::vector<tr::S5Observation<double>>> track_obs;

    std::uint64_t processed = 0, skipped = 0;
    std::uint32_t width = 0, height = 0;
    for (const auto& frame : images) {
        if (processed >= args.max_frames)
            break;
        cv::OwnedImage<std::uint8_t> img;
        try {
            img = cv::read_png(frame.path);
        } catch (const std::exception&) {
            ++skipped;
            continue;
        }
        const SE3* pose = nearest_pose(gt, frame.t_s);
        if (!pose)
            continue;
        const auto rep = frontend.step(std::as_const(img).view(), frame.t_s, frame.path);
        width = rep.width;
        height = rep.height;

        const auto clone_index = static_cast<std::uint32_t>(clone_poses.size());
        clone_poses.push_back(*pose);
        clone_t.push_back(frame.t_s);
        clone_image.push_back(frame.path);

        std::vector<FrameObs> fobs;
        for (const auto& t : rep.tracks) {
            const cam::Vec3<double> bearing = camera.unproject(cam::Vec2<double>{t.u, t.v});
            if (!(bearing[2] > 0.0))
                continue;
            track_obs[t.id].push_back(
                tr::S5Observation<double>{clone_index, 0, {bearing[0] / bearing[2], bearing[1] / bearing[2]}});
            fobs.push_back(FrameObs{t.id, t.u, t.v});
        }
        per_frame.push_back(std::move(fobs));
        ++processed;
    }

    if (clone_poses.empty()) {
        std::cerr << "s5_inspect: no frames processed\n";
        return 1;
    }

    // ── Build the decoupled S5 input and run the real triangulator. ───────────
    tr::S5Input<double> in;
    in.clone_poses = clone_poses;
    in.extrinsics.push_back(SE3(euroc_R_imu_cam(), euroc_p_imu_cam()));
    std::vector<std::uint64_t> track_ids;
    for (const auto& [id, obs] : track_obs) {
        if (obs.size() < args.min_obs)
            continue;
        tr::S5Track<double> t;
        t.feature_id = id;
        t.obs = obs;
        in.tracks.push_back(std::move(t));
        track_ids.push_back(id);
    }

    const bt::S5TriangulationInspector inspector(458.654, args.px_noise, args.min_obs);
    const auto result = inspector.run(in);

    // Map feature_id → triangulated world point (for reprojection residuals).
    std::map<std::uint64_t, Vec3> landmark_of;
    std::uint64_t n_ok = 0;
    for (const auto& lm : result.landmarks)
        if (lm.success) {
            landmark_of[lm.feature_id] = lm.p_world;
            ++n_ok;
        }

    std::error_code ec;
    std::filesystem::create_directories(args.out, ec);

    // ── scene.json: landmark cloud + per-landmark covariance + camera. ────────
    {
        json landmarks = json::array(), landmark_cov = json::array();
        for (const auto& lm : result.landmarks) {
            if (!lm.success)
                continue;
            bool finite = true;
            for (double c : lm.cov)
                finite = finite && std::isfinite(c);
            if (!finite)
                continue;
            landmarks.push_back(v3(lm.p_world));
            landmark_cov.push_back(json(lm.cov));
        }
        const auto R_ic = euroc_R_imu_cam().quaternion();
        json frustum = json::array();
        const double corners[4][2] = {{0, 0}, {double(width), 0}, {double(width), double(height)}, {0, double(height)}};
        for (const auto& cxy : corners) {
            const cam::Vec3<double> r = camera.unproject(cam::Vec2<double>{cxy[0], cxy[1]});
            frustum.push_back(json::array({r[0], r[1], r[2]}));
        }
        json scene{{"landmarks", std::move(landmarks)},
                   {"landmark_cov", std::move(landmark_cov)},
                   {"camera",
                    json{{"R_imu_cam", json::array({R_ic[0], R_ic[1], R_ic[2], R_ic[3]})},
                         {"p_imu_cam", v3(euroc_p_imu_cam())},
                         {"frustum_rays", std::move(frustum)}}}};
        std::ofstream os(args.out + "/scene.json");
        os << scene.dump(1) << '\n';
    }

    // ── run.jsonl: the camera path through the clone window (viewer frames). ───
    {
        std::ofstream os(args.out + "/run.jsonl");
        for (std::size_t i = 0; i < clone_poses.size(); ++i) {
            const auto q = clone_poses[i].rotation().quaternion();
            json rec{{"type", "est"},
                     {"t", clone_t[i]},
                     {"q", json::array({q[0], q[1], q[2], q[3]})},
                     {"p", v3(clone_poses[i].translation())}};
            os << rec.dump() << '\n';
        }
    }

    // ── frames.jsonl: per-frame reprojection residuals over the image. ────────
    {
        std::ofstream os(args.out + "/frames.jsonl");
        for (std::size_t f = 0; f < per_frame.size(); ++f) {
            const SE3& pose = clone_poses[f];
            const SO3 R_wc = pose.rotation() * euroc_R_imu_cam();  // world ← camera
            const Vec3 center = pose.translation() + pose.rotation().matrix() * euroc_p_imu_cam();
            json residuals = json::array();
            for (const auto& o : per_frame[f]) {
                auto it = landmark_of.find(o.id);
                if (it == landmark_of.end())
                    continue;
                const Vec3 d{{it->second[0] - center[0], it->second[1] - center[1], it->second[2] - center[2]}};
                const Vec3 pc = R_wc.inverse() * d;  // camera-frame point
                if (!(pc[2] > 0.0))
                    continue;
                const cam::Vec2<double> rp = camera.project(cam::Vec3<double>{pc[0], pc[1], pc[2]});
                const double err = std::sqrt((rp[0] - o.u) * (rp[0] - o.u) + (rp[1] - o.v) * (rp[1] - o.v));
                residuals.push_back(
                    json{{"id", o.id}, {"u", o.u}, {"v", o.v}, {"pu", rp[0]}, {"pv", rp[1]}, {"err", err}});
            }
            json rec{{"kind", "s5_residuals"},
                     {"stage", "S5"},
                     {"frame", f},
                     {"t", clone_t[f]},
                     {"image", clone_image[f]},
                     {"width", width},
                     {"height", height},
                     {"residuals", std::move(residuals)}};
            os << rec.dump() << '\n';
        }
    }

    std::cout << "s5_inspect: processed " << processed << " frames";
    if (skipped)
        std::cout << " (" << skipped << " unreadable, skipped)";
    std::cout << "\n  tracks:    " << in.tracks.size() << " with >=" << args.min_obs << " obs, " << n_ok
              << " triangulated\n"
              << "  scene:     " << args.out << "/scene.json   (3-D landmark cloud + covariance)\n"
              << "  frames:    " << args.out << "/frames.jsonl  (reprojection residuals)\n"
              << "  render:    node docs-site/scripts/gen-overlay.mjs " << args.out << "\n";
    return 0;
}
