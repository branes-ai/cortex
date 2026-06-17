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
#include <branes/sdk/sfm/init_window.hpp>  // so3_from_matrix
#include <branes/sdk/vio_estimator.hpp>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <random>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace {

namespace ev = branes::sdk::eval;
namespace euroc = branes::sdk::euroc;
namespace msckf = branes::sdk::msckf;
using T = double;
using Vec3 = branes::math::lie::detail::Vec<T, 3>;
using branes::sdk::FrontendObservation;
using branes::sdk::ImuMeasurement;
using branes::sdk::MsckfBackend;
using branes::sdk::VioConfig;

struct Args {
    std::string out;
    std::string source = "synthetic";
    std::string dataset;
    double noise = 1.0;
    bool sweep = false;
    bool video = false;
    std::string robot = "default";
    bool invariant = false;
    bool help = false;
};

Args parse(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string_view v = argv[i];
        auto next_is_value = [&] { return i + 1 < argc && std::string_view(argv[i + 1]).substr(0, 2) != "--"; };
        if (v == "--help" || v == "-h")
            a.help = true;
        else if (v == "--invariant")
            a.invariant = true;
        else if (v == "--sweep")
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
    }
    return a;
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
RunResult run_synthetic(const ev::SyntheticData<T>& w,
                        const VioConfig& cfg,
                        double ns,
                        std::uint64_t seed,
                        const Args& args,
                        std::ofstream* traj,
                        std::ofstream* stream,
                        std::ofstream* frames) {
    using Cal = MsckfBackend<T>::CameraCalibration;
    Cal cal;
    cal.intrinsics = w.camera;
    cal.extrinsics.R_imu_cam = w.R_imu_cam;
    cal.extrinsics.p_imu_cam = w.p_imu_cam;
    MsckfBackend<T> backend(std::vector<Cal>{cal});
    backend.initialize(cfg);

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

// ── EuRoC source (real images through the KLT front end) ───────────────────
// Gated: returns false (with a hint) when the dataset is absent/unreadable.
bool run_euroc(const Args& args, const VioConfig& cfg, RunResult& out, std::ofstream* stream, std::ofstream* frames) {
    using Backend = MsckfBackend<T>;
    using Estimator = branes::sdk::VioEstimator<T, Backend>;
    using Mat3 = branes::math::lie::detail::Mat<T, 3, 3>;

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

    typename Backend::CameraCalibration cal;
    cal.intrinsics = typename Backend::Camera(
        458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05);
    Mat3 R{};
    R(0, 0) = 0.0148655429818;
    R(0, 1) = -0.999880929698;
    R(0, 2) = 0.00414029679422;
    R(1, 0) = 0.999557249008;
    R(1, 1) = 0.0149672133247;
    R(1, 2) = 0.025715529948;
    R(2, 0) = -0.0257744366974;
    R(2, 1) = 0.00375618835797;
    R(2, 2) = 0.999660727178;
    cal.extrinsics.R_imu_cam = branes::sdk::sfm::so3_from_matrix<T>(R);
    cal.extrinsics.p_imu_cam = Vec3{{-0.0216401454975, -0.064676986768, 0.00981073058949}};

    Estimator est(Backend(std::vector<typename Backend::CameraCalibration>{cal}));
    est.configure(cfg);
    est.activate();

    std::vector<ev::StampedPose<T>> traj;
    std::size_t imu_idx = 0, feat_total = 0, n = 0;
    for (std::size_t f = 0; f < images.size(); ++f) {
        const double t = images[f].t_s;
        std::size_t end = imu_idx;
        while (end < imu.size() && imu[end].timestamp_s <= t)
            ++end;
        if (end > imu_idx) {
            est.feed_imu(std::span<const ImuMeasurement<T>>{imu.data() + imu_idx, end - imu_idx});
            imu_idx = end;
        }
        branes::cv::OwnedImage<std::uint8_t> img;
        try {
            img = branes::cv::read_png(images[f].path);
        } catch (const std::exception&) {
            continue;
        }
        est.feed_image(t, std::as_const(img).view());
        if (!est.backend().initialized())
            continue;  // skip pre-init frames so they don't pollute the ATE
        ev::StampedPose<T> sp;
        sp.t_s = t;
        sp.pose = est.current_pose();
        traj.push_back(sp);
        ++n;
        const auto tf = est.tracked_features();
        feat_total += tf.size();
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
    const auto assoc = ev::associate<T>(traj, gt, 0.02);
    out.ate_rms_m = assoc.estimated.empty() ? 0 : ev::ate_rmse<T>(assoc.estimated, assoc.reference);
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    const Args args = parse(argc, argv);
    if (args.help) {
        std::cout
            << "vio_pipeline — end-to-end VIO noise->robustness demo\n"
               "  --source synthetic|euroc   stream source (default synthetic)\n"
               "  --dataset ROOT             EuRoC mav0 root (for --source euroc)\n"
               "  --out DIR                  write JSONL stream + CSVs (+ scene with --video)\n"
               "  --noise S                  additive sensor-noise scale (synthetic; 1 = matched)\n"
               "  --sweep                    noise level -> robustness curve (synthetic)\n"
               "  --video                    emit per-frame scene + overlay data (frames.jsonl)\n"
               "  --invariant                use the Right-Invariant EKF (R-IEKF) backend instead of standard MSCKF\n"
               "  --robot ground|drone       motion aggressiveness (synthetic)\n";
        return 0;
    }
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
        }
        return f;
    };

    if (args.source == "euroc") {
        auto stream = open("run.jsonl");
        auto frames = args.video ? open("frames.jsonl") : std::ofstream{};
        RunResult r;
        if (!run_euroc(args, VioConfig{}, r, &stream, args.video ? &frames : nullptr))
            return 0;
        std::cout << "  source euroc    : " << args.dataset << "\n  frames tracked  : " << r.frames
                  << "\n  mean features   : " << r.mean_features << "\n  ATE (Horn)      : " << r.ate_rms_m
                  << " m\n  NIS (normalized): " << r.nis_normalized << "\n";
        if (args.video)
            std::cout << "  overlay:  node docs-site/scripts/gen-overlay.mjs " << args.out << "\n";
        return 0;
    }

    const ev::SyntheticData<T> world = ev::generate_world<T>(robot_preset(args.robot));
    const VioConfig cfg;
    std::cout << "synthetic world: " << world.imu.size() << " IMU, " << world.frames.size()
              << " frames, robot=" << args.robot << "\n";
    if (!args.out.empty())
        write_scene_json(args.out + "/scene.json", world);  // static cloud + camera, for the 3D viewer

    if (args.sweep) {
        auto sweep = open("noise_sweep.csv");
        if (sweep.is_open())
            sweep << "noise_scale,ate_rms_m,final_err_m,nis_normalized,mean_features\n";
        std::cout << "\n  noise   ATE(m)   final(m)   NIS   features\n  "
                     "------------------------------------------------\n";
        for (const double n : {0.0, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0}) {
            const RunResult r = args.invariant
                                    ? run_synthetic_invariant(world, cfg, n, 0xC0FFEE, args, nullptr, nullptr, nullptr)
                                    : run_synthetic(world, cfg, n, 0xC0FFEE, args, nullptr, nullptr, nullptr);
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
        const RunResult r =
            args.invariant
                ? run_synthetic_invariant(
                      world, cfg, args.noise, 0xC0FFEE, args, &traj, &stream, args.video ? &frames : nullptr)
                : run_synthetic(world, cfg, args.noise, 0xC0FFEE, args, &traj, &stream, args.video ? &frames : nullptr);
        std::cout << "\n  noise scale     : " << args.noise << "\n  frames tracked  : " << r.frames
                  << "\n  mean features   : " << r.mean_features << "\n  ATE (RMS pos)   : " << r.ate_rms_m
                  << " m\n  final pos error : " << r.final_err_m << " m\n  NIS (normalized): " << r.nis_normalized
                  << "  (1 = consistent)\n";
        if (args.video)
            std::cout << "  overlay:  node docs-site/scripts/gen-overlay.mjs " << args.out << "\n";
    }
    if (!args.out.empty())
        std::cout << "\n  artifacts in " << args.out << "/\n";
    return 0;
}
