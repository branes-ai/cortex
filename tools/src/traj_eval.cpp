// SPDX-License-Identifier: MIT
//
// traj_eval — score one estimated trajectory against EuRoC ground truth, the
// same way for every estimator (docs/benchmarks/euroc-baseline.mdx).
//
// The estimate is a TUM file ("t x y z qx qy qz qw", seconds, the IMU pose in
// the world frame), as vio_pipeline writes to <out>/trajectory.tum. Each
// estimated pose is matched to the nearest ground-truth sample within 20 ms
// (ev::associate). Reported:
//
//   ate_se3      translational RMSE after a rigid (6-DoF) Horn alignment, as
//                vio_pipeline's ATE; the stricter score, since a VIO observes
//                roll and pitch
//   ate_4dof     RMSE after a position + yaw alignment: what a VIO can't
//                observe is removed and nothing else; the usual convention
//                for comparing VIO estimators
//   rpe          per segment length (default 8, 16, 24, 32, 40 m of
//                ground-truth travel): mean translation (m) and rotation (deg)
//                error of the relative motion, and their means over lengths
//
//   ./traj_eval --dataset .../V1_01_easy/mav0 --est run/trajectory.tum [--json]
//   ./traj_eval --dataset ... --est ... --from 40      # skip the first 40 s of GT
//   ./traj_eval --dataset ... --est ... --lengths 4,8  # other segment lengths
//
// --from/--to are seconds after the first ground-truth stamp, so two
// estimators can be scored over the same window; without them every pose is
// kept, including one stamped just before the first ground-truth sample. Exit status: 0 scored,
// 1 nothing to score (no file, no matches), 2 bad arguments.

#include <branes/sdk/euroc/asl_replay.hpp>
#include <branes/sdk/eval/trajectory_metrics.hpp>
#include <branes/tools/tum_trajectory.hpp>

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

namespace bs = branes::sdk;
namespace ev = branes::sdk::eval;
using T = double;
using json = nlohmann::json;

struct Args {
    std::string dataset, est;
    double from_s = -std::numeric_limits<double>::infinity();  ///< default: no window
    double to_s = std::numeric_limits<double>::infinity();
    std::vector<double> lengths{8.0, 16.0, 24.0, 32.0, 40.0};
    bool json = false;
    bool help = false;
};

std::vector<double> parse_lengths(const std::string& s) {
    std::vector<double> out;
    std::stringstream ss(s);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        const double L = std::stod(tok);
        if (!(L > 0.0))
            throw std::invalid_argument("--lengths: every length must be > 0 m");
        out.push_back(L);
    }
    if (out.empty())
        throw std::invalid_argument("--lengths needs at least one length");
    return out;
}

Args parse(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string_view v = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc)
                throw std::invalid_argument(std::string(v) + " needs a value");
            return argv[++i];
        };
        if (v == "--help" || v == "-h")
            a.help = true;
        else if (v == "--dataset")
            a.dataset = next();
        else if (v == "--est")
            a.est = next();
        else if (v == "--from")
            a.from_s = std::stod(next());
        else if (v == "--to")
            a.to_s = std::stod(next());
        else if (v == "--lengths")
            a.lengths = parse_lengths(next());
        else if (v == "--json")
            a.json = true;
        else
            throw std::invalid_argument("unknown flag " + std::string(v));
    }
    if (!(a.to_s > a.from_s))
        throw std::invalid_argument("--to must be after --from");
    return a;
}

void usage() {
    std::cout << "usage: traj_eval --dataset <mav0 dir> --est <trajectory.tum> [--from S] [--to S]\n"
                 "                 [--lengths 8,16,24,32,40] [--json]\n";
}

int run(const Args& args) {
    const auto gt = bs::euroc::parse_groundtruth<T>(args.dataset);
    if (gt.empty()) {
        std::cerr << "traj_eval: no ground truth under " << args.dataset << '\n';
        return 1;
    }
    std::ifstream f(args.est);
    if (!f) {
        std::cerr << "traj_eval: cannot read " << args.est << '\n';
        return 1;
    }
    const double t0 = gt.front().t_s;
    std::vector<ev::StampedPose<T>> est;
    for (const auto& sp : branes::tools::read_tum<T>(f))
        if (sp.t_s >= t0 + args.from_s && sp.t_s <= t0 + args.to_s)
            est.push_back(sp);

    const auto m = ev::associate<T>(est, gt, 0.02);
    if (m.estimated.empty()) {
        std::cerr << "traj_eval: no estimated pose within 20 ms of ground truth in the window\n";
        return 1;
    }
    const auto segs = ev::rpe_by_distance<T>(m.estimated, m.reference, args.lengths);

    json j;
    j["poses"] = m.estimated.size();
    j["start_s"] = m.estimated.front().t_s - t0;  // after the first GT stamp
    j["coverage_s"] = m.estimated.back().t_s - m.estimated.front().t_s;
    j["gt_span_s"] = gt.back().t_s - t0;
    j["ate_se3"] = ev::ate_rmse<T>(m.estimated, m.reference);
    j["ate_4dof"] = ev::ate_rmse_4dof<T>(m.estimated, m.reference);
    j["rpe"] = json::object();
    double tsum = 0.0, rsum = 0.0;
    for (const auto& s : segs) {
        std::ostringstream key;
        key << s.length_m;
        j["rpe"][key.str()] = {{"trans_m", s.translation_m}, {"rot_deg", s.rotation_deg}, {"n", s.pairs}};
        tsum += s.translation_m;
        rsum += s.rotation_deg;
    }
    const double nseg = static_cast<double>(segs.size());
    j["rpe_trans_m"] = segs.empty() ? json(nullptr) : json(tsum / nseg);
    j["rpe_rot_deg"] = segs.empty() ? json(nullptr) : json(rsum / nseg);

    if (args.json) {
        std::cout << j.dump() << '\n';
        return 0;
    }
    std::printf("poses %zu  from %.1f s  coverage %.1f / %.1f s\n",
                m.estimated.size(),
                j["start_s"].get<double>(),
                j["coverage_s"].get<double>(),
                j["gt_span_s"].get<double>());
    std::printf("ATE se3 %.3f m   ATE 4dof %.3f m\n", j["ate_se3"].get<double>(), j["ate_4dof"].get<double>());
    for (const auto& s : segs)
        std::printf("  RPE %4g m: %.3f m  %.2f deg  (n=%zu)\n", s.length_m, s.translation_m, s.rotation_deg, s.pairs);
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parse(argc, argv);
        if (args.help || args.dataset.empty() || args.est.empty()) {
            usage();
            return args.help ? 0 : 2;
        }
        return run(args);
    } catch (const std::invalid_argument& e) {
        std::cerr << "traj_eval: " << e.what() << '\n';
        usage();
        return 2;
    } catch (const std::exception& e) {
        std::cerr << "traj_eval: " << e.what() << '\n';
        return 1;
    }
}
