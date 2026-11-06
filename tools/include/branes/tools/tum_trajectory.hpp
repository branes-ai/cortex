// SPDX-License-Identifier: MIT
//
// branes/tools/tum_trajectory.hpp — read and write a trajectory in the TUM
// text format: one pose per line, "t x y z qx qy qz qw" (seconds; the IMU
// pose in the world frame; Hamilton quaternion, scalar last).
//
// vio_pipeline writes its EuRoC trajectory this way (trajectory.tum), and
// traj_eval reads it, so any estimator that writes the same format is scored
// by the same code (docs/benchmarks/euroc-baseline.mdx). Lines that are
// empty or start with '#' are skipped.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_TUM_TRAJECTORY_HPP
#define BRANES_TOOLS_TUM_TRAJECTORY_HPP

#include <branes/math/lie/se3.hpp>
#include <branes/sdk/eval/trajectory_metrics.hpp>

#include <algorithm>
#include <cmath>
#include <istream>
#include <limits>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace branes::tools {

/// Write `traj` as TUM lines at full double precision.
template <math::Scalar T>
void write_tum(std::ostream& os, const std::vector<sdk::eval::StampedPose<T>>& traj) {
    const auto old = os.precision(std::numeric_limits<double>::max_digits10);
    for (const auto& sp : traj) {
        const auto& p = sp.pose.translation();
        const auto& q = sp.pose.rotation().quaternion();  // (w, x, y, z)
        os << sp.t_s << ' ' << static_cast<double>(p[0]) << ' ' << static_cast<double>(p[1]) << ' '
           << static_cast<double>(p[2]) << ' ' << static_cast<double>(q[1]) << ' ' << static_cast<double>(q[2]) << ' '
           << static_cast<double>(q[3]) << ' ' << static_cast<double>(q[0]) << '\n';
    }
    os.precision(old);
}

/// Read TUM lines, sorted by time. Throws std::runtime_error on a line that
/// does not hold eight finite numbers or has a zero-norm quaternion.
template <math::Scalar T>
[[nodiscard]] std::vector<sdk::eval::StampedPose<T>> read_tum(std::istream& is) {
    std::vector<sdk::eval::StampedPose<T>> out;
    std::string line;
    std::size_t lineno = 0;
    while (std::getline(is, line)) {
        ++lineno;
        const auto first = line.find_first_not_of(" \t\r");
        if (first == std::string::npos || line[first] == '#')
            continue;
        std::istringstream ls(line);
        double v[8];
        for (double& x : v)
            if (!(ls >> x) || !std::isfinite(x))
                throw std::runtime_error("read_tum: line " + std::to_string(lineno) + " is not 8 finite numbers");
        const double qn = v[4] * v[4] + v[5] * v[5] + v[6] * v[6] + v[7] * v[7];
        if (!(qn > 0.0))
            throw std::runtime_error("read_tum: zero-norm quaternion on line " + std::to_string(lineno));
        const typename math::lie::SO3<T>::Quaternion q{
            {static_cast<T>(v[7]), static_cast<T>(v[4]), static_cast<T>(v[5]), static_cast<T>(v[6])}};
        const math::lie::detail::Vec<T, 3> p{{static_cast<T>(v[1]), static_cast<T>(v[2]), static_cast<T>(v[3])}};
        sdk::eval::StampedPose<T> sp;
        sp.t_s = v[0];
        sp.pose = math::lie::SE3<T>(math::lie::SO3<T>(q), p);
        out.push_back(sp);
    }
    std::stable_sort(out.begin(), out.end(), [](const auto& a, const auto& b) { return a.t_s < b.t_s; });
    return out;
}

}  // namespace branes::tools

#endif  // BRANES_TOOLS_TUM_TRAJECTORY_HPP
