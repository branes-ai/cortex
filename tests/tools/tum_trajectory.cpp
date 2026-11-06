// TUM trajectory I/O (docs/benchmarks/euroc-baseline.mdx): vio_pipeline writes
// it and traj_eval reads it, so the round trip must be exact.

#include <branes/tools/tum_trajectory.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace {

namespace ev = branes::sdk::eval;
using T = double;
using SE3 = branes::math::lie::SE3<T>;
using SO3 = branes::math::lie::SO3<T>;
using Vec3 = branes::math::lie::detail::Vec<T, 3>;

}  // namespace

TEST_CASE("TUM round trip is exact at full precision", "[tools][tum]") {
    std::vector<ev::StampedPose<T>> traj;
    for (int k = 0; k < 20; ++k) {
        const double s = 0.1 * k;
        traj.push_back(ev::StampedPose<T>{
            1403636579.763555527 + 0.05 * k,
            SE3(SO3::exp(Vec3{{0.3 * s, -0.2 * s, 1.1 * s}}), Vec3{{1.0 / 3.0 + s, -s * s, 1e-7 * k}})});
    }
    std::stringstream ss;
    branes::tools::write_tum<T>(ss, traj);
    const auto back = branes::tools::read_tum<T>(ss);
    REQUIRE(back.size() == traj.size());
    for (std::size_t i = 0; i < traj.size(); ++i) {
        REQUIRE(back[i].t_s == traj[i].t_s);
        for (std::size_t a = 0; a < 3; ++a)
            REQUIRE(back[i].pose.translation()[a] == traj[i].pose.translation()[a]);
        // SO3 renormalizes on construction: equal to the last bit or two.
        const auto d = (back[i].pose.rotation().inverse() * traj[i].pose.rotation()).log();
        for (std::size_t a = 0; a < 3; ++a)
            REQUIRE(std::abs(d[a]) < 1e-15);
    }
}

TEST_CASE("TUM reader skips comments and blanks, sorts by time", "[tools][tum]") {
    std::istringstream is("# t x y z qx qy qz qw\n"
                          "\n"
                          "2.0 1 2 3 0 0 0 1\n"
                          "  # indented comment\n"
                          "1.0 4 5 6 0 0 0 2\n");  // unnormalized: still the identity
    const auto t = branes::tools::read_tum<T>(is);
    REQUIRE(t.size() == 2);
    REQUIRE(t[0].t_s == 1.0);
    REQUIRE(t[0].pose.translation()[0] == 4.0);
    REQUIRE(t[1].t_s == 2.0);
    REQUIRE(t[0].pose.rotation().log()[2] == 0.0);
}

TEST_CASE("TUM reader rejects malformed lines", "[tools][tum]") {
    std::istringstream short_line("1.0 1 2 3 0 0 0\n");
    REQUIRE_THROWS_AS(branes::tools::read_tum<T>(short_line), std::runtime_error);
    std::istringstream nan_line("1.0 nan 2 3 0 0 0 1\n");
    REQUIRE_THROWS_AS(branes::tools::read_tum<T>(nan_line), std::runtime_error);
    std::istringstream zero_q("1.0 1 2 3 0 0 0 0\n");
    REQUIRE_THROWS_AS(branes::tools::read_tum<T>(zero_q), std::runtime_error);
}
