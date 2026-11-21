// R-IEKF S6 stage bench: invariant camera update (issue #612, epic #444).
//
// Locks three invariants of the R-IEKF msckf_update stage:
//   (1) H matches finite differences of the world-frame retraction;
//   (2) the marginalized H annihilates the 4-DoF gauge (global yaw + 3 translations);
//   (3) P⁺ ⪯ P⁻: the update does not inflate the covariance.

#include <branes/tools/bench/s6_riekf_update_bench.hpp>

#include <catch2/catch_test_macros.hpp>

namespace {
namespace bn = branes::tools::bench;
using B = bn::S6RieкfUpdateBench;
}  // namespace

TEST_CASE("S6 R-IEKF: every built-in fixture passes all invariants", "[tools][bench][s6][riekf]") {
    for (const auto& nf : B::builtin_fixtures()) {
        const auto in = B::decode_input<double>(nf.fixture.input);
        const auto out = B::run<double>(in, bn::kShipped);
        const auto rs = B::invariants<double>(in, out, nf.fixture);
        for (const auto& res : rs) {
            INFO(nf.name << " / " << res.name << " = " << res.value);
            REQUIRE(res.pass);
        }
    }
}

TEST_CASE("S6 R-IEKF: H annihilates the 4-DoF gauge at the synthetic scene",
          "[tools][bench][s6][riekf][observability]") {
    const auto sc = B::make_scene();
    B::Input<double> in;
    in.clones = sc.clones;
    in.p_f = sc.feats[0];
    in.obs = B::track(sc.clones, sc.feats[0]);
    in.sigma = 0.01;
    const auto out = B::run<double>(in, bn::kShipped);
    REQUIRE(out.applied);
    const auto rs = B::invariants<double>(in, out, {});
    const auto it =
        std::find_if(rs.begin(), rs.end(), [](const auto& r) { return r.name == "jacobian.annihilates_gauge"; });
    REQUIRE(it != rs.end());
    INFO("gauge leak = " << it->value);
    REQUIRE(it->pass);
}

TEST_CASE("S6 R-IEKF: degenerate track (1 obs) is rejected -- clones unchanged", "[tools][bench][s6][riekf][reject]") {
    using B = branes::tools::bench::S6RieкfUpdateBench;
    B::Input<double> in;
    // 3 identity clones
    for (int c = 0; c < 3; ++c) {
        branes::sdk::msckf::InvariantClone<double> cl;
        cl.p = branes::math::lie::detail::Vec<double, 3>{{0.5 * c, 0.0, 0.0}};
        in.clones.push_back(cl);
    }
    in.p_f = branes::math::lie::detail::Vec<double, 3>{{0.0, 0.0, 3.0}};
    in.sigma = 0.01;
    // only 1 observation — invariant_update returns false immediately
    in.obs = {{0, 0, branes::math::lie::detail::Vec<double, 2>{{0.0, 0.0}}}};

    const auto clones_before = in.clones;
    const auto out = B::run<double>(in, branes::tools::bench::kShipped);

    REQUIRE_FALSE(out.applied);
    REQUIRE(out.clones_after.size() == clones_before.size());
    for (std::size_t i = 0; i < clones_before.size(); ++i) {
        const auto qb = clones_before[i].R.quaternion();
        const auto qa = out.clones_after[i].R.quaternion();
        for (int k = 0; k < 4; ++k)
            REQUIRE(qb[k] == qa[k]);
        for (int k = 0; k < 3; ++k)
            REQUIRE(clones_before[i].p[k] == out.clones_after[i].p[k]);
    }
}
