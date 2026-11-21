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
    const auto rs = B::invariants<double>(in, out, {});
    for (const auto& res : rs) {
        if (res.name == "jacobian.annihilates_gauge") {
            INFO("gauge leak = " << res.value);
            REQUIRE(res.pass);
        }
    }
}
