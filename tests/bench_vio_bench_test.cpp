// vio_bench component tests (issue #200).
//
// Exercises the parts of the benchmark that don't need a dataset: the
// RAPL wrap/aggregate delta math, the analytic operator FLOP/byte model,
// the KITTI-style RPE drift metric, and the report JSON serialization.
// The full EuRoC run is dataset-gated and lives in the vio_bench tool.

#include <branes/bench/energy_backend.hpp>
#include <branes/bench/operator_profile.hpp>
#include <branes/bench/rapl.hpp>
#include <branes/bench/report.hpp>
#include <branes/bench/tegrastats.hpp>
#include <branes/sdk/eval/trajectory_metrics.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace bb = branes::bench;
namespace ev = branes::sdk::eval;
using T = double;
using Vec3 = branes::math::lie::detail::Vec<T, 3>;
using SE3 = branes::math::lie::SE3<T>;
using SO3 = branes::math::lie::SO3<T>;

TEST_CASE("RAPL delta handles monotonic increase and counter wrap", "[bench][rapl]") {
    using bb::rapl::ZoneReading;
    // Simple increase.
    REQUIRE(bb::rapl::delta_uj({"pkg", 100, 1000}, {"pkg", 450, 1000}) == 350);
    // Wrap: counter went 900 → 50 with range 1000 ⇒ (1000-900)+50 = 150.
    REQUIRE(bb::rapl::delta_uj({"pkg", 900, 1000}, {"pkg", 50, 1000}) == 150);
    // Wrap with unknown range ⇒ unrecoverable ⇒ 0 (no false huge delta).
    REQUIRE(bb::rapl::delta_uj({"pkg", 900, 0}, {"pkg", 50, 0}) == 0);
}

TEST_CASE("RAPL total_delta sums matching zones and ignores unmatched", "[bench][rapl]") {
    std::vector<bb::rapl::ZoneReading> before = {{"package-0", 1000, 100000}, {"dram", 500, 100000}};
    std::vector<bb::rapl::ZoneReading> after = {
        {"package-0", 4000, 100000}, {"dram", 1500, 100000}, {"psys", 9, 100000}};  // psys only in 'after' → ignored
    REQUIRE(bb::rapl::total_delta_uj(before, after) == (3000 + 1000));
}

TEST_CASE("operator profile scales with the runtime counts", "[bench][profile]") {
    bb::PipelineCounts c;
    c.frames = 100;
    c.width = 752;
    c.height = 480;
    c.pyramid_levels = 3;
    c.avg_tracked_features = 150.0;
    c.imu_samples = 20000;
    c.camera_updates = 100;
    c.avg_state_dim = 81.0;  // 15 + 6*11
    c.avg_update_rows = 300.0;

    const auto prof = bb::build_pipeline_profile(c);
    REQUIRE(prof.size() == 5);
    for (const auto& s : prof) {
        REQUIRE(s.flops > 0.0);
        REQUIRE(s.bytes > 0.0);
    }
    // Doubling the frame count doubles per-frame stages (e.g. FAST).
    bb::PipelineCounts c2 = c;
    c2.frames = 200;
    const double fast1 = bb::fast_profile(c).flops;
    const double fast2 = bb::fast_profile(c2).flops;
    REQUIRE_THAT(fast2, Catch::Matchers::WithinRel(2.0 * fast1, 1e-9));
    // MSCKF propagate grows ~cubically in state dimension.
    bb::PipelineCounts cd = c;
    cd.avg_state_dim = 2.0 * c.avg_state_dim;
    REQUIRE_THAT(bb::msckf_propagate_profile(cd).flops,
                 Catch::Matchers::WithinRel(8.0 * bb::msckf_propagate_profile(c).flops, 1e-9));
}

TEST_CASE("RPE drift is zero for an identical trajectory, positive under error", "[bench][rpe]") {
    std::vector<ev::StampedPose<T>> gt, est;
    for (int k = 0; k < 40; ++k) {
        const T s = static_cast<T>(k);
        const Vec3 p{{0.5 * s, 0.0, 0.0}};  // 0.5 m straight-line segments
        gt.push_back({0.1 * k, SE3(SO3{}, p)});
        est.push_back({0.1 * k, SE3(SO3{}, p)});
    }
    auto d0 = ev::rpe_drift(est, gt, 1);
    REQUIRE_THAT(d0.translation_pct, Catch::Matchers::WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(d0.rotation_deg_per_m, Catch::Matchers::WithinAbs(0.0, 1e-9));

    // Inject a constant per-step translational error → positive % drift.
    for (std::size_t k = 0; k < est.size(); ++k)
        est[k].pose = SE3(SO3{}, gt[k].pose.translation() + Vec3{{0.0, 0.05 * (k % 2 ? 1.0 : -1.0), 0.0}});
    auto d1 = ev::rpe_drift(est, gt, 1);
    REQUIRE(d1.translation_pct > 0.0);
}

TEST_CASE("report JSON serialization emits the expected keys", "[bench][report]") {
    bb::BenchReport r;
    r.sequence = "test/mav0";  // plain path
    r.ate_m = 0.12;
    r.ate_gate_m = 0.5;
    r.real_time_factor = 1.5;
    r.rapl_available = true;
    r.avg_power_w = 18.0;
    r.accuracy_within_gate = true;
    r.fps_per_watt = 1.1;
    r.counts.frames = 100;
    r.profile = {{"fast", 1.0e9, 2.0e6}, {"klt", 3.0e9, 4.0e6}};

    std::ostringstream os;
    bb::to_json(os, r);
    const std::string j = os.str();
    REQUIRE(j.find("\"sequence\": \"test/mav0\"") != std::string::npos);
    REQUIRE(j.find("\"ate_m\": 0.12") != std::string::npos);
    REQUIRE(j.find("\"real_time_factor\": 1.5") != std::string::npos);
    REQUIRE(j.find("\"rapl_available\": true") != std::string::npos);
    REQUIRE(j.find("\"operator_profile\"") != std::string::npos);
    REQUIRE(j.find("\"name\": \"fast\"") != std::string::npos);

    // A path with a quote/backslash must be escaped to stay valid JSON.
    bb::BenchReport r2;
    r2.sequence = R"(a"b\c)";
    std::ostringstream os2;
    bb::to_json(os2, r2);
    const std::string j2 = os2.str();
    REQUIRE(j2.find(R"("sequence": "a\"b\\c")") != std::string::npos);
}

TEST_CASE("tegrastats parser prefers a total rail and handles both formats", "[bench][tegrastats]") {
    namespace tg = branes::bench::tegrastats;
    // Modern Orin format with a total input rail (VDD_IN) — use it directly,
    // not the sum of components.
    REQUIRE_THAT(tg::parse_power_mw("RAM 1/2MB VDD_GPU_SOC 1200mW/1200mW VDD_CPU_CV 400mW/400mW VDD_IN 4000mW/4000mW"),
                 Catch::Matchers::WithinAbs(4000.0, 1e-9));
    // No total rail → sum the VDD_* components.
    REQUIRE_THAT(tg::parse_power_mw("VDD_GPU_SOC 1200mW/1200mW VDD_CPU_CV 400mW/400mW"),
                 Catch::Matchers::WithinAbs(1600.0, 1e-9));
    // Legacy POM format with a total input rail.
    REQUIRE_THAT(tg::parse_power_mw("RAM x POM_5V_IN 4148/4148 POM_5V_GPU 0/0 POM_5V_CPU 415/415"),
                 Catch::Matchers::WithinAbs(4148.0, 1e-9));
    // No power tokens → 0.
    REQUIRE_THAT(tg::parse_power_mw("RAM 1/2MB CPU [1%@1200]"), Catch::Matchers::WithinAbs(0.0, 1e-9));
}

TEST_CASE("power→energy integrator is trapezoidal", "[bench][tegrastats]") {
    namespace tg = branes::bench::tegrastats;
    // Constant 10 W over 2 s ⇒ 20 J.
    REQUIRE_THAT(tg::integrate_energy_j({{0.0, 10.0}, {1.0, 10.0}, {2.0, 10.0}}),
                 Catch::Matchers::WithinAbs(20.0, 1e-9));
    // Ramp 0→10 W over 1 s ⇒ trapezoid area 5 J.
    REQUIRE_THAT(tg::integrate_energy_j({{0.0, 0.0}, {1.0, 10.0}}), Catch::Matchers::WithinAbs(5.0, 1e-9));
    // Fewer than two samples ⇒ 0.
    REQUIRE_THAT(tg::integrate_energy_j({{0.0, 10.0}}), Catch::Matchers::WithinAbs(0.0, 1e-9));
}

TEST_CASE("external counter-file backend reports the µJ delta", "[bench][energy]") {
    namespace fs = std::filesystem;
    const fs::path p = fs::temp_directory_path() / "cortex_energy_counter_uj";
    {
        std::ofstream f(p);
        f << "1000000\n";  // 1.0 J cumulative at start
    }
    bb::CounterFileBackend backend(p.string());
    REQUIRE(backend.available());
    {
        std::ofstream f(p);
        f << "3500000\n";  // 3.5 J cumulative now ⇒ delta 2.5 J
    }
    REQUIRE_THAT(backend.joules(), Catch::Matchers::WithinAbs(2.5, 1e-9));
    REQUIRE(std::string(backend.name()) == "external");
    fs::remove(p);

    // A missing path is unavailable and yields 0, not a crash.
    bb::CounterFileBackend missing((fs::temp_directory_path() / "cortex_no_such_counter").string());
    REQUIRE_FALSE(missing.available());
    REQUIRE_THAT(missing.joules(), Catch::Matchers::WithinAbs(0.0, 1e-9));
}

TEST_CASE("energy_kind_from_string parses known kinds", "[bench][energy]") {
    bb::EnergyKind k{};
    REQUIRE(bb::energy_kind_from_string("rapl", k));
    REQUIRE(k == bb::EnergyKind::Rapl);
    REQUIRE(bb::energy_kind_from_string("tegrastats", k));
    REQUIRE(k == bb::EnergyKind::Tegrastats);
    REQUIRE(bb::energy_kind_from_string("external", k));
    REQUIRE(k == bb::EnergyKind::External);
    REQUIRE_FALSE(bb::energy_kind_from_string("nonsense", k));
}
