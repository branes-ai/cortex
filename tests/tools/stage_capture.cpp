// Stage-boundary stream capture (issue #446).
//
// A synthetic MSCKF run with a StageRecorder tapped onto the backend: every
// captured fixture replays bit-identically through its stage bench, the
// recomputed S6 chain matches the run, the triggers select what they name,
// and a tap that captures nothing — or everything — leaves the run unchanged.

#include <branes/sdk/eval/synthetic_world.hpp>
#include <branes/sdk/msckf_backend.hpp>
#include <branes/tools/bench/stage_recorder.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <map>
#include <random>
#include <string>
#include <vector>

namespace {

namespace bn = branes::tools::bench;
namespace sdk = branes::sdk;
using Backend = sdk::MsckfBackend<double>;
using Recorder = bn::StageRecorder<double>;

const sdk::eval::SyntheticData<double>& world() {
    static const auto w = sdk::eval::generate_world<double>(sdk::eval::SyntheticConfig<double>{});
    return w;
}

/// A backend with the synthetic world's camera.
Backend make_backend() {
    const auto& w = world();
    Backend::CameraCalibration cal;
    cal.intrinsics = w.camera;
    cal.extrinsics.R_imu_cam = w.R_imu_cam;
    cal.extrinsics.p_imu_cam = w.p_imu_cam;
    Backend be(std::vector<Backend::CameraCalibration>{cal});
    be.initialize(sdk::VioConfig{});
    return be;
}

/// Run the synthetic world through the backend for `frames` camera frames,
/// with deterministic pixel noise (so updates carry a real residual).
void run(Backend& be, std::size_t frames) {
    const auto& w = world();
    std::size_t k = 0;
    std::uint32_t draw = 0;
    for (std::size_t f = 0; f < std::min(frames, w.frames.size()); ++f) {
        for (; k < w.imu.size() && w.imu[k].timestamp_s <= w.frames[f].t; ++k)
            be.process_imu(w.imu[k]);
        auto obs = w.frames[f].obs;
        for (auto& o : obs) {
            o.u += 0.5 * bn::s6::pseudo_normal(draw++);
            o.v += 0.5 * bn::s6::pseudo_normal(draw++);
        }
        be.process_camera(w.frames[f].t, std::span<const sdk::FrontendObservation<double>>(obs));
    }
}

// The filter initializes within the first 8 frames; 32 frames reach post-init
// frame 21, past the first full-window marginalizations (S9 from frame 11).
constexpr std::size_t kFrames = 32;

/// The synthetic world's camera, for S0.
std::vector<Recorder::Intrinsics> intrinsics() {
    return {Recorder::intrinsics_of(world().camera, 752.0, 480.0)};
}

/// Run one captured fixture through its bench in double.
bn::FixtureRun replay(const bn::NamedFixture& nf) {
    const std::string& s = nf.fixture.stage;
    auto go = [&]<class B>() { return bn::run_fixture<B>(nf, bn::kShipped, bn::TypeList<double>{}); };
    if (s == "S0_sensor_model")
        return go.operator()<bn::S0SensorModelBench>();
    if (s == "S2_propagation")
        return go.operator()<bn::S2PropagationBench>();
    if (s == "S3_augmentation")
        return go.operator()<bn::S3AugmentationBench>();
    if (s == "S5_triangulation")
        return go.operator()<bn::S5TriangulationBench>();
    if (s == "S6a_jacobians")
        return go.operator()<bn::S6aJacobiansBench>();
    if (s == "S6b_nullspace_projection")
        return go.operator()<bn::S6bNullspaceProjectionBench>();
    if (s == "S6c_compression")
        return go.operator()<bn::S6cCompressionBench>();
    if (s == "S6d_gating")
        return go.operator()<bn::S6dGatingBench>();
    if (s == "S6e_ekf_update")
        return go.operator()<bn::S6eEkfUpdateBench>();
    if (s == "S9_marginalization")
        return go.operator()<bn::S9MarginalizationBench>();
    FAIL("no bench for stage " << s);
    return {};
}

double value_of(const bn::FixtureRun& run, std::string_view name) {
    for (const auto& r : run.types.front().report.results())
        if (r.name == name)
            return r.value;
    FAIL("no result named " << name);
    return 0.0;
}

double nis_per_dof(const bn::NamedFixture& nf) {
    const auto& e = nf.fixture.expected;
    return bn::unpack_num(e.at("nis")) / e.at("dof").get<double>();
}

}  // namespace

TEST_CASE("captured fixtures replay bit-identically through their stage benches", "[tools][capture]") {
    bn::CaptureOptions opt;
    opt.first_frame = opt.last_frame = 13;  // the window is full: S9 runs, and S6 on the oldest clone's tracks
    opt.source = "synthetic_world (default config), 0.5 px noise";
    Recorder rec(opt, intrinsics());
    auto be = make_backend();
    be.set_stage_tap(&rec);
    run(be, kFrames);
    rec.finish();
    REQUIRE(be.initialized());

    std::map<std::string, int> per_stage;
    for (const auto& nf : rec.fixtures()) {
        INFO(nf.name);
        ++per_stage[nf.fixture.stage];
        REQUIRE(nf.fixture.kind == bn::FixtureKind::Captured);
        REQUIRE(nf.fixture.arithmetic == "double");
        const auto r = replay(nf);
        REQUIRE(value_of(r, "replay.bit_identical") == 0.0);
    }
    for (const auto& s : bn::capturable_stages()) {
        INFO(s);
        REQUIRE(per_stage[s] > 0);
    }
    REQUIRE(rec.stats().chain_mismatches == 0);
    REQUIRE(rec.stats().fixtures == rec.fixtures().size());
}

TEST_CASE("captured fixtures written to disk load and replay", "[tools][capture]") {
    // Unique per run: gcc and clang ctest (or CI jobs) may share the temp directory.
    const auto dir = std::filesystem::temp_directory_path() /
                     ("branes_stage_capture_test_" + std::to_string(std::random_device{}()) + "_" +
                      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::remove_all(dir);
    bn::CaptureOptions opt;
    opt.stages = {"S3_augmentation", "S6e_ekf_update", "S9_marginalization"};
    opt.first_frame = opt.last_frame = 13;
    Recorder rec(opt, {}, Recorder::write_to(dir));
    auto be = make_backend();
    be.set_stage_tap(&rec);
    run(be, kFrames);
    rec.finish();
    REQUIRE(rec.fixtures().empty());  // all went to the sink
    std::size_t n = 0;
    for (const auto& e : std::filesystem::recursive_directory_iterator(dir)) {
        if (e.path().extension() != ".json")
            continue;
        const bn::NamedFixture nf{e.path().stem().string(), bn::load(e.path())};
        INFO(nf.name);
        REQUIRE(e.path().parent_path().filename() == nf.fixture.stage);
        REQUIRE(value_of(replay(nf), "replay.bit_identical") == 0.0);
        ++n;
    }
    REQUIRE(n == rec.stats().fixtures);
    REQUIRE(n >= 3);
    std::filesystem::remove_all(dir);
}

TEST_CASE("a tap leaves the run unchanged whether it captures nothing or everything", "[tools][capture]") {
    auto plain = make_backend(), idle = make_backend(), full = make_backend();
    run(plain, kFrames);

    sdk::msckf::StageTap<double, sdk::msckf::FullCovariance<double>> nothing;
    idle.set_stage_tap(&nothing);
    run(idle, kFrames);

    bn::CaptureOptions every;  // every stage, over frames that include S9 and S6 updates
    every.first_frame = 12;
    every.last_frame = 14;
    Recorder rec(every, intrinsics());
    full.set_stage_tap(&rec);
    run(full, kFrames);
    REQUIRE(rec.stats().fixtures > 0);

    REQUIRE(bn::pack(idle.state()) == bn::pack(plain.state()));
    REQUIRE(bn::pack(full.state()) == bn::pack(plain.state()));
    REQUIRE(full.nis_consistency().samples() == plain.nis_consistency().samples());
    REQUIRE(full.nis_consistency().report().normalized == plain.nis_consistency().report().normalized);
}

TEST_CASE("the NIS trigger captures exactly the S6 updates above the threshold", "[tools][capture]") {
    // Every S6d decision in the run, then the median NIS/dof as the trigger.
    bn::CaptureOptions all;
    all.stages = {"S6d_gating"};
    all.first_frame = 13;
    all.last_frame = 21;
    Recorder every(all);
    auto be = make_backend();
    be.set_stage_tap(&every);
    run(be, kFrames);
    std::vector<double> nis;
    for (const auto& nf : every.fixtures())
        nis.push_back(nis_per_dof(nf));
    REQUIRE(nis.size() >= 4);
    auto sorted = nis;
    std::sort(sorted.begin(), sorted.end());
    const double thr = sorted[sorted.size() / 2];
    const auto above =
        static_cast<std::size_t>(std::count_if(nis.begin(), nis.end(), [&](double x) { return x > thr; }));

    bn::CaptureOptions opt;
    opt.nis_per_dof_above = thr;
    opt.first_frame = all.first_frame;
    opt.last_frame = all.last_frame;
    Recorder rec(opt, intrinsics());
    auto be2 = make_backend();
    be2.set_stage_tap(&rec);
    run(be2, kFrames);
    rec.finish();
    std::size_t gates = 0;
    for (const auto& nf : rec.fixtures()) {
        INFO(nf.name);
        const auto& s = nf.fixture.stage;
        REQUIRE((s == "S5_triangulation" || s.rfind("S6", 0) == 0));  // only S6 boundaries fire
        if (s == "S6d_gating") {
            REQUIRE(nis_per_dof(nf) > thr);
            ++gates;
        }
    }
    REQUIRE(gates == above);
    REQUIRE(rec.stats().boundaries == above);
}

TEST_CASE("the first-violation trigger captures only the first failing boundary", "[tools][capture]") {
    bn::CaptureOptions opt;
    opt.first_violation = true;
    Recorder rec(opt, intrinsics());
    auto be = make_backend();
    be.set_stage_tap(&rec);
    run(be, kFrames);
    rec.finish();
    REQUIRE(rec.stats().first_violation_frame.has_value());
    REQUIRE(rec.stats().boundaries == 1);
    std::vector<std::string> captured;
    bool fails = false;
    for (const auto& nf : rec.fixtures()) {
        captured.push_back(nf.name);
        fails = fails || !replay(nf).pass();
    }
    REQUIRE(fails);

    // Every fixture of every stage up to that frame, in run order: the first
    // one that fails its bench belongs to the captured boundary.
    bn::CaptureOptions upto;
    upto.last_frame = *rec.stats().first_violation_frame;
    Recorder all(upto, intrinsics());
    auto be2 = make_backend();
    be2.set_stage_tap(&all);
    run(be2, kFrames);
    all.finish();
    std::string first_failing;
    for (const auto& nf : all.fixtures())
        if (!replay(nf).pass()) {
            first_failing = nf.name;
            break;
        }
    INFO(first_failing);
    REQUIRE(std::find(captured.begin(), captured.end(), first_failing) != captured.end());
}

TEST_CASE("S0 capture starts at the first initialized frame", "[tools][capture]") {
    // While initializing, the backend normalizes pixels for its dynamic-init
    // buffer before any on_frame; those must not be filed under frame 0.
    bn::CaptureOptions opt;
    opt.stages = {"S0_sensor_model"};
    Recorder rec(opt, intrinsics());
    REQUIRE(rec.wants(sdk::msckf::TapStage::S0_sensor_model));
    const auto cam = world().camera;
    auto s0 = [&](double u, double v) { rec.on_s0(0, u, v, sdk::msckf::stages::s0_sensor_model::apply(cam, u, v)); };
    s0(100.0, 100.0);  // pre-init
    s0(200.0, 150.0);  // pre-init
    rec.on_frame(1.0);
    s0(300.0, 250.0);  // frame 0
    rec.finish();
    REQUIRE(rec.fixtures().size() == 1);
    REQUIRE(rec.fixtures().front().fixture.input.at("pixels").size() == 1);
    REQUIRE(value_of(replay(rec.fixtures().front()), "replay.bit_identical") == 0.0);
}

TEST_CASE("capture options are validated", "[tools][capture]") {
    bn::CaptureOptions bad_stage;
    bad_stage.stages = {"S6_update"};
    REQUIRE_THROWS_AS(Recorder(bad_stage), std::invalid_argument);
    bn::CaptureOptions bad_range;
    bad_range.first_frame = 5;
    bad_range.last_frame = 4;
    REQUIRE_THROWS_AS(Recorder(bad_range), std::invalid_argument);
    bn::CaptureOptions bad_nis;
    bad_nis.nis_per_dof_above = -1.0;
    REQUIRE_THROWS_AS(Recorder(bad_nis), std::invalid_argument);
}
