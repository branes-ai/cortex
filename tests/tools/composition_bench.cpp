// Composition bench and live-assertion mode (issue #447).
//
// The filter loop as a tape of stage operations: its built-in fixtures pass,
// an injected fault is caught at the stage that introduced it (in the
// composition bench and in live mode on a running filter), and a loop captured
// from a run replays bit-identically.

#include <branes/sdk/eval/synthetic_world.hpp>
#include <branes/sdk/msckf_backend.hpp>
#include <branes/tools/bench/c_filter_loop_bench.hpp>
#include <branes/tools/bench/stage_recorder.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace {

namespace bn = branes::tools::bench;
namespace sdk = branes::sdk;
using Loop = bn::CFilterLoopBench;
using Backend = sdk::MsckfBackend<double>;
using St = sdk::msckf::State<double>;

bn::FixtureRun run(const bn::NamedFixture& nf, std::string_view variant, const std::vector<std::string>& types) {
    return bn::run_fixture<Loop>(nf, variant, bn::bench_types<Loop>(), types);
}

double value_of(const bn::TypeRun& t, std::string_view name) {
    for (const auto& r : t.report.results())
        if (r.name == name)
            return r.value;
    FAIL("no result named " << name);
    return 0.0;
}

/// Index of the first operation of `kind` in a fixture's tape.
double first_op(const bn::Fixture& f, Loop::OpKind kind) {
    const auto in = Loop::decode_input<double>(f.input);
    for (std::size_t k = 0; k < in.ops.size(); ++k)
        if (in.ops[k].kind == kind)
            return static_cast<double>(k);
    FAIL("no operation of that kind");
    return -1.0;
}

const sdk::eval::SyntheticData<double>& world() {
    static const auto w = sdk::eval::generate_world<double>(sdk::eval::SyntheticConfig<double>{});
    return w;
}

Backend make_backend() {
    Backend::CameraCalibration cal;
    cal.intrinsics = world().camera;
    cal.extrinsics.R_imu_cam = world().R_imu_cam;
    cal.extrinsics.p_imu_cam = world().p_imu_cam;
    Backend be(std::vector<Backend::CameraCalibration>{cal});
    be.initialize(sdk::VioConfig{});
    return be;
}

void run_world(Backend& be, std::size_t frames) {
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

/// A defective S3 in a running filter: after the clone, its cross-covariance
/// with the IMU pose is perturbed (the backend's live state, as a buggy stage
/// would leave it), then the boundary is reported on.
class FaultyS3 final : public sdk::msckf::StageTap<double, sdk::msckf::FullCovariance<double>> {
public:
    FaultyS3(StageTap& inner, std::uint64_t frame) : inner_(inner), frame_at_(frame) {}
    void on_frame(double t) override {
        frame_ = next_++;
        inner_.on_frame(t);
    }
    [[nodiscard]] bool wants(sdk::msckf::TapStage s) const override {
        return s == sdk::msckf::TapStage::S3_augmentation || inner_.wants(s);
    }
    void
    on_s0(std::uint32_t c, double u, double v, const sdk::msckf::stages::s0_sensor_model::Result<double>& r) override {
        inner_.on_s0(c, u, v, r);
    }
    void on_s2(const St& b,
               const sdk::msckf::Propagator<double>& p,
               const Vec3& g,
               const Vec3& a,
               double dt,
               const St& af) override {
        inner_.on_s2(b, p, g, a, dt, af);
    }
    void
    on_s3(const St& b, double t, const sdk::msckf::stages::s3_augmentation::Diagnostics& d, const St& af) override {
        if (frame_ == frame_at_) {
            auto& live = const_cast<St&>(af);  // the injected defect
            const std::size_t c = live.clone_offset(live.clones.size() - 1);
            for (std::size_t i = 0; i < 6; ++i)
                for (std::size_t j = 0; j < 6; ++j) {
                    live.cov.P(c + i, j) += 1e-3;
                    live.cov.P(j, c + i) += 1e-3;
                }
        }
        if (inner_.wants(sdk::msckf::TapStage::S3_augmentation))
            inner_.on_s3(b, t, d, af);
    }
    void on_s6(const St& b,
               const sdk::msckf::CameraUpdater<double>& u,
               std::uint64_t id,
               const sdk::msckf::FeatureTrack<double>& tr,
               const sdk::msckf::stages::s6_msckf_update::Diagnostics<double>& d,
               const St& af) override {
        inner_.on_s6(b, u, id, tr, d, af);
    }
    void on_s9(const St& b,
               std::size_t i,
               const sdk::msckf::stages::s9_marginalization::Diagnostics& d,
               const St& af) override {
        inner_.on_s9(b, i, d, af);
    }

private:
    StageTap& inner_;
    std::uint64_t frame_at_;
    std::uint64_t next_ = 0, frame_ = 0;
};

}  // namespace

TEST_CASE("composition bench: every built-in fixture passes the shipped loop", "[tools][bench][composition]") {
    for (const auto& nf : Loop::builtin_fixtures()) {
        const auto r = run(nf, bn::kShipped, {"double", "float"});
        for (const auto& t : r.types) {
            const auto bad = t.report.first_failure();
            INFO(nf.name << " / " << t.type << " / " << (bad ? std::string(bad->name) : std::string("-")));
            REQUIRE(t.report.all_pass());
            REQUIRE(value_of(t, "loop.first_violating_op") == -1.0);
        }
    }
}

TEST_CASE("composition bench: the known answer holds exactly and P stays 0", "[tools][bench][composition]") {
    const auto r = run({"ka", Loop::known_answer()}, bn::kShipped, {"double"});
    const auto& t = r.types.front();
    REQUIRE(value_of(t, "known_answer.residual") < 1e-12);
    REQUIRE(value_of(t, "loop.nis_per_dof") < 1e-20);
}

TEST_CASE("composition bench: an injected fault is caught at the stage that introduced it",
          "[tools][bench][composition]") {
    for (const auto& nf : {bn::NamedFixture{"ka", Loop::known_answer()}, bn::NamedFixture{"cap", Loop::captured()}}) {
        INFO(nf.name);
        const auto s3 = run(nf, "fault_s3_clone_jacobian", {"double"}).types.front();
        REQUIRE_FALSE(s3.report.all_pass());
        REQUIRE(s3.report.first_failure()->stage == branes::sdk::eval::inv::Stage::S3_augmentation);
        REQUIRE(value_of(s3, "loop.first_violating_op") == first_op(nf.fixture, Loop::OpKind::S3));

        const auto s6a = run(nf, "fault_s6a_jacobian", {"double"}).types.front();
        REQUIRE_FALSE(s6a.report.all_pass());
        REQUIRE(s6a.report.first_failure()->stage == branes::sdk::eval::inv::Stage::S6a_jacobians);
        REQUIRE(value_of(s6a, "loop.first_violating_op") == first_op(nf.fixture, Loop::OpKind::S6));
    }
}

TEST_CASE("composition bench: a loop captured from a run replays bit-identically", "[tools][bench][composition]") {
    bn::LoopTape<double> tape(16, 17);
    auto be = make_backend();
    be.set_stage_tap(&tape);
    run_world(be, 30);
    REQUIRE(tape.started());
    const auto f = tape.fixture("synthetic_world, test");
    REQUIRE(f.kind == bn::FixtureKind::Captured);
    REQUIRE(tape.frames_seen() > 17);  // the run went past the tape's last frame
    const auto r = run({"tape", f}, bn::kShipped, {"double"});
    REQUIRE(value_of(r.types.front(), "replay.bit_identical") == 0.0);
}

TEST_CASE("composition bench: the committed fixture files load and pass", "[tools][bench][composition]") {
    const std::filesystem::path dir = std::filesystem::path(BRANES_BENCH_FIXTURE_DIR) / "c_filter_loop";
    std::size_t n = 0;
    for (const auto& e : std::filesystem::directory_iterator(dir)) {
        const bn::NamedFixture nf{e.path().stem().string(), bn::load(e.path())};
        INFO(nf.name);
        REQUIRE(run(nf, bn::kShipped, {"double"}).pass());
        ++n;
    }
    REQUIRE(n == 3);
}

TEST_CASE("live mode: a fault injected into a running filter is reported at its stage and frame",
          "[tools][capture][composition]") {
    bn::CaptureOptions opt;
    opt.first_violation = true;
    bn::StageRecorder<double> live(opt, {}, [](const bn::NamedFixture&) {});
    // Post-init frame 0: before any S6 update (so before #487's first rejection).
    FaultyS3 faulty(live, 0);
    auto be = make_backend();
    be.set_stage_tap(&faulty);
    run_world(be, 30);
    const auto& v = live.stats().first_violation;
    REQUIRE(v.has_value());
    REQUIRE(v->stage == "S3_augmentation");
    REQUIRE(v->frame == 0);
    INFO(v->invariant << " = " << v->value << " vs " << v->threshold);
    REQUIRE_FALSE(v->invariant.empty());
}

TEST_CASE("a fan-out tap feeds every tap the same boundaries", "[tools][capture][composition]") {
    bn::LoopTape<double> alone(16, 16), shared(16, 16);
    bn::CaptureOptions opt;
    opt.stages = {"S9_marginalization"};
    bn::StageRecorder<double> rec(opt);
    {
        auto be = make_backend();
        be.set_stage_tap(&alone);
        run_world(be, 30);
    }
    bn::StageTapFanout<double> fan;
    fan.add(&shared);
    fan.add(&rec);
    auto be = make_backend();
    be.set_stage_tap(&fan);
    run_world(be, 30);
    REQUIRE(Loop::encode_input(shared.input()) == Loop::encode_input(alone.input()));
    REQUIRE(Loop::encode_output(shared.output()) == Loop::encode_output(alone.output()));
    REQUIRE_FALSE(rec.fixtures().empty());
}
