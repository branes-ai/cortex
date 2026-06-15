// SPDX-License-Identifier: MIT
//
// Round-trip gate for the inter-stage VIO trace bus (branes/tools/vio_trace.hpp,
// issue #372). The schema is the keystone every per-stage inspector loads from,
// so what it writes must read back identically. These cases pin: the shared
// serialization vocabulary (observation, SE3, IMU, NavState, covariance block),
// the envelope (header + input/output), the writer/reader over a JSONL stream
// (including the self-describing banner), and the S4 worked template.
//
// Precision: the raw scalar/vector paths are asserted BIT-EXACT — nlohmann/json
// serializes a double with the shortest representation that re-parses to the
// same double, so `parse(dump(x)) == x` exactly. The literals below deliberately
// carry full double precision (not 0.1-style one-digit values) so the round-trip
// genuinely exercises the mantissa rather than passing trivially. The ONE place
// a tolerance is needed is the quaternion: `SO3(Quaternion)` re-normalizes on
// construction (so3.hpp), so unpack→SO3 perturbs the low bits — that tolerance
// is about the group constructor, not the trace.

#include <branes/tools/vio_trace.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <sstream>

namespace tr = branes::tools::trace;
using branes::math::lie::SE3;
using branes::math::lie::SO3;
using Catch::Matchers::WithinAbs;

namespace {

// Full-precision constants, so an exact round-trip means the whole mantissa
// survived — not just a digit or two.
constexpr double kRt = 1.4142135623730951;     // √2
constexpr double kE = 2.7182818284590452;      // e
constexpr double kPi = 3.1415926535897932;     // π
constexpr double kPhi = 1.6180339887498949;    // golden ratio
constexpr double kGamma = 0.5772156649015329;  // Euler–Mascheroni
constexpr double kLn2 = 0.6931471805599453;    // ln 2

// A representative non-identity pose: a small rotation + a translation.
SE3<double> sample_pose() {
    SO3<double>::Tangent phi{0.12345678901234567, -0.23456789012345678, 0.34567890123456789};
    SE3<double>::Vector3 t{kRt, -kE, kPi};
    return SE3<double>(SO3<double>::exp(phi), t);
}

// Raw scalar/vector serialization is lossless → compare bit-exact.
void require_vec3_exact(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    for (std::size_t i = 0; i < 3; ++i)
        REQUIRE_THAT(a[i], WithinAbs(b[i], 0.0));
}

// The quaternion passes through SO3's normalizing constructor on unpack, so a
// few ulps of drift is expected and acceptable — this is NOT a serialization
// tolerance.
void require_so3_close(const SO3<double>& a, const SO3<double>& b) {
    for (std::size_t i = 0; i < 4; ++i)
        REQUIRE_THAT(a.quaternion()[i], WithinAbs(b.quaternion()[i], 1e-15));
}

}  // namespace

TEST_CASE("shared vocabulary round-trips through JSON", "[tools][vio_trace]") {
    SECTION("FrontendObservation") {
        branes::sdk::FrontendObservation<double> o{42, 1, 320.0 + kRt, 240.0 - kE};
        auto back = tr::unpack_observation<double>(tr::pack(o));
        REQUIRE(back.feature_id == o.feature_id);
        REQUIRE(back.camera_id == o.camera_id);
        REQUIRE_THAT(back.u, WithinAbs(o.u, 0.0));  // bit-exact
        REQUIRE_THAT(back.v, WithinAbs(o.v, 0.0));
    }

    SECTION("SE3 pose") {
        auto X = sample_pose();
        auto back = tr::unpack_se3<double>(tr::pack(X));
        require_so3_close(X.rotation(), back.rotation());
        for (std::size_t i = 0; i < 3; ++i)
            REQUIRE_THAT(X.translation()[i], WithinAbs(back.translation()[i], 0.0));  // bit-exact
    }

    SECTION("ImuMeasurement") {
        branes::sdk::ImuMeasurement<double> m{kPi, {kGamma, -kLn2, kPhi}, {0.0, 0.0, 9.806649999999999}};
        auto back = tr::unpack_imu<double>(tr::pack(m));
        REQUIRE_THAT(back.timestamp_s, WithinAbs(m.timestamp_s, 0.0));
        require_vec3_exact(back.angular_velocity, m.angular_velocity);
        require_vec3_exact(back.linear_acceleration, m.linear_acceleration);
    }

    SECTION("NavState") {
        branes::sdk::NavState<double> s{kE, sample_pose(), {kGamma, -kPhi, kLn2}};
        auto back = tr::unpack_navstate<double>(tr::pack(s));
        REQUIRE_THAT(back.timestamp_s, WithinAbs(s.timestamp_s, 0.0));
        require_so3_close(s.T_world_imu.rotation(), back.T_world_imu.rotation());
        require_vec3_exact(back.velocity_world, s.velocity_world);
    }

    SECTION("covariance block") {
        // A tiny dense matrix view exposing operator()(i,j).
        struct Mat {
            std::array<std::array<double, 3>, 3> e;
            double operator()(std::size_t i, std::size_t j) const {
                return e[i][j];
            }
        } P{{{{kRt, 0.1 * kPi, 0.2 * kE}, {0.1 * kPi, kPhi, 0.3 * kLn2}, {0.2 * kE, 0.3 * kLn2, kPi}}}};
        auto j = tr::pack_covariance(P, 3);
        REQUIRE(j.size() == 3);
        for (std::size_t r = 0; r < 3; ++r)
            for (std::size_t c = 0; c < 3; ++c)
                REQUIRE_THAT(j.at(r).at(c).get<double>(), WithinAbs(P(r, c), 0.0));  // bit-exact
    }
}

TEST_CASE("S4 input/output round-trips through the envelope", "[tools][vio_trace]") {
    tr::S4Input<double> in;
    in.image_path = "euroc/MH_05/cam0/12.png";
    in.width = 752;
    in.height = 480;
    in.camera_id = 0;
    in.prev_tracks = {{10, 0, 320.0 + kRt, 240.0 - kE}, {11, 0, 410.0 + kPi, 180.0 + kLn2}};
    in.gyro_prior = sample_pose().rotation();

    tr::S4Output<double> out;
    out.observations = {{10, 0, 322.0 + kGamma, 241.0 + kPhi}};

    tr::TraceRecord rec = tr::make_record(tr::TraceHeader{12, 1.45, "S4"}, in, out);

    auto in2 = tr::get_input<tr::S4Input<double>>(rec);
    auto out2 = tr::get_output<tr::S4Output<double>>(rec);

    REQUIRE(in2.image_path == in.image_path);
    REQUIRE(in2.width == in.width);
    REQUIRE(in2.height == in.height);
    REQUIRE(in2.camera_id == in.camera_id);
    REQUIRE(in2.prev_tracks.size() == in.prev_tracks.size());
    REQUIRE_THAT(in2.prev_tracks[0].u, WithinAbs(in.prev_tracks[0].u, 0.0));  // bit-exact
    REQUIRE(in2.gyro_prior.has_value());
    require_so3_close(*in.gyro_prior, *in2.gyro_prior);
    REQUIRE(out2.observations.size() == out.observations.size());
    REQUIRE(out2.observations[0].feature_id == 10);
    REQUIRE_THAT(out2.observations[0].v, WithinAbs(out.observations[0].v, 0.0));

    SECTION("absent gyro prior serializes as null and reads back empty") {
        in.gyro_prior.reset();
        tr::TraceRecord r2 = tr::make_record(tr::TraceHeader{13, 1.50, "S4"}, in, out);
        REQUIRE(r2.input.at("gyro_prior").is_null());
        REQUIRE_FALSE(tr::get_input<tr::S4Input<double>>(r2).gyro_prior.has_value());
    }
}

TEST_CASE("writer/reader round-trips a JSONL stream", "[tools][vio_trace]") {
    std::stringstream ss;
    {
        tr::TraceWriter w(ss);
        w.write_banner("S4", "Visual frontend — per-frame I/O");
        for (std::uint64_t f = 0; f < 3; ++f) {
            tr::S4Input<double> in;
            in.image_path = "frame_" + std::to_string(f) + ".png";
            in.width = 752;
            in.height = 480;
            in.prev_tracks = {{f, 0, 100.0 + static_cast<double>(f), 200.0}};
            tr::S4Output<double> out;
            out.observations = {{f, 0, 101.0 + static_cast<double>(f), 201.0}};
            w.write(tr::make_record(tr::TraceHeader{f, 1.0 + 0.1 * static_cast<double>(f), "S4"}, in, out));
        }
    }

    // The banner is the first line and is self-describing.
    std::string first;
    std::getline(std::stringstream(ss.str()), first);
    auto banner = tr::json::parse(first);
    REQUIRE(banner.at("kind") == "meta");
    REQUIRE(banner.at("trace_schema") == std::string(tr::kSchemaVersion));

    // The reader skips the banner and yields exactly the records.
    tr::TraceReader r(ss);
    auto records = r.read_all();
    REQUIRE(records.size() == 3);
    for (std::uint64_t f = 0; f < 3; ++f) {
        REQUIRE(records[f].header.frame == f);
        REQUIRE(records[f].header.stage == "S4");
        REQUIRE_THAT(records[f].header.t_s, WithinAbs(1.0 + 0.1 * static_cast<double>(f), 0.0));
        auto out = tr::get_output<tr::S4Output<double>>(records[f]);
        REQUIRE(out.observations.size() == 1);
        REQUIRE(out.observations[0].feature_id == f);
    }
}
