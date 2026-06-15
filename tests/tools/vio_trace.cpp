// SPDX-License-Identifier: MIT
//
// Round-trip gate for the inter-stage VIO trace bus (branes/tools/vio_trace.hpp,
// issue #372). The schema is the keystone every per-stage inspector loads from,
// so what it writes must read back identically. These cases pin: the shared
// serialization vocabulary (observation, SE3, IMU, NavState, covariance block),
// the envelope (header + input/output), the writer/reader over a JSONL stream
// (including the self-describing banner), and the S4 worked template.

#include <branes/tools/vio_trace.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <sstream>

namespace tr = branes::tools::trace;
using branes::math::lie::SE3;
using branes::math::lie::SO3;
using Catch::Matchers::WithinAbs;

namespace {

// A representative non-identity pose: a small rotation + a translation.
SE3<double> sample_pose() {
    SO3<double>::Tangent phi{0.10, -0.20, 0.30};
    SE3<double>::Vector3 t{1.5, -2.5, 3.5};
    return SE3<double>(SO3<double>::exp(phi), t);
}

void require_vec3_eq(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    for (std::size_t i = 0; i < 3; ++i)
        REQUIRE_THAT(a[i], WithinAbs(b[i], 1e-12));
}

void require_so3_eq(const SO3<double>& a, const SO3<double>& b) {
    for (std::size_t i = 0; i < 4; ++i)
        REQUIRE_THAT(a.quaternion()[i], WithinAbs(b.quaternion()[i], 1e-12));
}

}  // namespace

TEST_CASE("shared vocabulary round-trips through JSON", "[tools][vio_trace]") {
    SECTION("FrontendObservation") {
        branes::sdk::FrontendObservation<double> o{42, 1, 320.5, 240.25};
        auto back = tr::unpack_observation<double>(tr::pack(o));
        REQUIRE(back.feature_id == o.feature_id);
        REQUIRE(back.camera_id == o.camera_id);
        REQUIRE_THAT(back.u, WithinAbs(o.u, 1e-12));
        REQUIRE_THAT(back.v, WithinAbs(o.v, 1e-12));
    }

    SECTION("SE3 pose") {
        auto X = sample_pose();
        auto back = tr::unpack_se3<double>(tr::pack(X));
        require_so3_eq(X.rotation(), back.rotation());
        for (std::size_t i = 0; i < 3; ++i)
            REQUIRE_THAT(X.translation()[i], WithinAbs(back.translation()[i], 1e-12));
    }

    SECTION("ImuMeasurement") {
        branes::sdk::ImuMeasurement<double> m{1.234, {0.01, -0.02, 0.03}, {0.0, 0.0, 9.81}};
        auto back = tr::unpack_imu<double>(tr::pack(m));
        REQUIRE_THAT(back.timestamp_s, WithinAbs(m.timestamp_s, 1e-12));
        require_vec3_eq(back.angular_velocity, m.angular_velocity);
        require_vec3_eq(back.linear_acceleration, m.linear_acceleration);
    }

    SECTION("NavState") {
        branes::sdk::NavState<double> s{2.5, sample_pose(), {0.5, -0.5, 0.25}};
        auto back = tr::unpack_navstate<double>(tr::pack(s));
        REQUIRE_THAT(back.timestamp_s, WithinAbs(s.timestamp_s, 1e-12));
        require_so3_eq(s.T_world_imu.rotation(), back.T_world_imu.rotation());
        require_vec3_eq(back.velocity_world, s.velocity_world);
    }

    SECTION("covariance block") {
        // A tiny dense matrix view exposing operator()(i,j).
        struct Mat {
            std::array<std::array<double, 3>, 3> e;
            double operator()(std::size_t i, std::size_t j) const { return e[i][j]; }
        } P{{{{1.0, 0.1, 0.2}, {0.1, 2.0, 0.3}, {0.2, 0.3, 3.0}}}};
        auto j = tr::pack_covariance(P, 3);
        REQUIRE(j.size() == 3);
        for (std::size_t r = 0; r < 3; ++r)
            for (std::size_t c = 0; c < 3; ++c)
                REQUIRE_THAT(j.at(r).at(c).get<double>(), WithinAbs(P(r, c), 1e-12));
    }
}

TEST_CASE("S4 input/output round-trips through the envelope", "[tools][vio_trace]") {
    tr::S4Input<double> in;
    in.image_path = "euroc/MH_05/cam0/12.png";
    in.width = 752;
    in.height = 480;
    in.camera_id = 0;
    in.prev_tracks = {{10, 0, 320.5, 240.0}, {11, 0, 410.0, 180.25}};
    in.gyro_prior = sample_pose().rotation();

    tr::S4Output<double> out;
    out.observations = {{10, 0, 322.0, 241.5}};

    tr::TraceRecord rec = tr::make_record(tr::TraceHeader{12, 1.45, "S4"}, in, out);

    auto in2 = tr::get_input<tr::S4Input<double>>(rec);
    auto out2 = tr::get_output<tr::S4Output<double>>(rec);

    REQUIRE(in2.image_path == in.image_path);
    REQUIRE(in2.width == in.width);
    REQUIRE(in2.height == in.height);
    REQUIRE(in2.camera_id == in.camera_id);
    REQUIRE(in2.prev_tracks.size() == in.prev_tracks.size());
    REQUIRE(in2.gyro_prior.has_value());
    require_so3_eq(*in.gyro_prior, *in2.gyro_prior);
    REQUIRE(out2.observations.size() == out.observations.size());
    REQUIRE(out2.observations[0].feature_id == 10);

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
        REQUIRE_THAT(records[f].header.t_s, WithinAbs(1.0 + 0.1 * static_cast<double>(f), 1e-12));
        auto out = tr::get_output<tr::S4Output<double>>(records[f]);
        REQUIRE(out.observations.size() == 1);
        REQUIRE(out.observations[0].feature_id == f);
    }
}
