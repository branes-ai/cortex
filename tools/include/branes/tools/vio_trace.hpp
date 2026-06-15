// SPDX-License-Identifier: MIT
//
// branes/tools/vio_trace.hpp — the inter-stage VIO trace bus (epic #371's
// keystone, issue #372).
//
// A serializable, human-readable envelope for the INPUT and OUTPUT of every
// VIO pipeline stage boundary S0–S10 (docs/arch/vio-pipeline-canonical.md).
// The running pipeline dumps one record per stage per frame; each per-stage
// inspector replays from the trace, re-runs the *real* stage operator on the
// recorded input, and renders. This is the concrete, writable/readable form of
// the prose pre/post-conditions registered in
// tools/include/branes/tools/vio_stage_contracts.hpp.
//
// ── The schema ──────────────────────────────────────────────────────────────
// One JSONL file = one stream of records, one record per line. Every record
// carries the SHARED HEADER (`frame`, `t_s`, `stage`) flattened at the top
// level, plus a per-stage `input` / `output` payload object:
//
//   {"frame":12,"t_s":1.45,"stage":"S4","input":{…},"output":{…}}
//
// The optional FIRST line is a self-describing `meta` banner — schema version,
// the stage, and a one-line description — so a learner can read a trace without
// reading code (the educational goal). Readers skip it transparently.
//
// Design notes:
//   • Human-readable & self-describing trumps compactness — field names are
//     spelled out (`feature_id`, not `id`-by-position), pixels/poses are plain
//     numbers, and the image is a *reference* (path + dims) the inspector
//     reloads, not an inlined pixel blob.
//   • Scalars serialize through `double` (JSON's only number) — the trace is a
//     real-data study artifact (float/double pipelines), not a bit-exact
//     checkpoint. Round-trip is exact for values representable in double.
//   • This header lives in the tools layer and depends on branes::sdk +
//     nlohmann/json only; it never reaches into middleware or hardware.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_VIO_TRACE_HPP
#define BRANES_TOOLS_VIO_TRACE_HPP

#include <branes/math/arithmetic.hpp>  // branes::math::Scalar
#include <branes/math/lie/se3.hpp>     // SE3, SO3, detail::Vec
#include <branes/sdk/vio_backend.hpp>  // FrontendObservation, ImuMeasurement, NavState

#include <nlohmann/json.hpp>

#include <array>
#include <cstdint>
#include <istream>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <vector>

namespace branes::tools::trace {

using json = nlohmann::json;
namespace lie = branes::math::lie;
namespace sdk = branes::sdk;

/// Schema identifier carried in the `meta` banner. Bump the version when a
/// record's wire shape changes incompatibly so old inspectors fail loudly.
inline constexpr std::string_view kSchemaVersion = "branes.vio.trace/1";

// ── Shared serialization vocabulary ─────────────────────────────────────────
// The handful of geometric/measurement types that recur at stage boundaries.
// `pack(...)` overloads write; `unpack_*<T>(...)` read. Inspectors and the
// per-stage payloads below build on these so every stage speaks one dialect.

/// JSON's only number is double; route every scalar through it.
template <branes::math::Scalar T>
[[nodiscard]] inline double num(const T& x) {
    return static_cast<double>(x);
}

/// 3-vector (point / velocity / bias) → `[x,y,z]`.
template <branes::math::Scalar T>
[[nodiscard]] inline json pack(const std::array<T, 3>& v) {
    return json::array({num(v[0]), num(v[1]), num(v[2])});
}

/// Fixed-size Lie vector (`detail::Vec<T,N>`) → `[…]`.
template <branes::math::Scalar T, std::size_t N>
[[nodiscard]] inline json pack(const lie::detail::Vec<T, N>& v) {
    json a = json::array();
    for (std::size_t i = 0; i < N; ++i)
        a.push_back(num(v[i]));
    return a;
}

/// Rotation → `{"q":[w,x,y,z]}` (the storage convention of SO3).
template <branes::math::Scalar T>
[[nodiscard]] inline json pack(const lie::SO3<T>& R) {
    return json{{"q", pack(R.quaternion())}};
}

/// Rigid pose → `{"q":[w,x,y,z],"t":[x,y,z]}`.
template <branes::math::Scalar T>
[[nodiscard]] inline json pack(const lie::SE3<T>& X) {
    return json{{"q", pack(X.rotation().quaternion())}, {"t", pack(X.translation())}};
}

/// One front-end observation → `{"feature_id","camera_id","u","v"}`.
template <branes::math::Scalar T>
[[nodiscard]] inline json pack(const sdk::FrontendObservation<T>& o) {
    return json{{"feature_id", o.feature_id}, {"camera_id", o.camera_id}, {"u", num(o.u)}, {"v", num(o.v)}};
}

/// One inertial sample → `{"t_s","gyro":[…],"accel":[…]}`.
template <branes::math::Scalar T>
[[nodiscard]] inline json pack(const sdk::ImuMeasurement<T>& m) {
    return json{{"t_s", m.timestamp_s}, {"gyro", pack(m.angular_velocity)}, {"accel", pack(m.linear_acceleration)}};
}

/// Navigation state → `{"t_s","pose":{q,t},"velocity":[…]}`.
template <branes::math::Scalar T>
[[nodiscard]] inline json pack(const sdk::NavState<T>& s) {
    return json{{"t_s", s.timestamp_s}, {"pose", pack(s.T_world_imu)}, {"velocity", pack(s.velocity_world)}};
}

/// A square covariance block → row-major array-of-arrays. `P(i,j)` is read for
/// `0≤i,j<n`; works for any dense matrix view exposing `operator()`.
template <class Matrix>
[[nodiscard]] inline json pack_covariance(const Matrix& P, std::size_t n) {
    json rows = json::array();
    for (std::size_t i = 0; i < n; ++i) {
        json row = json::array();
        for (std::size_t j = 0; j < n; ++j)
            row.push_back(static_cast<double>(P(i, j)));
        rows.push_back(std::move(row));
    }
    return rows;
}

// ── Readers (typed; the inverse of the pack overloads) ──────────────────────

template <branes::math::Scalar T = double>
[[nodiscard]] inline std::array<T, 3> unpack_vec3(const json& j) {
    return {static_cast<T>(j.at(0).get<double>()), static_cast<T>(j.at(1).get<double>()),
            static_cast<T>(j.at(2).get<double>())};
}

template <branes::math::Scalar T = double, std::size_t N>
[[nodiscard]] inline lie::detail::Vec<T, N> unpack_vecn(const json& j) {
    lie::detail::Vec<T, N> v{};
    for (std::size_t i = 0; i < N; ++i)
        v[i] = static_cast<T>(j.at(i).get<double>());
    return v;
}

template <branes::math::Scalar T = double>
[[nodiscard]] inline lie::SO3<T> unpack_so3(const json& j) {
    return lie::SO3<T>(unpack_vecn<T, 4>(j.at("q")));
}

template <branes::math::Scalar T = double>
[[nodiscard]] inline lie::SE3<T> unpack_se3(const json& j) {
    return lie::SE3<T>(unpack_so3<T>(j), unpack_vecn<T, 3>(j.at("t")));
}

template <branes::math::Scalar T = double>
[[nodiscard]] inline sdk::FrontendObservation<T> unpack_observation(const json& j) {
    sdk::FrontendObservation<T> o;
    o.feature_id = j.at("feature_id").get<std::uint64_t>();
    o.camera_id = j.at("camera_id").get<std::uint32_t>();
    o.u = static_cast<T>(j.at("u").get<double>());
    o.v = static_cast<T>(j.at("v").get<double>());
    return o;
}

template <branes::math::Scalar T = double>
[[nodiscard]] inline sdk::ImuMeasurement<T> unpack_imu(const json& j) {
    sdk::ImuMeasurement<T> m;
    m.timestamp_s = j.at("t_s").get<double>();
    m.angular_velocity = unpack_vec3<T>(j.at("gyro"));
    m.linear_acceleration = unpack_vec3<T>(j.at("accel"));
    return m;
}

template <branes::math::Scalar T = double>
[[nodiscard]] inline sdk::NavState<T> unpack_navstate(const json& j) {
    sdk::NavState<T> s;
    s.timestamp_s = j.at("t_s").get<double>();
    s.T_world_imu = unpack_se3<T>(j.at("pose"));
    s.velocity_world = unpack_vec3<T>(j.at("velocity"));
    return s;
}

// ── The envelope ────────────────────────────────────────────────────────────

/// The header every record shares: where in the run this boundary fired.
struct TraceHeader {
    std::uint64_t frame = 0;  ///< monotonic frame index (the run's clock tick)
    double t_s = 0.0;         ///< sensor timestamp (seconds)
    std::string stage;        ///< "S0"…"S10" — which boundary this record is
};

/// One stage-boundary record: the shared header plus the stage's input and
/// output payloads as JSON objects. The payloads are intentionally untyped at
/// this level — each stage's typed `SNInput`/`SNOutput` (below) pack into and
/// unpack out of them — so the bus stays one envelope across all stages.
struct TraceRecord {
    TraceHeader header;
    json input;
    json output;
};

inline void to_json(json& j, const TraceHeader& h) {
    j = json{{"frame", h.frame}, {"t_s", h.t_s}, {"stage", h.stage}};
}
inline void from_json(const json& j, TraceHeader& h) {
    j.at("frame").get_to(h.frame);
    j.at("t_s").get_to(h.t_s);
    j.at("stage").get_to(h.stage);
}

inline void to_json(json& j, const TraceRecord& r) {
    j = json{{"frame", r.header.frame}, {"t_s", r.header.t_s}, {"stage", r.header.stage}};
    j["input"] = r.input;
    j["output"] = r.output;
}
inline void from_json(const json& j, TraceRecord& r) {
    from_json(j, r.header);
    r.input = j.value("input", json::object());
    r.output = j.value("output", json::object());
}

// ── Writer / reader ─────────────────────────────────────────────────────────

/// Appends records to a JSONL stream, one compact line each. Optionally leads
/// with a self-describing `meta` banner.
class TraceWriter {
public:
    explicit TraceWriter(std::ostream& os) : os_(os) {}

    /// Write the self-describing banner as the first line. `stage` and
    /// `description` are free text for the human reading the file (e.g. the
    /// stage's contract title). Call before the first `write`.
    void write_banner(std::string_view stage = {}, std::string_view description = {}) {
        json b{{"kind", "meta"}, {"trace_schema", kSchemaVersion}};
        if (!stage.empty())
            b["stage"] = std::string(stage);
        if (!description.empty())
            b["description"] = std::string(description);
        os_ << b.dump() << '\n';
    }

    /// Append one record.
    void write(const TraceRecord& r) {
        json j = r;
        os_ << j.dump() << '\n';
    }

private:
    std::ostream& os_;
};

/// Streams records out of a JSONL trace. The `meta` banner and blank lines are
/// skipped transparently, so a reader sees only stage records.
class TraceReader {
public:
    explicit TraceReader(std::istream& is) : is_(is) {}

    /// Read the next stage record. Returns false at end of stream.
    [[nodiscard]] bool next(TraceRecord& out) {
        std::string line;
        while (std::getline(is_, line)) {
            if (line.empty())
                continue;
            json j = json::parse(line);
            if (j.contains("kind") && j["kind"] == "meta")  // skip the banner
                continue;
            out = j.get<TraceRecord>();
            return true;
        }
        return false;
    }

    /// Drain the whole stream into a vector.
    [[nodiscard]] std::vector<TraceRecord> read_all() {
        std::vector<TraceRecord> v;
        TraceRecord r;
        while (next(r))
            v.push_back(std::move(r));
        return v;
    }

private:
    std::istream& is_;
};

// ── S4 — Visual frontend (the worked template) ──────────────────────────────
// signature: track(image_t, prev_tracks) → observations[(id,cam,u,v)]
// (vio_stage_contracts.hpp kS4). Every other stage's SNInput/SNOutput follows
// this shape — typed struct + a templated to_json/from_json — as #374–#384
// decouple their operators.

/// S4 input: the image to track into (by reference — the inspector reloads the
/// pixels), the tracks carried in from the previous frame, and the optional
/// gyro-derived rotation prior.
template <branes::math::Scalar T = double>
struct S4Input {
    std::string image_path;                            ///< path the inspector reloads
    std::uint32_t width = 0;                           ///< image dims, for sanity/overlay
    std::uint32_t height = 0;                           //
    std::uint32_t camera_id = 0;                        ///< which camera produced it
    std::vector<sdk::FrontendObservation<T>> prev_tracks;  ///< tracks from the previous frame
    std::optional<lie::SO3<T>> gyro_prior;             ///< optional rotation prior (S2 → frontend)
};

/// S4 output: the surviving observations after tracking + RANSAC — the boundary
/// handed downstream to triangulation (S5) and the MSCKF update (S6).
template <branes::math::Scalar T = double>
struct S4Output {
    std::vector<sdk::FrontendObservation<T>> observations;
};

template <class T>
inline void to_json(json& j, const S4Input<T>& in) {
    j = json{{"image", json{{"path", in.image_path}, {"width", in.width}, {"height", in.height},
                            {"camera_id", in.camera_id}}}};
    json tracks = json::array();
    for (const auto& o : in.prev_tracks)
        tracks.push_back(pack(o));
    j["prev_tracks"] = std::move(tracks);
    j["gyro_prior"] = in.gyro_prior ? pack(*in.gyro_prior) : json(nullptr);
}

template <class T>
inline void from_json(const json& j, S4Input<T>& in) {
    const json& img = j.at("image");
    img.at("path").get_to(in.image_path);
    img.at("width").get_to(in.width);
    img.at("height").get_to(in.height);
    img.at("camera_id").get_to(in.camera_id);
    in.prev_tracks.clear();
    for (const auto& o : j.at("prev_tracks"))
        in.prev_tracks.push_back(unpack_observation<T>(o));
    in.gyro_prior.reset();
    if (auto it = j.find("gyro_prior"); it != j.end() && !it->is_null())
        in.gyro_prior = unpack_so3<T>(*it);
}

template <class T>
inline void to_json(json& j, const S4Output<T>& out) {
    json obs = json::array();
    for (const auto& o : out.observations)
        obs.push_back(pack(o));
    j = json{{"observations", std::move(obs)}};
}

template <class T>
inline void from_json(const json& j, S4Output<T>& out) {
    out.observations.clear();
    for (const auto& o : j.at("observations"))
        out.observations.push_back(unpack_observation<T>(o));
}

/// Build a record from a typed stage input/output pair. The header's `stage`
/// must be set by the caller (the typed payload doesn't carry it).
template <class In, class Out>
[[nodiscard]] inline TraceRecord make_record(const TraceHeader& header, const In& in, const Out& out) {
    TraceRecord r;
    r.header = header;
    r.input = in;
    r.output = out;
    return r;
}

/// Decode a record's input/output back into a typed payload.
template <class In>
[[nodiscard]] inline In get_input(const TraceRecord& r) {
    return r.input.get<In>();
}
template <class Out>
[[nodiscard]] inline Out get_output(const TraceRecord& r) {
    return r.output.get<Out>();
}

}  // namespace branes::tools::trace

#endif  // BRANES_TOOLS_VIO_TRACE_HPP
