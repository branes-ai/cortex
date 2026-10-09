// SPDX-License-Identifier: MIT
//
// branes/tools/bench/fixture.hpp — the stage-bench fixture model (issue #453,
// epic #444 §A). A fixture is one stage's input, plus — depending on its kind —
// the output it must produce or the truth it is compared with:
//
//   known_answer  synthetic input with an analytic expected output
//   ground_truth  input built with ground truth injected upstream (#266); a
//                 `truth` payload the output is compared with
//   captured      a stage-boundary record from a real run (#446): the input and
//                 the output the stage produced, replayed bit-for-bit
//
// ── The file format ─────────────────────────────────────────────────────────
// One JSON document per fixture, schema `branes.vio.fixture/1`:
//
//   {"schema":"branes.vio.fixture/1", "stage":"S9_marginalization",
//    "kind":"captured", "arithmetic":"double", "source":"…", "seed":0,
//    "description":"…", "input":{…}, "expected":{…}, "truth":{…}}
//
// `input` / `expected` / `truth` are stage-defined payloads built from the
// vocabulary below (numbers, vectors, matrices, rotations, the MSCKF state).
//
// Exactness: every number is written through `double`, which holds every value
// of float, posit<32,2> and double exactly, and nlohmann/json prints doubles in
// shortest round-trip form — so a payload reads back bit-identical. Non-finite
// values (JSON has none) are written as the strings "nan", "inf", "-inf".
// Rotations are stored as their unit quaternion and restored WITHOUT
// renormalizing (SO3::from_unit_quaternion), which would not be bit-idempotent.
//
// Header-only, C++20.

#ifndef BRANES_TOOLS_BENCH_FIXTURE_HPP
#define BRANES_TOOLS_BENCH_FIXTURE_HPP

#include <branes/math/lie/so3.hpp>
#include <branes/sdk/msckf/state.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace branes::tools::bench {

using json = nlohmann::json;

inline constexpr std::string_view kFixtureSchema = "branes.vio.fixture/1";

enum class FixtureKind { KnownAnswer, GroundTruth, Captured };

[[nodiscard]] inline std::string_view to_string(FixtureKind k) noexcept {
    switch (k) {
    case FixtureKind::KnownAnswer:
        return "known_answer";
    case FixtureKind::GroundTruth:
        return "ground_truth";
    case FixtureKind::Captured:
        return "captured";
    }
    return "?";
}

[[nodiscard]] inline FixtureKind fixture_kind(std::string_view s) {
    if (s == "known_answer")
        return FixtureKind::KnownAnswer;
    if (s == "ground_truth")
        return FixtureKind::GroundTruth;
    if (s == "captured")
        return FixtureKind::Captured;
    throw std::invalid_argument("fixture: unknown kind '" + std::string(s) + "'");
}

/// One stage fixture. `expected` (known_answer, captured) and `truth`
/// (ground_truth) are null when the kind does not carry them.
struct Fixture {
    std::string stage;  ///< S<N>_<transformation>, e.g. "S9_marginalization"
    FixtureKind kind = FixtureKind::KnownAnswer;
    std::string arithmetic = "double";  ///< type the payload was produced in (captured: the run's type)
    std::string source;                 ///< generator, dataset + frame, or capture site
    std::uint64_t seed = 0;
    std::string description;
    /// The variant the `expected` output was produced by or is valid for; ""
    /// means every variant must reproduce it (an implementation-independent
    /// known answer). A captured fixture records the variant that ran.
    std::string variant;
    json input;
    json expected;
    json truth;
};

// ── Numbers ─────────────────────────────────────────────────────────────────

[[nodiscard]] inline json pack_num(double x) {
    if (std::isnan(x))
        return "nan";
    if (std::isinf(x))
        return x > 0 ? "inf" : "-inf";
    return x;
}

[[nodiscard]] inline double unpack_num(const json& j) {
    if (j.is_string()) {
        const auto& s = j.get_ref<const std::string&>();
        if (s == "nan")
            return std::numeric_limits<double>::quiet_NaN();
        if (s == "inf")
            return std::numeric_limits<double>::infinity();
        if (s == "-inf")
            return -std::numeric_limits<double>::infinity();
        throw std::invalid_argument("fixture: bad number '" + s + "'");
    }
    return j.get<double>();
}

template <math::Scalar T>
[[nodiscard]] json pack(const T& x) {
    return pack_num(static_cast<double>(x));
}

template <math::Scalar T>
[[nodiscard]] T unpack_scalar(const json& j) {
    return T(unpack_num(j));
}

// ── Vectors and matrices ────────────────────────────────────────────────────

template <math::Scalar T>
[[nodiscard]] json pack_vec(std::span<const T> v) {
    json a = json::array();
    for (const T& x : v)
        a.push_back(pack(x));
    return a;
}

template <math::Scalar T>
[[nodiscard]] std::vector<T> unpack_vec(const json& j) {
    std::vector<T> v;
    v.reserve(j.size());
    for (const auto& x : j)
        v.push_back(unpack_scalar<T>(x));
    return v;
}

template <math::Scalar T, std::size_t N>
[[nodiscard]] json pack(const math::lie::detail::Vec<T, N>& v) {
    return pack_vec<T>(std::span<const T>(v.e));
}

template <math::Scalar T, std::size_t N>
[[nodiscard]] math::lie::detail::Vec<T, N> unpack_fixed(const json& j) {
    if (j.size() != N)
        throw std::invalid_argument("fixture: fixed vector has the wrong length");
    math::lie::detail::Vec<T, N> v;
    for (std::size_t i = 0; i < N; ++i)
        v[i] = unpack_scalar<T>(j[i]);
    return v;
}

/// Row-major matrix: {"rows":r, "cols":c, "data":[…]}.
template <math::Scalar T>
[[nodiscard]] json pack(const sdk::msckf::DynMat<T>& m) {
    return json{{"rows", m.rows}, {"cols", m.cols}, {"data", pack_vec<T>(std::span<const T>(m.d))}};
}

template <math::Scalar T>
[[nodiscard]] sdk::msckf::DynMat<T> unpack_mat(const json& j) {
    sdk::msckf::DynMat<T> m(j.at("rows").get<std::size_t>(), j.at("cols").get<std::size_t>());
    const auto& d = j.at("data");
    if (d.size() != m.d.size())
        throw std::invalid_argument("fixture: matrix data does not match rows*cols");
    for (std::size_t i = 0; i < m.d.size(); ++i)
        m.d[i] = unpack_scalar<T>(d[i]);
    return m;
}

// ── Rotations ───────────────────────────────────────────────────────────────

/// SO(3) as its unit quaternion [w, x, y, z].
template <math::Scalar T>
[[nodiscard]] json pack(const math::lie::SO3<T>& r) {
    return pack(r.quaternion());
}

/// Restore a rotation exactly. The stored quaternion must be canonical (w ≥ 0)
/// and unit-norm to the precision of T; anything else is a malformed fixture
/// (rejected here, never handed to SO3::from_unit_quaternion's precondition).
template <math::Scalar T>
[[nodiscard]] math::lie::SO3<T> unpack_so3(const json& j) {
    const auto q = unpack_fixed<T, 4>(j);
    double n2 = 0.0;
    for (std::size_t i = 0; i < 4; ++i) {
        const double x = static_cast<double>(q[i]);
        if (!std::isfinite(x))
            throw std::invalid_argument("fixture: rotation quaternion is not finite");
        n2 += x * x;
    }
    // Fixture numbers pass through double, so the norm is only unit to double
    // precision even when T is wider (long double): floor ε at double's.
    const double eps =
        std::max(static_cast<double>(std::numeric_limits<T>::epsilon()), std::numeric_limits<double>::epsilon());
    if (std::abs(std::sqrt(n2) - 1.0) > 64.0 * eps)
        throw std::invalid_argument("fixture: rotation quaternion is not unit-norm");
    if (static_cast<double>(q[0]) < 0.0)
        throw std::invalid_argument("fixture: rotation quaternion is not canonical (w < 0)");
    return math::lie::SO3<T>::from_unit_quaternion(q);
}

// ── The MSCKF state (full covariance) ───────────────────────────────────────

/// Navigation state, calibration blocks, clone window and covariance P.
template <math::Scalar T>
[[nodiscard]] json pack(const sdk::msckf::State<T>& s) {
    json j;
    j["R"] = pack(s.R);
    j["p"] = pack(s.p);
    j["v"] = pack(s.v);
    j["bg"] = pack(s.bg);
    j["ba"] = pack(s.ba);
    j["timestamp"] = pack_num(s.timestamp);
    j["calib"] = json::array();
    for (const auto& c : s.calib)
        j["calib"].push_back({{"R_imu_cam", pack(c.R_imu_cam)}, {"p_imu_cam", pack(c.p_imu_cam)}});
    j["clones"] = json::array();
    for (const auto& c : s.clones)
        j["clones"].push_back({{"R", pack(c.R)}, {"p", pack(c.p)}, {"timestamp", pack_num(c.timestamp)}});
    j["P"] = pack(s.cov.P);
    return j;
}

template <math::Scalar T>
[[nodiscard]] sdk::msckf::State<T> unpack_state(const json& j) {
    sdk::msckf::State<T> s(T{1});
    s.R = unpack_so3<T>(j.at("R"));
    s.p = unpack_fixed<T, 3>(j.at("p"));
    s.v = unpack_fixed<T, 3>(j.at("v"));
    s.bg = unpack_fixed<T, 3>(j.at("bg"));
    s.ba = unpack_fixed<T, 3>(j.at("ba"));
    s.timestamp = unpack_num(j.at("timestamp"));
    for (const auto& c : j.at("calib"))
        s.calib.push_back({unpack_so3<T>(c.at("R_imu_cam")), unpack_fixed<T, 3>(c.at("p_imu_cam"))});
    for (const auto& c : j.at("clones"))
        s.clones.push_back({unpack_so3<T>(c.at("R")), unpack_fixed<T, 3>(c.at("p")), unpack_num(c.at("timestamp"))});
    s.cov.P = unpack_mat<T>(j.at("P"));
    if (s.cov.P.rows != s.dim() || s.cov.P.cols != s.dim())
        throw std::invalid_argument("fixture: covariance does not match the state dimension");
    return s;
}

// ── Fixture files ───────────────────────────────────────────────────────────

[[nodiscard]] inline json to_json(const Fixture& f) {
    json j;
    j["schema"] = kFixtureSchema;
    j["stage"] = f.stage;
    j["kind"] = to_string(f.kind);
    j["arithmetic"] = f.arithmetic;
    j["source"] = f.source;
    j["seed"] = f.seed;
    j["description"] = f.description;
    if (!f.variant.empty())
        j["variant"] = f.variant;
    j["input"] = f.input;
    if (!f.expected.is_null())
        j["expected"] = f.expected;
    if (!f.truth.is_null())
        j["truth"] = f.truth;
    return j;
}

[[nodiscard]] inline Fixture from_json(const json& j) {
    if (j.value("schema", "") != kFixtureSchema)
        throw std::invalid_argument("fixture: schema is not " + std::string(kFixtureSchema));
    Fixture f;
    f.stage = j.at("stage").get<std::string>();
    f.kind = fixture_kind(j.at("kind").get<std::string>());
    f.arithmetic = j.value("arithmetic", "double");
    f.source = j.value("source", "");
    f.seed = j.value("seed", std::uint64_t{0});
    f.description = j.value("description", "");
    f.variant = j.value("variant", "");
    f.input = j.at("input");
    f.expected = j.value("expected", json());
    f.truth = j.value("truth", json());
    if (f.kind != FixtureKind::GroundTruth && f.expected.is_null())
        throw std::invalid_argument("fixture: a " + std::string(to_string(f.kind)) + " fixture needs 'expected'");
    if (f.kind == FixtureKind::GroundTruth && f.truth.is_null())
        throw std::invalid_argument("fixture: a ground_truth fixture needs 'truth'");
    return f;
}

inline void save(const Fixture& f, const std::filesystem::path& path) {
    if (path.has_parent_path())
        std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path);
    if (!out)
        throw std::runtime_error("fixture: cannot write " + path.string());
    out << to_json(f).dump(1) << '\n';
}

[[nodiscard]] inline Fixture load(const std::filesystem::path& path) {
    std::ifstream in(path);
    if (!in)
        throw std::runtime_error("fixture: cannot read " + path.string());
    return from_json(json::parse(in));
}

/// Build a captured fixture from a stage boundary: the stage's input and the
/// output it produced, recorded in `arithmetic` (a type_name<T>()). This is the writer #446 calls
/// from the running pipeline; benches call it to capture from a synthetic run.
[[nodiscard]] inline Fixture capture(std::string stage,
                                     std::string arithmetic,
                                     std::string source,
                                     json input,
                                     json output,
                                     std::string variant = "shipped") {
    Fixture f;
    f.stage = std::move(stage);
    f.kind = FixtureKind::Captured;
    f.arithmetic = std::move(arithmetic);
    f.source = std::move(source);
    f.description = "captured stage boundary";
    f.variant = std::move(variant);
    f.input = std::move(input);
    f.expected = std::move(output);
    return f;
}

}  // namespace branes::tools::bench

#endif  // BRANES_TOOLS_BENCH_FIXTURE_HPP
