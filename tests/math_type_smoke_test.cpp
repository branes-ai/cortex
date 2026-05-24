// Arithmetic-type plumbing smoke test (issue #25).
//
// Confirms that MTL5's dense matrix/vector expression machinery
// instantiates and *runs* against the validated arithmetic types — IEEE
// double/float plus Universal posit/cfloat/fixpnt — by evaluating the
// canonical linear-algebra expression `y = A*x + b` for each and
// checking the result. Also statically asserts that every type models
// the branes::math arithmetic concepts (the MTL5 mapping).
//
// This is the "type-genericity" guard: a Universal bump or an MTL5 bump
// that breaks scalar plumbing for any validated type fails here first.

#include <branes/math/arithmetic.hpp>

#include <catch2/catch_test_macros.hpp>
#include <mtl/mtl.hpp>
#include <universal/number/cfloat/cfloat.hpp>
#include <universal/number/fixpnt/fixpnt.hpp>
#include <universal/number/posit/posit.hpp>

#include <cstddef>
#include <cstdint>

namespace {

// IEEE-754 single-precision-equivalent cfloat.
using cfloat32 = sw::universal::cfloat<32, 8, std::uint32_t, true, false, false>;
// Q16.16 saturating fixed-point.
using fixedQ16 = sw::universal::fixpnt<32, 16, sw::universal::Saturate>;
using posit32 = sw::universal::posit<32, 2>;
using posit16 = sw::universal::posit<16, 1>;

// Concept conformance — the "mapping" of Universal types onto MTL5's
// operation-based scalar concepts. No specialization needed; this just
// records that the duck-typing holds for each validated type.
static_assert(branes::math::LinearAlgebraScalar<double>);
static_assert(branes::math::LinearAlgebraScalar<float>);
static_assert(branes::math::LinearAlgebraScalar<posit32>);
static_assert(branes::math::LinearAlgebraScalar<posit16>);
static_assert(branes::math::LinearAlgebraScalar<cfloat32>);
static_assert(branes::math::LinearAlgebraScalar<fixedQ16>);

static_assert(branes::math::OrderedField<double>);
static_assert(branes::math::OrderedField<posit32>);
static_assert(branes::math::OrderedField<fixedQ16>);

// Evaluate y = A*x + b for a small fixed system whose exact answer is
// representable in every validated type (all entries are small integers
// or exact binary fractions, so posit/cfloat/fixpnt round-trip cleanly).
//
//   A = [[2, 0, 1],     x = [1, 2, 4]^T   b = [1, -1, 0]^T
//        [0, 1, 0],
//        [1, 1, 1]]
//
//   A*x   = [6, 2, 7]^T
//   y     = [7, 1, 7]^T
template <branes::math::LinearAlgebraScalar T>
void check_axpb() {
    using namespace mtl;
    mat::dense2D<T> A(3, 3);
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            A(i, j) = T{0};
        }
    }
    A(0, 0) = T{2};
    A(0, 2) = T{1};
    A(1, 1) = T{1};
    A(2, 0) = T{1};
    A(2, 1) = T{1};
    A(2, 2) = T{1};

    vec::dense_vector<T> x(3);
    x[0] = T{1};
    x[1] = T{2};
    x[2] = T{4};

    vec::dense_vector<T> b(3);
    b[0] = T{1};
    b[1] = T{-1};
    b[2] = T{0};

    vec::dense_vector<T> y(3);
    y = A * x + b;  // MTL5 expression template, evaluated on assignment

    REQUIRE(double(y[0]) == 7.0);
    REQUIRE(double(y[1]) == 1.0);
    REQUIRE(double(y[2]) == 7.0);
}

}  // namespace

TEST_CASE("y = A*x + b instantiates for IEEE types", "[math][type-smoke]") {
    SECTION("double") {
        check_axpb<double>();
    }
    SECTION("float") {
        check_axpb<float>();
    }
}

TEST_CASE("y = A*x + b instantiates for Universal posit", "[math][type-smoke]") {
    SECTION("posit<32,2>") {
        check_axpb<posit32>();
    }
    SECTION("posit<16,1>") {
        check_axpb<posit16>();
    }
}

TEST_CASE("y = A*x + b instantiates for Universal cfloat", "[math][type-smoke]") {
    check_axpb<cfloat32>();
}

TEST_CASE("y = A*x + b instantiates for Universal fixed-point", "[math][type-smoke]") {
    check_axpb<fixedQ16>();
}
