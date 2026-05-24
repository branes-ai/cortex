// Gaussian pyramid tests (issue #49).
//
// No OpenCV / external reference binary is in-repo, so correctness is
// pinned by reference-independent invariants of an anti-aliased Gaussian
// pyramid: exact level-0 identity, per-level dimensions for the scale
// factor, constant-image preservation (normalized blur + resample of a
// constant is the same constant), DC (mean) preservation, monotonicity
// of a ramp, and deterministic reproducibility.

#include <branes/cv/image.hpp>
#include <branes/cv/pyramid.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace {

namespace cv = branes::cv;

template <class T>
double mean_of(const cv::Image<const T>& img) {
    double s = 0.0;
    for (std::size_t y = 0; y < img.height(); ++y)
        for (std::size_t x = 0; x < img.width(); ++x)
            s += static_cast<double>(img(y, x));
    return s / static_cast<double>(img.width() * img.height());
}

}  // namespace

TEST_CASE("pyramid levels have the expected octave dimensions", "[cv][pyramid]") {
    cv::OwnedImage<std::uint8_t> base(752, 480, 0);
    cv::Pyramid<std::uint8_t> pyr(base.view(), 4, 2.0);
    REQUIRE(pyr.num_levels() == 4);
    REQUIRE(pyr.scale_factor() == 2.0);
    REQUIRE((pyr.level_width(0) == 752 && pyr.level_height(0) == 480));
    REQUIRE((pyr.level_width(1) == 376 && pyr.level_height(1) == 240));
    REQUIRE((pyr.level_width(2) == 188 && pyr.level_height(2) == 120));
    REQUIRE((pyr.level_width(3) == 94 && pyr.level_height(3) == 60));
}

TEST_CASE("level 0 is a byte-exact copy of the base", "[cv][pyramid]") {
    cv::OwnedImage<std::uint8_t> base(16, 12, 0);
    for (std::size_t y = 0; y < 12; ++y)
        for (std::size_t x = 0; x < 16; ++x)
            base(y, x) = static_cast<std::uint8_t>((x * 5 + y * 11) & 0xFF);
    cv::Pyramid<std::uint8_t> pyr(base.view(), 3);
    auto l0 = pyr.level(0);
    bool equal = true;
    for (std::size_t y = 0; y < 12; ++y)
        for (std::size_t x = 0; x < 16; ++x)
            if (l0(y, x) != base(y, x))
                equal = false;
    REQUIRE(equal);
}

TEST_CASE("a constant image stays constant at every level", "[cv][pyramid]") {
    cv::OwnedImage<std::uint8_t> base(100, 80, 128);
    cv::Pyramid<std::uint8_t> pyr(base.view(), 5, 2.0);
    for (int l = 0; l < pyr.num_levels(); ++l) {
        auto img = pyr.level(l);
        for (std::size_t y = 0; y < img.height(); ++y)
            for (std::size_t x = 0; x < img.width(); ++x)
                REQUIRE(img(y, x) == 128);
    }
}

TEST_CASE("configurable non-integer scale factor", "[cv][pyramid]") {
    cv::OwnedImage<std::uint8_t> base(752, 480, 50);
    cv::Pyramid<std::uint8_t> pyr(base.view(), 3, 1.5);
    REQUIRE(pyr.scale_factor() == 1.5);
    REQUIRE(pyr.level_width(1) == 501);   // round(752 / 1.5)
    REQUIRE(pyr.level_height(1) == 320);  // round(480 / 1.5)
    REQUIRE(pyr.level_width(2) == 334);   // round(501 / 1.5)
}

TEST_CASE("downsampling preserves the mean (DC) of a ramp", "[cv][pyramid]") {
    cv::OwnedImage<std::uint8_t> base(128, 128, 0);
    for (std::size_t y = 0; y < 128; ++y)
        for (std::size_t x = 0; x < 128; ++x)
            base(y, x) = static_cast<std::uint8_t>(x * 2);  // 0..254 ramp in x
    cv::Pyramid<std::uint8_t> pyr(base.view(), 3, 2.0);
    const double m0 = mean_of(pyr.level(0));
    const double m1 = mean_of(pyr.level(1));
    const double m2 = mean_of(pyr.level(2));
    REQUIRE(std::abs(m1 - m0) < 1.0);  // mean preserved within ~1 gray level
    REQUIRE(std::abs(m2 - m0) < 1.0);
}

TEST_CASE("a horizontal ramp stays monotonic after downsampling", "[cv][pyramid]") {
    cv::OwnedImage<std::uint8_t> base(64, 64, 0);
    for (std::size_t y = 0; y < 64; ++y)
        for (std::size_t x = 0; x < 64; ++x)
            base(y, x) = static_cast<std::uint8_t>(x * 4);
    cv::Pyramid<std::uint8_t> pyr(base.view(), 2, 2.0);
    auto l1 = pyr.level(1);
    for (std::size_t y = 0; y < l1.height(); ++y)
        for (std::size_t x = 1; x < l1.width(); ++x)
            REQUIRE(l1(y, x) >= l1(y, x - 1));  // non-decreasing in x
}

TEST_CASE("pyramid is deterministic and works for float", "[cv][pyramid]") {
    cv::OwnedImage<float> base(40, 30, 0.0f);
    for (std::size_t y = 0; y < 30; ++y)
        for (std::size_t x = 0; x < 40; ++x)
            base(y, x) = static_cast<float>(std::sin(0.1 * x) + 0.2 * y);
    cv::Pyramid<float> a(base.view(), 3, 2.0);
    cv::Pyramid<float> b(base.view(), 3, 2.0);
    auto la = a.level(2);
    auto lb = b.level(2);
    REQUIRE(la.width() == lb.width());
    for (std::size_t y = 0; y < la.height(); ++y)
        for (std::size_t x = 0; x < la.width(); ++x)
            REQUIRE(la(y, x) == lb(y, x));  // bit-exact reproducibility

    // A constant float image is preserved exactly too.
    cv::OwnedImage<float> flat(50, 50, 0.5f);
    cv::Pyramid<float> fp(flat.view(), 4, 2.0);
    for (int l = 0; l < fp.num_levels(); ++l) {
        auto img = fp.level(l);
        for (std::size_t y = 0; y < img.height(); ++y)
            for (std::size_t x = 0; x < img.width(); ++x)
                REQUIRE(img(y, x) == 0.5f);
    }
}
