// KLT pyramidal tracker tests (issue #38).
//
// No EuRoC subset is in-repo, so tracking accuracy is validated against a
// synthetic known translation: a richly-textured image A and a copy B
// shifted by a known sub-pixel (dx, dy). The tracker must recover that
// displacement for feature points to sub-pixel accuracy, and report
// Lost on a textureless image and OutOfBounds when a feature leaves the
// frame.

#include <branes/cv/fast.hpp>
#include <branes/cv/image.hpp>
#include <branes/cv/klt.hpp>
#include <branes/cv/pyramid.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <vector>

namespace {

namespace cv = branes::cv;

// A richly-textured, non-periodic-enough field with gradients in both
// directions everywhere (so the LK Hessian is well-conditioned).
double pattern(double x, double y) {
    return 128.0 + 60.0 * std::sin(0.21 * x + 0.10 * y) + 50.0 * std::cos(0.13 * x - 0.17 * y) +
           40.0 * std::sin(0.07 * x) * std::cos(0.09 * y);
}

cv::OwnedImage<float> render(int w, int h, double dx, double dy) {
    cv::OwnedImage<float> img(static_cast<std::size_t>(w), static_cast<std::size_t>(h));
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            img(static_cast<std::size_t>(y), static_cast<std::size_t>(x)) = static_cast<float>(pattern(x - dx, y - dy));
    return img;
}

}  // namespace

TEST_CASE("KLT recovers a known sub-pixel translation", "[cv][klt]") {
    constexpr int W = 240, H = 200;
    const double dx = 1.7, dy = -1.1;
    const auto a = render(W, H, 0.0, 0.0);
    const auto b = render(W, H, dx, dy);  // B(x,y) = A(x-dx, y-dy)
    const cv::Pyramid<float> pa(a.view(), 3, 2.0);
    const cv::Pyramid<float> pb(b.view(), 3, 2.0);

    std::vector<cv::KeyPoint> pts = {{60, 60, 0}, {100, 80, 0}, {150, 120, 0}, {90, 150, 0}, {180, 90, 0}};
    const auto tracked = cv::track_klt_pyramidal(pa, pb, pts);

    REQUIRE(tracked.size() == pts.size());
    for (std::size_t i = 0; i < pts.size(); ++i) {
        REQUIRE(tracked[i].status == cv::TrackStatus::Tracked);
        REQUIRE(std::abs((tracked[i].x - pts[i].x) - dx) < 0.3);
        REQUIRE(std::abs((tracked[i].y - pts[i].y) - dy) < 0.3);
    }
}

TEST_CASE("KLT reports Lost on a textureless image", "[cv][klt]") {
    cv::OwnedImage<float> flat(120, 120, 100.0f);
    const cv::Pyramid<float> p(flat.view(), 3, 2.0);
    std::vector<cv::KeyPoint> pts = {{60, 60, 0}};
    const auto tracked = cv::track_klt_pyramidal(p, p, pts);
    REQUIRE(tracked[0].status == cv::TrackStatus::Lost);
}

TEST_CASE("KLT reports OutOfBounds when the window leaves the frame", "[cv][klt]") {
    constexpr int W = 200, H = 200;
    const auto a = render(W, H, 0.0, 0.0);
    const cv::Pyramid<float> p(a.view(), 3, 2.0);
    // A point within hw+1 px of the right edge: its support window leaves
    // a pyramid level, so the result is flagged OutOfBounds.
    std::vector<cv::KeyPoint> pts = {{196, 100, 0}};
    const auto tracked = cv::track_klt_pyramidal(p, p, pts);
    REQUIRE(tracked[0].status == cv::TrackStatus::OutOfBounds);
}

TEST_CASE("KLT zero displacement is recovered as ~zero flow", "[cv][klt]") {
    constexpr int W = 160, H = 160;
    const auto a = render(W, H, 0.0, 0.0);
    const cv::Pyramid<float> p(a.view(), 3, 2.0);
    std::vector<cv::KeyPoint> pts = {{80, 80, 0}};
    const auto tracked = cv::track_klt_pyramidal(p, p, pts);
    REQUIRE(tracked[0].status == cv::TrackStatus::Tracked);
    REQUIRE(std::abs(tracked[0].x - 80.0) < 0.05);
    REQUIRE(std::abs(tracked[0].y - 80.0) < 0.05);
}
