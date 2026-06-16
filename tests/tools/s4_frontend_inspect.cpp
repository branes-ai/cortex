// SPDX-License-Identifier: MIT
//
// Gate for the S4 frontend inspector (branes/tools/s4_frontend_inspect.hpp,
// issue #374). The real inspector runs over EuRoC, which CI does not vendor, so
// here we drive S4FrontendInspector with synthetic high-contrast frames and pin
// the behaviour the overlay + downstream study rely on: frame 0 detects (no
// tracks carried in), a static frame keeps its tracks with ~0 forward-backward
// residual and incrementing age, a panned frame yields flow matching the pan,
// the FB gate culls when enabled, and the report serializes to the overlay
// schema.

#include <branes/cv/image.hpp>
#include <branes/tools/s4_frontend_inspect.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cstdint>

namespace bt = branes::tools;
namespace cv = branes::cv;
using Catch::Matchers::WithinAbs;

namespace {

// A deterministic high-contrast field with sharp bright squares — FAST fires at
// the square corners and KLT localizes them well. Sampling px(x+shift) moves a
// feature at column x0 to x0-shift, i.e. a leftward pan of `shift` px.
cv::OwnedImage<std::uint8_t> textured(int shift = 0) {
    constexpr std::size_t W = 240, H = 180;
    cv::OwnedImage<std::uint8_t> img(W, H, 40);
    auto px = [](std::size_t x, std::size_t y) -> std::uint8_t {
        // A smooth-ish background so KLT has gradients everywhere.
        return static_cast<std::uint8_t>(60 + ((x * 7 + y * 13) % 64));
    };
    auto v = img.view();
    for (std::size_t y = 0; y < H; ++y)
        for (std::size_t x = 0; x < W; ++x)
            v(y, x) = px(x + static_cast<std::size_t>(shift), y);
    // Sharp bright squares on a grid → strong, well-separated corners.
    for (std::size_t cy = 24; cy < H - 16; cy += 36)
        for (std::size_t cx = 24; cx < W - 16; cx += 36) {
            const std::size_t sx = cx - static_cast<std::size_t>(shift % 36);
            for (std::size_t y = cy; y < cy + 7 && y < H; ++y)
                for (std::size_t x = sx; x < sx + 7 && x < W; ++x)
                    v(y, x) = 250;
        }
    return img;
}

}  // namespace

TEST_CASE("S4 inspector: first frame detects, nothing tracked in", "[tools][s4_inspect]") {
    bt::S4FrontendInspector insp;  // defaults
    const auto img = textured();
    const auto rep = insp.step(std::as_const(img).view(), 1.0, "f0.png");

    REQUIRE(rep.frame == 0);
    REQUIRE(rep.n_tracked == 0);
    REQUIRE(rep.n_new > 0);
    REQUIRE(rep.n_new == rep.detections.size());
    REQUIRE(rep.pyramid_levels == 3);  // default FrontendParams
    REQUIRE(rep.pyramid_sizes.size() == 3);
    REQUIRE(rep.pyramid_sizes[0].first == rep.width);
    for (const auto& t : rep.tracks) {
        REQUIRE(t.status == "new");
        REQUIRE(t.age == 0);
        REQUIRE(t.fb_residual < 0.0);  // not computed for a new track
        REQUIRE(t.pu == t.u);          // new tracks have no previous position
    }
}

TEST_CASE("S4 inspector: a static frame keeps tracks at ~0 FB residual, age grows", "[tools][s4_inspect]") {
    bt::S4FrontendInspector insp;
    const auto img = textured();
    (void)insp.step(std::as_const(img).view(), 1.0, "f0.png");
    const auto r1 = insp.step(std::as_const(img).view(), 1.05, "f1.png");  // identical frame

    REQUIRE(r1.frame == 1);
    REQUIRE(r1.n_tracked > 0);
    std::uint32_t aged = 0;
    for (const auto& t : r1.tracks) {
        if (t.status != "tracked")
            continue;
        ++aged;
        REQUIRE(t.age == 1);
        REQUIRE(t.fb_residual >= 0.0);
        REQUIRE(t.fb_residual < 0.5);             // identical frames → round-trip is exact-ish
        REQUIRE_THAT(t.u, WithinAbs(t.pu, 0.5));  // no motion
        REQUIRE_THAT(t.v, WithinAbs(t.pv, 0.5));
    }
    REQUIRE(aged > 0);

    // A third identical frame: the survivors age again.
    const auto r2 = insp.step(std::as_const(img).view(), 1.10, "f2.png");
    bool saw_age2 = false;
    for (const auto& t : r2.tracks)
        if (t.status == "tracked" && t.age >= 2)
            saw_age2 = true;
    REQUIRE(saw_age2);
}

TEST_CASE("S4 inspector: flow follows a horizontal pan", "[tools][s4_inspect]") {
    bt::S4FrontendInspector insp;
    const auto f0 = textured(0);
    const auto f1 = textured(3);  // content shifted: features move ~+3 px in x
    (void)insp.step(std::as_const(f0).view(), 1.0, "f0.png");
    const auto r1 = insp.step(std::as_const(f1).view(), 1.05, "f1.png");

    // textured(shift) samples px(x+shift), so a feature at column x0 in f0 lands
    // at x0-shift in f1: the flow is leftward by ~3 px (u < pu).
    int moved_with_pan = 0, total = 0;
    double sum_dx = 0.0;
    for (const auto& t : r1.tracks) {
        if (t.status != "tracked")
            continue;
        ++total;
        const double dx = t.pu - t.u;  // leftward motion is positive
        sum_dx += dx;
        if (dx > 1.0)
            ++moved_with_pan;
    }
    REQUIRE(total > 0);
    REQUIRE(moved_with_pan * 2 > total);                // the majority moved with the pan
    REQUIRE_THAT(sum_dx / total, WithinAbs(3.0, 1.5));  // mean flow ≈ the 3 px pan
}

TEST_CASE("S4 inspector: FB gate off measures without culling; report serializes", "[tools][s4_inspect]") {
    branes::sdk::FrontendParams fe;
    fe.fb_max_residual = 0.0;  // measure only
    bt::S4FrontendInspector measure(fe);
    const auto a = textured(0);
    const auto b = textured(3);
    (void)measure.step(std::as_const(a).view(), 1.0, "a.png");
    const auto m = measure.step(std::as_const(b).view(), 1.05, "b.png");
    REQUIRE(m.n_fb_culled == 0);  // gate disabled → never culls

    SECTION("serialization carries the overlay schema") {
        const auto j = bt::to_json(m);
        REQUIRE(j.at("frame") == 1);
        REQUIRE(j.at("tracks").is_array());
        REQUIRE(j.at("counts").contains("tracked"));
        REQUIRE(j.at("pyramid").at("levels") == m.pyramid_levels);
        REQUIRE(j.at("grid").at("cols") == m.grid_cols);
        REQUIRE(j.at("nfeat") == m.tracks.size());
        // every track record carries the fields the renderer reads
        for (const auto& t : j.at("tracks")) {
            REQUIRE(t.contains("u"));
            REQUIRE(t.contains("pu"));
            REQUIRE(t.contains("fb"));
            REQUIRE(t.contains("status"));
        }
    }
}
