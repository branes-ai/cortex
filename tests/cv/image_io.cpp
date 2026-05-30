// Image container + I/O tests (issue #48).
//
// Acceptance: a round-trip read+write of a 752x480 EuRoC-sized image
// preserves all bytes. Covered here for PGM (write_pgm -> read_pgm) and,
// for the read-only PNG path, by encoding a PNG with stb_image_write and
// decoding it back through read_png. Also exercises the Image / OwnedImage
// containers (indexing, views, stride).

#include <branes/cv/image.hpp>
#include <branes/cv/image_io.hpp>

#include <catch2/catch_test_macros.hpp>

// Encode PNGs in this TU so read_png has a real fixture to decode.
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <stb_image_write.h>

namespace {

namespace cv = branes::cv;

// Deterministic, full-range 752x480 pattern.
cv::OwnedImage<std::uint8_t> make_pattern(std::size_t w, std::size_t h) {
    cv::OwnedImage<std::uint8_t> img(w, h);
    for (std::size_t y = 0; y < h; ++y)
        for (std::size_t x = 0; x < w; ++x)
            img(y, x) = static_cast<std::uint8_t>((x * 7 + y * 13) & 0xFF);
    return img;
}

std::string temp_path(const char* name) {
    return (std::filesystem::temp_directory_path() / name).string();
}

}  // namespace

TEST_CASE("Image view indexing and stride", "[cv][image]") {
    std::vector<std::uint8_t> buf(6 * 4, 0);
    cv::Image<std::uint8_t> img(buf.data(), 4, 3, 6);  // 4x3 in a stride-6 buf
    REQUIRE(img.width() == 4);
    REQUIRE(img.height() == 3);
    REQUIRE(img.stride() == 6);
    REQUIRE_FALSE(img.is_contiguous());
    img(2, 3) = 42;
    REQUIRE(buf[2 * 6 + 3] == 42);  // row*stride + col
    REQUIRE(img.row(2).size() == 4);
}

TEST_CASE("OwnedImage hands out consistent views", "[cv][image]") {
    cv::OwnedImage<std::uint16_t> img(5, 4, 7);
    REQUIRE(img.size() == 20);
    img(1, 2) = 1000;
    REQUIRE(img.view()(1, 2) == 1000);
    const auto& cimg = img;
    REQUIRE(cimg.view()(0, 0) == 7);  // fill value
}

TEST_CASE("PGM round-trip preserves all bytes (752x480)", "[cv][image][io]") {
    constexpr std::size_t W = 752, H = 480;  // EuRoC MAV resolution
    const auto src = make_pattern(W, H);
    const auto path = temp_path("branes_cv_roundtrip.pgm");

    cv::write_pgm(path, src.view());
    const auto loaded = cv::read_pgm(path);

    REQUIRE(loaded.width() == W);
    REQUIRE(loaded.height() == H);
    bool all_equal = true;
    for (std::size_t i = 0; i < W * H; ++i)
        if (loaded.data()[i] != src.data()[i]) {
            all_equal = false;
            break;
        }
    REQUIRE(all_equal);
    std::filesystem::remove(path);
}

TEST_CASE("16-bit PGM round-trips big-endian", "[cv][image][io]") {
    cv::OwnedImage<std::uint16_t> src(8, 4);
    for (std::size_t i = 0; i < src.size(); ++i)
        src.data()[i] = static_cast<std::uint16_t>(i * 4096 + 123);
    const auto path = temp_path("branes_cv_roundtrip16.pgm");
    cv::write_pgm(path, src.view());

    // Read the 16-bit data back manually (read_pgm targets 8-bit) and
    // confirm the big-endian byte order matches the source samples.
    std::ifstream in(path, std::ios::binary);
    std::string magic;
    int w, h, maxv;
    in >> magic >> w >> h >> maxv;
    in.get();  // consume the single whitespace after maxval
    REQUIRE(maxv == 65535);
    for (std::size_t i = 0; i < src.size(); ++i) {
        const int hi = in.get();
        const int lo = in.get();
        const auto px = static_cast<std::uint16_t>((hi << 8) | lo);
        REQUIRE(px == src.data()[i]);
    }
    in.close();  // Windows refuses to remove a file while a handle is open.
    std::filesystem::remove(path);
}

TEST_CASE("read_png decodes an encoded grayscale PNG byte-exactly", "[cv][image][io]") {
    constexpr std::size_t W = 64, H = 48;
    const auto src = make_pattern(W, H);
    const auto path = temp_path("branes_cv_roundtrip.png");

    const int ok =
        stbi_write_png(path.c_str(), static_cast<int>(W), static_cast<int>(H), 1, src.data(), static_cast<int>(W));
    REQUIRE(ok != 0);

    const auto loaded = cv::read_png(path);
    REQUIRE(loaded.width() == W);
    REQUIRE(loaded.height() == H);
    bool all_equal = true;
    for (std::size_t i = 0; i < W * H; ++i)
        if (loaded.data()[i] != src.data()[i]) {
            all_equal = false;
            break;
        }
    REQUIRE(all_equal);  // PNG is lossless for grayscale
    std::filesystem::remove(path);
}
