// SPDX-License-Identifier: MIT
//
// branes/cv/image_io.cpp — compiled image I/O (PNG/JPEG decode).
//
// Owns the single STB_IMAGE_IMPLEMENTATION translation unit for the
// project's CV layer and implements read_png on top of it. PGM read/write
// is header-only (see image_io.hpp).

#include <branes/cv/image_io.hpp>

#include <cstring>
#include <stdexcept>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

namespace branes::cv {

OwnedImage<std::uint8_t> read_png(const std::string& path) {
    int w = 0;
    int h = 0;
    int channels = 0;
    // Force a single (grayscale) channel; stb converts as needed.
    std::uint8_t* pixels = stbi_load(path.c_str(), &w, &h, &channels, 1);
    if (pixels == nullptr) {
        throw std::runtime_error("read_png: failed to decode " + path + " (" + stbi_failure_reason() + ")");
    }
    if (w <= 0 || h <= 0) {  // defensive: stb should never return non-positive
        stbi_image_free(pixels);
        throw std::runtime_error("read_png: non-positive dimensions for " + path);
    }
    OwnedImage<std::uint8_t> img(static_cast<std::size_t>(w), static_cast<std::size_t>(h));
    std::memcpy(img.data(), pixels, static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
    stbi_image_free(pixels);
    return img;
}

}  // namespace branes::cv
