// SPDX-License-Identifier: MIT
//
// branes/cv/image_io.hpp — grayscale image read/write.
//
// PGM (Netpbm) is the primary interchange format for the test harnesses:
// trivial, lossless, byte-exact. `read_pgm` / `write_pgm` are header-only
// (pure parsing + std::ofstream). PNG/JPEG are read-only via stb_image
// and live in the compiled `branes::cv` library (image_io.cpp owns the
// single STB_IMAGE_IMPLEMENTATION).
//
// C++20.

#ifndef BRANES_CV_IMAGE_IO_HPP
#define BRANES_CV_IMAGE_IO_HPP

#include <branes/cv/image.hpp>

#include <cctype>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <istream>
#include <stdexcept>
#include <string>

namespace branes::cv {

namespace detail {

/// Read the next PGM header token, skipping whitespace and `#` comments.
inline std::string next_pgm_token(std::istream& in) {
    std::string tok;
    char c;
    // Skip leading whitespace and comment lines.
    while (in.get(c)) {
        if (c == '#') {
            while (in.get(c) && c != '\n') {}
        } else if (!std::isspace(static_cast<unsigned char>(c))) {
            tok.push_back(c);
            break;
        }
    }
    while (in.get(c) && !std::isspace(static_cast<unsigned char>(c))) {
        tok.push_back(c);
    }
    return tok;
}

}  // namespace detail

/// Read an 8-bit PGM (binary `P5` or ASCII `P2`). Throws
/// std::runtime_error on a malformed file or a maxval > 255 (16-bit PGM
/// is out of scope for this reader). Returns a contiguous OwnedImage.
inline OwnedImage<std::uint8_t> read_pgm(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in)
        throw std::runtime_error("read_pgm: cannot open " + path);

    const std::string magic = detail::next_pgm_token(in);
    if (magic != "P5" && magic != "P2") {
        throw std::runtime_error("read_pgm: not a PGM (P2/P5): " + path);
    }
    const std::size_t width = std::stoul(detail::next_pgm_token(in));
    const std::size_t height = std::stoul(detail::next_pgm_token(in));
    const long maxval = std::stol(detail::next_pgm_token(in));
    if (maxval <= 0 || maxval > 255) {
        throw std::runtime_error("read_pgm: unsupported maxval (expect 1..255)");
    }

    OwnedImage<std::uint8_t> img(width, height);
    if (magic == "P5") {
        in.read(reinterpret_cast<char*>(img.data()), static_cast<std::streamsize>(width * height));
        if (static_cast<std::size_t>(in.gcount()) != width * height) {
            throw std::runtime_error("read_pgm: truncated pixel data");
        }
    } else {  // P2 ASCII
        for (std::size_t i = 0; i < width * height; ++i) {
            img.data()[i] = static_cast<std::uint8_t>(std::stoi(detail::next_pgm_token(in)));
        }
    }
    return img;
}

/// Write a binary (`P5`) PGM. Defined for integral pixel types: 8-bit is
/// written one byte per pixel (maxval 255), 16-bit big-endian per the
/// Netpbm spec (maxval 65535). Throws on I/O failure.
template <PixelType T>
    requires std::integral<T>
void write_pgm(const std::string& path, const Image<T>& img) {
    std::ofstream out(path, std::ios::binary);
    if (!out)
        throw std::runtime_error("write_pgm: cannot open " + path);

    constexpr bool wide = sizeof(T) >= 2;
    const int maxval = wide ? 65535 : 255;
    out << "P5\n" << img.width() << ' ' << img.height() << '\n' << maxval << '\n';
    for (std::size_t r = 0; r < img.height(); ++r) {
        for (std::size_t c = 0; c < img.width(); ++c) {
            const auto px = static_cast<std::uint32_t>(img(r, c));
            if constexpr (wide) {
                out.put(static_cast<char>((px >> 8) & 0xFF));  // big-endian
                out.put(static_cast<char>(px & 0xFF));
            } else {
                out.put(static_cast<char>(px & 0xFF));
            }
        }
    }
    if (!out)
        throw std::runtime_error("write_pgm: write failed for " + path);
}

/// Read a PNG/JPEG (or any stb_image-supported format) as 8-bit
/// grayscale. Defined in the compiled branes::cv library (image_io.cpp).
/// Throws std::runtime_error if the file cannot be decoded.
OwnedImage<std::uint8_t> read_png(const std::string& path);

}  // namespace branes::cv

#endif  // BRANES_CV_IMAGE_IO_HPP
