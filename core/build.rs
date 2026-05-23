// Generates the C++ header for the `#[cxx::bridge]` module in
// `src/bridge.rs`. The output lives at the build dir Corrosion exposes
// to consuming CMake targets; C++ TUs `#include <branes/core/bridge.rs.h>`.

fn main() {
    cxx_build::bridge("src/bridge.rs")
        .std("c++20")
        .compile("cortex_core_bridge");
    println!("cargo:rerun-if-changed=src/bridge.rs");
    println!("cargo:rerun-if-changed=build.rs");
}
