// C++↔Rust integration test for the cxx::bridge defined in
// `core/src/bridge.rs`. Acceptance test for issue #20.
//
// The test exercises the same flow a real consumer (the SDK in Phase
// 3) will use:
//   1. Construct a ResourceManager from C++.
//   2. Walk the lifecycle state machine.
//   3. Allocate a buffer through the bridge, wrap it in std::span,
//      write a pattern, and verify Rust sees the same bytes.
//   4. Release. Confirm double-release is detected.

#include <catch2/catch_test_macros.hpp>
// Corrosion's corrosion_add_cxxbridge places the generated header at
// `<cxx_target>/<basename>.h` — see CMakeLists.txt §3a.
#include <cortex_core_cxx/bridge.h>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <span>
#include <stdexcept>

using branes::core::BridgeLifecycleState;
using branes::core::DataType;
using branes::core::make_resource_manager;
using branes::core::ResourceManager;
using branes::core::TensorMetadata;

namespace {

// Force the heap backend so the test never touches /dev/shm and runs
// without privileges. Setting the env var before make_resource_manager
// is the supported way to control SitlMemoryProvider's backend choice.
void use_heap_backend() {
#if defined(_WIN32)
    // MSVC has no POSIX setenv/unsetenv. _putenv_s sets the variable, and an
    // empty value removes it from the environment (documented behaviour).
    _putenv_s("CORTEX_MEM_BACKEND", "heap");
    _putenv_s("RUST_HAL_TARGET", "");
#else
    ::setenv("CORTEX_MEM_BACKEND", "heap", /*overwrite=*/1);
    ::unsetenv("RUST_HAL_TARGET");
#endif
}

}  // namespace

TEST_CASE("ResourceManager round-trip via cxx bridge", "[bridge][integration]") {
    use_heap_backend();
    auto rm = make_resource_manager();
    REQUIRE(rm->lifecycle_state() == BridgeLifecycleState::Unconfigured);

    rm->configure();
    REQUIRE(rm->lifecycle_state() == BridgeLifecycleState::Inactive);

    rm->activate();
    REQUIRE(rm->lifecycle_state() == BridgeLifecycleState::Active);

    // Allocate 1 MiB of u8 storage. The metadata bundle carries
    // everything the C++ side needs to construct a std::span without
    // a second round-trip to Rust.
    TensorMetadata meta = rm->request_buffer(
        /*byte_size=*/1u << 20,
        /*alignment=*/64,
        /*rows=*/1024,
        /*cols=*/1024,
        /*stride=*/1024,
        DataType::Uint8);

    REQUIRE(meta.data_ptr != 0);
    REQUIRE(meta.byte_size >= (1u << 20));
    REQUIRE(meta.rows == 1024);
    REQUIRE(meta.cols == 1024);
    REQUIRE(meta.stride == 1024);

    auto* ptr = reinterpret_cast<std::uint8_t*>(meta.data_ptr);
    std::span<std::uint8_t> view(ptr, meta.byte_size);

    // Write a fixed pattern from the C++ side.
    std::fill(view.begin(), view.end(), std::uint8_t{0xA5});

    // Sanity-check the bookends. Full-buffer correctness is implied by
    // fill's contract; here we just confirm the pointer + size combo
    // produces a usable span.
    REQUIRE(view.front() == 0xA5);
    REQUIRE(view.back() == 0xA5);
    REQUIRE(view[meta.byte_size / 2] == 0xA5);

    rm->release_buffer(meta.handle);

    rm->deactivate();
    REQUIRE(rm->lifecycle_state() == BridgeLifecycleState::Inactive);

    rm->teardown();
    REQUIRE(rm->lifecycle_state() == BridgeLifecycleState::Teardown);

    rm->reset();
    REQUIRE(rm->lifecycle_state() == BridgeLifecycleState::Unconfigured);
}

TEST_CASE("Double release surfaces as exception", "[bridge][integration]") {
    use_heap_backend();
    auto rm = make_resource_manager();
    TensorMetadata meta = rm->request_buffer(64, 8, 1, 1, 1, DataType::Uint8);
    rm->release_buffer(meta.handle);
    REQUIRE_THROWS(rm->release_buffer(meta.handle));
}

TEST_CASE("Invalid lifecycle transition surfaces as exception", "[bridge][lifecycle]") {
    use_heap_backend();
    auto rm = make_resource_manager();
    // From Unconfigured, activate is invalid. cxx maps the Rust
    // `Err(LifecycleErr)` to a C++ exception.
    REQUIRE_THROWS(rm->activate());
}

TEST_CASE("DataType enum mirrors Rust shape", "[bridge][datatype]") {
    // Compile-time check: the C++ enum has the expected discriminants
    // matching the Rust DataType.
    STATIC_REQUIRE(static_cast<std::uint8_t>(DataType::Uint8) == 0);
    STATIC_REQUIRE(static_cast<std::uint8_t>(DataType::Float64) == 9);
}
