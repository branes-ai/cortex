#[cxx::bridge(namespace = "branes::core")]
mod ffi {
    // Define the C-Structs for your tensor metadata here
    struct TensorMetadata {
        data_ptr: usize, // Raw pointer to the zero-copy buffer
        rows: u32,
        cols: u32,
    }

    extern "Rust" {
        // Expose Rust functions for C++ to call
        fn request_kpu_buffer(size: usize) -> TensorMetadata;
    }
}
