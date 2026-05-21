fn main() {
    cxx_build::bridge("src/lib.rs")
        .compile("cortex_core_rust_bridge");
    
    println!("cargo:rerun-if-changed=src/lib.rs");
}
