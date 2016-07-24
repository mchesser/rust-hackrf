extern crate gcc;
extern crate pkg_config;

fn main() {
    // Attempt to link to libhackrf
    if let Err(_) = pkg_config::probe_library("hackrf") {
        // Failed to link to system hackrf so attempt to build it instead.
        gcc::compile_library("libhackrf.a", &["libhackrf/hackrf.c"]);

        // Attempt to link to libusb
        if let Err(_) = pkg_config::probe_library("libusb-1.0") {
            // For now just try to link to everything manually
            println!("cargo:rustc-link-lib=libusb-1.0");
            println!("cargo:rustc-link-lib=pthread")
        }
    }
}
