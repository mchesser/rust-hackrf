extern crate gcc;
extern crate pkg_config;

fn main() {
    gcc::compile_library("libhackrf.a", &["libhackrf/hackrf.c"]);
    pkg_config::find_library("libusb-1.0").unwrap();
}