HackRF
====

A library for interacting with a HackRF. Currently only receiving is supported.

## Documentation

 - [Link](http://mchesser.github.io/rust-hackrf/doc/hackrf/index.html)

## Examples

```rust
extern crate hackrf;

use hackrf::{HackRF, HackRFResult};

const SAMP_RATE: u64 = 2_000_000; // 2 MHz

fn main() {
   if let Err(e) = run() {
       println!("{:?}", e);
   }
}

fn run() -> HackRFResult<()> {
    let context = try!(hackrf::init());

    let mut device = try!(HackRF::open(&context));
    try!(device.set_samp_rate(SAMP_RATE as f64));
    try!(device.set_freq(150_000_000));

    let mut rx_stream = try!(device.rx_stream());

    // Take 10 seconds worth of samples
    for _ in 0..10 * SAMP_RATE {
        let (i, q) = rx_stream.next_sample();
        // .. Do something with samples ..
    }

    Ok(())
}
```
