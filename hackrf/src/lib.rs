//! # HackRF
//!
//! A library for interacting with a HackRF.
//!
//! ## Examples
//! ```
//! extern crate hackrf;
//!
//! use hackrf::{HackRF, HackRFResult};
//!
//! const SAMP_RATE: u64 = 2_000_000; // 2 MHz
//!
//! fn main() {
//!    if let Err(e) = run() {
//!        println!("{:?}", e);
//!    }
//! }
//!
//! fn run() -> HackRFResult<()> {
//!     try!(hackrf::init());
//!
//!     let mut device = try!(HackRF::open());
//!     try!(device.set_samp_rate(SAMP_RATE as f64));
//!     try!(device.set_freq(150_000_000));
//!
//!     let mut rx_stream = try!(device.rx_stream());
//!
//!     // Take 10 seconds worth of samples
//!     for _ in 0..10 * SAMP_RATE {
//!         let (i, q) = rx_stream.next_sample();
//!         // .. Do something with samples ..
//!     }
//!
//!     try!(hackrf::exit());
//!     Ok(())
//! }
//! ```

extern crate hackrf_sys as ffi;

use std::ffi::CStr;
use std::os::raw::{c_int, c_void};

use std::sync::{Mutex, Condvar};
use std::{mem, slice, ops};

use ffi::hackrf_error::*;

/// The maximum size of each of the buffers used for receiving.
/// Note: this must be greater than or equal to the buffer size chosen for libusb.
const BUFFER_SIZE: usize = 262144;

/// The number of buffers to use to better handle overrun.
const NUM_BUFFERS: usize = 5;

/// An internal macro for handling hackrf errors.
macro_rules! hackrf_try {
    ($e:expr) => ({
        match $e {
            HACKRF_SUCCESS => (),
            error => return Err(parse_error(error)),
        }
    });
}

/// A error that can occur when trying to interact with a HackRF device.
#[derive(Debug)]
pub struct HackRFError {
    pub desc: String,
    pub code: i32
}

/// A specialized Result type for HackRF operations
pub type HackRFResult<T> = Result<T, HackRFError>;

/// An internal function for obtaining the description from a HackRF error code.
fn parse_error(error_code: ffi::hackrf_error) -> HackRFError {
    unsafe {
        let desc = CStr::from_ptr(ffi::hackrf_error_name(error_code)).to_string_lossy();
        HackRFError {
            desc: desc.to_string(),
            code: error_code as i32
        }
    }
}

/// Adjust a gain value to the nearest valid step
fn adjust_gain(gain: u32, min: u32, max: u32, step: u32) -> u32 {
    if gain < min { min }
    else if gain > max { max }
    else { (gain as f32 / step as f32).round() as u32 * step }
}

/// Initialize the HackRF library
pub fn init() -> HackRFResult<()> {
    unsafe {
        hackrf_try!(ffi::hackrf_init());
    }
    Ok(())
}


/// Exit the HackRF library
pub fn exit() -> HackRFResult<()> {
    unsafe {
        hackrf_try!(ffi::hackrf_exit());
    }
    Ok(())
}

/// A HackRF device
pub struct HackRF {
    inner: *mut ffi::hackrf_device
}

impl Drop for HackRF {
    fn drop(&mut self) {
        unsafe {
            ffi::hackrf_close(self.inner);
        }
    }
}

impl HackRF {
    /// Attempt to open a connected HackRF device
    pub fn open() -> HackRFResult<HackRF> {
        unsafe {
            let mut device = std::mem::zeroed();
            hackrf_try!(ffi::hackrf_open(&mut device));
            Ok(HackRF { inner: device })
        }
    }

    /// Closes the current HackRF device
    pub fn close(self) -> HackRFResult<()> {
        unsafe {
            hackrf_try!(ffi::hackrf_close(self.inner));
            mem::forget(self);
        }
        Ok(())
    }

    /// Sets the center frequency (in Hz) of the HackRF: 5-6800 MHz
    pub fn set_freq(&mut self, freq: u64) -> HackRFResult<()> {
        // TODO: Consider checking that the frequency is in a valid range.
        unsafe {
            hackrf_try!(ffi::hackrf_set_freq(self.inner, freq));
        }
        Ok(())
    }

    /// Sets amp gain to on/off
    /// Warning: Enabling this in the presence of high power signals could damage the HackRF.
    pub fn set_amp_enable(&mut self, amp_enable: bool) -> HackRFResult<()> {
        unsafe {
            let value = if amp_enable { 1 } else { 0 };
            hackrf_try!(ffi::hackrf_set_amp_enable(self.inner, value));
        }
        Ok(())
    }

    /// Set antenna enable to on/off.
    pub fn set_antenna_enable(&mut self, antenna_enable: bool) -> HackRFResult<()> {
        unsafe {
            let value = if antenna_enable { 1 } else { 0 };
            hackrf_try!(ffi::hackrf_set_antenna_enable(self.inner, value));
        }
        Ok(())
    }

    /// Set LNA gain: 0-40 dB, 8dB steps.
    /// If the specified gain falls outside of the valid range, the gain will be clamped
    pub fn set_lna_gain(&mut self, gain: u32) -> HackRFResult<()> {
        unsafe {
            hackrf_try!(ffi::hackrf_set_lna_gain(self.inner, adjust_gain(gain, 0, 40, 8)));
        }
        Ok(())
    }

    /// Set VGA (IF) gain: 0-62 dB, with 2dB steps.
    /// If the specified gain falls outside of the valid range, the gain will be clamped
    pub fn set_vga_gain(&mut self, gain: u32) -> HackRFResult<()> {
        unsafe {
            hackrf_try!(ffi::hackrf_set_vga_gain(self.inner, adjust_gain(gain, 0, 62, 2)));
        }
        Ok(())
    }

    /// Sets the sample rate in Hz: 2-20 MHz
    pub fn set_samp_rate(&mut self, samp_rate: f64) -> HackRFResult<()> {
        unsafe {
            hackrf_try!(ffi::hackrf_set_sample_rate(self.inner, samp_rate));

            // Automatically compute a good baseband filter bandwidth
            let bandwidth = ffi::hackrf_compute_baseband_filter_bw_round_down_lt(samp_rate as u32);
            try!(self.set_baseband_filter_bw(bandwidth));
        }
        Ok(())
    }


    /// Sets the baseband filter bandwidth.
    /// Note: This must be performed *after* setting the sample rate.
    pub fn set_baseband_filter_bw(&mut self, filter_bw: u32) -> HackRFResult<()> {
        unsafe {
            let adjusted_filter_bw = ffi::hackrf_compute_baseband_filter_bw(filter_bw);
            hackrf_try!(ffi::hackrf_set_baseband_filter_bandwidth(self.inner, adjusted_filter_bw));
        }
        Ok(())
    }

    /// Start an RX stream
    pub fn rx_stream(&mut self) -> HackRFResult<RxStream> {
        let buffers = TransferBuffers::new(NUM_BUFFERS);
        let shared_data = {
            let data = Box::new(RxSharedData(Mutex::new(buffers), Condvar::new()));
            Box::into_raw(data)
        };

        unsafe {
            match ffi::hackrf_start_rx(self.inner, rx_callback, shared_data as *mut c_void) {
                HACKRF_SUCCESS => Ok(RxStream::new(shared_data, self)),
                error => {
                    // Ensure the allocated data gets deallocated correctly.
                    mem::drop(Box::from_raw(shared_data));
                    Err(parse_error(error))
                }
            }
        }
    }
}

/// A structure that manages receiving I/Q samples from the HackRF
pub struct RxStream<'a> {
    local_index: usize,
    local_buffer: StackVec,
    shared_data: *mut RxSharedData,
    hackrf_device: &'a mut HackRF,
    lookup_table: [f32; 256],
}

impl<'a> RxStream<'a> {
    /// Create a new instance of a RxStream
    fn new(shared_data: *mut RxSharedData, hackrf_device: &'a mut HackRF) -> RxStream<'a> {
        fn gen_lookup_table() -> [f32; 256] {
            let mut data = [0.0; 256];
            for i in 0..256 {
                data[i] = (i as i8) as f32 / 128.0;
            }
            data
        }

        RxStream {
            local_index: 0,
            local_buffer: StackVec::new(),
            shared_data: shared_data,
            hackrf_device: hackrf_device,
            lookup_table: gen_lookup_table()
        }
    }

    /// Return the next I/Q sample converted to a floating point number.
    pub fn next_sample(&mut self) -> (f32, f32) {
        let (raw_i, raw_q) = self.next_sample_raw();
        (self.lookup_table[raw_i as usize], self.lookup_table[raw_q as usize])
    }

    /// Return the next raw I/Q sample from the HackRF.
    pub fn next_sample_raw(&mut self) -> (u8, u8) {
        if self.local_index + 1 < self.local_buffer.len() {
            let &RxSharedData(ref buffers, ref cvar) = unsafe { &(*self.shared_data) };

            // Obtain a lock to the shared data.
            let mut buffers = {
                let lock = buffers.lock().unwrap();
                if lock.used == 0 {
                    // If there is no data to read, then wait for data to arrive
                    cvar.wait(lock).unwrap()
                }
                else { lock }
            };

            // Get the oldest shared data and copy it into the local buffer.
            let index = buffers.head;
            self.local_buffer.copy_slice(&buffers.data[index]);
            buffers.used -= 1;

            self.local_index = 0;
        }

        let i = self.local_index;
        self.local_index += 2;
        (self.local_buffer[i], self.local_buffer[i + 1])
    }
}

impl<'a> Drop for RxStream<'a> {
    fn drop(&mut self) {
        unsafe {
            // TODO: investigate if this can ever fail under reasonable conditions
            ffi::hackrf_stop_rx(self.hackrf_device.inner);
            mem::drop(Box::from_raw(self.shared_data));
        }
    }
}

/// Data shared between the RxStream and callback context
struct RxSharedData(Mutex<TransferBuffers>, Condvar);

/// A stack allocated vector with a maximum size
struct StackVec {
    data: [u8; BUFFER_SIZE],
    len: usize,
}

impl StackVec {
    fn new() -> StackVec {
        StackVec {
            data: [0; BUFFER_SIZE],
            len: 0
        }
    }

    /// Copy a slice into the vector, setting the vectors length to the length of the slice.
    /// Panics if the slice is too large.
    fn copy_slice(&mut self, slice: &[u8]) {
        self.data[0..slice.len()].clone_from_slice(slice);
        self.len = slice.len();
    }
}

impl ops::Deref for StackVec {
    type Target = [u8];

    fn deref(&self) -> &[u8] {
        &self.data[0..self.len]
    }
}

/// Data for maintaining the currently stored samples
struct TransferBuffers {
    data: Vec<StackVec>,
    head: usize,
    used: usize,
}

impl TransferBuffers {
    fn new(num_buffers: usize) -> TransferBuffers {
        TransferBuffers {
            data: (0..num_buffers).map(|_| StackVec::new()).collect(),
            head: 0,
            used: 0,
        }
    }
}

/// Copy the source buffer into the next free circular buffer.
fn copy_to_next_buffer(src_buffer: &[u8], buffers: &mut TransferBuffers) {
    let dest_index = (buffers.head + buffers.used) %  buffers.data.len();
    buffers.data[dest_index].copy_slice(src_buffer);

    if buffers.used >= buffers.data.len() {
        // Out of space in the buffer... Just overwrite old data for now...
        buffers.head = (buffers.head + 1) % buffers.data.len();
    }
    else {
        buffers.used += 1;
    }
}

unsafe extern "C" fn rx_callback(transfer: *mut ffi::hackrf_transfer) -> c_int {
    let src_buffer = slice::from_raw_parts((*transfer).buffer, (*transfer).valid_length as usize);
    let shared_data: *mut RxSharedData = mem::transmute((*transfer).rx_ctx);

    let &RxSharedData(ref buffers, ref cvar) = &(*shared_data);
    copy_to_next_buffer(src_buffer, &mut *buffers.lock().unwrap());
    cvar.notify_one();

    0
}
