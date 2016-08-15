//! A library for interacting with a HackRF device.
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
//!     let context = try!(hackrf::init());
//!
//!     let mut device = try!(HackRF::open(&context));
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
//!     Ok(())
//! }
//! ```

extern crate hackrf_sys as ffi;

use std::ffi::CStr;
use std::os::raw::{c_int, c_void};

use std::sync::mpsc::{SyncSender, Receiver, TrySendError, sync_channel};
use std::sync::atomic::{AtomicUsize, ATOMIC_USIZE_INIT, Ordering};

use std::{mem, slice, ops, fmt};
use std::rc::Rc;

use ffi::hackrf_error::*;

pub mod util;

/// The maximum size of each of the buffers used for receiving.
/// Note: this must be greater than or equal to the buffer size chosen for libusb.
const BUFFER_SIZE: usize = 262144;

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
///
/// ## Possible Error Values
///
///     HACKRF_ERROR_INVALID_PARAM = -2
///     HACKRF_ERROR_NOT_FOUND = -5
///     HACKRF_ERROR_BUSY = -6
///     HACKRF_ERROR_NO_MEM = -11
///     HACKRF_ERROR_LIBUSB = -1000
///     HACKRF_ERROR_THREAD = -1001
///     HACKRF_ERROR_STREAMING_THREAD_ERR = -1002
///     HACKRF_ERROR_STREAMING_STOPPED = -1003
///     HACKRF_ERROR_STREAMING_EXIT_CALLED = -1004
///     HACKRF_ERROR_OTHER = -9999
///
/// See [libhackrf](https://github.com/mossmann/hackrf/tree/master/host/libhackrf) for more info.
#[derive(Debug)]
pub struct HackRFError {
    /// The description of the code.
    pub desc: String,

    /// The code returned by `libhackrf`.
    pub code: i32,
}

impl fmt::Display for HackRFError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Error({}): {}", self.code, self.desc)
    }
}

/// A specialized Result type for HackRF operations
pub type HackRFResult<T> = Result<T, HackRFError>;

/// An internal function for obtaining the description from a HackRF error code.
fn parse_error(error_code: ffi::hackrf_error) -> HackRFError {
    unsafe {
        let desc = CStr::from_ptr(ffi::hackrf_error_name(error_code)).to_string_lossy();
        HackRFError {
            desc: desc.to_string(),
            code: error_code as i32,
        }
    }
}

/// Adjust a gain value to the nearest valid step
fn adjust_gain(gain: u32, min: u32, max: u32, step: u32) -> u32 {
    if gain < min { min }
    else if gain > max { max }
    else { (gain as f32 / step as f32).round() as u32 * step }
}

/// The HackRF context.
///
/// This struct is used ensure that `libhackrf` is initialized before it can be used. A context can
/// be obtained by calling hackrf::init().
///
/// ## Example
/// ```
/// let context = try!(hackrf::init());
/// // Use context
/// ```
pub struct HackRFContext(Rc<HackRFContextInner>);

/// Value for keeping track of any object that contains a reference to the context.
///
/// When this value is dropped, `libhackrf` will be closed.
struct HackRFContextInner;

impl Drop for HackRFContextInner {
    fn drop(&mut self) {
        unsafe {
            ffi::hackrf_exit();
        }
    }
}

/// Initialize `libhackrf`, returning a context that can be used to connect to HackRF devices.
/// Once all references to the returned context go out of scope, the library will be closed
/// automatically.
pub fn init() -> HackRFResult<HackRFContext> {
    unsafe {
        hackrf_try!(ffi::hackrf_init());
    }
    Ok(HackRFContext(Rc::new(HackRFContextInner)))
}

/// A structure for managaing a HackRF device.
///
/// This structure contains methods for configuring the various settings of the HackRF. It also
/// provides a method for managing received samples as a blocking stream. Currently transmitting
/// samples and non-blocking streams are not supported.
///
/// ## Implementation notes
///
/// This library is a lightweight wrapper around the basic HackRF FFI interface. In addition to
/// the basic functionality it provides a blocking stream interface for receiving samples (See:
/// [`RxStream`](./struct.RxStream.html)).
///
/// Rust's type system is used to ensure that resources are not destroy or modified while the HackRF
/// is receiving samples (although it is possible there are problems with the implementation.). To
/// stop receiving samples it is sufficient to let the stream drop out of scope. For example:
///
/// ```
/// let context = try!(hackrf::init());
/// let mut device = try!(HackRF::open(&context));
///
/// // Configure HackRF
///
/// {
///     let mut rx_stream = try!(device.rx_stream());
///     // Get samples from rx stream
/// }
///
/// // Re-configure HackRF
///
/// {
///     let mut rx_stream = try!(device.rx_stream());
///     // Get samples from rx stream
/// }
/// ```
pub struct HackRF {
    inner: *mut ffi::hackrf_device,
    _context: Rc<HackRFContextInner>,
}

impl Drop for HackRF {
    fn drop(&mut self) {
        unsafe {
            ffi::hackrf_close(self.inner);
        }
    }
}

impl HackRF {
    /// Attempt to open a connected HackRF device.
    pub fn open(context: &HackRFContext) -> HackRFResult<HackRF> {
        unsafe {
            let mut device = std::mem::zeroed();
            hackrf_try!(ffi::hackrf_open(&mut device));
            Ok(HackRF { inner: device, _context: context.0.clone() })
        }
    }

    /// Sets the center frequency (in Hz) of the HackRF: 5-6800 MHz.
    pub fn set_freq(&mut self, freq: u64) -> HackRFResult<()> {
        // TODO: Consider checking that the frequency is in a valid range.
        unsafe {
            hackrf_try!(ffi::hackrf_set_freq(self.inner, freq));
        }
        Ok(())
    }

    /// Sets amp gain to on/off.
    ///
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
    /// If the specified gain falls outside of the valid range, the gain will be clamped.
    pub fn set_lna_gain(&mut self, gain: u32) -> HackRFResult<()> {
        unsafe {
            hackrf_try!(ffi::hackrf_set_lna_gain(self.inner, adjust_gain(gain, 0, 40, 8)));
        }
        Ok(())
    }

    /// Set VGA (IF) gain: 0-62 dB, with 2dB steps.
    /// If the specified gain falls outside of the valid range, the gain will be clamped.
    pub fn set_vga_gain(&mut self, gain: u32) -> HackRFResult<()> {
        unsafe {
            hackrf_try!(ffi::hackrf_set_vga_gain(self.inner, adjust_gain(gain, 0, 62, 2)));
        }
        Ok(())
    }

    /// Sets the sample rate in Hz: 2-20 MHz.
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
    ///
    /// Note: This must be performed *after* setting the sample rate.
    pub fn set_baseband_filter_bw(&mut self, filter_bw: u32) -> HackRFResult<()> {
        unsafe {
            let adjusted_filter_bw = ffi::hackrf_compute_baseband_filter_bw(filter_bw);
            hackrf_try!(ffi::hackrf_set_baseband_filter_bandwidth(self.inner, adjusted_filter_bw));
        }
        Ok(())
    }

    /// Start an RX stream.
    pub fn rx_stream(&mut self, bound: usize) -> HackRFResult<RxStream> {
        let mut rx_stream = RxStream::new(bound, self);
        let sender = (&mut *rx_stream.sender) as *mut SyncSender<StackVec>;
        unsafe {
            match ffi::hackrf_start_rx(self.inner, rx_callback, sender as *mut c_void) {
                HACKRF_SUCCESS => Ok(rx_stream),
                error => Err(parse_error(error)),
            }
        }
    }
}

static OVERFLOW_COUNT: AtomicUsize = ATOMIC_USIZE_INIT;

/// A structure that manages receiving I/Q samples from the HackRF.
pub struct RxStream<'a> {
    hackrf_device: &'a HackRF,
    lookup_table: [f32; 256],
    local_index: usize,
    local_buffer: StackVec,
    sender: Box<SyncSender<StackVec>>,
    receiver: Receiver<StackVec>,
}

impl<'a> RxStream<'a> {
    /// Create a new instance of a RxStream
    fn new(bound: usize, hackrf_device: &'a HackRF) -> RxStream<'a> {
        fn gen_lookup_table() -> [f32; 256] {
            let mut data = [0.0; 256];
            for i in 0..256 {
                data[i] = (i as i8) as f32 / 128.0;
            }
            data
        }

        // Reset the overflow counter
        OVERFLOW_COUNT.store(0, Ordering::Relaxed);

        let (sender, receiver) = sync_channel(bound);
        RxStream {
            hackrf_device: hackrf_device,
            lookup_table: gen_lookup_table(),
            local_index: 0,
            local_buffer: StackVec::new(),
            sender: Box::new(sender),
            receiver: receiver,
        }
    }

    // Returns a reference to the internal receiver
    pub fn receiver(&mut self) -> &mut Receiver<StackVec> {
        &mut self.receiver
    }

    /// Return the next I/Q sample converted to floating point numbers. If there is no data
    /// avaliable, then this function is blocking.
    pub fn next_sample(&mut self) -> Option<(f32, f32)> {
        let sample = self.next_sample_raw();
        sample.map(|(i, q)| (self.lookup_table[i as usize], self.lookup_table[q as usize]))
    }

    /// Return the next raw I/Q sample from the HackRF. If there is no data avaliable, then this
    /// function is blocking.
    pub fn next_sample_raw(&mut self) -> Option<(u8, u8)> {
        if self.local_buffer.len() < self.local_index + 1 {
            self.local_buffer = match self.receiver.recv() {
                Ok(buffer) => buffer,
                Err(_) => return None,
            };
            self.local_index = 0;
        }

        let i = self.local_index;
        self.local_index += 2;
        Some((self.local_buffer[i], self.local_buffer[i + 1]))
    }

    /// Return how many times the stream dropped data frames
    pub fn overflow_count(&self) -> usize {
        OVERFLOW_COUNT.load(Ordering::Relaxed)
    }
}

impl<'a> Drop for RxStream<'a> {
    fn drop(&mut self) {
        unsafe {
            // TODO: investigate if this can ever fail under reasonable conditions
            ffi::hackrf_stop_rx(self.hackrf_device.inner);
        }
    }
}

/// A stack allocated vector with a maximum size
pub struct StackVec {
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
        self.data[0..slice.len()].copy_from_slice(slice);
        self.len = slice.len();
    }
}

impl ops::Deref for StackVec {
    type Target = [u8];

    fn deref(&self) -> &[u8] {
        &self.data[0..self.len]
    }
}

/// The call back that is used in the stream abstraction
unsafe extern "C" fn rx_callback(transfer: *mut ffi::hackrf_transfer) -> c_int {
    let src_buffer = slice::from_raw_parts((*transfer).buffer, (*transfer).valid_length as usize);

    // If the input buffer is too long, then an error has occured somewhere
    if src_buffer.len() > BUFFER_SIZE {
        return -1;
    }

    // Copy the transferred data into an owned structure
    let mut data = StackVec::new();
    data.copy_slice(src_buffer);

    let sender_raw: *mut SyncSender<StackVec> = mem::transmute((*transfer).rx_ctx);
    let sender = &mut *sender_raw;

    if let Err(e) = sender.try_send(data) {
        match e {
            // If the buffer is full drop the message and increment the overflow count
            TrySendError::Full(_) => { OVERFLOW_COUNT.fetch_add(1, Ordering::Relaxed); },

            // Report an error back to libhackrf
            TrySendError::Disconnected(_) => return -1,
        }
    }

    // Succesfully sent the data
    0
}

#[cfg(test)]
mod test {
    #[test]
    fn init_test() {
        super::init();
    }
}
