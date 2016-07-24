#![allow(bad_style)]

use std::os::raw::{c_void, c_int, c_uint, c_char, c_uchar, c_double};

#[derive(Clone, Copy)]
#[repr(i32)]
pub enum hackrf_error {
    HACKRF_SUCCESS = 0,
    HACKRF_TRUE = 1,
    HACKRF_ERROR_INVALID_PARAM = -2,
    HACKRF_ERROR_NOT_FOUND = -5,
    HACKRF_ERROR_BUSY = -6,
    HACKRF_ERROR_NO_MEM = -11,
    HACKRF_ERROR_LIBUSB = -1000,
    HACKRF_ERROR_THREAD = -1001,
    HACKRF_ERROR_STREAMING_THREAD_ERR = -1002,
    HACKRF_ERROR_STREAMING_STOPPED = -1003,
    HACKRF_ERROR_STREAMING_EXIT_CALLED = -1004,
    HACKRF_ERROR_OTHER = -9999,
}

#[derive(Clone, Copy)]
#[repr(u32)]
pub enum hackrf_board_id {
    BOARD_ID_JELLYBEAN = 0,
    BOARD_ID_JAWBREAKER = 1,
    BOARD_ID_HACKRF_ONE = 2,
    BOARD_ID_INVALID = 255,
}

#[derive(Clone, Copy)]
#[repr(u32)]
pub enum hackrf_usb_board_id {
    USB_BOARD_ID_JAWBREAKER = 24651,
    USB_BOARD_ID_HACKRF_ONE = 24713,
    USB_BOARD_ID_RAD1O = 52245,
    USB_BOARD_ID_INVALID = 65535,
}

#[derive(Clone, Copy)]
#[repr(u32)]
pub enum rf_path_filter {
    RF_PATH_FILTER_BYPASS = 0,
    RF_PATH_FILTER_LOW_PASS = 1,
    RF_PATH_FILTER_HIGH_PASS = 2,
}

#[derive(Clone, Copy)]
#[repr(u32)]
pub enum transceiver_mode_t {
    TRANSCEIVER_MODE_OFF = 0,
    TRANSCEIVER_MODE_RX = 1,
    TRANSCEIVER_MODE_TX = 2,
    TRANSCEIVER_MODE_SS = 3,
    TRANSCEIVER_MODE_CPLD_UPDATE = 4,
}

pub enum hackrf_device { }

#[repr(C)]
pub struct hackrf_transfer {
    pub device: *mut hackrf_device,
    pub buffer: *mut u8,
    pub buffer_length: c_int,
    pub valid_length: c_int,
    pub rx_ctx: *mut c_void,
    pub tx_ctx: *mut c_void,
}

#[repr(C)]
pub struct read_partid_serialno_t {
    pub part_id: [u32; 2],
    pub serial_no: [u32; 4],
}

#[repr(C)]
pub struct hackrf_device_list_t {
    pub serial_numbers: *mut *mut c_char,
    pub usb_board_ids: *mut hackrf_usb_board_id,
    pub usb_device_index: *mut c_int,
    pub devicecount: c_int,
    pub usb_devices: *mut *mut c_void,
    pub usb_devicecount: c_int,
}

pub type hackrf_sample_block_cb_fn = unsafe extern "C" fn(transfer: *mut hackrf_transfer) -> c_int;

extern "C" {
    pub fn hackrf_init() -> hackrf_error;
    pub fn hackrf_exit() -> hackrf_error;
    pub fn hackrf_device_list() -> *mut hackrf_device_list_t;
    pub fn hackrf_device_list_open(list: *mut hackrf_device_list_t, idx: c_int, device: *mut *mut hackrf_device) -> hackrf_error;
    pub fn hackrf_device_list_free(list: *mut hackrf_device_list_t);
    pub fn hackrf_open(device: *mut *mut hackrf_device) -> hackrf_error;
    pub fn hackrf_open_by_serial(desired_serial_number: *const c_char, device: *mut *mut hackrf_device) -> hackrf_error;
    pub fn hackrf_close(device: *mut hackrf_device) -> hackrf_error;
    pub fn hackrf_start_rx(device: *mut hackrf_device, callback: hackrf_sample_block_cb_fn, rx_ctx: *mut c_void) -> hackrf_error;
    pub fn hackrf_stop_rx(device: *mut hackrf_device) -> hackrf_error;
    pub fn hackrf_start_tx(device: *mut hackrf_device, callback: hackrf_sample_block_cb_fn, tx_ctx: *mut c_void) -> hackrf_error;
    pub fn hackrf_stop_tx(device: *mut hackrf_device) -> hackrf_error;
    pub fn hackrf_is_streaming(device: *mut hackrf_device) -> hackrf_error;
    pub fn hackrf_max2837_read(device: *mut hackrf_device, register_number: u8, value: *mut u16) -> hackrf_error;
    pub fn hackrf_max2837_write(device: *mut hackrf_device, register_number: u8, value: u16) -> hackrf_error;
    pub fn hackrf_si5351c_read(device: *mut hackrf_device, register_number: u16, value: *mut u16) -> hackrf_error;
    pub fn hackrf_si5351c_write(device: *mut hackrf_device, register_number: u16, value: u16) -> hackrf_error;
    pub fn hackrf_set_baseband_filter_bandwidth(device: *mut hackrf_device, bandwidth_hz: u32) -> hackrf_error;
    pub fn hackrf_rffc5071_read(device: *mut hackrf_device, register_number: u8, value: *mut u16) -> hackrf_error;
    pub fn hackrf_rffc5071_write(device: *mut hackrf_device, register_number: u8, value: u16) -> hackrf_error;
    pub fn hackrf_spiflash_erase(device: *mut hackrf_device) -> hackrf_error;
    pub fn hackrf_spiflash_write(device: *mut hackrf_device, address: u32, length: u16, data: *mut c_uchar) -> hackrf_error;
    pub fn hackrf_spiflash_read(device: *mut hackrf_device, address: u32, length: u16, data: *mut c_uchar) -> hackrf_error;
    pub fn hackrf_cpld_write(device: *mut hackrf_device, data: *mut c_uchar, total_length: c_uint) -> hackrf_error;
    pub fn hackrf_board_id_read(device: *mut hackrf_device, value: *mut u8) -> hackrf_error;
    pub fn hackrf_version_string_read(device: *mut hackrf_device, version: *mut c_char, length: u8) -> hackrf_error;
    pub fn hackrf_set_freq(device: *mut hackrf_device, freq_hz: u64) -> hackrf_error;
    pub fn hackrf_set_freq_explicit(device: *mut hackrf_device, if_freq_hz: u64, lo_freq_hz: u64, path: rf_path_filter) -> hackrf_error;
    pub fn hackrf_set_sample_rate_manual(device: *mut hackrf_device, freq_hz: u32, divider: u32) -> hackrf_error;
    pub fn hackrf_set_sample_rate(device: *mut hackrf_device, freq_hz: c_double) -> hackrf_error;
    pub fn hackrf_set_amp_enable(device: *mut hackrf_device, value: u8) -> hackrf_error;
    pub fn hackrf_board_partid_serialno_read(device: *mut hackrf_device, read_partid_serialno: *mut read_partid_serialno_t) -> hackrf_error;
    pub fn hackrf_set_lna_gain(device: *mut hackrf_device, value: u32) -> hackrf_error;
    pub fn hackrf_set_vga_gain(device: *mut hackrf_device, value: u32) -> hackrf_error;
    pub fn hackrf_set_txvga_gain(device: *mut hackrf_device, value: u32) -> hackrf_error;
    pub fn hackrf_set_antenna_enable(device: *mut hackrf_device, value: u8) -> hackrf_error;
    pub fn hackrf_error_name(errcode: hackrf_error) -> *const c_char;
    pub fn hackrf_board_id_name(board_id: hackrf_board_id) -> *const c_char;
    pub fn hackrf_usb_board_id_name(usb_board_id: hackrf_usb_board_id) -> *const c_char;
    pub fn hackrf_filter_path_name(path: rf_path_filter) -> *const c_char;
    pub fn hackrf_compute_baseband_filter_bw_round_down_lt(bandwidth_hz: u32) -> u32;
    pub fn hackrf_compute_baseband_filter_bw(bandwidth_hz: u32) -> u32;
}
