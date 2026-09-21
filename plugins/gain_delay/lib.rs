#![no_std]
use core::panic::PanicInfo;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

const MAX_FRAMES: usize = 1024;
static mut INPUT_BUFFER: [f32; MAX_FRAMES * 2] = [0.0; MAX_FRAMES * 2];
static mut OUTPUT_BUFFER: [f32; MAX_FRAMES * 2] = [0.0; MAX_FRAMES * 2];

// Sovereign ABI parameter storage
static mut GAIN: f32 = 1.0;
static mut FEEDBACK: f32 = 0.3;
const DELAY_CAPACITY: usize = 4800; // 100ms buffer at 48kHz
static mut DELAY_BUF_L: [f32; DELAY_CAPACITY] = [0.0; DELAY_CAPACITY];
static mut DELAY_BUF_R: [f32; DELAY_CAPACITY] = [0.0; DELAY_CAPACITY];
static mut WRITE_POS: usize = 0;

static PARAM_GAIN_NAME: &[u8] = b"Gain\0";
static PARAM_FB_NAME: &[u8] = b"Feedback\0";
static PARAM_UNKNOWN_NAME: &[u8] = b"Unknown\0";

#[no_mangle]
pub extern "C" fn sov_init(_sample_rate: u32) {
    unsafe {
        WRITE_POS = 0;
        for i in 0..DELAY_CAPACITY {
            DELAY_BUF_L[i] = 0.0;
            DELAY_BUF_R[i] = 0.0;
        }
    }
}

#[no_mangle]
pub extern "C" fn sov_get_input_buffer() -> *mut f32 {
    core::ptr::addr_of_mut!(INPUT_BUFFER) as *mut f32
}

#[no_mangle]
pub extern "C" fn sov_get_output_buffer() -> *const f32 {
    core::ptr::addr_of!(OUTPUT_BUFFER) as *const f32
}

#[no_mangle]
pub extern "C" fn sov_get_num_params() -> u32 {
    2
}

#[no_mangle]
pub extern "C" fn sov_get_param_name(param_id: u32) -> *const u8 {
    match param_id {
        1 => PARAM_GAIN_NAME.as_ptr(),
        2 => PARAM_FB_NAME.as_ptr(),
        _ => PARAM_UNKNOWN_NAME.as_ptr(),
    }
}

#[no_mangle]
pub extern "C" fn sov_set_param(param_id: u32, value: f32) {
    unsafe {
        match param_id {
            1 => GAIN = if value < 0.0 { 0.0 } else if value > 4.0 { 4.0 } else { value },
            2 => FEEDBACK = if value < 0.0 { 0.0 } else if value > 0.95 { 0.95 } else { value },
            _ => {}
        }
    }
}

#[no_mangle]
pub extern "C" fn sov_get_param(param_id: u32) -> f32 {
    unsafe {
        match param_id {
            1 => GAIN,
            2 => FEEDBACK,
            _ => 0.0,
        }
    }
}

#[no_mangle]
pub extern "C" fn sov_process(num_frames: u32) {
    let frames = if (num_frames as usize) > MAX_FRAMES { MAX_FRAMES } else { num_frames as usize };
    unsafe {
        let gain = GAIN;
        let fb = FEEDBACK;
        let delay_len = 2400; // 50ms delay at 48kHz

        for i in 0..frames {
            let in_l = INPUT_BUFFER[i] * gain;
            let in_r = INPUT_BUFFER[MAX_FRAMES + i] * gain;

            let read_pos = if WRITE_POS >= delay_len {
                WRITE_POS - delay_len
            } else {
                WRITE_POS + DELAY_CAPACITY - delay_len
            };

            let delayed_l = DELAY_BUF_L[read_pos];
            let delayed_r = DELAY_BUF_R[read_pos];

            DELAY_BUF_L[WRITE_POS] = in_l + delayed_l * fb;
            DELAY_BUF_R[WRITE_POS] = in_r + delayed_r * fb;

            WRITE_POS = (WRITE_POS + 1) % DELAY_CAPACITY;

            OUTPUT_BUFFER[i] = in_l + delayed_l;
            OUTPUT_BUFFER[MAX_FRAMES + i] = in_r + delayed_r;
        }
    }
}
