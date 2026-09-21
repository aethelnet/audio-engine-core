#![no_std]
use core::panic::PanicInfo;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

const MAX_FRAMES: usize = 1024;
static mut INPUT_BUFFER: [f32; MAX_FRAMES * 2] = [0.0; MAX_FRAMES * 2];
static mut OUTPUT_BUFFER: [f32; MAX_FRAMES * 2] = [0.0; MAX_FRAMES * 2];
static mut SIDECHAIN_BUFFER: [f32; MAX_FRAMES * 2] = [0.0; MAX_FRAMES * 2];

// Param 1: Ducking Depth (0.0 to 1.0, default 0.8)
// Param 2: Threshold (0.0 to 1.0, default 0.1)
static mut DUCK_DEPTH: f32 = 0.8;
static mut THRESHOLD: f32 = 0.1;
static mut ENVELOPE: f32 = 0.0;

static PARAM_DUCK_NAME: &[u8] = b"Ducking\0";
static PARAM_THRESH_NAME: &[u8] = b"Threshold\0";
static PARAM_UNKNOWN: &[u8] = b"Unknown\0";

#[no_mangle]
pub extern "C" fn sov_init(_sample_rate: u32) {
    unsafe {
        ENVELOPE = 0.0;
        DUCK_DEPTH = 0.8;
        THRESHOLD = 0.1;
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
pub extern "C" fn sov_get_sidechain_buffer() -> *mut f32 {
    core::ptr::addr_of_mut!(SIDECHAIN_BUFFER) as *mut f32
}

#[no_mangle]
pub extern "C" fn sov_supports_sidechain() -> u32 {
    1
}

#[no_mangle]
pub extern "C" fn sov_get_num_params() -> u32 {
    2
}

#[no_mangle]
pub extern "C" fn sov_get_param_name(param_id: u32) -> *const u8 {
    match param_id {
        1 => PARAM_DUCK_NAME.as_ptr(),
        2 => PARAM_THRESH_NAME.as_ptr(),
        _ => PARAM_UNKNOWN.as_ptr(),
    }
}

#[no_mangle]
pub extern "C" fn sov_set_param(param_id: u32, value: f32) {
    unsafe {
        match param_id {
            1 => DUCK_DEPTH = if value < 0.0 { 0.0 } else if value > 1.0 { 1.0 } else { value },
            2 => THRESHOLD = if value < 0.001 { 0.001 } else if value > 1.0 { 1.0 } else { value },
            _ => {}
        }
    }
}

#[no_mangle]
pub extern "C" fn sov_get_param(param_id: u32) -> f32 {
    unsafe {
        match param_id {
            1 => DUCK_DEPTH,
            2 => THRESHOLD,
            _ => 0.0,
        }
    }
}

#[inline(always)]
fn abs_f32(x: f32) -> f32 {
    if x < 0.0 { -x } else { x }
}

#[no_mangle]
pub extern "C" fn sov_process(num_frames: u32) {
    // Standard stereo processing (no sidechain active: passthrough)
    let frames = if (num_frames as usize) > MAX_FRAMES { MAX_FRAMES } else { num_frames as usize };
    unsafe {
        for i in 0..frames {
            OUTPUT_BUFFER[i] = INPUT_BUFFER[i];
            OUTPUT_BUFFER[MAX_FRAMES + i] = INPUT_BUFFER[MAX_FRAMES + i];
        }
    }
}

#[no_mangle]
pub extern "C" fn sov_process_sidechain(num_frames: u32) {
    let frames = if (num_frames as usize) > MAX_FRAMES { MAX_FRAMES } else { num_frames as usize };
    unsafe {
        let depth = DUCK_DEPTH;
        let thresh = THRESHOLD;
        let mut env = ENVELOPE;

        for i in 0..frames {
            let sc_l = abs_f32(SIDECHAIN_BUFFER[i]);
            let sc_r = abs_f32(SIDECHAIN_BUFFER[MAX_FRAMES + i]);
            let sc_peak = if sc_l > sc_r { sc_l } else { sc_r };

            // Fast attack, smooth release envelope follower
            if sc_peak > env {
                env += 0.5 * (sc_peak - env);
            } else {
                env += 0.05 * (sc_peak - env);
            }

            let over = if env > thresh { env - thresh } else { 0.0 };
            let duck = if over * 2.0 > 1.0 { 1.0 } else { over * 2.0 };
            let gain = 1.0 - (depth * duck);

            OUTPUT_BUFFER[i] = INPUT_BUFFER[i] * gain;
            OUTPUT_BUFFER[MAX_FRAMES + i] = INPUT_BUFFER[MAX_FRAMES + i] * gain;
        }

        ENVELOPE = env;
    }
}
