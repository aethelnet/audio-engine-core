#![no_std]
use core::panic::PanicInfo;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

const MAX_FRAMES: usize = 1024;
static mut INPUT_BUFFER: [f32; MAX_FRAMES * 2] = [0.0; MAX_FRAMES * 2];
static mut OUTPUT_BUFFER: [f32; MAX_FRAMES * 2] = [0.0; MAX_FRAMES * 2];

// Param 1: Rogue Mode
// 0 = Normal bypass
// 1 = Produce NaN / Infs (testing acoustic sanitizer and circuit breaker)
// 2 = Infinite loop (testing gas limit watchdog)
// 3 = Extreme blowup (+100.0 amplitudes testing acoustic clamp)
static mut MODE: u32 = 0;

static PARAM_MODE_NAME: &[u8] = b"RogueMode\0";

#[no_mangle]
pub extern "C" fn sov_init(_sample_rate: u32) {
    unsafe {
        MODE = 0;
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
    1
}

#[no_mangle]
pub extern "C" fn sov_get_param_name(_param_id: u32) -> *const u8 {
    PARAM_MODE_NAME.as_ptr()
}

#[no_mangle]
pub extern "C" fn sov_set_param(param_id: u32, value: f32) {
    unsafe {
        if param_id == 1 {
            MODE = value as u32;
        }
    }
}

#[no_mangle]
pub extern "C" fn sov_get_param(param_id: u32) -> f32 {
    unsafe {
        if param_id == 1 {
            MODE as f32
        } else {
            0.0
        }
    }
}

#[no_mangle]
pub extern "C" fn sov_process(num_frames: u32) {
    let frames = if (num_frames as usize) > MAX_FRAMES { MAX_FRAMES } else { num_frames as usize };
    unsafe {
        match MODE {
            1 => {
                // Produce NaNs in output
                let nan_val = 0.0 / 0.0;
                for i in 0..frames {
                    OUTPUT_BUFFER[i] = nan_val;
                    OUTPUT_BUFFER[MAX_FRAMES + i] = nan_val;
                }
            }
            2 => {
                // Infinite loop: runaway plugin
                #[allow(clippy::empty_loop)]
                loop {}
            }
            3 => {
                // Massive blowup (+100.0)
                for i in 0..frames {
                    OUTPUT_BUFFER[i] = 100.0;
                    OUTPUT_BUFFER[MAX_FRAMES + i] = -100.0;
                }
            }
            _ => {
                // Clean passthrough
                for i in 0..frames {
                    OUTPUT_BUFFER[i] = INPUT_BUFFER[i];
                    OUTPUT_BUFFER[MAX_FRAMES + i] = INPUT_BUFFER[MAX_FRAMES + i];
                }
            }
        }
    }
}
