#![no_std]
use core::panic::PanicInfo;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

const MAX_FRAMES: usize = 1024;
// Planar stereo buffers: Left channel [0..1024], Right channel [1024..2048]
static mut INPUT_BUFFER: [f32; MAX_FRAMES * 2] = [0.0; MAX_FRAMES * 2];
static mut OUTPUT_BUFFER: [f32; MAX_FRAMES * 2] = [0.0; MAX_FRAMES * 2];

// Parameters
static mut DRIVE: f32 = 2.0;
static mut TONE: f32 = 0.5;
static mut MIX: f32 = 1.0;
static mut LP_LEFT: f32 = 0.0;
static mut LP_RIGHT: f32 = 0.0;

#[no_mangle]
pub extern "C" fn dsp_init(_sample_rate: u32) {
    unsafe {
        LP_LEFT = 0.0;
        LP_RIGHT = 0.0;
    }
}

#[no_mangle]
pub extern "C" fn dsp_get_input_buffer() -> *mut f32 {
    unsafe { INPUT_BUFFER.as_mut_ptr() }
}

#[no_mangle]
pub extern "C" fn dsp_get_output_buffer() -> *const f32 {
    unsafe { OUTPUT_BUFFER.as_ptr() }
}

#[no_mangle]
pub extern "C" fn dsp_set_param(param_id: u32, value: f32) {
    unsafe {
        match param_id {
            1 => DRIVE = if value < 0.1 { 0.1 } else if value > 20.0 { 20.0 } else { value },
            2 => TONE = if value < 0.01 { 0.01 } else if value > 0.99 { 0.99 } else { value },
            3 => MIX = if value < 0.0 { 0.0 } else if value > 1.0 { 1.0 } else { value },
            _ => {}
        }
    }
}

#[no_mangle]
pub extern "C" fn dsp_get_param(param_id: u32) -> f32 {
    unsafe {
        match param_id {
            1 => DRIVE,
            2 => TONE,
            3 => MIX,
            _ => 0.0,
        }
    }
}

#[inline(always)]
fn soft_saturate(x: f32) -> f32 {
    // Pade approximant of tanh(x)
    if x > 3.0 {
        1.0
    } else if x < -3.0 {
        -1.0
    } else {
        let x2 = x * x;
        (x * (27.0 + x2)) / (27.0 + 9.0 * x2)
    }
}

#[no_mangle]
pub extern "C" fn dsp_process(num_frames: u32) {
    let frames = if (num_frames as usize) > MAX_FRAMES { MAX_FRAMES } else { num_frames as usize };
    unsafe {
        let drive = DRIVE;
        let tone_alpha = TONE;
        let mix = MIX;
        let dry_mix = 1.0 - mix;

        for i in 0..frames {
            let in_l = INPUT_BUFFER[i];
            let in_r = INPUT_BUFFER[MAX_FRAMES + i];

            // Overdrive + soft saturate
            let sat_l = soft_saturate(in_l * drive);
            let sat_r = soft_saturate(in_r * drive);

            // Simple 1-pole lowpass tone filter
            LP_LEFT += tone_alpha * (sat_l - LP_LEFT);
            LP_RIGHT += tone_alpha * (sat_r - LP_RIGHT);

            // Dry/Wet mix
            OUTPUT_BUFFER[i] = (in_l * dry_mix) + (LP_LEFT * mix);
            OUTPUT_BUFFER[MAX_FRAMES + i] = (in_r * dry_mix) + (LP_RIGHT * mix);
        }
    }
}
