#include "audio_core/dsp/baxandall.hpp"
#include "audio_core/dsp/buttercomp2.hpp"
#include "audio_core/dsp/clip_only2.hpp"
#include "audio_core/dsp/purest_drive.hpp"

#include <cmath>
#include <algorithm>
#include <numbers>

static constexpr size_t kMaxFrames = 4096;
static float g_left_buffer[kMaxFrames];
static float g_right_buffer[kMaxFrames];

// Real-Time C++20 Airwindows DSP Instances
static audio_core::dsp::PurestDrive g_drive;
static audio_core::dsp::Baxandall g_baxandall;
static audio_core::dsp::ButterComp2 g_comp;
static audio_core::dsp::ClipOnly2 g_limiter;

static uint32_t g_sample_rate = 48000;
static float g_drive_amount = 0.4f;
static bool g_drive_enabled = true;
static bool g_eq_enabled = true;
static bool g_comp_enabled = true;
static bool g_limiter_enabled = true;

// Synthesizer voice for standalone testing
static double g_phase = 0.0;
static float g_synth_freq = 110.0f; // A2
static bool g_synth_active = false;
static int g_synth_waveform = 0; // 0: Saw, 1: Square, 2: Sine

extern "C" {

void wasm_init(uint32_t sample_rate) {
    g_sample_rate = sample_rate > 0 ? sample_rate : 48000;
    g_drive.init(g_sample_rate);
    g_baxandall.init(g_sample_rate);
    g_comp.init(g_sample_rate);
    g_limiter.init(g_sample_rate);
    
    // Default musical parameters
    g_drive_amount = 0.4f;
    g_drive.set_parameter(0, g_drive_amount);     // 40% saturation
    g_baxandall.set_parameter(0, 3.0f); // +3dB Bass warmth
    g_baxandall.set_parameter(1, 1.5f); // +1.5dB Treble sheen
    g_comp.set_parameter(0, 0.35f);    // Smooth compression
    g_comp.set_parameter(1, 1.05f);    // Output makeup
}

float* wasm_get_left_buffer() { return g_left_buffer; }
float* wasm_get_right_buffer() { return g_right_buffer; }

void wasm_set_drive(float amount) {
    g_drive_amount = std::clamp(amount, 0.0f, 1.0f);
    g_drive.set_parameter(0, g_drive_amount);
}

void wasm_set_bass(float db) {
    g_baxandall.set_parameter(0, std::clamp(db, -15.0f, 15.0f));
}

void wasm_set_treble(float db) {
    g_baxandall.set_parameter(1, std::clamp(db, -15.0f, 15.0f));
}

void wasm_set_comp(float amount) {
    g_comp.set_parameter(0, std::clamp(amount, 0.0f, 1.0f));
}

void wasm_set_limiter(int enable) {
    g_limiter_enabled = (enable != 0);
}

void wasm_set_synth(int active, float freq, int waveform) {
    g_synth_active = (active != 0);
    g_synth_freq = std::clamp(freq, 20.0f, 20000.0f);
    g_synth_waveform = waveform;
}

void wasm_process(uint32_t frames) {
    if (frames > kMaxFrames) frames = kMaxFrames;

    // 1. If internal synth voice is active, generate oscillator signal
    if (g_synth_active) {
        const double phase_inc = (static_cast<double>(g_synth_freq) * 2.0 * std::numbers::pi) / static_cast<double>(g_sample_rate);
        for (uint32_t i = 0; i < frames; ++i) {
            float s = 0.0f;
            if (g_synth_waveform == 0) {
                // Naive Sawtooth
                s = static_cast<float>(1.0 - (g_phase / std::numbers::pi));
            } else if (g_synth_waveform == 1) {
                // Square wave
                s = (g_phase < std::numbers::pi) ? 0.65f : -0.65f;
            } else {
                // Pure Sine wave
                s = static_cast<float>(std::sin(g_phase));
            }
            g_left_buffer[i] = s * 0.35f;
            g_right_buffer[i] = s * 0.35f;

            g_phase += phase_inc;
            if (g_phase >= 2.0 * std::numbers::pi) {
                g_phase -= 2.0 * std::numbers::pi;
            }
        }
    }

    // 2. Sequential Airwindows DSP processing in exact order:
    // PurestDrive -> Baxandall EQ -> ButterComp2 -> ClipOnly2
    if (g_drive_enabled) {
        if (g_drive_amount > 0.001f) {
            const float in_gain = 1.0f + 3.0f * g_drive_amount;
            const float out_gain = 1.0f / (1.0f + 1.2f * g_drive_amount);
            for (uint32_t i = 0; i < frames; ++i) {
                g_left_buffer[i] *= in_gain;
                g_right_buffer[i] *= in_gain;
            }
            g_drive.process_stereo(g_left_buffer, g_right_buffer, frames);
            for (uint32_t i = 0; i < frames; ++i) {
                g_left_buffer[i] *= out_gain;
                g_right_buffer[i] *= out_gain;
            }
        } else {
            g_drive.process_stereo(g_left_buffer, g_right_buffer, frames);
        }
    }
    if (g_eq_enabled) {
        g_baxandall.process_stereo(g_left_buffer, g_right_buffer, frames);
    }
    if (g_comp_enabled) {
        g_comp.process_stereo(g_left_buffer, g_right_buffer, frames);
    }
    if (g_limiter_enabled) {
        g_limiter.process_stereo(g_left_buffer, g_right_buffer, frames);
    }
}

} // extern "C"
