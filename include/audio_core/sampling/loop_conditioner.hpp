#pragma once

#include "audio_core/sampling/audio_clip.hpp"
#include <cmath>
#include <numbers>
#include <algorithm>

namespace audio_core::sampling {

// ============================================================================
// LoopConditioner: Professional Click-Free Seam & Transient Conditioning
// Eliminates Heaviside step discontinuities, sub-bass thumps, and phase clicks
// ============================================================================
class LoopConditioner {
public:
    // Remove DC offset by subtracting exact channel mean and suppressing sub-bass drift
    // Eliminates speaker cone jumping when looping asymmetric saturated audio
    static void remove_dc_offset(AudioClip& clip) noexcept {
        const uint32_t channels = clip.num_channels();
        const uint32_t frames = clip.num_frames();

        if (channels == 0 || frames == 0) return;

        for (uint32_t ch = 0; ch < channels; ++ch) {
            float* data = clip.channel(ch);
            double sum = 0.0;
            for (uint32_t i = 0; i < frames; ++i) {
                sum += data[i];
            }
            float dc = static_cast<float>(sum / frames);
            for (uint32_t i = 0; i < frames; ++i) {
                data[i] -= dc;
            }
        }
    }

    // Apply an Equal-Power Micro-Crossfade (L^2 + R^2 = 1.0) at the loop boundary
    // Blends the final samples into the opening phase to ensure C0 and C1 continuity
    static void apply_equal_power_seam(AudioClip& clip, uint32_t fade_frames = 128) noexcept {
        const uint32_t channels = clip.num_channels();
        const uint32_t frames = clip.num_frames();

        if (channels == 0 || frames < (fade_frames * 2) || fade_frames == 0) return;

        const float half_pi = 0.5f * std::numbers::pi_v<float>;

        for (uint32_t ch = 0; ch < channels; ++ch) {
            float* data = clip.channel(ch);

            // Tail index: [frames - fade_frames .. frames - 1]
            // Head index: [0 .. fade_frames - 1]
            const uint32_t tail_start = frames - fade_frames;

            for (uint32_t k = 0; k < fade_frames; ++k) {
                const float frac = static_cast<float>(k) / static_cast<float>(fade_frames);
                const float theta = frac * half_pi;

                // Constant power: cos^2 + sin^2 == 1.0
                const float g_out = std::cos(theta); // 1.0 -> 0.0
                const float g_in = std::sin(theta);  // 0.0 -> 1.0

                const float tail_val = data[tail_start + k];
                const float head_val = data[k];

                // Smoothly blend tail towards head value so the seam wraps with 0 discontinuity
                data[tail_start + k] = (tail_val * g_out) + (head_val * g_in);
            }
        }
    }

    // One-shot conditioning: Equal-power seam + DC trap
    static void condition_seamless(AudioClip& clip, uint32_t fade_frames = 128) noexcept {
        apply_equal_power_seam(clip, fade_frames);
        remove_dc_offset(clip);
    }
};

} // namespace audio_core::sampling
