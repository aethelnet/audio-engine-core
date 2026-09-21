#pragma once

#include "audio_core/sampling/audio_clip.hpp"
#include <vector>
#include <cmath>
#include <cstdint>
#include <algorithm>

namespace audio_core::sampling {

// ============================================================================
// SampleRepairEngine: Heaviside Discontinuity Inpainting & Zero-Crossing Snapper
// Heals mid-wave chop clicks while providing creative click bypass for lo-fi aesthetics.
// ============================================================================
class SampleRepairEngine {
public:
    struct Discontinuity {
        uint32_t channel{0};
        uint32_t frame_index{0};
        float step_magnitude{0.0f};
    };

    // Detect abrupt step discontinuities (mid-cycle cuts / Heaviside jumps)
    // threshold: step jump amplitude (default 0.20 = 20% full-scale jump within 1 sample)
    static std::vector<Discontinuity> detect_discontinuities(const AudioClip& clip, float threshold = 0.20f) {
        std::vector<Discontinuity> results;
        const uint32_t channels = clip.num_channels();
        const uint32_t frames = clip.num_frames();
        if (channels == 0 || frames < 4) return results;

        for (uint32_t ch = 0; ch < channels; ++ch) {
            const float* data = clip.channel(ch);
            for (uint32_t i = 1; i < frames - 1; ++i) {
                float delta = std::abs(data[i] - data[i - 1]);
                if (delta >= threshold) {
                    // Check local curvature to confirm an isolated step jump rather than high-frequency noise
                    float d_prev = std::abs(data[i - 1] - (i > 1 ? data[i - 2] : 0.0f));
                    float d_next = std::abs((i + 2 < frames ? data[i + 2] : data[i + 1]) - data[i + 1]);

                    // An isolated cut typically has a jump much larger than surrounding inter-sample deltas
                    if (delta > d_prev * 1.8f || delta > d_next * 1.8f || delta > 0.40f) {
                        results.push_back(Discontinuity{
                            .channel = ch,
                            .frame_index = i,
                            .step_magnitude = delta
                        });
                        // Skip forward past the click window
                        i += 8;
                    }
                }
            }
        }
        return results;
    }

    // Cubic Hermite $C^1$ Inpainting of a single seam discontinuity
    // Replaces the jump with a continuous spline matching amplitude AND slope on both sides
    static void inpaint_seam(float* buffer, uint32_t frames, uint32_t cut_pos, uint32_t radius = 24) noexcept {
        if (!buffer || frames == 0 || radius == 0) return;
        if (cut_pos < radius + 2 || cut_pos + radius + 2 >= frames) return;

        const uint32_t start = cut_pos - radius;
        const uint32_t end   = cut_pos + radius;
        const uint32_t span  = end - start;

        // Boundary points & slopes
        const float x0 = buffer[start];
        const float x1 = buffer[end];
        const float m0 = (buffer[start] - buffer[start - 1]) * static_cast<float>(span);
        const float m1 = (buffer[end + 1] - buffer[end]) * static_cast<float>(span);

        for (uint32_t k = 0; k <= span; ++k) {
            const float t = static_cast<float>(k) / static_cast<float>(span);
            const float t2 = t * t;
            const float t3 = t2 * t;

            // Hermite basis functions
            const float h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
            const float h10 = t3 - 2.0f * t2 + t;
            const float h01 = -2.0f * t3 + 3.0f * t2;
            const float h11 = t3 - t2;

            buffer[start + k] = h00 * x0 + h10 * m0 + h01 * x1 + h11 * m1;
        }
    }

    // Find the nearest zero crossing with matching slope
    static uint32_t find_nearest_zero_crossing(const float* buffer, uint32_t frames,
                                               uint32_t target_pos, uint32_t search_radius = 64,
                                               bool match_positive_slope = true) noexcept {
        if (!buffer || frames < 2) return target_pos;
        if (target_pos >= frames) target_pos = frames - 1;

        uint32_t min_idx = (target_pos > search_radius) ? (target_pos - search_radius) : 0;
        uint32_t max_idx = std::min<uint32_t>(frames - 2, target_pos + search_radius);

        uint32_t best_pos = target_pos;
        uint32_t best_dist = UINT32_MAX;

        for (uint32_t i = min_idx; i <= max_idx; ++i) {
            // Check for zero-crossing: sign change or exact zero
            if ((buffer[i] <= 0.0f && buffer[i + 1] >= 0.0f) ||
                (buffer[i] >= 0.0f && buffer[i + 1] <= 0.0f)) {
                bool is_positive_slope = (buffer[i + 1] >= buffer[i]);
                if (is_positive_slope == match_positive_slope || std::abs(buffer[i]) < 1e-4f) {
                    uint32_t dist = (i >= target_pos) ? (i - target_pos) : (target_pos - i);
                    if (dist < best_dist) {
                        best_dist = dist;
                        best_pos = (std::abs(buffer[i]) < std::abs(buffer[i + 1])) ? i : (i + 1);
                    }
                }
            }
        }
        return best_pos;
    }

    // Auto-heal all detected discontinuities across an AudioClip
    // Returns number of healed discontinuities
    static uint32_t heal_clip(AudioClip& clip, float threshold = 0.20f,
                              uint32_t heal_radius = 24, bool creative_click_bypass = false) {
        if (creative_click_bypass) {
            // User wants the raw lo-fi / MPC chopping clicks! Do not touch the waveform.
            return 0;
        }

        auto discontinuities = detect_discontinuities(clip, threshold);
        for (const auto& d : discontinuities) {
            float* channel_data = clip.channel(d.channel);
            if (channel_data) {
                inpaint_seam(channel_data, clip.num_frames(), d.frame_index, heal_radius);
            }
        }
        return static_cast<uint32_t>(discontinuities.size());
    }

    // Snap all slice markers in the AudioClip to the nearest zero-crossing
    static void snap_all_slices_to_zero_crossings(AudioClip& clip, uint32_t search_radius = 64) {
        auto& slices = clip.slices();
        const float* src = clip.channel(0);
        const uint32_t frames = clip.num_frames();
        if (!src || frames == 0) return;

        for (auto& s : slices) {
            if (s.start_frame > 0) {
                s.start_frame = find_nearest_zero_crossing(src, frames, s.start_frame, search_radius, true);
            }
            if (s.end_frame < frames) {
                s.end_frame = find_nearest_zero_crossing(src, frames, s.end_frame, search_radius, false);
            }
        }
    }
};

} // namespace audio_core::sampling
