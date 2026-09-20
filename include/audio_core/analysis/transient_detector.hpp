#pragma once

#include <vector>
#include <cstdint>
#include <cmath>
#include <algorithm>

namespace audio_core::analysis {

struct TransientMarker {
    uint32_t sample_offset{0};
    float strength{0.0f};      // Peak transient energy [0..1]
};

struct AudioAnalysisResult {
    float peak_l{0.0f};
    float peak_r{0.0f};
    float rms_l{0.0f};
    float rms_r{0.0f};
    float dc_offset_l{0.0f};
    float dc_offset_r{0.0f};
    std::vector<TransientMarker> onsets;
};

// ============================================================================
// TransientDetector: Fast Sample-Accurate Audio Analysis & Onset Detection
// Detects drum hits, synth attacks, DC offsets, and slice points for loop rearranging
// ============================================================================
class TransientDetector {
public:
    explicit TransientDetector(uint32_t sample_rate = 48000)
        : m_sample_rate(sample_rate) {}

    void set_sample_rate(uint32_t sr) noexcept {
        if (sr > 0) m_sample_rate = sr;
    }

    // Full analysis of planar stereo audio data
    [[nodiscard]] AudioAnalysisResult analyze(const float* left, const float* right, uint32_t frames,
                                             float sensitivity = 0.5f) const {
        AudioAnalysisResult result{};
        if (!left || frames == 0) return result;

        const float* r = right ? right : left;

        float peak_l = 0.0f, peak_r = 0.0f;
        double sum_sq_l = 0.0, sum_sq_r = 0.0;
        double sum_l = 0.0, sum_r = 0.0;

        // 1. Basic energy and DC statistics
        for (uint32_t i = 0; i < frames; ++i) {
            float abs_l = std::abs(left[i]);
            float abs_r = std::abs(r[i]);
            if (abs_l > peak_l) peak_l = abs_l;
            if (abs_r > peak_r) peak_r = abs_r;

            sum_sq_l += left[i] * left[i];
            sum_sq_r += r[i] * r[i];
            sum_l += left[i];
            sum_r += r[i];
        }

        result.peak_l = peak_l;
        result.peak_r = peak_r;
        result.rms_l = static_cast<float>(std::sqrt(sum_sq_l / frames));
        result.rms_r = static_cast<float>(std::sqrt(sum_sq_r / frames));
        result.dc_offset_l = static_cast<float>(sum_l / frames);
        result.dc_offset_r = static_cast<float>(sum_r / frames);

        // 2. Dual-Envelope Energy & HF-Flux Tracking for Onset Detection
        // Min refractory period between consecutive onsets (e.g. 25ms @ 48kHz = 1200 samples)
        const uint32_t min_interval_samples = (m_sample_rate * 25) / 1000;
        const float threshold = std::max(0.01f, (1.0f - sensitivity) * 0.1f);

        std::vector<float> odf(frames, 0.0f);
        float env_fast = 0.0f;
        float env_slow = 0.0f;

        const float alpha_fast = 0.15f;
        const float alpha_slow = 0.005f;

        for (uint32_t i = 0; i < frames; ++i) {
            float inst_amp = 0.5f * (std::abs(left[i]) + std::abs(r[i]));
            float prev_l = (i > 0) ? left[i - 1] : left[0];
            float prev_r = (i > 0) ? r[i - 1] : r[0];
            float hf_flux = 0.5f * (std::abs(left[i] - prev_l) + std::abs(r[i] - prev_r));

            float signal = inst_amp + 2.0f * hf_flux;
            env_fast += alpha_fast * (signal - env_fast);
            env_slow += alpha_slow * (signal - env_slow);

            float diff = env_fast - env_slow;
            odf[i] = diff > 0.0f ? diff : 0.0f;
        }

        // 3. Peak picking on ODF with onset start backtracking
        uint32_t last_onset = 0;

        for (uint32_t i = 1; i < frames - 1; ++i) {
            if (odf[i] > threshold && odf[i] > odf[i - 1] && odf[i] >= odf[i + 1]) {
                if (result.onsets.empty() || (i - last_onset) >= min_interval_samples) {
                    // Backtrack to the start of the rise for sample-accurate onset alignment
                    uint32_t onset_start = i;
                    float noise_floor = threshold * 0.2f;
                    while (onset_start > 0 && odf[onset_start] > noise_floor && (i - onset_start) < 200) {
                        onset_start--;
                    }
                    result.onsets.push_back(TransientMarker{
                        .sample_offset = onset_start,
                        .strength = std::min(1.0f, odf[i] / (threshold + 0.1f))
                    });
                    last_onset = i;
                }
            }
        }

        return result;
    }

    // Helper: Find nearest zero crossing to avoid clicks during slicing
    [[nodiscard]] static uint32_t find_nearest_zero_crossing(const float* channel, uint32_t center_sample,
                                                             uint32_t max_search_radius, uint32_t total_frames) noexcept {
        if (!channel || total_frames == 0) return center_sample;

        uint32_t best_sample = center_sample;
        float min_abs = std::abs(channel[std::min(center_sample, total_frames - 1)]);

        uint32_t start = (center_sample > max_search_radius) ? (center_sample - max_search_radius) : 0;
        uint32_t end = std::min(total_frames - 1, center_sample + max_search_radius);

        for (uint32_t i = start; i < end; ++i) {
            // Check for true zero-crossing (sign change)
            if ((channel[i] <= 0.0f && channel[i + 1] >= 0.0f) || (channel[i] >= 0.0f && channel[i + 1] <= 0.0f)) {
                return (std::abs(channel[i]) < std::abs(channel[i + 1])) ? i : (i + 1);
            }
            float val = std::abs(channel[i]);
            if (val < min_abs) {
                min_abs = val;
                best_sample = i;
            }
        }
        return best_sample;
    }

private:
    uint32_t m_sample_rate{48000};
};

} // namespace audio_core::analysis
