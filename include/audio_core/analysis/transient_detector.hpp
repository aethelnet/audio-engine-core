#pragma once

#include <vector>
#include <cstdint>
#include <cmath>
#include <algorithm>

namespace audio_core::analysis {

enum class TransientBand : uint8_t {
    Broadband = 0,
    SubBass = 1,      // < 180 Hz (Kicks, 808 sub thumps)
    MidPunch = 2,     // 180 Hz - 3.5 kHz (Snares, Claps, Rimshots, Toms, Plucks)
    HighCrisp = 3     // > 3.5 kHz (Hi-Hats, Shakers, Cymbals, Acoustic Ticks)
};

struct TransientSpan {
    uint32_t start_frame{0};      // Backtracked onset start (zero-crossing aligned)
    uint32_t peak_frame{0};       // Frame of maximum instantaneous impulsive pressure
    uint32_t decay_end_frame{0};  // Frame where transient burst decays into stationary sustain (zero-crossing aligned)
    float strength{0.0f};         // Peak transient energy [0..1]
    float crest_factor{0.0f};     // Peak / RMS ratio within the transient span
    TransientBand band{TransientBand::Broadband};

    [[nodiscard]] uint32_t length_frames() const noexcept {
        return (decay_end_frame > start_frame) ? (decay_end_frame - start_frame) : 0;
    }
};

struct TransientMarker {
    uint32_t sample_offset{0};    // Kept for backward compatibility with existing code & tests
    float strength{0.0f};         // Peak transient energy [0..1]
    uint32_t peak_offset{0};
    uint32_t decay_end_offset{0};
    float crest_factor{0.0f};
    TransientBand band{TransientBand::Broadband};

    [[nodiscard]] TransientSpan to_span() const noexcept {
        return TransientSpan{
            .start_frame = sample_offset,
            .peak_frame = peak_offset ? peak_offset : sample_offset,
            .decay_end_frame = decay_end_offset ? decay_end_offset : (sample_offset + 480),
            .strength = strength,
            .crest_factor = crest_factor,
            .band = band
        };
    }
};

struct AudioAnalysisResult {
    float peak_l{0.0f};
    float peak_r{0.0f};
    float rms_l{0.0f};
    float rms_r{0.0f};
    float dc_offset_l{0.0f};
    float dc_offset_r{0.0f};
    std::vector<TransientMarker> onsets;
    std::vector<TransientSpan> transient_spans;
};

// ============================================================================
// TransientDetector: Multiband Sample-Accurate Audio Analysis & Onset Detection
// Detects drum hits, synth attacks, DC offsets, and slice points for loop rearranging
// ============================================================================
class TransientDetector {
public:
    explicit TransientDetector(uint32_t sample_rate = 48000)
        : m_sample_rate(sample_rate) {}

    void set_sample_rate(uint32_t sr) noexcept {
        if (sr > 0) m_sample_rate = sr;
    }

    void set_min_interval_ms(float ms) noexcept {
        m_min_interval_ms = std::max(2.0f, ms);
    }

    [[nodiscard]] float min_interval_ms() const noexcept {
        return m_min_interval_ms;
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

        // 2. 3-Band Complementary Filterbank (SubBass < 180Hz, MidPunch 180-3500Hz, HighCrisp > 3500Hz)
        const float sr = static_cast<float>(m_sample_rate);
        const float fc_sub = 180.0f;
        const float fc_high = 3500.0f;

        const float alpha_lp1 = 1.0f - std::exp(-2.0f * std::numbers::pi_v<float> * fc_sub / sr);
        const float alpha_lp2 = 1.0f - std::exp(-2.0f * std::numbers::pi_v<float> * fc_high / sr);

        std::vector<float> mono(frames, 0.0f);
        std::vector<float> band_low(frames, 0.0f);
        std::vector<float> band_mid(frames, 0.0f);
        std::vector<float> band_high(frames, 0.0f);
        std::vector<float> odf(frames, 0.0f);

        float s_lp1 = 0.0f;
        float s_lp2 = 0.0f;

        for (uint32_t i = 0; i < frames; ++i) {
            float m = 0.5f * (left[i] + r[i]);
            mono[i] = m;

            // Complementary filterbank: y_low + y_mid + y_high == m exactly
            s_lp1 += alpha_lp1 * (m - s_lp1);
            s_lp2 += alpha_lp2 * (m - s_lp2);

            float y_low = s_lp1;
            float y_high = m - s_lp2;
            float y_mid = s_lp2 - s_lp1;

            band_low[i] = y_low;
            band_mid[i] = y_mid;
            band_high[i] = y_high;
        }

        // 3. Multiband Spectral Flux & Dual-Envelope Tracking
        float env_fast = 0.0f;
        float env_slow = 0.0f;
        const float alpha_fast = 0.15f;
        const float alpha_slow = 0.005f;

        float prev_low = 0.0f, prev_mid = 0.0f, prev_high = 0.0f;

        for (uint32_t i = 0; i < frames; ++i) {
            float cur_low = std::abs(band_low[i]);
            float cur_mid = std::abs(band_mid[i]);
            float cur_high = std::abs(band_high[i]);

            float flux_low = std::max(0.0f, cur_low - prev_low);
            float flux_mid = std::max(0.0f, cur_mid - prev_mid);
            float flux_high = std::max(0.0f, cur_high - prev_high);

            prev_low = cur_low;
            prev_mid = cur_mid;
            prev_high = cur_high;

            // Multiband Onset Function with high-frequency emphasis for crisp ticks
            float composite_signal = cur_mid + 2.0f * flux_mid
                                   + 1.2f * (cur_low + 2.0f * flux_low)
                                   + 2.5f * (cur_high + 3.0f * flux_high);

            env_fast += alpha_fast * (composite_signal - env_fast);
            env_slow += alpha_slow * (composite_signal - env_slow);

            float diff = env_fast - env_slow;
            odf[i] = (diff > 0.0f) ? diff : 0.0f;
        }

        // 4. Adaptive Peak Picking with Inter-Onset-Interval (IOI) Gate
        const float interval_ms = std::max(3.0f, m_min_interval_ms * (1.2f - 0.4f * sensitivity));
        const uint32_t min_interval_samples = static_cast<uint32_t>((m_sample_rate * interval_ms) / 1000.0f);
        const float threshold = std::max(0.008f, (1.0f - sensitivity) * 0.08f);

        uint32_t last_onset = 0;
        bool is_first = true;

        for (uint32_t i = 1; i < frames - 1; ++i) {
            if (odf[i] > threshold && odf[i] > odf[i - 1] && odf[i] >= odf[i + 1]) {
                if (is_first || (i - last_onset) >= min_interval_samples) {
                    // Backtrack to the start of the rise for sample-accurate onset alignment
                    uint32_t onset_start = i;
                    float noise_floor = threshold * 0.2f;
                    while (onset_start > 0 && odf[onset_start] > noise_floor && (i - onset_start) < 200) {
                        onset_start--;
                    }

                    // Snap to exact 0 if within 64 samples of beginning, otherwise snap to nearest zero-crossing
                    if (onset_start < 64) {
                        onset_start = 0;
                    } else {
                        onset_start = find_nearest_zero_crossing(mono.data(), onset_start, 16, frames);
                    }

                    // 1. Peak Frame: highest instantaneous magnitude in onset window
                    uint32_t peak_frame = onset_start;
                    float peak_amp = 0.0f;
                    uint32_t search_end = std::min(frames, onset_start + 400);
                    for (uint32_t s = onset_start; s < search_end; ++s) {
                        float a = std::abs(mono[s]);
                        if (a > peak_amp) {
                            peak_amp = a;
                            peak_frame = s;
                        }
                    }

                    // Advance onset_start if it was located in silence before the actual attack
                    uint32_t first_active = onset_start;
                    while (first_active < peak_frame && std::abs(mono[first_active]) < 1e-4f) {
                        first_active++;
                    }
                    if (first_active > onset_start && first_active <= peak_frame) {
                        onset_start = first_active;
                    }

                    // 2. Decay End Frame: search from peak_frame until amplitude drops below 25% of peak_amp
                    uint32_t max_decay = std::min(frames, peak_frame + std::min(1920u, (m_sample_rate * 40) / 1000));
                    uint32_t decay_end = peak_frame + std::min(240u, (m_sample_rate * 5) / 1000);
                    float decay_thresh = std::max(0.02f, 0.25f * peak_amp);
                    for (uint32_t s = peak_frame + 64; s < max_decay; ++s) {
                        if (std::abs(mono[s]) <= decay_thresh) {
                            decay_end = s;
                            break;
                        }
                        decay_end = s;
                    }
                    decay_end = find_nearest_zero_crossing(mono.data(), decay_end, 32, frames);
                    if (decay_end <= peak_frame) decay_end = std::min(frames, peak_frame + 240);

                    // 3. Crest factor and energy over [onset_start, decay_end]
                    double span_sum_sq = 0.0;
                    float span_peak = 0.0f;
                    double e_low = 0.0, e_mid = 0.0, e_high = 0.0;

                    for (uint32_t s = onset_start; s < decay_end; ++s) {
                        float val = mono[s];
                        span_sum_sq += val * val;
                        if (std::abs(val) > span_peak) span_peak = std::abs(val);

                        e_low += std::abs(band_low[s]);
                        e_mid += std::abs(band_mid[s]);
                        e_high += std::abs(band_high[s]);
                    }

                    uint32_t span_len = decay_end - onset_start;
                    float span_rms = (span_len > 0) ? static_cast<float>(std::sqrt(span_sum_sq / span_len)) : 0.0f;
                    float crest = (span_rms > 1e-5f) ? (span_peak / span_rms) : 1.0f;

                    // 4. Spectral Band classification
                    TransientBand band = TransientBand::Broadband;
                    if (e_low > 1.2 * (e_mid + e_high) && e_low > 1e-3) {
                        band = TransientBand::SubBass;
                    } else if (e_high > 1.2 * (e_mid + e_low) && e_high > 1e-3) {
                        band = TransientBand::HighCrisp;
                    } else {
                        band = TransientBand::MidPunch;
                    }

                    TransientSpan span{
                        .start_frame = onset_start,
                        .peak_frame = peak_frame,
                        .decay_end_frame = decay_end,
                        .strength = std::min(1.0f, odf[i] / (threshold + 0.1f)),
                        .crest_factor = crest,
                        .band = band
                    };

                    TransientMarker marker{
                        .sample_offset = onset_start,
                        .strength = span.strength,
                        .peak_offset = peak_frame,
                        .decay_end_offset = decay_end,
                        .crest_factor = crest,
                        .band = band
                    };

                    result.onsets.push_back(marker);
                    result.transient_spans.push_back(span);
                    last_onset = i;
                    is_first = false;
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
    float m_min_interval_ms{10.0f};
};

} // namespace audio_core::analysis
