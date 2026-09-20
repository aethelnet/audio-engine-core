#pragma once

#include "audio_core/types.hpp"
#include "audio_core/dsp/fft.hpp"
#include <vector>
#include <cmath>
#include <numbers>
#include <algorithm>
#include <string>
#include <numeric>

namespace audio_core::analysis {

// ============================================================================
// RoomMode: Detected Acoustic Low-Frequency Resonance
// ============================================================================
struct RoomMode {
    float frequency_hz{0.0f};
    float peak_gain_db{0.0f};
    float q_factor{0.0f};
    float recommended_notch_gain_db{0.0f}; // Suggested notch cut (-gain)
};

// ============================================================================
// FarinaSweepGenerator: Logarithmic Exponential Sine Sweep (ESS)
// Based on Angelo Farina's AES 2000 swept-sine impulse response method:
// - Generates excitation sweep s(t)
// - Generates inverse deconvolution filter k(t) with -6 dB/octave tilt
// - Separates non-linear harmonic distortion into negative time
// ============================================================================
class FarinaSweepGenerator {
public:
    struct SweepParams {
        float start_freq{20.0f};      // Starting frequency in Hz
        float stop_freq{20000.0f};    // Stop frequency in Hz
        float duration_sec{1.0f};     // Duration in seconds
        uint32_t sample_rate{48000};  // Sample rate in Hz
        float fade_sec{0.01f};        // Half-cosine fade in/out duration in seconds
    };

    FarinaSweepGenerator() noexcept = default;
    explicit FarinaSweepGenerator(const SweepParams& params) noexcept
        : m_params(params) {}

    void set_params(const SweepParams& params) noexcept {
        m_params = params;
    }

    [[nodiscard]] const SweepParams& params() const noexcept { return m_params; }

    // Generates the forward excitation sweep s(t)
    [[nodiscard]] std::vector<float> generate_sweep() const {
        const size_t total_samples = static_cast<size_t>(std::round(m_params.duration_sec * static_cast<float>(m_params.sample_rate)));
        if (total_samples == 0) return {};

        std::vector<float> sweep(total_samples);
        const float w1 = 2.0f * std::numbers::pi_v<float> * m_params.start_freq;
        const float w2 = 2.0f * std::numbers::pi_v<float> * m_params.stop_freq;
        const float T = m_params.duration_sec;
        const float L = T / std::log(w2 / w1);

        const size_t fade_samples = static_cast<size_t>(std::round(m_params.fade_sec * static_cast<float>(m_params.sample_rate)));

        for (size_t n = 0; n < total_samples; ++n) {
            const float t = static_cast<float>(n) / static_cast<float>(m_params.sample_rate);
            const float phase = w1 * L * (std::exp(t / L) - 1.0f);
            float sample = std::sin(phase);

            // Half-cosine fade-in
            if (n < fade_samples && fade_samples > 0) {
                float window = 0.5f * (1.0f - std::cos(std::numbers::pi_v<float> * static_cast<float>(n) / static_cast<float>(fade_samples)));
                sample *= window;
            }
            // Half-cosine fade-out
            if (n >= total_samples - fade_samples && fade_samples > 0) {
                size_t fade_idx = total_samples - 1 - n;
                float window = 0.5f * (1.0f - std::cos(std::numbers::pi_v<float> * static_cast<float>(fade_idx) / static_cast<float>(fade_samples)));
                sample *= window;
            }

            sweep[n] = sample;
        }

        return sweep;
    }

    // Generates the inverse deconvolution filter k(t)
    // Time-reversed sweep with exponential envelope scaling (-6 dB/octave amplitude tilt)
    [[nodiscard]] std::vector<float> generate_inverse_filter() const {
        std::vector<float> sweep = generate_sweep();
        const size_t total_samples = sweep.size();
        if (total_samples == 0) return {};

        std::vector<float> inv_filter(total_samples);
        const float w1 = 2.0f * std::numbers::pi_v<float> * m_params.start_freq;
        const float w2 = 2.0f * std::numbers::pi_v<float> * m_params.stop_freq;
        const float T = m_params.duration_sec;
        const float L = T / std::log(w2 / w1);

        float weighted_energy = 0.0f;
        for (size_t n = 0; n < total_samples; ++n) {
            const float t = static_cast<float>(n) / static_cast<float>(m_params.sample_rate);
            // Amplitude envelope drops exponentially: A(t) = exp(-t / L)
            const float env = std::exp(-t / L);
            // Time reversal: sample from the end of the forward sweep
            inv_filter[total_samples - 1 - n] = sweep[n] * env;
            weighted_energy += sweep[n] * sweep[n] * env;
        }

        // Normalize so that direct convolve(sweep, inv_filter) has peak amplitude of 1.0
        if (weighted_energy > 1e-12f) {
            const float norm = 1.0f / weighted_energy;
            for (auto& val : inv_filter) {
                val *= norm;
            }
        }

        return inv_filter;
    }

private:
    SweepParams m_params;
};

// ============================================================================
// AcousticMeasurementEngine: Impulse Response Deconvolution & Room Diagnostics
// ============================================================================
class AcousticMeasurementEngine {
public:
    // Deconvolve recorded room signal with Farina inverse filter to extract impulse response h(t)
    [[nodiscard]] static std::vector<float> deconvolve(const std::vector<float>& recorded,
                                                       const std::vector<float>& inverse_filter) {
        if (recorded.empty() || inverse_filter.empty()) return {};
        return dsp::FastFourierTransform::convolve(recorded.data(), recorded.size(),
                                                   inverse_filter.data(), inverse_filter.size());
    }

    // Extract Time-of-Flight (arrival time of direct sound) with parabolic sub-sample peak interpolation
    // Returns: true if clear peak found, false if below noise floor
    static bool analyze_time_of_flight(const std::vector<float>& ir,
                                       uint32_t sample_rate,
                                       size_t sweep_len,
                                       float& out_tof_seconds,
                                       float& out_distance_meters,
                                       float speed_of_sound = 343.0f) noexcept {
        if (ir.empty() || sample_rate == 0) return false;

        // In Farina deconvolution, linear impulse response appears at or after the sweep duration
        // Search window: starting near sweep_len - 1
        size_t start_idx = (sweep_len > 100) ? (sweep_len - 100) : 0;
        if (start_idx >= ir.size()) start_idx = 0;

        float max_val = 0.0f;
        size_t peak_idx = start_idx;

        for (size_t i = start_idx; i < ir.size(); ++i) {
            float abs_val = std::abs(ir[i]);
            if (abs_val > max_val) {
                max_val = abs_val;
                peak_idx = i;
            }
        }

        if (max_val < 1e-6f) return false;

        // Sub-sample parabolic interpolation around peak_idx
        float delta = 0.0f;
        if (peak_idx > 0 && peak_idx + 1 < ir.size()) {
            float y0 = std::abs(ir[peak_idx - 1]);
            float y1 = std::abs(ir[peak_idx]);
            float y2 = std::abs(ir[peak_idx + 1]);
            float denom = 2.0f * (y0 - 2.0f * y1 + y2);
            if (std::abs(denom) > 1e-9f) {
                delta = (y0 - y2) / denom;
                delta = std::clamp(delta, -0.5f, 0.5f);
            }
        }

        // Relative delay from the expected zero-latency reference point (sweep_len - 1)
        int64_t ref_idx = static_cast<int64_t>(sweep_len) - 1;
        double delay_samples = (static_cast<double>(peak_idx) + static_cast<double>(delta)) - static_cast<double>(ref_idx);
        if (delay_samples < 0.0) delay_samples = 0.0;

        out_tof_seconds = static_cast<float>(delay_samples / static_cast<double>(sample_rate));
        out_distance_meters = out_tof_seconds * speed_of_sound;
        return true;
    }

    // Detect acoustic room modes (low-frequency resonance peaks under Schroeder frequency)
    [[nodiscard]] static std::vector<RoomMode> detect_room_modes(const std::vector<float>& ir,
                                                                 uint32_t sample_rate,
                                                                 size_t peak_offset,
                                                                 size_t analysis_window = 8192,
                                                                 float max_freq_hz = 250.0f,
                                                                 float prominence_threshold_db = 6.0f) {
        if (ir.empty() || sample_rate == 0) return {};

        // Window impulse response starting from direct sound arrival
        size_t start = (peak_offset < ir.size()) ? peak_offset : 0;
        size_t len = std::min<size_t>(analysis_window, ir.size() - start);
        if (len < 512) return {};

        // Window with half-Hann
        std::vector<float> windowed_ir(len);
        for (size_t i = 0; i < len; ++i) {
            float t = static_cast<float>(i) / static_cast<float>(len);
            float w = 0.5f * (1.0f + std::cos(std::numbers::pi_v<float> * t)); // Right half of Hann
            windowed_ir[i] = ir[start + i] * w;
        }

        // FFT size
        size_t fft_size = dsp::FastFourierTransform::next_power_of_two(len);
        std::vector<dsp::FastFourierTransform::Complex> X = dsp::FastFourierTransform::forward_real(windowed_ir.data(), len);
        std::vector<float> mag_db = dsp::FastFourierTransform::magnitude_spectrum_db(X);

        // Frequency resolution
        const float bin_width = static_cast<float>(sample_rate) / static_cast<float>(fft_size);
        const size_t max_bin = static_cast<size_t>(std::ceil(max_freq_hz / bin_width));
        const size_t min_bin = static_cast<size_t>(std::floor(20.0f / bin_width));

        if (max_bin >= mag_db.size() || min_bin >= max_bin) return {};

        // Compute median level in [min_bin, max_bin]
        std::vector<float> band_levels;
        band_levels.reserve(max_bin - min_bin);
        for (size_t k = min_bin; k <= max_bin; ++k) {
            band_levels.push_back(mag_db[k]);
        }
        std::sort(band_levels.begin(), band_levels.end());
        const float median_db = band_levels[band_levels.size() / 2];

        std::vector<RoomMode> modes;

        // Peak search in [min_bin + 1, max_bin - 1]
        for (size_t k = min_bin + 1; k < max_bin; ++k) {
            if (mag_db[k] > mag_db[k - 1] && mag_db[k] > mag_db[k + 1]) {
                float peak_val = mag_db[k];
                float prominence = peak_val - median_db;

                if (prominence >= prominence_threshold_db) {
                    float f0 = static_cast<float>(k) * bin_width;

                    // Calculate -3 dB bandwidth
                    float target_level = peak_val - 3.0f;
                    size_t left_bin = k;
                    while (left_bin > min_bin && mag_db[left_bin] > target_level) {
                        --left_bin;
                    }
                    size_t right_bin = k;
                    while (right_bin < max_bin && mag_db[right_bin] > target_level) {
                        ++right_bin;
                    }

                    float bw_hz = std::max(static_cast<float>(right_bin - left_bin) * bin_width, bin_width);
                    float q = f0 / bw_hz;

                    if (q >= 2.0f) { // High-Q resonance
                        RoomMode mode;
                        mode.frequency_hz = f0;
                        mode.peak_gain_db = prominence;
                        mode.q_factor = q;
                        mode.recommended_notch_gain_db = -std::min(prominence, 18.0f); // Cut peak down
                        modes.push_back(mode);
                    }
                }
            }
        }

        return modes;
    }
};

} // namespace audio_core::analysis
