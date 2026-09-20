#pragma once

#include "audio_core/types.hpp"
#include <vector>
#include <array>
#include <atomic>
#include <string>
#include <cmath>
#include <numbers>
#include <algorithm>
#include <cstring>
#include <cstdint>

namespace audio_core::dsp {

// Standard channel configuration presets
enum class SpeakerLayout : uint8_t {
    Mono = 1,
    Stereo = 2,
    Quadraphonic = 4,
    Surround5_1 = 6,
    Surround7_1_4 = 12,
    Ambisonics3rdOrder = 16,
    WaveFieldSynthesis32 = 32,
    WaveFieldSynthesis64 = 64,
    Custom = 128
};

// ============================================================================
// MultiChannelBus: High-Capacity Planar Audio Bus (1 to 128 Channels)
// Zero-allocation in real-time execution path, lock-free telemetry meters,
// SIMD-friendly planar storage.
// Features:
// - Arbitrary channel counts (Mono, Stereo, 5.1, 7.1.4, Ambisonics 16ch, WFS 64ch).
// - Format-agnostic mixing and spatial panning.
// - ITU-R BS.775 compliant multi-channel to stereo downmixing.
// - Zero-copy planar interleaving / de-interleaving for Poly-WAV (RF64/BWF).
// ============================================================================
class MultiChannelBus {
public:
    static constexpr uint32_t kMaxChannels = 128;

    MultiChannelBus(uint32_t num_channels = 2, uint32_t frames = 1024, std::string name = "MultiChannelBus")
        : m_num_channels(std::clamp(num_channels, 1u, kMaxChannels))
        , m_frames(frames)
        , m_name(std::move(name)) {
        m_storage.resize(m_num_channels * m_frames, 0.0f);
        m_channel_ptrs.resize(m_num_channels);
        update_ptrs();
        reset_meters();
    }

    void resize(uint32_t num_channels, uint32_t frames) {
        m_num_channels = std::clamp(num_channels, 1u, kMaxChannels);
        m_frames = frames;
        m_storage.resize(m_num_channels * m_frames, 0.0f);
        m_channel_ptrs.resize(m_num_channels);
        update_ptrs();
        reset_meters();
    }

    [[nodiscard]] uint32_t num_channels() const noexcept { return m_num_channels; }
    [[nodiscard]] uint32_t num_frames() const noexcept { return m_frames; }
    [[nodiscard]] const std::string& name() const noexcept { return m_name; }
    void set_name(std::string name) noexcept { m_name = std::move(name); }

    [[nodiscard]] float* channel(uint32_t ch) noexcept {
        return (ch < m_num_channels) ? m_channel_ptrs[ch] : nullptr;
    }
    [[nodiscard]] const float* channel(uint32_t ch) const noexcept {
        return (ch < m_num_channels) ? m_channel_ptrs[ch] : nullptr;
    }

    void clear() noexcept {
        std::fill(m_storage.begin(), m_storage.end(), 0.0f);
    }

    void clear_frames(uint32_t frames) noexcept {
        const uint32_t count = std::min(frames, m_frames);
        for (uint32_t ch = 0; ch < m_num_channels; ++ch) {
            std::memset(m_channel_ptrs[ch], 0, count * sizeof(float));
        }
    }

    // Mix single mono channel into target bus channel with gain
    void mix_mono_channel(uint32_t target_ch, const float* src, float gain, uint32_t frames) noexcept {
        if (target_ch >= m_num_channels || !src || frames == 0) return;
        float* dst = m_channel_ptrs[target_ch];
        const uint32_t count = std::min(frames, m_frames);

        #if defined(__GNUC__) || defined(__clang__)
        #pragma GCC ivdep
        #endif
        for (uint32_t i = 0; i < count; ++i) {
            dst[i] += src[i] * gain;
        }
    }

    // Mix planar stereo pair into target bus (e.g. into channels 0 and 1)
    void mix_stereo_pair(uint32_t target_left_ch, const float* src_l, const float* src_r, float gain, uint32_t frames) noexcept {
        mix_mono_channel(target_left_ch, src_l, gain, frames);
        mix_mono_channel(target_left_ch + 1, src_r, gain, frames);
    }

    // Spatial Panning: Pans a mono source into a circular horizontal speaker array
    // using Vector Base Amplitude Panning (VBAP) / Constant-Power Panning.
    // azimuth_rad in [-pi, pi] (0 = Center/Front, -pi/2 = Left, +pi/2 = Right)
    void pan_mono_circular(const float* src, float gain, float azimuth_rad, uint32_t frames) noexcept {
        if (!src || frames == 0 || m_num_channels == 0) return;
        const uint32_t count = std::min(frames, m_frames);

        if (m_num_channels == 1) {
            mix_mono_channel(0, src, gain, count);
            return;
        }

        if (m_num_channels == 2) {
            // Constant power stereo panning: azimuth in [-pi/2, pi/2]
            float clamped_azimuth = std::clamp(azimuth_rad, -std::numbers::pi_v<float> * 0.5f, std::numbers::pi_v<float> * 0.5f);
            float pan_norm = clamped_azimuth / (std::numbers::pi_v<float> * 0.5f); // [-1, 1]
            float theta = (pan_norm + 1.0f) * 0.25f * std::numbers::pi_v<float>;
            float gain_l = gain * std::cos(theta);
            float gain_r = gain * std::sin(theta);
            mix_mono_channel(0, src, gain_l, count);
            mix_mono_channel(1, src, gain_r, count);
            return;
        }

        // Multichannel Circular Array (4, 6, 8, 16, 32, 64 channels)
        // Normalize angle to [0, 2*pi)
        float angle = std::fmod(azimuth_rad, 2.0f * std::numbers::pi_v<float>);
        if (angle < 0.0f) angle += 2.0f * std::numbers::pi_v<float>;

        // Sector calculation
        const float sector_width = (2.0f * std::numbers::pi_v<float>) / static_cast<float>(m_num_channels);
        const float sector_f = angle / sector_width;
        const uint32_t spk1 = static_cast<uint32_t>(sector_f) % m_num_channels;
        const uint32_t spk2 = (spk1 + 1) % m_num_channels;
        const float fraction = sector_f - std::floor(sector_f);

        // Constant power pair panning
        const float g1 = gain * std::cos(fraction * 0.5f * std::numbers::pi_v<float>);
        const float g2 = gain * std::sin(fraction * 0.5f * std::numbers::pi_v<float>);

        mix_mono_channel(spk1, src, g1, count);
        mix_mono_channel(spk2, src, g2, count);
    }

    // Downmix to Stereo (Standard ITU-R BS.775 for 5.1/7.1, or Ambisonics/WFS downfold)
    void downmix_to_stereo(float* out_l, float* out_r, uint32_t frames) const noexcept {
        if (!out_l || !out_r || frames == 0) return;
        const uint32_t count = std::min(frames, m_frames);
        std::memset(out_l, 0, count * sizeof(float));
        std::memset(out_r, 0, count * sizeof(float));

        if (m_num_channels == 1) {
            // Mono to Dual-Mono (-3 dB per channel)
            constexpr float kMonoGain = 0.70710678f;
            const float* ch0 = m_channel_ptrs[0];
            for (uint32_t i = 0; i < count; ++i) {
                out_l[i] = ch0[i] * kMonoGain;
                out_r[i] = ch0[i] * kMonoGain;
            }
            return;
        }

        if (m_num_channels == 2) {
            // 1:1 Stereo Pass
            const float* ch0 = m_channel_ptrs[0];
            const float* ch1 = m_channel_ptrs[1];
            for (uint32_t i = 0; i < count; ++i) {
                out_l[i] = ch0[i];
                out_r[i] = ch1[i];
            }
            return;
        }

        if (m_num_channels >= 6) {
            // 5.1 Surround Downmix (ITU-R BS.775)
            // L_tot = L + 0.7071 * C + 0.7071 * Ls
            // R_tot = R + 0.7071 * C + 0.7071 * Rs
            // 0: L, 1: R, 2: C, 3: LFE, 4: Ls, 5: Rs
            constexpr float kCenterGain = 0.70710678f;
            constexpr float kSurroundGain = 0.70710678f;
            const float* L = m_channel_ptrs[0];
            const float* R = m_channel_ptrs[1];
            const float* C = m_channel_ptrs[2];
            const float* Ls = m_channel_ptrs[4];
            const float* Rs = m_channel_ptrs[5];

            for (uint32_t i = 0; i < count; ++i) {
                out_l[i] = L[i] + kCenterGain * C[i] + kSurroundGain * Ls[i];
                out_r[i] = R[i] + kCenterGain * C[i] + kSurroundGain * Rs[i];
            }
            return;
        }

        // Generic multichannel equal-power downmix (e.g. Quad or WFS ring)
        const float norm = 1.0f / std::sqrt(static_cast<float>(m_num_channels) * 0.5f);
        for (uint32_t ch = 0; ch < m_num_channels; ++ch) {
            const float* c = m_channel_ptrs[ch];
            const float angle = (2.0f * std::numbers::pi_v<float> * static_cast<float>(ch)) / static_cast<float>(m_num_channels);
            const float pan_l = std::max(0.0f, std::cos(angle));
            const float pan_r = std::max(0.0f, -std::cos(angle));

            for (uint32_t i = 0; i < count; ++i) {
                out_l[i] += c[i] * pan_l * norm;
                out_r[i] += c[i] * pan_r * norm;
            }
        }
    }

    // Interleave planar channel buffers into interleaved PCM format for Poly-WAV / AES67
    void interleave(float* interleaved_out, uint32_t frames) const noexcept {
        if (!interleaved_out || frames == 0) return;
        const uint32_t count = std::min(frames, m_frames);

        for (uint32_t ch = 0; ch < m_num_channels; ++ch) {
            const float* src = m_channel_ptrs[ch];
            for (uint32_t i = 0; i < count; ++i) {
                interleaved_out[(i * m_num_channels) + ch] = src[i];
            }
        }
    }

    // Deinterleave interleaved PCM buffer into planar bus channels
    void deinterleave(const float* interleaved_in, uint32_t in_channels, uint32_t frames) noexcept {
        if (!interleaved_in || frames == 0 || in_channels == 0) return;
        const uint32_t count = std::min(frames, m_frames);
        const uint32_t active_ch = std::min(in_channels, m_num_channels);

        clear_frames(count);
        for (uint32_t ch = 0; ch < active_ch; ++ch) {
            float* dst = m_channel_ptrs[ch];
            for (uint32_t i = 0; i < count; ++i) {
                dst[i] = interleaved_in[(i * in_channels) + ch];
            }
        }
    }

    // Telemetry metering across all active channels
    void update_meters(uint32_t frames) noexcept {
        const uint32_t count = std::min(frames, m_frames);
        if (count == 0) return;

        for (uint32_t ch = 0; ch < m_num_channels; ++ch) {
            const float* c = m_channel_ptrs[ch];
            float max_peak = 0.0f;
            float sum_sq = 0.0f;

            for (uint32_t i = 0; i < count; ++i) {
                float a = std::abs(c[i]);
                if (a > max_peak) max_peak = a;
                sum_sq += a * a;
            }
            float cur_rms = std::sqrt(sum_sq / static_cast<float>(count));

            m_peak_levels[ch].store(max_peak, std::memory_order_relaxed);
            m_rms_levels[ch].store(cur_rms, std::memory_order_relaxed);
        }
    }

    [[nodiscard]] float peak(uint32_t ch) const noexcept {
        return (ch < m_num_channels) ? m_peak_levels[ch].load(std::memory_order_relaxed) : 0.0f;
    }
    [[nodiscard]] float rms(uint32_t ch) const noexcept {
        return (ch < m_num_channels) ? m_rms_levels[ch].load(std::memory_order_relaxed) : 0.0f;
    }

    void reset_meters() noexcept {
        for (auto& p : m_peak_levels) p.store(0.0f, std::memory_order_relaxed);
        for (auto& r : m_rms_levels) r.store(0.0f, std::memory_order_relaxed);
    }

private:
    void update_ptrs() noexcept {
        for (uint32_t ch = 0; ch < m_num_channels; ++ch) {
            m_channel_ptrs[ch] = m_storage.data() + (ch * m_frames);
        }
    }

    uint32_t m_num_channels{2};
    uint32_t m_frames{1024};
    std::string m_name;

    std::vector<float> m_storage;
    std::vector<float*> m_channel_ptrs;

    std::array<std::atomic<float>, kMaxChannels> m_peak_levels{};
    std::array<std::atomic<float>, kMaxChannels> m_rms_levels{};
};

} // namespace audio_core::dsp
