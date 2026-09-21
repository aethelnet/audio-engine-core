#pragma once

#include "audio_core/types.hpp"
#include <memory>
#include <atomic>
#include <string_view>
#include <array>
#include <cmath>
#include <numbers>
#include <algorithm>

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
#include <xmmintrin.h>
#include <pmmintrin.h>
#endif

namespace audio_core {

// Enable Flush-to-Zero and Denormals-are-Zero to prevent CPU pipeline stalls
inline void enable_ftz_daz() noexcept {
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
#endif
}

// ============================================================================
// IProcessor: Unified Interface for Real-Time DSP Modules (Airwindows & Wasm)
// Zero allocations in audio thread, lock-free parameter access
// ============================================================================
class IProcessor {
public:
    virtual ~IProcessor() = default;

    virtual void init(uint32_t sample_rate) noexcept = 0;
    virtual void reset() noexcept = 0;
    virtual void process_stereo(Sample* left, Sample* right, uint32_t frames) noexcept = 0;
    virtual void process_stereo_sidechain(Sample* left, Sample* right,
                                          const Sample* sc_left, const Sample* sc_right,
                                          uint32_t frames) noexcept {
        (void)sc_left; (void)sc_right;
        process_stereo(left, right, frames);
    }
    [[nodiscard]] virtual bool supports_sidechain() const noexcept { return false; }

    virtual void set_parameter(uint32_t index, float value) noexcept = 0;
    [[nodiscard]] virtual float get_parameter(uint32_t index) const noexcept = 0;
    [[nodiscard]] virtual const char* name() const noexcept = 0;
};

// ============================================================================
// InsertSlot: Hardened Slot Hosting an IProcessor in the Channel Strip
// Defends against DC drift, NaNs, Infs, and blown-up community WASM plugins.
// Features:
// 1. DC-Blocking Highpass (5.0 Hz IIR AC coupling emulation)
// 2. NaN / Inf Sanitizer & Acoustic Safety Clamp (+24 dBFS ceiling)
// 3. Auto-Bypass Circuit Breaker on corrupt/exploding plugins
// ============================================================================
class InsertSlot {
public:
    static constexpr float kDefaultDcCutoffHz = 5.0f;
    static constexpr float kMaxSafeAmplitude = 16.0f; // +24 dBFS ceiling before hard acoustic protection
    static constexpr uint32_t kCircuitBreakerFaultLimit = 8; // Number of NaNs/Infs before tripping circuit breaker

    InsertSlot() noexcept {
        update_dc_coeff();
    }

    void init(uint32_t sample_rate) noexcept {
        if (sample_rate > 0) m_sample_rate = sample_rate;
        update_dc_coeff();
        reset();
        if (m_processor) {
            m_processor->init(m_sample_rate);
        }
    }

    void reset() noexcept {
        m_prev_x_l = 0.0f;
        m_prev_y_l = 0.0f;
        m_prev_x_r = 0.0f;
        m_prev_y_r = 0.0f;
        m_corrupt_samples_detected.store(0, std::memory_order_relaxed);
        m_circuit_breaker_tripped.store(false, std::memory_order_relaxed);
        m_has_fault.store(false, std::memory_order_relaxed);
        if (m_processor) {
            m_processor->reset();
        }
    }

    void set_processor(std::shared_ptr<IProcessor> proc) noexcept {
        m_processor = std::move(proc);
        if (m_processor) {
            m_processor->init(m_sample_rate);
        }
        reset();
    }

    [[nodiscard]] IProcessor* processor() noexcept {
        return m_processor.get();
    }

    [[nodiscard]] const IProcessor* processor() const noexcept {
        return m_processor.get();
    }

    void set_bypass(bool bypass) noexcept {
        m_bypass.store(bypass, std::memory_order_relaxed);
    }

    [[nodiscard]] bool is_bypassed() const noexcept {
        return m_bypass.load(std::memory_order_relaxed);
    }

    [[nodiscard]] bool is_empty() const noexcept {
        return m_processor == nullptr;
    }

    void set_dc_block_enabled(bool enabled) noexcept {
        m_dc_block_enabled.store(enabled, std::memory_order_relaxed);
    }

    [[nodiscard]] bool is_dc_block_enabled() const noexcept {
        return m_dc_block_enabled.load(std::memory_order_relaxed);
    }

    [[nodiscard]] bool is_circuit_breaker_tripped() const noexcept {
        return m_circuit_breaker_tripped.load(std::memory_order_relaxed);
    }

    void reset_circuit_breaker() noexcept {
        m_circuit_breaker_tripped.store(false, std::memory_order_relaxed);
        m_bypass.store(false, std::memory_order_relaxed);
        m_has_fault.store(false, std::memory_order_relaxed);
    }

    [[nodiscard]] bool has_fault() const noexcept {
        return m_has_fault.load(std::memory_order_relaxed);
    }

    void clear_fault() noexcept {
        m_has_fault.store(false, std::memory_order_relaxed);
    }

    [[nodiscard]] uint64_t corrupt_samples_detected() const noexcept {
        return m_corrupt_samples_detected.load(std::memory_order_relaxed);
    }

    inline void process_stereo(Sample* left, Sample* right, uint32_t frames,
                               const Sample* sc_left = nullptr, const Sample* sc_right = nullptr) noexcept {
        if (m_bypass.load(std::memory_order_relaxed) || !m_processor || frames == 0) {
            return;
        }

        // 1. Run underlying DSP processor (Airwindows or WASM container or ODE Compressor)
        if (sc_left && sc_right && m_processor->supports_sidechain()) {
            m_processor->process_stereo_sidechain(left, right, sc_left, sc_right, frames);
        } else {
            m_processor->process_stereo(left, right, frames);
        }

        // 2. Hardened Interceptor & Sanitizer Pass (Zero allocations)
        uint32_t bad_samples_in_block = 0;
        const bool dc_block = m_dc_block_enabled.load(std::memory_order_relaxed);
        const float alpha = m_alpha;

        for (uint32_t i = 0; i < frames; ++i) {
            float sl = left[i];
            float sr = right[i];

            // NaN / Inf protection
            if (!std::isfinite(sl)) {
                sl = 0.0f;
                bad_samples_in_block++;
            }
            if (!std::isfinite(sr)) {
                sr = 0.0f;
                bad_samples_in_block++;
            }

            // Acoustic safety clamp: prevent ears/DAC destruction (+24 dBFS ceiling)
            sl = std::clamp(sl, -kMaxSafeAmplitude, kMaxSafeAmplitude);
            sr = std::clamp(sr, -kMaxSafeAmplitude, kMaxSafeAmplitude);

            // DC-Blocking 1-pole highpass filter (AC-coupling emulation)
            if (dc_block) {
                float yl = alpha * (m_prev_y_l + sl - m_prev_x_l);
                m_prev_x_l = sl;
                m_prev_y_l = yl;
                sl = yl;

                float yr = alpha * (m_prev_y_r + sr - m_prev_x_r);
                m_prev_x_r = sr;
                m_prev_y_r = yr;
                sr = yr;
            }

            left[i] = sl;
            right[i] = sr;
        }

        // 3. Circuit breaker evaluation
        if (bad_samples_in_block > 0) {
            m_has_fault.store(true, std::memory_order_relaxed);
            m_corrupt_samples_detected.fetch_add(bad_samples_in_block, std::memory_order_relaxed);

            if (bad_samples_in_block >= kCircuitBreakerFaultLimit) {
                // Trip circuit breaker immediately: auto-bypass rogue plugin
                m_circuit_breaker_tripped.store(true, std::memory_order_relaxed);
                m_bypass.store(true, std::memory_order_relaxed);
                // Zero out corrupted block to prevent downstream ear-splitting pops
                for (uint32_t j = 0; j < frames; ++j) {
                    left[j] = 0.0f;
                    right[j] = 0.0f;
                }
            }
        }
    }

private:
    void update_dc_coeff() noexcept {
        const float dt = 1.0f / static_cast<float>(m_sample_rate);
        const float rc = 1.0f / (2.0f * std::numbers::pi_v<float> * kDefaultDcCutoffHz);
        m_alpha = rc / (rc + dt);
    }

    std::shared_ptr<IProcessor> m_processor{nullptr};
    std::atomic<bool> m_bypass{false};
    std::atomic<bool> m_dc_block_enabled{true};
    std::atomic<bool> m_circuit_breaker_tripped{false};
    std::atomic<bool> m_has_fault{false};
    std::atomic<uint64_t> m_corrupt_samples_detected{0};

    uint32_t m_sample_rate{48000};
    float m_alpha{0.999346f};

    float m_prev_x_l{0.0f};
    float m_prev_y_l{0.0f};
    float m_prev_x_r{0.0f};
    float m_prev_y_r{0.0f};
};

constexpr size_t kMaxTrackInsertSlots = 4;
constexpr size_t kMaxBusInsertSlots = 4;

} // namespace audio_core
