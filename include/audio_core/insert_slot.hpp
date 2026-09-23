#pragma once

#include "audio_core/types.hpp"
#include <memory>
#include <atomic>
#include <string_view>
#include <array>
#include <vector>
#include <mutex>
#include <cmath>
#include <numbers>
#include <algorithm>
#include <cstring>

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

    // Latency reporting for Plugin Delay Compensation (PDC)
    [[nodiscard]] virtual uint32_t latency_samples() const noexcept { return 0; }

    // Fault reporting for sandboxed plugins (WASM/Circuit Breaker)
    [[nodiscard]] virtual bool has_fault() const noexcept { return false; }
    [[nodiscard]] virtual const char* fault_reason() const noexcept { return nullptr; }
    virtual void clear_fault() noexcept {}
};

// ============================================================================
// InsertSlot: Hardened Slot Hosting an IProcessor in the Channel Strip
// Defends against DC drift, NaNs, Infs, and blown-up community WASM plugins.
// Features:
// 1. Lock-Free Atomic Plugin Hot-Swap (RCU-style epoch deferred reclamation)
// 2. DC-Blocking Highpass (5.0 Hz IIR AC coupling emulation)
// 3. NaN / Inf Sanitizer & Acoustic Safety Clamp (+24 dBFS ceiling)
// 4. Auto-Bypass Circuit Breaker on corrupt/exploding/trapping plugins
// ============================================================================
class InsertSlot {
public:
    static constexpr float kDefaultDcCutoffHz = 5.0f;
    static constexpr float kMaxSafeAmplitude = 16.0f; // +24 dBFS ceiling before hard acoustic protection
    static constexpr uint32_t kCircuitBreakerFaultLimit = 8; // Number of NaNs/Infs/Traps before tripping circuit breaker

    struct RetiredProcessor {
        std::shared_ptr<IProcessor> proc;
        uint64_t retire_epoch;
    };

    InsertSlot() noexcept {
        update_dc_coeff();
    }

    ~InsertSlot() {
        m_active_processor.store(nullptr, std::memory_order_release);
        std::lock_guard<std::mutex> lock(m_control_mutex);
        m_owned_processor.reset();
        m_graveyard.clear();
    }

    // Non-copyable and non-movable (contains atomics and mutex)
    InsertSlot(const InsertSlot&) = delete;
    InsertSlot& operator=(const InsertSlot&) = delete;
    InsertSlot(InsertSlot&&) = delete;
    InsertSlot& operator=(InsertSlot&&) = delete;

    void init(uint32_t sample_rate) noexcept {
        if (sample_rate > 0) m_sample_rate = sample_rate;
        update_dc_coeff();
        reset();
        IProcessor* proc = m_active_processor.load(std::memory_order_acquire);
        if (proc) {
            proc->init(m_sample_rate);
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
        m_consecutive_faults.store(0, std::memory_order_relaxed);
        IProcessor* proc = m_active_processor.load(std::memory_order_acquire);
        if (proc) {
            proc->reset();
        }
    }

    // Atomic Lock-Free Processor Hot-Swap:
    // Safely swaps active processor pointer without memory allocation or blocking in audio thread.
    // Old processor is safely deferred in the graveyard until audio thread clears the epoch.
    void swap_processor(std::shared_ptr<IProcessor> new_proc) noexcept {
        if (new_proc) {
            new_proc->init(m_sample_rate);
        }

        std::lock_guard<std::mutex> lock(m_control_mutex);
        const uint64_t current_epoch = m_audio_block_epoch.load(std::memory_order_acquire);

        if (m_owned_processor) {
            m_graveyard.push_back(RetiredProcessor{std::move(m_owned_processor), current_epoch});
        }
        m_owned_processor = std::move(new_proc);
        m_active_processor.store(m_owned_processor ? m_owned_processor.get() : nullptr, std::memory_order_release);

        prune_graveyard_locked(current_epoch);
        reset();
    }

    void set_processor(std::shared_ptr<IProcessor> proc) noexcept {
        swap_processor(std::move(proc));
    }

    [[nodiscard]] IProcessor* processor() noexcept {
        return m_active_processor.load(std::memory_order_relaxed);
    }

    [[nodiscard]] const IProcessor* processor() const noexcept {
        return m_active_processor.load(std::memory_order_relaxed);
    }

    [[nodiscard]] std::shared_ptr<IProcessor> shared_processor() const noexcept {
        std::lock_guard<std::mutex> lock(m_control_mutex);
        return m_owned_processor;
    }

    [[nodiscard]] bool is_empty() const noexcept {
        return m_active_processor.load(std::memory_order_relaxed) == nullptr;
    }

    void set_bypass(bool bypass) noexcept {
        m_bypass.store(bypass, std::memory_order_relaxed);
    }

    [[nodiscard]] bool is_bypassed() const noexcept {
        return m_bypass.load(std::memory_order_relaxed);
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
        m_consecutive_faults.store(0, std::memory_order_relaxed);
    }

    [[nodiscard]] bool has_fault() const noexcept {
        return m_has_fault.load(std::memory_order_relaxed);
    }

    void clear_fault() noexcept {
        m_has_fault.store(false, std::memory_order_relaxed);
        m_consecutive_faults.store(0, std::memory_order_relaxed);
    }

    [[nodiscard]] uint64_t corrupt_samples_detected() const noexcept {
        return m_corrupt_samples_detected.load(std::memory_order_relaxed);
    }

    [[nodiscard]] uint32_t consecutive_faults() const noexcept {
        return m_consecutive_faults.load(std::memory_order_relaxed);
    }

    [[nodiscard]] uint32_t latency_samples() const noexcept {
        if (m_bypass.load(std::memory_order_relaxed) || m_circuit_breaker_tripped.load(std::memory_order_relaxed)) {
            return 0;
        }
        IProcessor* proc = m_active_processor.load(std::memory_order_acquire);
        return proc ? proc->latency_samples() : 0;
    }

    [[nodiscard]] uint64_t audio_block_epoch() const noexcept {
        return m_audio_block_epoch.load(std::memory_order_acquire);
    }

    [[nodiscard]] size_t graveyard_size() const noexcept {
        std::lock_guard<std::mutex> lock(m_control_mutex);
        return m_graveyard.size();
    }

    void prune_graveyard() noexcept {
        std::lock_guard<std::mutex> lock(m_control_mutex);
        const uint64_t current_epoch = m_audio_block_epoch.load(std::memory_order_acquire);
        prune_graveyard_locked(current_epoch);
    }

    void force_prune_graveyard() noexcept {
        std::lock_guard<std::mutex> lock(m_control_mutex);
        m_graveyard.clear();
    }

    inline void process_stereo(Sample* left, Sample* right, uint32_t frames,
                               const Sample* sc_left = nullptr, const Sample* sc_right = nullptr) noexcept {
        m_audio_block_epoch.fetch_add(1, std::memory_order_relaxed);

        IProcessor* proc = m_active_processor.load(std::memory_order_acquire);
        if (m_bypass.load(std::memory_order_relaxed) || !proc || frames == 0) {
            return;
        }

        // 1. Run underlying DSP processor (Airwindows or WASM container or ODE Compressor)
        if (sc_left && sc_right && proc->supports_sidechain()) {
            proc->process_stereo_sidechain(left, right, sc_left, sc_right, frames);
        } else {
            proc->process_stereo(left, right, frames);
        }

        // 2. Fault detector for rogue / trapping sandboxed plugins
        if (proc->has_fault()) {
            m_has_fault.store(true, std::memory_order_relaxed);
            uint32_t faults = m_consecutive_faults.fetch_add(1, std::memory_order_relaxed) + 1;
            if (faults >= kCircuitBreakerFaultLimit) {
                m_circuit_breaker_tripped.store(true, std::memory_order_relaxed);
                m_bypass.store(true, std::memory_order_relaxed);
            }
            // Fail-safe: silence trapped output immediately and skip downstream DC blocker
            std::memset(left, 0, frames * sizeof(Sample));
            std::memset(right, 0, frames * sizeof(Sample));
            return;
        } else {
            m_consecutive_faults.store(0, std::memory_order_relaxed);
        }

        // 3. Hardened Interceptor & Sanitizer Pass (Zero allocations)
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

        // 4. Circuit breaker evaluation for NaNs/Infs
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

    void prune_graveyard_locked(uint64_t current_epoch) noexcept {
        auto it = std::remove_if(m_graveyard.begin(), m_graveyard.end(),
            [current_epoch](const RetiredProcessor& item) {
                return (current_epoch >= item.retire_epoch + 2);
            });
        m_graveyard.erase(it, m_graveyard.end());
    }

    // Atomic active processor pointer (lock-free read on real-time audio thread)
    std::atomic<IProcessor*> m_active_processor{nullptr};
    std::atomic<uint64_t> m_audio_block_epoch{0};

    // Control thread ownership & deferred reclamation graveyard
    mutable std::mutex m_control_mutex;
    std::shared_ptr<IProcessor> m_owned_processor{nullptr};
    std::vector<RetiredProcessor> m_graveyard;

    std::atomic<bool> m_bypass{false};
    std::atomic<bool> m_dc_block_enabled{true};
    std::atomic<bool> m_circuit_breaker_tripped{false};
    std::atomic<bool> m_has_fault{false};
    std::atomic<uint32_t> m_consecutive_faults{0};
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
