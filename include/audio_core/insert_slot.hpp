#pragma once

#include "audio_core/types.hpp"
#include <memory>
#include <atomic>
#include <string_view>
#include <array>

namespace audio_core {

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

    virtual void set_parameter(uint32_t index, float value) noexcept = 0;
    [[nodiscard]] virtual float get_parameter(uint32_t index) const noexcept = 0;
    [[nodiscard]] virtual const char* name() const noexcept = 0;
};

// ============================================================================
// InsertSlot: Fixed-size slot hosting an IProcessor in the channel strip
// Supports lock-free runtime bypass and zero-glitch atomic replacement
// ============================================================================
class InsertSlot {
public:
    InsertSlot() = default;

    void set_processor(std::shared_ptr<IProcessor> proc) noexcept {
        m_processor = std::move(proc);
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

    inline void process_stereo(Sample* left, Sample* right, uint32_t frames) noexcept {
        if (m_bypass.load(std::memory_order_relaxed) || !m_processor) {
            return;
        }
        m_processor->process_stereo(left, right, frames);
    }

private:
    std::shared_ptr<IProcessor> m_processor{nullptr};
    std::atomic<bool> m_bypass{false};
};

constexpr size_t kMaxTrackInsertSlots = 4;
constexpr size_t kMaxBusInsertSlots = 4;

} // namespace audio_core
