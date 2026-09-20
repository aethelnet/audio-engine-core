#pragma once

#include "backends/audio_backend.hpp"
#include <memory>
#include <atomic>
#include <string>

namespace audio_core {

// ============================================================================
// AAudioBackend: Native Android High-Performance Audio Driver
// - Exclusive hardware sharing mode (AAUDIO_SHARING_MODE_EXCLUSIVE)
// - Native low-latency path (AAUDIO_PERFORMANCE_MODE_LOW_LATENCY)
// - Direct float PCM streaming with hardware xrun/underrun monitoring
// ============================================================================
class AAudioBackend : public AudioBackend {
public:
    AAudioBackend();
    ~AAudioBackend() override;

    AAudioBackend(const AAudioBackend&) = delete;
    AAudioBackend& operator=(const AAudioBackend&) = delete;
    AAudioBackend(AAudioBackend&&) = delete;
    AAudioBackend& operator=(AAudioBackend&&) = delete;

    // Initialize low-latency stream (exclusive sharing mode, low-latency performance)
    bool init(uint32_t sample_rate, uint32_t channels, uint32_t buffer_size) override;
    bool start() override;
    void stop() override;

    void process_callback(Sample* out, uint32_t num_frames) noexcept;

    [[nodiscard]] bool is_running() const noexcept {
        return m_running.load(std::memory_order_relaxed);
    }

    [[nodiscard]] uint32_t actual_sample_rate() const noexcept override {
        return m_sample_rate;
    }

    [[nodiscard]] uint32_t actual_buffer_size() const noexcept override {
        return m_buffer_size;
    }

    [[nodiscard]] uint32_t channels() const noexcept {
        return m_channels;
    }

    // Hardware xrun (underrun/overrun) telemetry counter
    [[nodiscard]] int32_t xrun_count() const noexcept;

    // Maximum buffer capacity in frames
    [[nodiscard]] int32_t buffer_capacity() const noexcept;

    // Simulate an audio callback tick (useful for test harness and mock mode)
    void simulate_render_block(Sample* buffer, uint32_t frames);

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;

    uint32_t m_sample_rate{48000};
    uint32_t m_channels{2};
    uint32_t m_buffer_size{192};
    std::atomic<bool> m_running{false};
    bool m_initialized{false};
};

} // namespace audio_core
