#pragma once

#include "backends/audio_backend.hpp"
#include <memory>

// Forward declaration of miniaudio struct
struct ma_device;

namespace audio_core {

class DesktopBackend : public AudioBackend {
public:
    DesktopBackend();
    ~DesktopBackend() override;

    bool init(uint32_t sample_rate, uint32_t channels, uint32_t buffer_size) override;
    bool start() override;
    void stop() override;

    [[nodiscard]] uint32_t actual_sample_rate() const noexcept override { return m_sample_rate; }
    [[nodiscard]] uint32_t actual_buffer_size() const noexcept override { return m_buffer_size; }

private:
    static void miniaudio_data_callback(ma_device* pDevice, void* pOutput, const void* pInput, uint32_t frameCount);

    uint32_t m_sample_rate{kDefaultSampleRate};
    uint32_t m_channels{kDefaultChannels};
    uint32_t m_buffer_size{kDefaultBufferSize};
    bool m_initialized{false};
    bool m_running{false};

    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace audio_core
