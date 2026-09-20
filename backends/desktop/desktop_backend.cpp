#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wextra"
#endif

#define MINIAUDIO_IMPLEMENTATION
#include "third_party/miniaudio/miniaudio.h"

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include "backends/desktop/desktop_backend.hpp"
#include <iostream>

namespace audio_core {

struct DesktopBackend::Impl {
    ma_device device;
};

DesktopBackend::DesktopBackend() : m_impl(std::make_unique<Impl>()) {}

DesktopBackend::~DesktopBackend() {
    stop();
    if (m_initialized) {
        ma_device_uninit(&m_impl->device);
        m_initialized = false;
    }
}

void DesktopBackend::miniaudio_data_callback(ma_device* pDevice, void* pOutput, const void* /*pInput*/, uint32_t frameCount) {
    auto* self = static_cast<DesktopBackend*>(pDevice->pUserData);
    if (!self || !self->m_callback) {
        return;
    }

    auto* out = static_cast<Sample*>(pOutput);
    self->m_callback(out, frameCount, self->m_channels);
}

bool DesktopBackend::init(uint32_t sample_rate, uint32_t channels, uint32_t buffer_size) {
    if (m_initialized) {
        stop();
        ma_device_uninit(&m_impl->device);
        m_initialized = false;
    }

    m_sample_rate = sample_rate;
    m_channels = channels;
    m_buffer_size = buffer_size;

    ma_device_config config = ma_device_config_init(ma_device_type_playback);
    config.playback.format = ma_format_f32;
    config.playback.channels = channels;
    config.sampleRate = sample_rate;
    config.dataCallback = miniaudio_data_callback;
    config.pUserData = this;
    config.periodSizeInFrames = buffer_size;

    ma_result result = ma_device_init(nullptr, &config, &m_impl->device);
    if (result != MA_SUCCESS) {
        std::cerr << "[DesktopBackend] Failed to initialize miniaudio device: " << result << std::endl;
        return false;
    }

    m_sample_rate = m_impl->device.sampleRate;
    m_channels = m_impl->device.playback.channels;
    m_initialized = true;

    return true;
}

bool DesktopBackend::start() {
    if (!m_initialized) return false;
    if (m_running) return true;

    ma_result result = ma_device_start(&m_impl->device);
    if (result != MA_SUCCESS) {
        std::cerr << "[DesktopBackend] Failed to start audio device: " << result << std::endl;
        return false;
    }

    m_running = true;
    return true;
}

void DesktopBackend::stop() {
    if (m_running && m_initialized) {
        ma_device_stop(&m_impl->device);
        m_running = false;
    }
}

} // namespace audio_core
