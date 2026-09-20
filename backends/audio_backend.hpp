#pragma once

#include "audio_core/types.hpp"
#include <functional>

namespace audio_core {

class AudioBackend {
public:
    using AudioCallback = std::function<void(Sample* output, uint32_t frames, uint32_t channels)>;

    virtual ~AudioBackend() = default;

    virtual bool init(uint32_t sample_rate, uint32_t channels, uint32_t buffer_size) = 0;
    virtual bool start() = 0;
    virtual void stop() = 0;

    void set_callback(AudioCallback cb) {
        m_callback = std::move(cb);
    }

    [[nodiscard]] virtual uint32_t actual_sample_rate() const noexcept = 0;
    [[nodiscard]] virtual uint32_t actual_buffer_size() const noexcept = 0;

protected:
    AudioCallback m_callback;
};

} // namespace audio_core
