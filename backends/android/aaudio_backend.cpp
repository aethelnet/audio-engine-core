#include "backends/android/aaudio_backend.hpp"
#include <iostream>
#include <vector>
#include <cstring>

#if defined(__ANDROID__)
#include <aaudio/AAudio.h>
#else
// Software Emulation of AAudio Native API for Linux Host Testing & Verification
typedef struct AAudioStreamStruct AAudioStream;
typedef struct AAudioStreamBuilderStruct AAudioStreamBuilder;

typedef int32_t aaudio_result_t;
typedef int32_t aaudio_direction_t;
typedef int32_t aaudio_performance_mode_t;
typedef int32_t aaudio_sharing_mode_t;
typedef int32_t aaudio_format_t;
typedef int32_t aaudio_data_callback_result_t;

enum {
    AAUDIO_OK = 0,
    AAUDIO_ERROR_BASE = -900,
    AAUDIO_ERROR_DISCONNECTED = -899,
    AAUDIO_ERROR_INVALID_STATE = -898,
    AAUDIO_ERROR_INTERNAL = -897
};

enum {
    AAUDIO_DIRECTION_OUTPUT = 0,
    AAUDIO_DIRECTION_INPUT = 1
};

enum {
    AAUDIO_PERFORMANCE_MODE_NONE = 10,
    AAUDIO_PERFORMANCE_MODE_POWER_SAVING = 11,
    AAUDIO_PERFORMANCE_MODE_LOW_LATENCY = 12
};

enum {
    AAUDIO_SHARING_MODE_EXCLUSIVE = 0,
    AAUDIO_SHARING_MODE_SHARED = 1
};

enum {
    AAUDIO_FORMAT_UNSPECIFIED = 0,
    AAUDIO_FORMAT_PCM_I16 = 1,
    AAUDIO_FORMAT_PCM_FLOAT = 2
};

enum {
    AAUDIO_CALLBACK_RESULT_CONTINUE = 0,
    AAUDIO_CALLBACK_RESULT_STOP = 1
};

typedef aaudio_data_callback_result_t (*AAudioStream_dataCallback)(
    AAudioStream* stream,
    void* userData,
    void* audioData,
    int32_t numFrames);

typedef void (*AAudioStream_errorCallback)(
    AAudioStream* stream,
    void* userData,
    aaudio_result_t error);

struct AAudioStreamBuilderStruct {
    aaudio_direction_t direction{AAUDIO_DIRECTION_OUTPUT};
    aaudio_performance_mode_t performance_mode{AAUDIO_PERFORMANCE_MODE_LOW_LATENCY};
    aaudio_sharing_mode_t sharing_mode{AAUDIO_SHARING_MODE_EXCLUSIVE};
    aaudio_format_t format{AAUDIO_FORMAT_PCM_FLOAT};
    int32_t sample_rate{48000};
    int32_t channel_count{2};
    int32_t buffer_capacity_frames{384};
    AAudioStream_dataCallback data_callback{nullptr};
    AAudioStream_errorCallback error_callback{nullptr};
    void* user_data{nullptr};
};

struct AAudioStreamStruct {
    AAudioStreamBuilderStruct config;
    bool is_started{false};
    int32_t xrun_count{0};
};

inline aaudio_result_t AAudio_createStreamBuilder(AAudioStreamBuilder** builder) {
    *builder = new AAudioStreamBuilderStruct();
    return AAUDIO_OK;
}

inline void AAudioStreamBuilder_setDirection(AAudioStreamBuilder* b, aaudio_direction_t d) {
    if (b) b->direction = d;
}

inline void AAudioStreamBuilder_setPerformanceMode(AAudioStreamBuilder* b, aaudio_performance_mode_t m) {
    if (b) b->performance_mode = m;
}

inline void AAudioStreamBuilder_setSharingMode(AAudioStreamBuilder* b, aaudio_sharing_mode_t s) {
    if (b) b->sharing_mode = s;
}

inline void AAudioStreamBuilder_setFormat(AAudioStreamBuilder* b, aaudio_format_t f) {
    if (b) b->format = f;
}

inline void AAudioStreamBuilder_setSampleRate(AAudioStreamBuilder* b, int32_t rate) {
    if (b) b->sample_rate = rate;
}

inline void AAudioStreamBuilder_setChannelCount(AAudioStreamBuilder* b, int32_t channels) {
    if (b) b->channel_count = channels;
}

inline void AAudioStreamBuilder_setBufferCapacityInFrames(AAudioStreamBuilder* b, int32_t frames) {
    if (b) b->buffer_capacity_frames = frames;
}

inline void AAudioStreamBuilder_setDataCallback(AAudioStreamBuilder* b, AAudioStream_dataCallback cb, void* userData) {
    if (b) {
        b->data_callback = cb;
        b->user_data = userData;
    }
}

inline void AAudioStreamBuilder_setErrorCallback(AAudioStreamBuilder* b, AAudioStream_errorCallback cb, void* userData) {
    if (b) {
        b->error_callback = cb;
        b->user_data = userData;
    }
}

inline aaudio_result_t AAudioStreamBuilder_openStream(AAudioStreamBuilder* b, AAudioStream** stream) {
    if (!b || !stream) return AAUDIO_ERROR_INTERNAL;
    auto* s = new AAudioStreamStruct();
    s->config = *b;
    *stream = s;
    return AAUDIO_OK;
}

inline aaudio_result_t AAudioStreamBuilder_delete(AAudioStreamBuilder* b) {
    delete b;
    return AAUDIO_OK;
}

inline aaudio_result_t AAudioStream_requestStart(AAudioStream* s) {
    if (!s) return AAUDIO_ERROR_INVALID_STATE;
    s->is_started = true;
    return AAUDIO_OK;
}

inline aaudio_result_t AAudioStream_requestStop(AAudioStream* s) {
    if (!s) return AAUDIO_ERROR_INVALID_STATE;
    s->is_started = false;
    return AAUDIO_OK;
}

inline aaudio_result_t AAudioStream_close(AAudioStream* s) {
    delete s;
    return AAUDIO_OK;
}

inline int32_t AAudioStream_getSampleRate(AAudioStream* s) {
    return s ? s->config.sample_rate : 48000;
}

inline int32_t AAudioStream_getChannelCount(AAudioStream* s) {
    return s ? s->config.channel_count : 2;
}

inline int32_t AAudioStream_getFramesPerBurst(AAudioStream* s) {
    return s ? (s->config.buffer_capacity_frames / 2) : 192;
}

inline int32_t AAudioStream_getBufferCapacityInFrames(AAudioStream* s) {
    return s ? s->config.buffer_capacity_frames : 384;
}

inline int32_t AAudioStream_getXRunCount(AAudioStream* s) {
    return s ? s->xrun_count : 0;
}

#endif

namespace audio_core {

struct AAudioBackend::Impl {
    AAudioStream* stream{nullptr};
};

static aaudio_data_callback_result_t aaudio_data_callback(
    AAudioStream* /*stream*/,
    void* userData,
    void* audioData,
    int32_t numFrames) {
    auto* self = static_cast<AAudioBackend*>(userData);
    if (!self || !self->is_running()) {
        return AAUDIO_CALLBACK_RESULT_STOP;
    }

    auto* out = static_cast<Sample*>(audioData);
    self->process_callback(out, static_cast<uint32_t>(numFrames));
    return AAUDIO_CALLBACK_RESULT_CONTINUE;
}

void AAudioBackend::process_callback(Sample* out, uint32_t num_frames) noexcept {
    if (m_callback) {
        m_callback(out, num_frames, m_channels);
    } else if (out) {
        std::memset(out, 0, static_cast<size_t>(num_frames) * m_channels * sizeof(Sample));
    }
}

static void aaudio_error_callback(
    AAudioStream* /*stream*/,
    void* userData,
    aaudio_result_t error) {
    auto* self = static_cast<AAudioBackend*>(userData);
    if (!self) return;

    std::cerr << "[AAudioBackend] Error callback received code: " << error << std::endl;
    if (error == AAUDIO_ERROR_DISCONNECTED) {
        std::cerr << "[AAudioBackend] Audio device disconnected (headset unplugged / routed). Stopping stream." << std::endl;
        self->stop();
    }
}

AAudioBackend::AAudioBackend() : m_impl(std::make_unique<Impl>()) {}

AAudioBackend::~AAudioBackend() {
    stop();
    if (m_impl->stream) {
        AAudioStream_close(m_impl->stream);
        m_impl->stream = nullptr;
    }
}

bool AAudioBackend::init(uint32_t sample_rate, uint32_t channels, uint32_t buffer_size) {
    if (m_initialized) {
        stop();
        if (m_impl->stream) {
            AAudioStream_close(m_impl->stream);
            m_impl->stream = nullptr;
        }
        m_initialized = false;
    }

    m_sample_rate = sample_rate;
    m_channels = channels;
    m_buffer_size = buffer_size;

    AAudioStreamBuilder* builder = nullptr;
    aaudio_result_t result = AAudio_createStreamBuilder(&builder);
    if (result != AAUDIO_OK || !builder) {
        std::cerr << "[AAudioBackend] Failed to create stream builder: " << result << std::endl;
        return false;
    }

    AAudioStreamBuilder_setDirection(builder, AAUDIO_DIRECTION_OUTPUT);
    AAudioStreamBuilder_setPerformanceMode(builder, AAUDIO_PERFORMANCE_MODE_LOW_LATENCY);
    AAudioStreamBuilder_setSharingMode(builder, AAUDIO_SHARING_MODE_EXCLUSIVE);
    AAudioStreamBuilder_setFormat(builder, AAUDIO_FORMAT_PCM_FLOAT);
    AAudioStreamBuilder_setSampleRate(builder, static_cast<int32_t>(sample_rate));
    AAudioStreamBuilder_setChannelCount(builder, static_cast<int32_t>(channels));
    AAudioStreamBuilder_setBufferCapacityInFrames(builder, static_cast<int32_t>(buffer_size * 2));
    AAudioStreamBuilder_setDataCallback(builder, aaudio_data_callback, this);
    AAudioStreamBuilder_setErrorCallback(builder, aaudio_error_callback, this);

    result = AAudioStreamBuilder_openStream(builder, &m_impl->stream);
    if (result != AAUDIO_OK) {
        std::cerr << "[AAudioBackend] Exclusive mode unavailable (" << result << "). Falling back to SHARED mode." << std::endl;
        AAudioStreamBuilder_setSharingMode(builder, AAUDIO_SHARING_MODE_SHARED);
        result = AAudioStreamBuilder_openStream(builder, &m_impl->stream);
    }

    AAudioStreamBuilder_delete(builder);

    if (result != AAUDIO_OK || !m_impl->stream) {
        std::cerr << "[AAudioBackend] Failed to open AAudio stream: " << result << std::endl;
        return false;
    }

    m_sample_rate = static_cast<uint32_t>(AAudioStream_getSampleRate(m_impl->stream));
    m_channels = static_cast<uint32_t>(AAudioStream_getChannelCount(m_impl->stream));
    m_buffer_size = static_cast<uint32_t>(AAudioStream_getFramesPerBurst(m_impl->stream));
    m_initialized = true;

    return true;
}

bool AAudioBackend::start() {
    if (!m_initialized || !m_impl->stream) return false;
    if (m_running.load(std::memory_order_relaxed)) return true;

    aaudio_result_t result = AAudioStream_requestStart(m_impl->stream);
    if (result != AAUDIO_OK) {
        std::cerr << "[AAudioBackend] Failed to start AAudio stream: " << result << std::endl;
        return false;
    }

    m_running.store(true, std::memory_order_release);
    return true;
}

void AAudioBackend::stop() {
    if (!m_running.load(std::memory_order_relaxed)) return;
    m_running.store(false, std::memory_order_release);

    if (m_impl->stream) {
        AAudioStream_requestStop(m_impl->stream);
    }
}

int32_t AAudioBackend::xrun_count() const noexcept {
    if (!m_impl->stream) return 0;
    return AAudioStream_getXRunCount(m_impl->stream);
}

int32_t AAudioBackend::buffer_capacity() const noexcept {
    if (!m_impl->stream) return 0;
    return AAudioStream_getBufferCapacityInFrames(m_impl->stream);
}

void AAudioBackend::simulate_render_block(Sample* buffer, uint32_t frames) {
    if (m_callback && buffer && frames > 0) {
        m_callback(buffer, frames, m_channels);
    }
}

} // namespace audio_core
