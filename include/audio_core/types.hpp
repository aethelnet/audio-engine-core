#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>
#include <string_view>

namespace audio_core {

using Sample = float;

constexpr uint32_t kDefaultSampleRate = 48000;
constexpr uint32_t kDefaultChannels = 2;
constexpr uint32_t kDefaultBufferSize = 256;

struct ProcessContext {
    uint32_t sample_rate{kDefaultSampleRate};
    uint32_t num_frames{kDefaultBufferSize};
    double bpm{120.0};
    uint64_t sample_position{0};
    bool is_playing{false};
};

// Non-allocating multi-channel view over planar audio data
class AudioBufferView {
public:
    constexpr AudioBufferView(Sample** channel_ptrs, uint32_t num_channels, uint32_t num_frames) noexcept
        : m_channels(channel_ptrs), m_num_channels(num_channels), m_num_frames(num_frames) {}

    [[nodiscard]] constexpr uint32_t num_channels() const noexcept { return m_num_channels; }
    [[nodiscard]] constexpr uint32_t num_frames() const noexcept { return m_num_frames; }

    [[nodiscard]] constexpr Sample* channel(uint32_t ch) noexcept {
        return m_channels[ch];
    }

    [[nodiscard]] constexpr const Sample* channel(uint32_t ch) const noexcept {
        return m_channels[ch];
    }

    void clear() noexcept {
        for (uint32_t ch = 0; ch < m_num_channels; ++ch) {
            Sample* ptr = m_channels[ch];
            for (uint32_t i = 0; i < m_num_frames; ++i) {
                ptr[i] = 0.0f;
            }
        }
    }

private:
    Sample** m_channels{nullptr};
    uint32_t m_num_channels{0};
    uint32_t m_num_frames{0};
};

// Pre-allocated planar audio buffer (Allocated in constructor, never in RT thread)
class AudioBuffer {
public:
    AudioBuffer(uint32_t channels, uint32_t frames)
        : m_channels(channels), m_frames(frames), m_storage(channels * frames, 0.0f), m_channel_ptrs(channels) {
        update_channel_ptrs();
    }

    void resize(uint32_t channels, uint32_t frames) {
        m_channels = channels;
        m_frames = frames;
        m_storage.assign(channels * frames, 0.0f);
        m_channel_ptrs.resize(channels);
        update_channel_ptrs();
    }

    [[nodiscard]] AudioBufferView view() noexcept {
        return AudioBufferView(m_channel_ptrs.data(), m_channels, m_frames);
    }

    [[nodiscard]] AudioBufferView view_frames(uint32_t frames) noexcept {
        return AudioBufferView(m_channel_ptrs.data(), m_channels, frames < m_frames ? frames : m_frames);
    }

    [[nodiscard]] uint32_t num_channels() const noexcept { return m_channels; }
    [[nodiscard]] uint32_t num_frames() const noexcept { return m_frames; }

    void clear() noexcept {
        view().clear();
    }

private:
    void update_channel_ptrs() noexcept {
        for (uint32_t ch = 0; ch < m_channels; ++ch) {
            m_channel_ptrs[ch] = m_storage.data() + (ch * m_frames);
        }
    }

    uint32_t m_channels;
    uint32_t m_frames;
    std::vector<Sample> m_storage;
    std::vector<Sample*> m_channel_ptrs;
};

// MIDI Message representation
enum class MidiStatus : uint8_t {
    NoteOff = 0x80,
    NoteOn = 0x90,
    PolyAftertouch = 0xA0,
    ControlChange = 0xB0,
    ProgramChange = 0xC0,
    ChannelAftertouch = 0xD0,
    PitchBend = 0xE0
};

struct MidiEvent {
    uint32_t frame_offset{0};
    uint8_t status{0};
    uint8_t data1{0};
    uint8_t data2{0};

    [[nodiscard]] constexpr MidiStatus type() const noexcept {
        return static_cast<MidiStatus>(status & 0xF0);
    }
    [[nodiscard]] constexpr uint8_t channel() const noexcept {
        return status & 0x0F;
    }
    [[nodiscard]] constexpr uint8_t note() const noexcept {
        return data1;
    }
    [[nodiscard]] constexpr uint8_t velocity() const noexcept {
        return data2;
    }
};

// Parameter change event for lock-free dispatch
struct ParameterEvent {
    uint32_t parameter_id{0};
    float value{0.0f};
    uint32_t ramp_frames{0}; // 0 = instant, >0 = smooth transition
};

} // namespace audio_core
