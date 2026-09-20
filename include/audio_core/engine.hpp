#pragma once

#include "audio_core/types.hpp"
#include "audio_core/ring_buffer.hpp"
#include "audio_core/audio_graph.hpp"
#include "audio_core/wasm_host.hpp"
#include <chrono>
#include <atomic>

namespace audio_core {

class Engine {
public:
    explicit Engine(uint32_t sample_rate = kDefaultSampleRate, uint32_t buffer_size = kDefaultBufferSize);
    ~Engine() = default;

    // Called from Non-RT Thread (UI / MIDI / Network)
    bool send_midi(const MidiEvent& event) noexcept;
    bool set_parameter(uint32_t param_id, float value) noexcept;
    void note_on(uint8_t note, uint8_t velocity) noexcept;
    void note_off(uint8_t note) noexcept;
    void all_notes_off() noexcept;

    // Master Sandboxed Plugin Slot
    bool load_master_plugin(const std::string& filepath);
    void set_plugin_parameter(uint32_t param_id, float value) noexcept;

    // Called strictly from the Real-Time Audio Callback Thread
    void process_interleaved(Sample* output, uint32_t num_frames, uint32_t channels) noexcept;

    // Real-Time Stats (thread-safe reads)
    [[nodiscard]] float cpu_load_percent() const noexcept {
        return m_cpu_load.load(std::memory_order_relaxed);
    }
    [[nodiscard]] uint32_t sample_rate() const noexcept {
        return m_sample_rate;
    }
    [[nodiscard]] uint32_t buffer_size() const noexcept {
        return m_buffer_size;
    }

private:
    uint32_t m_sample_rate;
    uint32_t m_buffer_size;

    AudioGraph m_graph;
    AudioBuffer m_planar_buffer;
    WasmDspPlugin m_master_plugin;

    // Lock-Free Queues (SPSC)
    RingBuffer<MidiEvent> m_midi_queue{1024};
    RingBuffer<ParameterEvent> m_param_queue{1024};

    // Performance telemetry
    std::atomic<float> m_cpu_load{0.0f};
};

} // namespace audio_core
