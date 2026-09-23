#pragma once

#include "audio_core/types.hpp"
#include "audio_core/ring_buffer.hpp"
#include "audio_core/audio_graph.hpp"
#include "audio_core/mixer_graph.hpp"
#include "audio_core/clock/timeline_clock.hpp"
#include "audio_core/wasm_host.hpp"
#include "audio_core/protocol/command_packet.hpp"
#include "backends/audio_backend.hpp"

#include <chrono>
#include <atomic>
#include <memory>
#include <string>

namespace audio_core {

class Engine {
public:
    explicit Engine(uint32_t sample_rate = kDefaultSampleRate, uint32_t buffer_size = kDefaultBufferSize);
    ~Engine();

    bool init(uint32_t sample_rate = kDefaultSampleRate, uint32_t buffer_size = kDefaultBufferSize);
    bool start();
    void stop();
    [[nodiscard]] bool is_running() const noexcept { return m_running.load(std::memory_order_relaxed); }

    // Core Subsystem Accessors
    [[nodiscard]] MixerGraph& mixer() noexcept { return m_mixer; }
    [[nodiscard]] const MixerGraph& mixer() const noexcept { return m_mixer; }

    [[nodiscard]] clock::TimelineClock& clock() noexcept { return m_clock; }
    [[nodiscard]] const clock::TimelineClock& clock() const noexcept { return m_clock; }

    [[nodiscard]] AudioGraph& synth() noexcept { return m_synth_graph; }
    [[nodiscard]] const AudioGraph& synth() const noexcept { return m_synth_graph; }

    // Backend Interface (Desktop/PipeWire/Headless)
    void attach_backend(std::unique_ptr<AudioBackend> backend);
    [[nodiscard]] AudioBackend* backend() const noexcept { return m_backend.get(); }

    // Transport & Playhead Controls
    void transport_play() noexcept;
    void transport_pause() noexcept;
    void transport_stop() noexcept;
    void transport_seek(double beat) noexcept;
    void set_tempo(double bpm) noexcept;

    // Session Persistence
    bool save_session(const std::string& filepath, const std::string& project_name = "Untitled Project");
    bool load_session(const std::string& filepath);

    // Master Volume Controls
    void set_master_volume(float vol) noexcept { m_mixer.set_master_volume(vol); }
    [[nodiscard]] float master_volume() const noexcept { return m_mixer.master_volume(); }

    // Non-RT Command & MIDI Ingest (Thread-Safe SPSC)
    bool send_midi(const MidiEvent& event) noexcept;
    bool set_parameter(uint32_t param_id, float value) noexcept;
    bool send_command(const protocol::MixerCommand& cmd) noexcept;
    void note_on(uint8_t note, uint8_t velocity) noexcept;
    void note_off(uint8_t note) noexcept;
    void all_notes_off() noexcept;

    // Master Sandboxed Plugin Slot (Legacy & Experimental)
    bool load_master_plugin(const std::string& filepath);
    void set_plugin_parameter(uint32_t param_id, float value) noexcept;

    // Called strictly from the Real-Time Audio Callback Thread
    void process_interleaved(Sample* output, uint32_t num_frames, uint32_t channels) noexcept;

    // Real-Time Stats (thread-safe reads)
    [[nodiscard]] float cpu_load_percent() const noexcept {
        return m_cpu_load.load(std::memory_order_relaxed);
    }
    [[nodiscard]] uint32_t sample_rate() const noexcept { return m_sample_rate; }
    [[nodiscard]] uint32_t buffer_size() const noexcept { return m_buffer_size; }
    [[nodiscard]] uint64_t total_frames_rendered() const noexcept {
        return m_total_frames.load(std::memory_order_relaxed);
    }

private:
    uint32_t m_sample_rate;
    uint32_t m_buffer_size;
    std::atomic<bool> m_running{false};

    MixerGraph m_mixer;
    clock::TimelineClock m_clock;
    AudioGraph m_synth_graph; // Embedded polyphonic synth
    std::unique_ptr<AudioBackend> m_backend;

    AudioBuffer m_planar_buffer;
    WasmDspPlugin m_master_plugin;

    // Lock-Free Queues (SPSC)
    RingBuffer<MidiEvent> m_midi_queue{1024};
    RingBuffer<ParameterEvent> m_param_queue{1024};
    RingBuffer<protocol::MixerCommand> m_command_queue{1024};

    // Performance telemetry
    std::atomic<float> m_cpu_load{0.0f};
    std::atomic<uint64_t> m_total_frames{0};
};

} // namespace audio_core
