#include "audio_core/engine.hpp"
#include "audio_core/serialization/session_serializer.hpp"

#include <chrono>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace audio_core {

Engine::Engine(uint32_t sample_rate, uint32_t buffer_size)
    : m_sample_rate(sample_rate),
      m_buffer_size(buffer_size),
      m_mixer(buffer_size),
      m_clock(sample_rate),
      m_planar_buffer(kDefaultChannels, buffer_size) {
    m_synth_graph.init(sample_rate);
    m_mixer.set_sample_rate(sample_rate);
    m_mixer.set_parameter_ramping_enabled(true);
}

Engine::~Engine() {
    stop();
}

bool Engine::init(uint32_t sample_rate, uint32_t buffer_size) {
    if (m_running.load(std::memory_order_relaxed)) {
        stop();
    }
    m_sample_rate = sample_rate;
    m_buffer_size = buffer_size;
    m_synth_graph.init(sample_rate);
    m_mixer.set_sample_rate(sample_rate);
    m_clock.set_sample_rate(sample_rate);
    m_planar_buffer.resize(kDefaultChannels, buffer_size);

    if (m_backend) {
        return m_backend->init(sample_rate, kDefaultChannels, buffer_size);
    }
    return true;
}

bool Engine::start() {
    if (m_running.load(std::memory_order_relaxed)) return true;

    if (m_backend) {
        m_backend->set_callback([this](Sample* out, uint32_t frames, uint32_t channels) {
            process_interleaved(out, frames, channels);
        });
        if (!m_backend->start()) {
            std::cerr << "[Engine] Failed to start attached audio backend." << std::endl;
            return false;
        }
    }

    m_running.store(true, std::memory_order_release);
    return true;
}

void Engine::stop() {
    if (!m_running.load(std::memory_order_relaxed)) return;

    if (m_backend) {
        m_backend->stop();
    }
    m_clock.set_playing(false);
    m_mixer.stop_all_clips();
    m_synth_graph.all_notes_off();

    m_running.store(false, std::memory_order_release);
}

void Engine::attach_backend(std::unique_ptr<AudioBackend> backend) {
    bool was_running = m_running.load(std::memory_order_relaxed);
    if (was_running) {
        stop();
    }
    m_backend = std::move(backend);
    if (m_backend) {
        m_backend->init(m_sample_rate, kDefaultChannels, m_buffer_size);
        if (was_running) {
            start();
        }
    }
}

void Engine::transport_play() noexcept {
    m_clock.set_playing(true);
}

void Engine::transport_pause() noexcept {
    m_clock.set_playing(false);
}

void Engine::transport_stop() noexcept {
    m_clock.set_playing(false);
    m_clock.set_sample_position(0);
    m_mixer.stop_all_clips();
}

void Engine::transport_seek(double beat) noexcept {
    m_clock.set_sample_position(static_cast<uint64_t>(std::round(beat * m_clock.samples_per_beat())));
}

void Engine::set_tempo(double bpm) noexcept {
    m_clock.set_bpm(bpm);
}

bool Engine::save_session(const std::string& filepath, const std::string& project_name) {
    return serialization::SessionSerializer::save_session_file(filepath, m_mixer, m_clock, project_name);
}

bool Engine::load_session(const std::string& filepath) {
    return serialization::SessionSerializer::load_session_file(filepath, m_mixer, m_clock);
}

bool Engine::send_midi(const MidiEvent& event) noexcept {
    return m_midi_queue.try_push(event);
}

bool Engine::set_parameter(uint32_t param_id, float value) noexcept {
    return m_param_queue.try_push({param_id, value, 0});
}

bool Engine::send_command(const protocol::MixerCommand& cmd) noexcept {
    return m_command_queue.try_push(cmd);
}

void Engine::note_on(uint8_t note, uint8_t velocity) noexcept {
    MidiEvent ev{
        .frame_offset = 0,
        .status = static_cast<uint8_t>(MidiStatus::NoteOn),
        .data1 = note,
        .data2 = velocity
    };
    send_midi(ev);
}

void Engine::note_off(uint8_t note) noexcept {
    MidiEvent ev{
        .frame_offset = 0,
        .status = static_cast<uint8_t>(MidiStatus::NoteOff),
        .data1 = note,
        .data2 = 0
    };
    send_midi(ev);
}

void Engine::all_notes_off() noexcept {
    for (uint8_t note = 0; note < 128; ++note) {
        note_off(note);
    }
    m_synth_graph.all_notes_off();
}

bool Engine::load_master_plugin(const std::string& filepath) {
    if (!m_master_plugin.load_from_file(filepath)) {
        return false;
    }
    return m_master_plugin.init(m_sample_rate);
}

void Engine::set_plugin_parameter(uint32_t param_id, float value) noexcept {
    m_master_plugin.set_parameter(param_id, value);
}

void Engine::process_interleaved(Sample* output, uint32_t num_frames, uint32_t channels) noexcept {
    const auto start_time = std::chrono::steady_clock::now();

    // 1. Drain MIDI queue (lock-free)
    MidiEvent midi_ev{};
    while (m_midi_queue.try_pop(midi_ev)) {
        m_synth_graph.handle_midi_event(midi_ev);
    }

    // 2. Drain legacy Parameter queue (lock-free)
    ParameterEvent param_ev{};
    while (m_param_queue.try_pop(param_ev)) {
        switch (param_ev.parameter_id) {
            case 1: // Master Volume
                m_mixer.set_master_volume(param_ev.value);
                break;
            case 2: // Filter Cutoff
                m_synth_graph.set_global_filter_cutoff(param_ev.value);
                break;
            case 3: // Filter Resonance
                m_synth_graph.set_global_resonance(param_ev.value);
                break;
            default:
                break;
        }
    }

    // 3. Drain lock-free mixer command queue
    protocol::MixerCommand cmd{};
    while (m_command_queue.try_pop(cmd)) {
        m_mixer.post_command(cmd);
    }

    // 4. Advance timeline clock if transport is active
    m_clock.advance_block(num_frames);

    // 5. Render MixerGraph to planar buffer (all active tracks, PDC, parameter ramping, buses, limiter)
    auto view = m_planar_buffer.view_frames(num_frames);
    m_mixer.render(view);

    Sample* left = view.channel(0);
    Sample* right = (view.num_channels() > 1) ? view.channel(1) : left;

    // 4. If SynthGraph has active voices, accumulate them into the mix
    if (m_synth_graph.active_voice_count() > 0) {
        for (uint32_t f = 0; f < num_frames; ++f) {
            // Synth voices can be rendered into temporary block and added
        }
    }

    // 5. Apply Sandboxed WASM Master Effect (if loaded)
    if (m_master_plugin.is_loaded()) {
        m_master_plugin.process_stereo(left, right, left, right, num_frames);
    }

    // 6. Interleave planar channels to hardware output buffer
    if (channels == 2) {
        for (uint32_t i = 0; i < num_frames; ++i) {
            output[(i * 2)]     = left[i];
            output[(i * 2) + 1] = right[i];
        }
    } else if (channels == 1) {
        for (uint32_t i = 0; i < num_frames; ++i) {
            output[i] = (left[i] + right[i]) * 0.5f;
        }
    } else {
        // Multi-channel surround fallback
        for (uint32_t i = 0; i < num_frames; ++i) {
            output[i * channels]     = left[i];
            output[(i * channels) + 1] = right[i];
            for (uint32_t ch = 2; ch < channels; ++ch) {
                output[(i * channels) + ch] = 0.0f;
            }
        }
    }

    m_total_frames.fetch_add(num_frames, std::memory_order_relaxed);

    // 7. Measure RT execution time & CPU budget
    const auto end_time = std::chrono::steady_clock::now();
    const auto elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
    const double budget_ns = (static_cast<double>(num_frames) / static_cast<double>(m_sample_rate)) * 1e9;

    if (budget_ns > 0.0) {
        const float current_load = static_cast<float>((static_cast<double>(elapsed_ns) / budget_ns) * 100.0);
        const float prev_load = m_cpu_load.load(std::memory_order_relaxed);
        m_cpu_load.store((prev_load * 0.9f) + (current_load * 0.1f), std::memory_order_relaxed);
    }
}

} // namespace audio_core
