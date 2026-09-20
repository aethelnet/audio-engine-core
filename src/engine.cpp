#include "audio_core/engine.hpp"
#include <chrono>

namespace audio_core {

Engine::Engine(uint32_t sample_rate, uint32_t buffer_size)
    : m_sample_rate(sample_rate),
      m_buffer_size(buffer_size),
      m_planar_buffer(kDefaultChannels, buffer_size) {
    m_graph.init(sample_rate);
}

bool Engine::send_midi(const MidiEvent& event) noexcept {
    return m_midi_queue.try_push(event);
}

bool Engine::set_parameter(uint32_t param_id, float value) noexcept {
    return m_param_queue.try_push({param_id, value, 0});
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
        m_graph.handle_midi_event(midi_ev);
    }

    // 2. Drain Parameter queue (lock-free)
    ParameterEvent param_ev{};
    while (m_param_queue.try_pop(param_ev)) {
        switch (param_ev.parameter_id) {
            case 1: // Master Volume
                m_graph.set_master_volume(param_ev.value);
                break;
            case 2: // Filter Cutoff
                m_graph.set_global_filter_cutoff(param_ev.value);
                break;
            case 3: // Filter Resonance
                m_graph.set_global_resonance(param_ev.value);
                break;
            default:
                break;
        }
    }

    // 3. Render audio into internal planar buffer
    auto view = m_planar_buffer.view_frames(num_frames);
    m_graph.render(view);

    // 4. Apply Sandboxed WASM DSP Master Effect (if loaded)
    Sample* left = view.channel(0);
    Sample* right = (view.num_channels() > 1) ? view.channel(1) : left;
    if (m_master_plugin.is_loaded()) {
        m_master_plugin.process_stereo(left, right, left, right, num_frames);
    }

    // 5. Interleave planar channels to hardware output
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
        // Fallback for multi-channel surround
        for (uint32_t i = 0; i < num_frames; ++i) {
            output[i * channels] = left[i];
            if (channels > 1) output[(i * channels) + 1] = right[i];
            for (uint32_t ch = 2; ch < channels; ++ch) {
                output[(i * channels) + ch] = 0.0f;
            }
        }
    }

    // 5. Measure RT execution time & CPU budget
    const auto end_time = std::chrono::steady_clock::now();
    const auto elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
    const double budget_ns = (static_cast<double>(num_frames) / static_cast<double>(m_sample_rate)) * 1e9;

    if (budget_ns > 0.0) {
        const float current_load = static_cast<float>((static_cast<double>(elapsed_ns) / budget_ns) * 100.0);
        // Exponential moving average for smooth telemetry display
        const float prev_load = m_cpu_load.load(std::memory_order_relaxed);
        m_cpu_load.store((prev_load * 0.9f) + (current_load * 0.1f), std::memory_order_relaxed);
    }
}

} // namespace audio_core
