#include "audio_core/engine.hpp"
#include "audio_core/serialization/session_serializer.hpp"
#include "audio_core/sampling/wav_reader.hpp"

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
      m_planar_buffer(kDefaultChannels, buffer_size),
      m_synth_buffer(kDefaultChannels, buffer_size) {
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
    m_mixer.clock().set_sample_rate(sample_rate);
    m_planar_buffer.resize(kDefaultChannels, buffer_size);
    m_synth_buffer.resize(kDefaultChannels, buffer_size);

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
    m_mixer.clock().set_playing(false);
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
    m_mixer.clock().set_playing(true);
}

void Engine::transport_pause() noexcept {
    m_clock.set_playing(false);
    m_mixer.clock().set_playing(false);
}

void Engine::transport_stop() noexcept {
    m_clock.set_playing(false);
    m_clock.set_sample_position(0);
    m_mixer.clock().set_playing(false);
    m_mixer.clock().set_sample_position(0);
    m_mixer.stop_all_clips();
}

void Engine::transport_seek(double beat) noexcept {
    const uint64_t pos = static_cast<uint64_t>(std::round(beat * m_clock.samples_per_beat()));
    m_clock.set_sample_position(pos);
    m_mixer.clock().set_sample_position(pos);
}

void Engine::set_tempo(double bpm) noexcept {
    m_clock.set_bpm(bpm);
    m_mixer.clock().set_bpm(bpm);
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

    // 4. If SynthGraph has active voices, render and accumulate them into the mix
    if (m_synth_graph.active_voice_count() > 0) {
        auto synth_view = m_synth_buffer.view_frames(num_frames);
        m_synth_graph.render(synth_view);
        const Sample* s_left = synth_view.channel(0);
        const Sample* s_right = (synth_view.num_channels() > 1) ? synth_view.channel(1) : s_left;
        for (uint32_t f = 0; f < num_frames; ++f) {
            left[f] += s_left[f];
            right[f] += s_right[f];
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

BounceResult Engine::render_offline(std::vector<float>& out_left,
                                   std::vector<float>& out_right,
                                   const BounceOptions& options) {
    const auto start_time = std::chrono::steady_clock::now();
    BounceResult result{};
    result.sample_rate = m_sample_rate;

    if (m_running.load(std::memory_order_relaxed)) {
        result.success = false;
        result.error_message = "Cannot perform offline bounce while engine is running real-time playback. Call stop() first.";
        return result;
    }

    // 1. Determine total render length
    uint64_t total_frames = options.total_frames;
    if (total_frames == 0) {
        total_frames = m_mixer.auto_detect_project_frames();
        if (total_frames == 0) {
            total_frames = static_cast<uint64_t>(m_clock.samples_per_bar() * 4);
        }
    }

    // 2. Plugin Delay Compensation (PDC) Pre-Roll Analysis
    m_mixer.update_pdc_delay_compensation();
    const uint32_t pdc_lat = options.apply_pdc_flush ? m_mixer.total_latency_samples() : 0;
    const uint64_t final_target_frames = total_frames + options.tail_frames;

    out_left.clear();
    out_right.clear();
    out_left.reserve(final_target_frames);
    out_right.reserve(final_target_frames);

    // 3. Save initial transport state
    const uint64_t prev_pos = m_clock.sample_position();
    const bool prev_playing = m_clock.is_playing();

    // 4. Initialize transport for offline bounce
    m_clock.set_sample_position(options.start_frame);
    m_clock.set_playing(true);
    m_mixer.clock().set_sample_position(options.start_frame);
    m_mixer.clock().set_playing(true);
    m_mixer.reset_playback_state(static_cast<double>(options.start_frame));

    const uint32_t block_size = m_buffer_size;
    uint32_t pdc_discarded = 0;
    uint64_t timeline_rendered = 0;
    bool tail_phase = false;

    // 5. Offline Render Loop (faster than realtime, zero hardware sync)
    while (out_left.size() < final_target_frames) {
        const uint64_t needed = final_target_frames - out_left.size();
        uint32_t chunk_frames = static_cast<uint32_t>(std::min<uint64_t>(block_size, needed + (pdc_lat > pdc_discarded ? (pdc_lat - pdc_discarded) : 0)));
        if (chunk_frames == 0) break;

        // Drain any pending MIDI events
        MidiEvent midi_ev{};
        while (m_midi_queue.try_pop(midi_ev)) {
            m_synth_graph.handle_midi_event(midi_ev);
        }

        // Drain any pending parameter events
        ParameterEvent param_ev{};
        while (m_param_queue.try_pop(param_ev)) {
            switch (param_ev.parameter_id) {
                case 1: m_mixer.set_master_volume(param_ev.value); break;
                case 2: m_synth_graph.set_global_filter_cutoff(param_ev.value); break;
                case 3: m_synth_graph.set_global_resonance(param_ev.value); break;
                default: break;
            }
        }

        // Drain any pending mixer commands
        protocol::MixerCommand cmd{};
        while (m_command_queue.try_pop(cmd)) {
            m_mixer.post_command(cmd);
        }

        // Check if we entered tail phase: stop triggering new clips once total_frames is reached
        if (!tail_phase && timeline_rendered >= total_frames) {
            tail_phase = true;
            m_mixer.stop_all_clips();
        }

        // Advance timeline clock
        m_clock.advance_block(chunk_frames);

        // Render MixerGraph
        auto view = m_planar_buffer.view_frames(chunk_frames);
        m_mixer.render(view);

        Sample* left = view.channel(0);
        Sample* right = (view.num_channels() > 1) ? view.channel(1) : left;

        // Accumulate Poly-Synth voices if active
        if (m_synth_graph.active_voice_count() > 0) {
            auto synth_view = m_synth_buffer.view_frames(chunk_frames);
            m_synth_graph.render(synth_view);
            const Sample* s_left = synth_view.channel(0);
            const Sample* s_right = (synth_view.num_channels() > 1) ? synth_view.channel(1) : s_left;
            for (uint32_t f = 0; f < chunk_frames; ++f) {
                left[f] += s_left[f];
                right[f] += s_right[f];
            }
        }

        // Master Sandboxed Plugin (if loaded)
        if (m_master_plugin.is_loaded()) {
            m_master_plugin.process_stereo(left, right, left, right, chunk_frames);
        }

        // PDC Latency Flushing & Buffer Appending
        uint32_t frame_offset = 0;
        if (pdc_discarded < pdc_lat) {
            const uint32_t to_discard = std::min(chunk_frames, pdc_lat - pdc_discarded);
            pdc_discarded += to_discard;
            frame_offset = to_discard;
        }

        const uint32_t valid_in_chunk = chunk_frames - frame_offset;
        const uint32_t to_append = static_cast<uint32_t>(std::min<uint64_t>(valid_in_chunk, final_target_frames - out_left.size()));

        for (uint32_t i = frame_offset; i < frame_offset + to_append; ++i) {
            out_left.push_back(left[i]);
            out_right.push_back(right[i]);
        }

        timeline_rendered += chunk_frames;

        if (options.progress_callback && final_target_frames > 0) {
            options.progress_callback(static_cast<float>(out_left.size()) / static_cast<float>(final_target_frames));
        }
    }

    // 6. Optional Peak Normalization
    if (options.normalize && !out_left.empty()) {
        float peak = 0.0f;
        for (size_t i = 0; i < out_left.size(); ++i) {
            const float al = std::abs(out_left[i]);
            const float ar = std::abs(out_right[i]);
            if (al > peak) peak = al;
            if (ar > peak) peak = ar;
        }
        if (peak > 1e-6f) {
            const float target_linear = std::pow(10.0f, options.target_peak_db / 20.0f);
            const float scale = target_linear / peak;
            for (size_t i = 0; i < out_left.size(); ++i) {
                out_left[i] *= scale;
                out_right[i] *= scale;
            }
        }
    }

    // 7. Calculate Audio Metrics
    float max_l = 0.0f, max_r = 0.0f;
    double sum_sq_l = 0.0, sum_sq_r = 0.0;
    for (size_t i = 0; i < out_left.size(); ++i) {
        const float al = std::abs(out_left[i]);
        const float ar = std::abs(out_right[i]);
        if (al > max_l) max_l = al;
        if (ar > max_r) max_r = ar;
        sum_sq_l += static_cast<double>(out_left[i]) * out_left[i];
        sum_sq_r += static_cast<double>(out_right[i]) * out_right[i];
    }

    result.frames_rendered = out_left.size();
    result.peak_l = max_l;
    result.peak_r = max_r;
    result.rms_l = out_left.empty() ? 0.0f : static_cast<float>(std::sqrt(sum_sq_l / out_left.size()));
    result.rms_r = out_right.empty() ? 0.0f : static_cast<float>(std::sqrt(sum_sq_r / out_right.size()));
    result.duration_seconds = static_cast<double>(result.frames_rendered) / static_cast<double>(m_sample_rate);

    const auto end_time = std::chrono::steady_clock::now();
    result.render_time_seconds = std::chrono::duration<double>(end_time - start_time).count();
    result.realtime_factor = (result.render_time_seconds > 0.0) ? (result.duration_seconds / result.render_time_seconds) : 0.0;
    result.success = true;

    // 8. Restore transport state
    m_clock.set_playing(prev_playing);
    m_clock.set_sample_position(prev_pos);
    m_mixer.clock().set_playing(prev_playing);
    m_mixer.clock().set_sample_position(prev_pos);
    m_mixer.reset_playback_state(static_cast<double>(prev_pos));
    m_mixer.stop_all_clips();

    return result;
}

BounceResult Engine::render_offline_wav(const std::string& filepath, const BounceOptions& options) {
    std::vector<float> left, right;
    auto result = render_offline(left, right, options);
    if (!result.success) {
        return result;
    }

    bool ok = sampling::WavReader::save_wav(filepath, left.data(), right.data(), left.size(), m_sample_rate, options.bits_per_sample);
    if (!ok) {
        result.success = false;
        result.error_message = "Failed to write WAV file to disk: " + filepath;
        return result;
    }

    result.output_path = filepath;
    return result;
}

} // namespace audio_core
