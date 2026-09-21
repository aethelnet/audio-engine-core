#include "backends/pipewire/pipewire_backend.hpp"

#include <pipewire/pipewire.h>
#include <pipewire/filter.h>
#include <spa/param/audio/dsp-utils.h>
#include <spa/param/audio/format-utils.h>

#include <iostream>
#include <vector>
#include <array>
#include <cstring>
#include <atomic>
#include <thread>
#include <chrono>
#include <cstdio>

namespace audio_core {

struct PipeWireBackend::Impl {
    MixerGraph& mixer;
    std::string node_name;
    uint32_t sample_rate{48000};
    std::atomic<bool> running{false};

    struct pw_thread_loop* loop{nullptr};
    struct pw_filter* filter{nullptr};

    // Output ports (Master Out L / R)
    void* port_out_l{nullptr};
    void* port_out_r{nullptr};

    // Virtual Input Sink ports per track (Track 1..32)
    struct TrackPortPair {
        void* port_in_l{nullptr};
        void* port_in_r{nullptr};
    };
    std::array<TrackPortPair, MixerGraph::kMaxTracks> track_ports{};

    AudioBuffer master_buffer{2, 2048};

    explicit Impl(MixerGraph& m) : mixer(m) {}

    static void on_process(void* userdata, struct spa_io_position* position) noexcept {
        auto* self = static_cast<Impl*>(userdata);
        if (!self || !self->running.load(std::memory_order_relaxed)) {
            return;
        }

        uint32_t n_samples = position ? position->clock.duration : 1024;
        if (n_samples == 0 || n_samples > 2048) {
            n_samples = 1024;
        }

        const uint32_t engine_max = self->mixer.buffer_frames();
        const uint32_t frames_to_process = std::min(n_samples, engine_max);

        // 1. Pull audio from Virtual Input Sinks (external apps patched into tracks)
        for (size_t i = 0; i < MixerGraph::kMaxTracks; ++i) {
            auto* track = self->mixer.get_track(static_cast<uint32_t>(i + 1));
            if (!track || !track->is_active()) {
                continue;
            }

            const auto& ports = self->track_ports[i];
            if (ports.port_in_l && ports.port_in_r) {
                const auto* src_l = static_cast<const float*>(pw_filter_get_dsp_buffer(ports.port_in_l, n_samples));
                const auto* src_r = static_cast<const float*>(pw_filter_get_dsp_buffer(ports.port_in_r, n_samples));

                // Only copy if external ports are actively providing buffers
                if (src_l || src_r) {
                    Sample* dst_l = track->buffer().view().channel(0);
                    Sample* dst_r = track->buffer().view().channel(1);
                    uint32_t track_max = std::min(frames_to_process, track->buffer().num_frames());

                    if (src_l) {
                        std::memcpy(dst_l, src_l, track_max * sizeof(float));
                    }
                    if (src_r) {
                        std::memcpy(dst_r, src_r, track_max * sizeof(float));
                    }
                }
            }
        }

        // 2. Execute Real-Time Channel Strips, Inserts, DAG Submixes & Master Summing
        auto master_view = self->master_buffer.view_frames(frames_to_process);
        self->mixer.render(master_view);

        // 3. Push Master Output to PipeWire Master Out Ports
        if (self->port_out_l && self->port_out_r) {
            auto* dst_l = static_cast<float*>(pw_filter_get_dsp_buffer(self->port_out_l, n_samples));
            auto* dst_r = static_cast<float*>(pw_filter_get_dsp_buffer(self->port_out_r, n_samples));

            const Sample* m_l = master_view.channel(0);
            const Sample* m_r = master_view.channel(1);

            if (dst_l) {
                std::memcpy(dst_l, m_l, frames_to_process * sizeof(float));
                if (n_samples > frames_to_process) {
                    std::memset(dst_l + frames_to_process, 0, (n_samples - frames_to_process) * sizeof(float));
                }
            }
            if (dst_r) {
                std::memcpy(dst_r, m_r, frames_to_process * sizeof(float));
                if (n_samples > frames_to_process) {
                    std::memset(dst_r + frames_to_process, 0, (n_samples - frames_to_process) * sizeof(float));
                }
            }
        }
    }

    static void on_state_changed(void* userdata, enum pw_filter_state old_state,
                                 enum pw_filter_state state, const char* error) noexcept {
        auto* self = static_cast<Impl*>(userdata);
        if (!self) return;

        if (state == PW_FILTER_STATE_ERROR) {
            std::cerr << "[PipeWireBackend] Filter error: " << (error ? error : "unknown") << std::endl;
        } else if (state == PW_FILTER_STATE_STREAMING) {
            self->running.store(true, std::memory_order_relaxed);
        } else if (state == PW_FILTER_STATE_PAUSED) {
            self->running.store(false, std::memory_order_relaxed);
        }
    }
};

PipeWireBackend::PipeWireBackend(MixerGraph& mixer)
    : m_impl(std::make_unique<Impl>(mixer)) {}

PipeWireBackend::~PipeWireBackend() {
    stop();
}

PipeWireBackend::PipeWireBackend(PipeWireBackend&&) noexcept = default;
PipeWireBackend& PipeWireBackend::operator=(PipeWireBackend&&) noexcept = default;

bool PipeWireBackend::init(const std::string& node_name, uint32_t sample_rate) {
    m_impl->node_name = node_name;
    m_impl->sample_rate = sample_rate;

    pw_init(nullptr, nullptr);

    m_impl->loop = pw_thread_loop_new(node_name.c_str(), nullptr);
    if (!m_impl->loop) {
        std::cerr << "[PipeWireBackend] Failed to create pw_thread_loop" << std::endl;
        return false;
    }

    if (pw_thread_loop_start(m_impl->loop) < 0) {
        std::cerr << "[PipeWireBackend] Failed to start pw_thread_loop" << std::endl;
        return false;
    }

    pw_thread_loop_lock(m_impl->loop);

    struct pw_properties* props = pw_properties_new(
        PW_KEY_MEDIA_TYPE, "Audio",
        PW_KEY_MEDIA_CATEGORY, "Filter",
        PW_KEY_MEDIA_ROLE, "DSP",
        PW_KEY_APP_NAME, node_name.c_str(),
        PW_KEY_NODE_NAME, "aethel_mixer_graph",
        PW_KEY_NODE_DESCRIPTION, "Aethel Engine Multitrack Virtual Sinks & Console Desk",
        PW_KEY_NODE_AUTOCONNECT, "true",
        nullptr);

    static const struct pw_filter_events events = {
        .version = PW_VERSION_FILTER_EVENTS,
        .state_changed = Impl::on_state_changed,
        .process = Impl::on_process,
    };

    m_impl->filter = pw_filter_new_simple(
        pw_thread_loop_get_loop(m_impl->loop),
        node_name.c_str(),
        props,
        &events,
        m_impl.get());

    if (!m_impl->filter) {
        pw_thread_loop_unlock(m_impl->loop);
        std::cerr << "[PipeWireBackend] Failed to create pw_filter" << std::endl;
        return false;
    }

    // Prepare 32-bit Float DSP format parameter
    uint8_t buffer[1024];
    struct spa_pod_builder b = SPA_POD_BUILDER_INIT(buffer, sizeof(buffer));
    struct spa_audio_info_dsp info{};
    info.format = SPA_AUDIO_FORMAT_DSP_F32;
    const struct spa_pod* params[1];
    params[0] = spa_format_audio_dsp_build(&b, SPA_PARAM_EnumFormat, &info);

    // 1. Create Master Output Ports
    struct pw_properties* p_out_l = pw_properties_new(
        PW_KEY_PORT_NAME, "Master Out L",
        PW_KEY_PORT_DIRECTION, "out",
        PW_KEY_AUDIO_CHANNEL, "FL",
        nullptr);
    m_impl->port_out_l = pw_filter_add_port(m_impl->filter, PW_DIRECTION_OUTPUT,
                                             PW_FILTER_PORT_FLAG_MAP_BUFFERS, 0, p_out_l, params, 1);

    struct pw_properties* p_out_r = pw_properties_new(
        PW_KEY_PORT_NAME, "Master Out R",
        PW_KEY_PORT_DIRECTION, "out",
        PW_KEY_AUDIO_CHANNEL, "FR",
        nullptr);
    m_impl->port_out_r = pw_filter_add_port(m_impl->filter, PW_DIRECTION_OUTPUT,
                                             PW_FILTER_PORT_FLAG_MAP_BUFFERS, 0, p_out_r, params, 1);

    // 2. Create Virtual Input Sinks for Mixer Tracks (so Bitwig, Renoise, Chrome can patch into any track)
    for (size_t i = 0; i < MixerGraph::kMaxTracks; ++i) {
        uint32_t trk_num = static_cast<uint32_t>(i + 1);
        std::string name_l = "Track " + std::to_string(trk_num) + " In L";
        std::string name_r = "Track " + std::to_string(trk_num) + " In R";

        struct pw_properties* p_in_l = pw_properties_new(
            PW_KEY_PORT_NAME, name_l.c_str(),
            PW_KEY_PORT_DIRECTION, "in",
            PW_KEY_AUDIO_CHANNEL, "FL",
            nullptr);
        m_impl->track_ports[i].port_in_l = pw_filter_add_port(m_impl->filter, PW_DIRECTION_INPUT,
                                                               PW_FILTER_PORT_FLAG_MAP_BUFFERS, 0, p_in_l, params, 1);

        struct pw_properties* p_in_r = pw_properties_new(
            PW_KEY_PORT_NAME, name_r.c_str(),
            PW_KEY_PORT_DIRECTION, "in",
            PW_KEY_AUDIO_CHANNEL, "FR",
            nullptr);
        m_impl->track_ports[i].port_in_r = pw_filter_add_port(m_impl->filter, PW_DIRECTION_INPUT,
                                                               PW_FILTER_PORT_FLAG_MAP_BUFFERS, 0, p_in_r, params, 1);
    }

    pw_thread_loop_unlock(m_impl->loop);
    return true;
}

bool PipeWireBackend::start() {
    if (!m_impl->filter || !m_impl->loop) return false;
    if (m_impl->running.load(std::memory_order_relaxed)) return true;

    pw_thread_loop_lock(m_impl->loop);
    int res = pw_filter_connect(m_impl->filter, PW_FILTER_FLAG_RT_PROCESS, nullptr, 0);
    pw_thread_loop_unlock(m_impl->loop);

    if (res < 0) {
        std::cerr << "[PipeWireBackend] pw_filter_connect failed: " << res << std::endl;
        return false;
    }

    m_impl->running.store(true, std::memory_order_relaxed);

    // Auto-connect Master Output to system physical playback sinks in background
    std::thread([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        FILE* fp = popen("pw-link -i", "r");
        if (!fp) return;
        char line[256];
        std::string sink_l, sink_r;
        while (fgets(line, sizeof(line), fp)) {
            std::string s(line);
            if (!s.empty() && s.back() == '\n') s.pop_back();
            if (s.find("playback_FL") != std::string::npos || s.find("playback_0") != std::string::npos || s.find("playback_1") != std::string::npos) {
                if (sink_l.empty()) sink_l = s;
            } else if (s.find("playback_FR") != std::string::npos || s.find("playback_2") != std::string::npos) {
                if (sink_r.empty()) sink_r = s;
            }
        }
        pclose(fp);

        if (!sink_l.empty()) {
            std::string cmd = "pw-link \"aethel_mixer_graph:Master Out L\" \"" + sink_l + "\" 2>/dev/null";
            (void)system(cmd.c_str());
        }
        if (!sink_r.empty()) {
            std::string cmd = "pw-link \"aethel_mixer_graph:Master Out R\" \"" + sink_r + "\" 2>/dev/null";
            (void)system(cmd.c_str());
        }
    }).detach();

    return true;
}

void PipeWireBackend::stop() {
    if (!m_impl || !m_impl->loop) return;

    m_impl->running.store(false, std::memory_order_relaxed);

    pw_thread_loop_lock(m_impl->loop);
    if (m_impl->filter) {
        pw_filter_disconnect(m_impl->filter);
        pw_filter_destroy(m_impl->filter);
        m_impl->filter = nullptr;
    }
    pw_thread_loop_unlock(m_impl->loop);

    pw_thread_loop_stop(m_impl->loop);
    pw_thread_loop_destroy(m_impl->loop);
    m_impl->loop = nullptr;

    pw_deinit();
}

bool PipeWireBackend::is_running() const noexcept {
    return m_impl && m_impl->running.load(std::memory_order_relaxed);
}

uint32_t PipeWireBackend::sample_rate() const noexcept {
    return m_impl ? m_impl->sample_rate : 48000;
}

} // namespace audio_core
