#pragma once

#include "audio_core/types.hpp"
#include "audio_core/dsp/console_processor.hpp"
#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <numbers>
#include <algorithm>
#include <atomic>

namespace audio_core {

struct MeterLevels {
    float peak_l{0.0f};
    float peak_r{0.0f};
    float rms_l{0.0f};
    float rms_r{0.0f};
};

// ============================================================================
// Track: A single audio/instrument channel in the Mixer Graph
// ============================================================================
class Track {
public:
    Track(uint32_t id, std::string name, uint32_t buffer_frames = 1024)
        : m_id(id), m_name(std::move(name)), m_buffer(2, buffer_frames) {
        m_console.set_mode(dsp::ConsoleMode::Channel);
    }

    [[nodiscard]] uint32_t id() const noexcept { return m_id; }
    [[nodiscard]] const std::string& name() const noexcept { return m_name; }

    void set_gain(float gain) noexcept { m_gain.store(std::max(0.0f, gain), std::memory_order_relaxed); }
    [[nodiscard]] float gain() const noexcept { return m_gain.load(std::memory_order_relaxed); }

    void set_pan(float pan) noexcept { m_pan.store(std::clamp(pan, -1.0f, 1.0f), std::memory_order_relaxed); }
    [[nodiscard]] float pan() const noexcept { return m_pan.load(std::memory_order_relaxed); }

    void set_mute(bool mute) noexcept { m_mute.store(mute, std::memory_order_relaxed); }
    [[nodiscard]] bool is_muted() const noexcept { return m_mute.load(std::memory_order_relaxed); }

    void set_solo(bool solo) noexcept { m_solo.store(solo, std::memory_order_relaxed); }
    [[nodiscard]] bool is_solo() const noexcept { return m_solo.load(std::memory_order_relaxed); }

    void set_target_bus(int32_t bus_id) noexcept { m_target_bus.store(bus_id, std::memory_order_relaxed); }
    [[nodiscard]] int32_t target_bus() const noexcept { return m_target_bus.load(std::memory_order_relaxed); }

    void set_send(uint32_t bus_id, float amount, bool pre_fader = false) noexcept {
        for (auto& s : m_sends) {
            if (s.bus_id == bus_id) {
                s.amount = std::clamp(amount, 0.0f, 2.0f);
                s.pre_fader = pre_fader;
                return;
            }
        }
        m_sends.push_back({bus_id, std::clamp(amount, 0.0f, 2.0f), pre_fader});
    }

    void reset_meters() noexcept {
        m_meter_peak_l.store(0.0f, std::memory_order_relaxed);
        m_meter_peak_r.store(0.0f, std::memory_order_relaxed);
        m_meter_rms_l.store(0.0f, std::memory_order_relaxed);
        m_meter_rms_r.store(0.0f, std::memory_order_relaxed);
    }

    void set_console_type(dsp::ConsoleType type) noexcept {
        m_console.set_type(type);
    }
    [[nodiscard]] dsp::ConsoleType console_type() const noexcept {
        return m_console.type();
    }

    [[nodiscard]] AudioBuffer& buffer() noexcept { return m_buffer; }
    [[nodiscard]] const AudioBuffer& buffer() const noexcept { return m_buffer; }

    [[nodiscard]] MeterLevels meter() const noexcept {
        return MeterLevels{
            m_meter_peak_l.load(std::memory_order_relaxed),
            m_meter_peak_r.load(std::memory_order_relaxed),
            m_meter_rms_l.load(std::memory_order_relaxed),
            m_meter_rms_r.load(std::memory_order_relaxed)
        };
    }

    // Called inside the RT render loop
    void process_channel_strip(uint32_t frames) noexcept {
        Sample* left = m_buffer.view().channel(0);
        Sample* right = m_buffer.view().channel(1);

        // 1. In-line Console Encode (Airwindows EveryConsole)
        m_console.process_stereo(left, right, frames);

        // 2. Measure Telemetry (Peak & RMS)
        float peak_l = 0.0f, peak_r = 0.0f;
        float sum_sq_l = 0.0f, sum_sq_r = 0.0f;

        for (uint32_t i = 0; i < frames; ++i) {
            float abs_l = std::abs(left[i]);
            float abs_r = std::abs(right[i]);
            if (abs_l > peak_l) peak_l = abs_l;
            if (abs_r > peak_r) peak_r = abs_r;
            sum_sq_l += left[i] * left[i];
            sum_sq_r += right[i] * right[i];
        }

        float rms_l = std::sqrt(sum_sq_l / static_cast<float>(frames));
        float rms_r = std::sqrt(sum_sq_r / static_cast<float>(frames));

        m_meter_peak_l.store(peak_l, std::memory_order_relaxed);
        m_meter_peak_r.store(peak_r, std::memory_order_relaxed);
        m_meter_rms_l.store(rms_l, std::memory_order_relaxed);
        m_meter_rms_r.store(rms_r, std::memory_order_relaxed);
    }

    struct SendInfo {
        uint32_t bus_id;
        float amount;
        bool pre_fader{false};
    };
    [[nodiscard]] const std::vector<SendInfo>& sends() const noexcept { return m_sends; }

private:
    uint32_t m_id;
    std::string m_name;
    AudioBuffer m_buffer;

    std::atomic<float> m_gain{1.0f};
    std::atomic<float> m_pan{0.0f};
    std::atomic<bool> m_mute{false};
    std::atomic<bool> m_solo{false};
    std::atomic<int32_t> m_target_bus{-1}; // -1 = Direct to Master

    std::vector<SendInfo> m_sends;
    dsp::ConsoleProcessor m_console;

    std::atomic<float> m_meter_peak_l{0.0f};
    std::atomic<float> m_meter_peak_r{0.0f};
    std::atomic<float> m_meter_rms_l{0.0f};
    std::atomic<float> m_meter_rms_r{0.0f};
};

// ============================================================================
// AudioBus: Submix or Master Summing Bus
// ============================================================================
class AudioBus {
public:
    AudioBus(uint32_t id, std::string name, uint32_t buffer_frames = 1024)
        : m_id(id), m_name(std::move(name)), m_buffer(2, buffer_frames) {
        m_console.set_mode(dsp::ConsoleMode::Buss);
    }

    [[nodiscard]] uint32_t id() const noexcept { return m_id; }
    [[nodiscard]] const std::string& name() const noexcept { return m_name; }

    void set_gain(float gain) noexcept { m_gain.store(std::max(0.0f, gain), std::memory_order_relaxed); }
    [[nodiscard]] float gain() const noexcept { return m_gain.load(std::memory_order_relaxed); }

    void set_console_type(dsp::ConsoleType type) noexcept {
        m_console.set_type(type);
    }
    [[nodiscard]] dsp::ConsoleType console_type() const noexcept {
        return m_console.type();
    }

    [[nodiscard]] AudioBuffer& buffer() noexcept { return m_buffer; }
    [[nodiscard]] const AudioBuffer& buffer() const noexcept { return m_buffer; }

    [[nodiscard]] MeterLevels meter() const noexcept {
        return MeterLevels{
            m_meter_peak_l.load(std::memory_order_relaxed),
            m_meter_peak_r.load(std::memory_order_relaxed),
            m_meter_rms_l.load(std::memory_order_relaxed),
            m_meter_rms_r.load(std::memory_order_relaxed)
        };
    }

    void clear() noexcept {
        m_buffer.clear();
    }

    void process_buss_strip(uint32_t frames) noexcept {
        Sample* left = m_buffer.view().channel(0);
        Sample* right = m_buffer.view().channel(1);

        // 1. In-line Console Decode (Reciprocal Airwindows expansion)
        m_console.process_stereo(left, right, frames);

        // 2. Telemetry
        float peak_l = 0.0f, peak_r = 0.0f;
        float sum_sq_l = 0.0f, sum_sq_r = 0.0f;

        for (uint32_t i = 0; i < frames; ++i) {
            float abs_l = std::abs(left[i]);
            float abs_r = std::abs(right[i]);
            if (abs_l > peak_l) peak_l = abs_l;
            if (abs_r > peak_r) peak_r = abs_r;
            sum_sq_l += left[i] * left[i];
            sum_sq_r += right[i] * right[i];
        }

        m_meter_peak_l.store(peak_l, std::memory_order_relaxed);
        m_meter_peak_r.store(peak_r, std::memory_order_relaxed);
        m_meter_rms_l.store(std::sqrt(sum_sq_l / static_cast<float>(frames)), std::memory_order_relaxed);
        m_meter_rms_r.store(std::sqrt(sum_sq_r / static_cast<float>(frames)), std::memory_order_relaxed);
    }

private:
    uint32_t m_id;
    std::string m_name;
    AudioBuffer m_buffer;

    std::atomic<float> m_gain{1.0f};
    dsp::ConsoleProcessor m_console;

    std::atomic<float> m_meter_peak_l{0.0f};
    std::atomic<float> m_meter_peak_r{0.0f};
    std::atomic<float> m_meter_rms_l{0.0f};
    std::atomic<float> m_meter_rms_r{0.0f};
};

// ============================================================================
// MixerGraph: Multi-Track, Submix Bus, and Master Summing Engine
// ============================================================================
class MixerGraph {
public:
    explicit MixerGraph(uint32_t buffer_frames = 1024)
        : m_buffer_frames(buffer_frames), m_master_bus(0, "Master", buffer_frames) {}

    Track* add_track(const std::string& name) {
        uint32_t new_id = static_cast<uint32_t>(m_tracks.size() + 1);
        m_tracks.push_back(std::make_unique<Track>(new_id, name, m_buffer_frames));
        return m_tracks.back().get();
    }

    AudioBus* add_submix_bus(const std::string& name) {
        uint32_t new_id = static_cast<uint32_t>(m_buses.size() + 1);
        m_buses.push_back(std::make_unique<AudioBus>(new_id, name, m_buffer_frames));
        return m_buses.back().get();
    }

    [[nodiscard]] Track* get_track(uint32_t id) noexcept {
        for (auto& t : m_tracks) {
            if (t->id() == id) return t.get();
        }
        return nullptr;
    }

    [[nodiscard]] AudioBus* get_bus(uint32_t id) noexcept {
        for (auto& b : m_buses) {
            if (b->id() == id) return b.get();
        }
        return nullptr;
    }

    [[nodiscard]] AudioBus& master_bus() noexcept { return m_master_bus; }
    [[nodiscard]] const AudioBus& master_bus() const noexcept { return m_master_bus; }
    [[nodiscard]] size_t track_count() const noexcept { return m_tracks.size(); }
    [[nodiscard]] size_t bus_count() const noexcept { return m_buses.size(); }

    // Constant-power panning law
    static inline std::pair<float, float> calculate_pan_gains(float pan) noexcept {
        // pan in [-1.0, 1.0] -> angle in [0, pi/2]
        float theta = (pan + 1.0f) * 0.25f * std::numbers::pi_v<float>;
        return {std::cos(theta), std::sin(theta)};
    }

    void set_master_limiter_enabled(bool enabled) noexcept {
        m_limiter_enabled.store(enabled, std::memory_order_relaxed);
    }
    [[nodiscard]] bool is_master_limiter_enabled() const noexcept {
        return m_limiter_enabled.load(std::memory_order_relaxed);
    }

    // Real-Time Render Pipeline: Fully lock-free, zero allocation
    void render(AudioBufferView& out_master) noexcept {
        const uint32_t frames = std::min(out_master.num_frames(), m_buffer_frames);

        // 1. Clear Master and Submix Buses
        m_master_bus.clear();
        for (auto& bus : m_buses) {
            bus->clear();
        }

        // 2. Evaluate Solo state
        bool any_solo = false;
        for (const auto& track : m_tracks) {
            if (track->is_solo()) {
                any_solo = true;
                break;
            }
        }

        // 3. Process each Track and Route/Accumulate
        for (auto& track : m_tracks) {
            if (track->is_muted() || (any_solo && !track->is_solo())) {
                track->reset_meters();
                continue;
            }

            // In-line Channel Strip processing
            track->process_channel_strip(frames);

            const float gain = track->gain();
            const auto [pan_l, pan_r] = calculate_pan_gains(track->pan());
            const float left_gain = gain * pan_l;
            const float right_gain = gain * pan_r;

            const Sample* trk_l = track->buffer().view().channel(0);
            const Sample* trk_r = track->buffer().view().channel(1);

            // Routing destination: Submix bus or Master
            const int32_t target_id = track->target_bus();
            AudioBus* target_bus = (target_id > 0) ? get_bus(static_cast<uint32_t>(target_id)) : &m_master_bus;
            if (!target_bus) target_bus = &m_master_bus;

            Sample* dst_l = target_bus->buffer().view().channel(0);
            Sample* dst_r = target_bus->buffer().view().channel(1);

            for (uint32_t i = 0; i < frames; ++i) {
                dst_l[i] += trk_l[i] * left_gain;
                dst_r[i] += trk_r[i] * right_gain;
            }

            // Auxiliary Sends (e.g. Reverb / Delay Busses)
            for (const auto& send : track->sends()) {
                AudioBus* send_bus = get_bus(send.bus_id);
                if (send_bus && send.amount > 0.0f) {
                    Sample* s_l = send_bus->buffer().view().channel(0);
                    Sample* s_r = send_bus->buffer().view().channel(1);
                    float s_gain = send.pre_fader ? send.amount : (gain * send.amount);
                    for (uint32_t i = 0; i < frames; ++i) {
                        s_l[i] += trk_l[i] * s_gain * pan_l;
                        s_r[i] += trk_r[i] * s_gain * pan_r;
                    }
                }
            }
        }

        // 4. Process Submix Buses and Accumulate into Master
        for (auto& bus : m_buses) {
            bus->process_buss_strip(frames);

            const float bus_gain = bus->gain();
            const Sample* b_l = bus->buffer().view().channel(0);
            const Sample* b_r = bus->buffer().view().channel(1);

            Sample* m_l = m_master_bus.buffer().view().channel(0);
            Sample* m_r = m_master_bus.buffer().view().channel(1);

            for (uint32_t i = 0; i < frames; ++i) {
                m_l[i] += b_l[i] * bus_gain;
                m_r[i] += b_r[i] * bus_gain;
            }
        }

        // 5. Process Master Bus Strip (Console Decode + Final Peak Safety)
        m_master_bus.process_buss_strip(frames);

        const float master_gain = m_master_bus.gain();
        const Sample* final_l = m_master_bus.buffer().view().channel(0);
        const Sample* final_r = m_master_bus.buffer().view().channel(1);

        Sample* out_l = out_master.channel(0);
        Sample* out_r = (out_master.num_channels() > 1) ? out_master.channel(1) : out_l;

        const bool limiter = m_limiter_enabled.load(std::memory_order_relaxed);

        // Copy to output buffer with optional transparent threshold limiter
        for (uint32_t i = 0; i < frames; ++i) {
            float val_l = final_l[i] * master_gain;
            float val_r = final_r[i] * master_gain;

            if (limiter) {
                // Transparent bit-accurate passthrough below 0.95, smooth asymptotic tanh ceiling above
                if (val_l > 0.95f) val_l = 0.95f + 0.05f * std::tanh((val_l - 0.95f) / 0.05f);
                else if (val_l < -0.95f) val_l = -0.95f + 0.05f * std::tanh((val_l + 0.95f) / 0.05f);

                if (val_r > 0.95f) val_r = 0.95f + 0.05f * std::tanh((val_r - 0.95f) / 0.05f);
                else if (val_r < -0.95f) val_r = -0.95f + 0.05f * std::tanh((val_r + 0.95f) / 0.05f);
            }

            out_l[i] = val_l;
            if (out_master.num_channels() > 1) {
                out_r[i] = val_r;
            }
        }
    }

private:
    uint32_t m_buffer_frames;
    std::atomic<bool> m_limiter_enabled{true};
    std::vector<std::unique_ptr<Track>> m_tracks;
    std::vector<std::unique_ptr<AudioBus>> m_buses;
    AudioBus m_master_bus;
};

} // namespace audio_core
