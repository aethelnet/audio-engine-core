#pragma once

#include "audio_core/types.hpp"
#include "audio_core/ring_buffer.hpp"
#include "audio_core/insert_slot.hpp"
#include "audio_core/dsp/console_processor.hpp"
#include "audio_core/protocol/command_packet.hpp"
#include "audio_core/protocol/telemetry_packet.hpp"
#include "audio_core/sampling/sample_tap.hpp"
#include "audio_core/network/aoip_receiver.hpp"
#include "audio_core/clock/timeline_clock.hpp"
#include "audio_core/sequencer/step_sequencer.hpp"
#include "audio_core/threading/audio_worker_pool.hpp"
#include <string>
#include <vector>
#include <array>
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
// Pre-allocated in fixed pool; zero allocations during playback
// ============================================================================
class Track {
public:
    Track(uint32_t id, std::string name, uint32_t buffer_frames = 1024)
        : m_id(id), m_name(std::move(name)), m_buffer(2, buffer_frames) {
        m_console.set_mode(dsp::ConsoleMode::Channel);
    }

    [[nodiscard]] uint32_t id() const noexcept { return m_id; }
    [[nodiscard]] const std::string& name() const noexcept { return m_name; }
    void set_name(std::string name) noexcept { m_name = std::move(name); }

    [[nodiscard]] bool is_active() const noexcept { return m_active.load(std::memory_order_relaxed); }
    void set_active(bool active) noexcept { m_active.store(active, std::memory_order_relaxed); }

    void activate(uint32_t id, std::string name) noexcept {
        m_id = id;
        m_name = std::move(name);
        m_gain.store(1.0f, std::memory_order_relaxed);
        m_pan.store(0.0f, std::memory_order_relaxed);
        m_mute.store(false, std::memory_order_relaxed);
        m_solo.store(false, std::memory_order_relaxed);
        m_target_bus.store(-1, std::memory_order_relaxed);
        for (auto& s : m_sends) {
            s.active = false;
        }
        m_clip.reset();
        m_clip_playhead.store(0.0, std::memory_order_relaxed);
        m_buffer.clear();
        reset_meters();
        m_active.store(true, std::memory_order_release);
    }

    void deactivate() noexcept {
        m_active.store(false, std::memory_order_release);
        for (auto& s : m_sends) {
            s.active = false;
        }
        m_clip.reset();
        m_clip_playhead.store(0.0, std::memory_order_relaxed);
        m_buffer.clear();
        reset_meters();
    }

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
            if (s.active && s.bus_id == bus_id) {
                if (amount <= 0.0001f) {
                    s.active = false;
                } else {
                    s.amount = std::clamp(amount, 0.0f, 2.0f);
                    s.pre_fader = pre_fader;
                }
                return;
            }
        }
        if (amount > 0.0001f) {
            for (auto& s : m_sends) {
                if (!s.active) {
                    s.bus_id = bus_id;
                    s.amount = std::clamp(amount, 0.0f, 2.0f);
                    s.pre_fader = pre_fader;
                    s.active = true;
                    return;
                }
            }
        }
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

    // Insert Slots (Pre-Console EQ, Dynamics, Saturation, Wasm)
    [[nodiscard]] InsertSlot& slot(size_t index) noexcept { return m_slots[index]; }
    [[nodiscard]] const InsertSlot& slot(size_t index) const noexcept { return m_slots[index]; }

    [[nodiscard]] AudioBuffer& buffer() noexcept { return m_buffer; }
    [[nodiscard]] const AudioBuffer& buffer() const noexcept { return m_buffer; }

    void set_clip(std::shared_ptr<sampling::AudioClip> clip, bool loop = true) noexcept {
        m_clip = std::move(clip);
        m_clip_loop.store(loop, std::memory_order_relaxed);
        m_clip_playhead.store(0.0, std::memory_order_relaxed);
    }

    void clear_clip() noexcept {
        m_clip.reset();
        m_clip_playhead.store(0.0, std::memory_order_relaxed);
    }

    [[nodiscard]] bool has_clip() const noexcept {
        return m_clip != nullptr;
    }

    [[nodiscard]] uint64_t clip_playhead() const noexcept {
        return static_cast<uint64_t>(std::round(m_clip_playhead.load(std::memory_order_relaxed)));
    }

    [[nodiscard]] double clip_playhead_f() const noexcept {
        return m_clip_playhead.load(std::memory_order_relaxed);
    }

    void set_clip_playhead(double playhead) noexcept {
        m_clip_playhead.store(playhead, std::memory_order_relaxed);
    }

    void set_sequencer(std::shared_ptr<sequencer::StepSequencer> seq) noexcept {
        m_sequencer = std::move(seq);
        m_sequencer_enabled.store(m_sequencer != nullptr, std::memory_order_relaxed);
    }

    void enable_sequencer(bool enable) noexcept {
        m_sequencer_enabled.store(enable, std::memory_order_relaxed);
    }

    [[nodiscard]] sequencer::StepSequencer* sequencer() noexcept { return m_sequencer.get(); }
    [[nodiscard]] const sequencer::StepSequencer* sequencer() const noexcept { return m_sequencer.get(); }
    [[nodiscard]] bool is_sequencer_enabled() const noexcept {
        return m_sequencer_enabled.load(std::memory_order_relaxed) && (m_sequencer != nullptr);
    }

    // Called inside RT render loop before channel strip processing
    void render_input(uint32_t frames, const clock::TimelineClock& clock,
                      const clock::BlockBoundaryEvents& boundary_events) noexcept {
        Sample* left = m_buffer.view().channel(0);
        Sample* right = m_buffer.view().channel(1);

        if (is_sequencer_enabled()) {
            m_sequencer->render(left, right, frames, clock, boundary_events);
        } else if (m_clip) {
            double ph = m_clip_playhead.load(std::memory_order_relaxed);
            m_clip->read_resampled(ph, clock.sample_rate(), left, right, frames, m_clip_loop.load(std::memory_order_relaxed));
            m_clip_playhead.store(ph, std::memory_order_relaxed);
        }
    }

    void render_input(uint32_t frames) noexcept {
        if (m_clip) {
            Sample* left = m_buffer.view().channel(0);
            Sample* right = m_buffer.view().channel(1);
            double ph = m_clip_playhead.load(std::memory_order_relaxed);
            m_clip->read_resampled(ph, m_clip->sample_rate(), left, right, frames, m_clip_loop.load(std::memory_order_relaxed));
            m_clip_playhead.store(ph, std::memory_order_relaxed);
        }
    }

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

        // 1. Process Modular Insert Slots (Baxandall EQ, ButterComp2, PurestDrive, WASM)
        for (auto& slot : m_slots) {
            slot.process_stereo(left, right, frames);
        }

        // 2. In-line Console Encode (Airwindows EveryConsole)
        m_console.process_stereo(left, right, frames);

        // 3. Measure Telemetry (Peak & RMS)
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

    static constexpr size_t kMaxTrackSends = 4;
    struct SendInfo {
        uint32_t bus_id{0};
        float amount{0.0f};
        bool pre_fader{false};
        bool active{false};
    };
    [[nodiscard]] const std::array<SendInfo, kMaxTrackSends>& sends() const noexcept { return m_sends; }

private:
    uint32_t m_id;
    std::string m_name;
    AudioBuffer m_buffer;

    std::atomic<bool> m_active{false};
    std::atomic<float> m_gain{1.0f};
    std::atomic<float> m_pan{0.0f};
    std::atomic<bool> m_mute{false};
    std::atomic<bool> m_solo{false};
    std::atomic<int32_t> m_target_bus{-1}; // -1 = Direct to Master

    std::array<InsertSlot, kMaxTrackInsertSlots> m_slots;
    std::array<SendInfo, kMaxTrackSends> m_sends{};
    dsp::ConsoleProcessor m_console;

    std::shared_ptr<sampling::AudioClip> m_clip{nullptr};
    std::atomic<bool> m_clip_loop{true};
    std::atomic<double> m_clip_playhead{0.0};

    std::shared_ptr<sequencer::StepSequencer> m_sequencer{nullptr};
    std::atomic<bool> m_sequencer_enabled{false};

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
    void set_name(std::string name) noexcept { m_name = std::move(name); }

    [[nodiscard]] bool is_active() const noexcept { return m_active.load(std::memory_order_relaxed); }
    void set_active(bool active) noexcept { m_active.store(active, std::memory_order_relaxed); }

    void activate(uint32_t id, std::string name) noexcept {
        m_id = id;
        m_name = std::move(name);
        m_gain.store(1.0f, std::memory_order_relaxed);
        m_target_bus.store(-1, std::memory_order_relaxed);
        m_buffer.clear();
        reset_meters();
        m_active.store(true, std::memory_order_release);
    }

    void deactivate() noexcept {
        m_active.store(false, std::memory_order_release);
        m_target_bus.store(-1, std::memory_order_relaxed);
        m_buffer.clear();
        reset_meters();
    }

    void set_gain(float gain) noexcept { m_gain.store(std::max(0.0f, gain), std::memory_order_relaxed); }
    [[nodiscard]] float gain() const noexcept { return m_gain.load(std::memory_order_relaxed); }

    void set_target_bus(int32_t bus_id) noexcept { m_target_bus.store(bus_id, std::memory_order_relaxed); }
    [[nodiscard]] int32_t target_bus() const noexcept { return m_target_bus.load(std::memory_order_relaxed); }

    void set_console_type(dsp::ConsoleType type) noexcept {
        m_console.set_type(type);
    }
    [[nodiscard]] dsp::ConsoleType console_type() const noexcept {
        return m_console.type();
    }

    // Insert Slots (Bus Glue Compressor, Master Limiter, Reverb)
    [[nodiscard]] InsertSlot& slot(size_t index) noexcept { return m_slots[index]; }
    [[nodiscard]] const InsertSlot& slot(size_t index) const noexcept { return m_slots[index]; }

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

    void reset_meters() noexcept {
        m_meter_peak_l.store(0.0f, std::memory_order_relaxed);
        m_meter_peak_r.store(0.0f, std::memory_order_relaxed);
        m_meter_rms_l.store(0.0f, std::memory_order_relaxed);
        m_meter_rms_r.store(0.0f, std::memory_order_relaxed);
    }

    void clear() noexcept {
        m_buffer.clear();
    }

    void process_buss_strip(uint32_t frames) noexcept {
        Sample* left = m_buffer.view().channel(0);
        Sample* right = m_buffer.view().channel(1);

        // 1. In-line Console Decode (Reciprocal Airwindows expansion)
        m_console.process_stereo(left, right, frames);

        // 2. Process Bus Insert Slots (Bus Glue Comp, Master EQ, etc.)
        for (auto& slot : m_slots) {
            slot.process_stereo(left, right, frames);
        }

        // 3. Telemetry
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

    std::atomic<bool> m_active{false};
    std::atomic<float> m_gain{1.0f};
    std::atomic<int32_t> m_target_bus{-1}; // -1 = Direct to Master, or target submix bus ID
    std::array<InsertSlot, kMaxBusInsertSlots> m_slots;
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
    static constexpr size_t kMaxTracks = 32;
    static constexpr size_t kMaxBuses = 16;
    static constexpr size_t kMaxSampleTaps = 4;

    explicit MixerGraph(uint32_t buffer_frames = 1024, bool enable_multithreading = true, uint32_t sample_rate = 48000)
        : m_buffer_frames(buffer_frames), m_master_bus(0, "Master", buffer_frames),
          m_clock(sample_rate, 120.0),
          m_worker_pool(enable_multithreading ? threading::AudioWorkerPool::kAutoDetect : 0) {
        m_master_bus.set_active(true);
        for (size_t i = 0; i < kMaxTracks; ++i) {
            m_tracks[i] = std::make_unique<Track>(static_cast<uint32_t>(i + 1), "Track " + std::to_string(i + 1), buffer_frames);
        }
        for (size_t i = 0; i < kMaxBuses; ++i) {
            m_buses[i] = std::make_unique<AudioBus>(static_cast<uint32_t>(i + 1), "Bus " + std::to_string(i + 1), buffer_frames);
        }
        for (size_t i = 0; i < kMaxSampleTaps; ++i) {
            m_taps[i] = std::make_unique<sampling::SampleTap>(sample_rate, 10.0f);
        }
        set_sample_rate(sample_rate);
        recompute_bus_order();
    }

    void set_sample_rate(uint32_t sample_rate) noexcept {
        if (sample_rate == 0) return;
        m_clock.set_sample_rate(sample_rate);

        for (auto& track : m_tracks) {
            if (track) {
                for (size_t i = 0; i < kMaxTrackInsertSlots; ++i) {
                    track->slot(i).init(sample_rate);
                }
            }
        }

        for (auto& bus : m_buses) {
            if (bus) {
                for (size_t i = 0; i < kMaxBusInsertSlots; ++i) {
                    bus->slot(i).init(sample_rate);
                }
            }
        }

        for (size_t i = 0; i < kMaxBusInsertSlots; ++i) {
            m_master_bus.slot(i).init(sample_rate);
        }

        for (auto& tap : m_taps) {
            if (tap) {
                tap->set_sample_rate(sample_rate);
            }
        }
    }

    [[nodiscard]] uint32_t sample_rate() const noexcept {
        return m_clock.sample_rate();
    }

    void set_worker_threads(uint32_t num_threads) {
        m_worker_pool.init(num_threads);
    }

    [[nodiscard]] uint32_t worker_threads() const noexcept {
        return m_worker_pool.num_workers();
    }

    [[nodiscard]] sampling::SampleTap* tap(size_t index) noexcept {
        return (index < kMaxSampleTaps) ? m_taps[index].get() : nullptr;
    }

    [[nodiscard]] const sampling::SampleTap* tap(size_t index) const noexcept {
        return (index < kMaxSampleTaps) ? m_taps[index].get() : nullptr;
    }

    [[nodiscard]] clock::TimelineClock& clock() noexcept { return m_clock; }
    [[nodiscard]] const clock::TimelineClock& clock() const noexcept { return m_clock; }

    // Ingest incoming AoIP network frames into all mapped active tracks
    void ingest_aoip(network::AoipReceiver& receiver, uint32_t frames) noexcept {
        for (auto& track : m_tracks) {
            if (track->is_active() && !track->has_clip()) {
                Sample* l = track->buffer().view().channel(0);
                Sample* r = track->buffer().view().channel(1);
                receiver.read_track_frames(track->id(), l, r, frames);
            }
        }
    }

    Track* allocate_track(const std::string& name) {
        for (size_t i = 0; i < kMaxTracks; ++i) {
            if (!m_tracks[i]->is_active()) {
                m_tracks[i]->activate(static_cast<uint32_t>(i + 1), name);
                return m_tracks[i].get();
            }
        }
        return nullptr; // Preallocated capacity reached
    }

    AudioBus* allocate_submix_bus(const std::string& name) {
        for (size_t i = 0; i < kMaxBuses; ++i) {
            if (!m_buses[i]->is_active()) {
                m_buses[i]->activate(static_cast<uint32_t>(i + 1), name);
                recompute_bus_order();
                return m_buses[i].get();
            }
        }
        return nullptr; // Preallocated capacity reached
    }

    // Backwards-compatible aliases
    Track* add_track(const std::string& name) {
        return allocate_track(name);
    }

    AudioBus* add_submix_bus(const std::string& name) {
        return allocate_submix_bus(name);
    }

    void release_track(uint32_t id) noexcept {
        if (id >= 1 && id <= kMaxTracks) {
            m_tracks[id - 1]->deactivate();
        }
    }

    void release_bus(uint32_t id) noexcept {
        if (id >= 1 && id <= kMaxBuses) {
            m_buses[id - 1]->deactivate();
            recompute_bus_order();
        }
    }

    bool set_bus_target_bus(uint32_t bus_id, int32_t target_bus_id) noexcept {
        if (bus_id < 1 || bus_id > kMaxBuses) return false;
        if (target_bus_id == static_cast<int32_t>(bus_id)) return false; // Self-loop cycle

        int32_t old_target = m_buses[bus_id - 1]->target_bus();
        m_buses[bus_id - 1]->set_target_bus(target_bus_id);

        if (!recompute_bus_order()) {
            // Cycle detected! Rollback
            m_buses[bus_id - 1]->set_target_bus(old_target);
            recompute_bus_order();
            return false;
        }
        return true;
    }

    bool recompute_bus_order() noexcept {
        std::array<int, kMaxBuses> in_degree{};
        std::array<bool, kMaxBuses> is_act{};
        size_t active_count = 0;

        for (size_t i = 0; i < kMaxBuses; ++i) {
            is_act[i] = m_buses[i]->is_active();
            if (is_act[i]) {
                active_count++;
            }
        }

        // Count in-degrees: if Bus i targets Bus j (1..16), Bus j has an incoming edge
        for (size_t i = 0; i < kMaxBuses; ++i) {
            if (!is_act[i]) continue;
            int32_t tgt = m_buses[i]->target_bus();
            if (tgt >= 1 && tgt <= static_cast<int32_t>(kMaxBuses)) {
                size_t tgt_idx = static_cast<size_t>(tgt - 1);
                if (is_act[tgt_idx]) {
                    in_degree[tgt_idx]++;
                }
            }
        }

        // Kahn's algorithm: queue nodes with in_degree == 0
        std::array<size_t, kMaxBuses> queue{};
        size_t q_head = 0;
        size_t q_tail = 0;

        for (size_t i = 0; i < kMaxBuses; ++i) {
            if (is_act[i] && in_degree[i] == 0) {
                queue[q_tail++] = i;
            }
        }

        std::array<size_t, kMaxBuses> order{};
        size_t sorted_count = 0;

        while (q_head < q_tail) {
            size_t u = queue[q_head++];
            order[sorted_count++] = u;

            int32_t tgt = m_buses[u]->target_bus();
            if (tgt >= 1 && tgt <= static_cast<int32_t>(kMaxBuses)) {
                size_t tgt_idx = static_cast<size_t>(tgt - 1);
                if (is_act[tgt_idx]) {
                    if (--in_degree[tgt_idx] == 0) {
                        queue[q_tail++] = tgt_idx;
                    }
                }
            }
        }

        if (sorted_count != active_count) {
            return false; // Cycle detected!
        }

        m_bus_render_order = order;
        m_bus_render_order_count = sorted_count;
        return true;
    }

    [[nodiscard]] Track* get_track(uint32_t id) noexcept {
        if (id >= 1 && id <= kMaxTracks) {
            auto* t = m_tracks[id - 1].get();
            return t->is_active() ? t : nullptr;
        }
        return nullptr;
    }

    [[nodiscard]] AudioBus* get_bus(uint32_t id) noexcept {
        if (id >= 1 && id <= kMaxBuses) {
            auto* b = m_buses[id - 1].get();
            return b->is_active() ? b : nullptr;
        }
        return nullptr;
    }

    [[nodiscard]] AudioBus& master_bus() noexcept { return m_master_bus; }
    [[nodiscard]] const AudioBus& master_bus() const noexcept { return m_master_bus; }

    [[nodiscard]] size_t track_count() const noexcept {
        size_t count = 0;
        for (const auto& t : m_tracks) {
            if (t && t->is_active()) ++count;
        }
        return count;
    }

    [[nodiscard]] size_t bus_count() const noexcept {
        size_t count = 0;
        for (const auto& b : m_buses) {
            if (b && b->is_active()) ++count;
        }
        return count;
    }

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

    // Lock-free command dispatch: UI or Network pushes binary POD packets
    bool post_command(const protocol::MixerCommand& cmd) noexcept {
        return m_command_queue.try_push(cmd);
    }

    // Lock-free telemetry capture: UI copies 60Hz state snapshot without blocking audio thread
    void capture_telemetry_snapshot(protocol::MixerTelemetryFrame& out_frame) const noexcept {
        out_frame.render_cycle = m_render_cycle.load(std::memory_order_relaxed);
        out_frame.active_tracks = static_cast<uint32_t>(track_count());
        out_frame.active_buses = static_cast<uint32_t>(bus_count());

        auto mm = m_master_bus.meter();
        out_frame.master_meter = {mm.peak_l, mm.peak_r, mm.rms_l, mm.rms_r};

        size_t active_idx = 0;
        for (size_t i = 0; i < kMaxTracks && active_idx < protocol::kMaxTelemetryTracks; ++i) {
            if (m_tracks[i]->is_active()) {
                auto tm = m_tracks[i]->meter();
                out_frame.track_meters[active_idx++] = {tm.peak_l, tm.peak_r, tm.rms_l, tm.rms_r};
            }
        }
    }

    // Real-Time Render Pipeline: Fully lock-free, zero allocation
    void render(AudioBufferView& out_master) noexcept {
        const uint32_t frames = std::min(out_master.num_frames(), m_buffer_frames);

        // 0. Drain and execute queued binary commands (Zero allocation, sample-exact)
        drain_commands();
        m_render_cycle.fetch_add(1, std::memory_order_relaxed);
        const auto boundary_events = m_clock.advance_block(frames);
        (void)boundary_events;

        // 1. Clear Master and Active Submix Buses
        m_master_bus.clear();
        for (auto& bus : m_buses) {
            if (bus->is_active()) {
                bus->clear();
            }
        }

        // 2. Evaluate Solo state across active tracks
        bool any_solo = false;
        for (const auto& track : m_tracks) {
            if (track->is_active() && track->is_solo()) {
                any_solo = true;
                break;
            }
        }

        // 3. Parallel Track Processing (Inserts, Console Encode, Meters, Taps)
        struct TrackRenderCtx {
            MixerGraph* self;
            uint32_t frames;
            const clock::TimelineClock* clock;
            const clock::BlockBoundaryEvents* events;
            bool any_solo;
        };

        TrackRenderCtx trk_ctx{
            .self = this,
            .frames = frames,
            .clock = &m_clock,
            .events = &boundary_events,
            .any_solo = any_solo
        };

        m_worker_pool.parallel_for(kMaxTracks, &trk_ctx, [](void* context, uint32_t track_idx) noexcept {
            auto* ctx = static_cast<TrackRenderCtx*>(context);
            auto* track = ctx->self->m_tracks[track_idx].get();
            if (!track->is_active()) return;

            if (track->is_muted() || (ctx->any_solo && !track->is_solo())) {
                track->reset_meters();
                return;
            }

            // Fill from active clip or step-sequencer
            track->render_input(ctx->frames, *ctx->clock, *ctx->events);

            const Sample* raw_l = track->buffer().view().channel(0);
            const Sample* raw_r = track->buffer().view().channel(1);

            // Pre-FX Tap
            for (auto& tap : ctx->self->m_taps) {
                if (tap && tap->is_active()) {
                    auto src = tap->source();
                    if (src.type == sampling::TapSourceType::TrackInput && src.source_id == track->id()) {
                        tap->record(raw_l, raw_r, ctx->frames, ctx->events);
                    }
                }
            }

            // In-line Channel Strip processing (Inserts + Console)
            track->process_channel_strip(ctx->frames);

            const Sample* trk_l = track->buffer().view().channel(0);
            const Sample* trk_r = track->buffer().view().channel(1);

            // Post-FX Tap
            for (auto& tap : ctx->self->m_taps) {
                if (tap && tap->is_active()) {
                    auto src = tap->source();
                    if (src.type == sampling::TapSourceType::TrackOutput && src.source_id == track->id()) {
                        tap->record(trk_l, trk_r, ctx->frames, ctx->events);
                    }
                }
            }
        });

        // 3b. Vectorized Bus & Master Accumulation (SIMD linear reduction)
        for (auto& track : m_tracks) {
            if (!track->is_active()) continue;
            if (track->is_muted() || (any_solo && !track->is_solo())) continue;

            const Sample* trk_l = track->buffer().view().channel(0);
            const Sample* trk_r = track->buffer().view().channel(1);

            const float gain = track->gain();
            const auto [pan_l, pan_r] = calculate_pan_gains(track->pan());
            const float left_gain = gain * pan_l;
            const float right_gain = gain * pan_r;

            // Routing destination: Submix bus or Master
            const int32_t target_id = track->target_bus();
            AudioBus* target_bus = (target_id > 0) ? get_bus(static_cast<uint32_t>(target_id)) : &m_master_bus;
            if (!target_bus) target_bus = &m_master_bus;

            Sample* dst_l = target_bus->buffer().view().channel(0);
            Sample* dst_r = target_bus->buffer().view().channel(1);

            #if defined(__GNUC__) || defined(__clang__)
            #pragma GCC ivdep
            #endif
            for (uint32_t i = 0; i < frames; ++i) {
                dst_l[i] += trk_l[i] * left_gain;
                dst_r[i] += trk_r[i] * right_gain;
            }

            // Auxiliary Sends (e.g. Reverb / Delay Busses)
            for (const auto& send : track->sends()) {
                if (!send.active || send.amount <= 0.0f) continue;
                AudioBus* send_bus = get_bus(send.bus_id);
                if (send_bus) {
                    Sample* s_l = send_bus->buffer().view().channel(0);
                    Sample* s_r = send_bus->buffer().view().channel(1);
                    float s_gain = send.pre_fader ? send.amount : (gain * send.amount);
                    #if defined(__GNUC__) || defined(__clang__)
                    #pragma GCC ivdep
                    #endif
                    for (uint32_t i = 0; i < frames; ++i) {
                        s_l[i] += trk_l[i] * s_gain * pan_l;
                        s_r[i] += trk_r[i] * s_gain * pan_r;
                    }
                }
            }
        }

        // 4. Process Active Submix Buses in Topological DAG Order
        for (size_t k = 0; k < m_bus_render_order_count; ++k) {
            size_t idx = m_bus_render_order[k];
            auto& bus = m_buses[idx];
            if (!bus->is_active()) {
                continue;
            }
            bus->process_buss_strip(frames);

            const float bus_gain = bus->gain();
            const Sample* b_l = bus->buffer().view().channel(0);
            const Sample* b_r = bus->buffer().view().channel(1);

            // Tap Submix Bus output (e.g. processed drum bus bounce!)
            for (auto& tap : m_taps) {
                if (tap && tap->is_active()) {
                    auto src = tap->source();
                    if (src.type == sampling::TapSourceType::BusOutput && src.source_id == bus->id()) {
                        tap->record(b_l, b_r, frames, &boundary_events);
                    }
                }
            }

            int32_t tgt_id = bus->target_bus();
            AudioBus* target = (tgt_id >= 1 && tgt_id <= static_cast<int32_t>(kMaxBuses))
                               ? get_bus(static_cast<uint32_t>(tgt_id))
                               : &m_master_bus;
            if (!target) target = &m_master_bus;

            Sample* dst_l = target->buffer().view().channel(0);
            Sample* dst_r = target->buffer().view().channel(1);

            for (uint32_t i = 0; i < frames; ++i) {
                dst_l[i] += b_l[i] * bus_gain;
                dst_r[i] += b_r[i] * bus_gain;
            }
        }

        // 5. Process Master Bus Strip (Console Decode + Master Inserts + Peak Safety)
        m_master_bus.process_buss_strip(frames);

        const float master_gain = m_master_bus.gain();
        const Sample* final_l = m_master_bus.buffer().view().channel(0);
        const Sample* final_r = m_master_bus.buffer().view().channel(1);

        // Tap Master Output (full mix bounce)
        for (auto& tap : m_taps) {
            if (tap && tap->is_active()) {
                auto src = tap->source();
                if (src.type == sampling::TapSourceType::MasterOutput) {
                    tap->record(final_l, final_r, frames, &boundary_events);
                }
            }
        }

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

    void drain_commands() noexcept {
        protocol::MixerCommand cmd;
        while (m_command_queue.try_pop(cmd)) {
            execute_command(cmd);
        }
    }

    void execute_command(const protocol::MixerCommand& cmd) noexcept {
        switch (cmd.type) {
            case protocol::MixerCommandType::SetTrackGain: {
                if (auto* trk = get_track(cmd.target_id)) {
                    trk->set_gain(cmd.value1);
                }
                break;
            }
            case protocol::MixerCommandType::SetTrackPan: {
                if (auto* trk = get_track(cmd.target_id)) {
                    trk->set_pan(cmd.value1);
                }
                break;
            }
            case protocol::MixerCommandType::SetTrackMute: {
                if (auto* trk = get_track(cmd.target_id)) {
                    trk->set_mute((cmd.flags & 1) != 0);
                }
                break;
            }
            case protocol::MixerCommandType::SetTrackSolo: {
                if (auto* trk = get_track(cmd.target_id)) {
                    trk->set_solo((cmd.flags & 1) != 0);
                }
                break;
            }
            case protocol::MixerCommandType::SetTrackTargetBus: {
                if (auto* trk = get_track(cmd.target_id)) {
                    trk->set_target_bus(static_cast<int32_t>(cmd.secondary_id));
                }
                break;
            }
            case protocol::MixerCommandType::SetTrackSend: {
                if (auto* trk = get_track(cmd.target_id)) {
                    trk->set_send(cmd.secondary_id, cmd.value1, (cmd.flags & 1) != 0);
                }
                break;
            }
            case protocol::MixerCommandType::SetTrackConsoleType: {
                if (auto* trk = get_track(cmd.target_id)) {
                    trk->set_console_type(static_cast<dsp::ConsoleType>(cmd.flags));
                }
                break;
            }
            case protocol::MixerCommandType::SetBusGain: {
                if (auto* bus = get_bus(cmd.target_id)) {
                    bus->set_gain(cmd.value1);
                }
                break;
            }
            case protocol::MixerCommandType::SetBusTargetBus: {
                set_bus_target_bus(cmd.target_id, static_cast<int32_t>(cmd.secondary_id));
                break;
            }
            case protocol::MixerCommandType::SetBusConsoleType: {
                if (auto* bus = get_bus(cmd.target_id)) {
                    bus->set_console_type(static_cast<dsp::ConsoleType>(cmd.flags));
                }
                break;
            }
            case protocol::MixerCommandType::SetMasterGain: {
                m_master_bus.set_gain(cmd.value1);
                break;
            }
            case protocol::MixerCommandType::SetMasterLimiter: {
                set_master_limiter_enabled((cmd.flags & 1) != 0);
                break;
            }
            case protocol::MixerCommandType::SetTrackSlotBypass: {
                if (auto* trk = get_track(cmd.target_id)) {
                    if (cmd.secondary_id < kMaxTrackInsertSlots) {
                        trk->slot(cmd.secondary_id).set_bypass((cmd.flags & 1) != 0);
                    }
                }
                break;
            }
            case protocol::MixerCommandType::SetTrackSlotParam: {
                if (auto* trk = get_track(cmd.target_id)) {
                    if (cmd.secondary_id < kMaxTrackInsertSlots) {
                        if (auto* proc = trk->slot(cmd.secondary_id).processor()) {
                            proc->set_parameter(cmd.flags, cmd.value1);
                        }
                    }
                }
                break;
            }
            case protocol::MixerCommandType::SetBusSlotBypass: {
                AudioBus* bus = (cmd.target_id == 0) ? &m_master_bus : get_bus(cmd.target_id);
                if (bus && cmd.secondary_id < kMaxBusInsertSlots) {
                    bus->slot(cmd.secondary_id).set_bypass((cmd.flags & 1) != 0);
                }
                break;
            }
            case protocol::MixerCommandType::SetBusSlotParam: {
                AudioBus* bus = (cmd.target_id == 0) ? &m_master_bus : get_bus(cmd.target_id);
                if (bus && cmd.secondary_id < kMaxBusInsertSlots) {
                    if (auto* proc = bus->slot(cmd.secondary_id).processor()) {
                        proc->set_parameter(cmd.flags, cmd.value1);
                    }
                }
                break;
            }
            case protocol::MixerCommandType::ResetMeters: {
                for (auto& trk : m_tracks) {
                    if (trk->is_active()) trk->reset_meters();
                }
                break;
            }
            case protocol::MixerCommandType::SetSampleRate: {
                uint32_t sr = (cmd.target_id > 0) ? cmd.target_id : cmd.flags;
                if (sr > 0) {
                    set_sample_rate(sr);
                }
                break;
            }
            default:
                break;
        }
    }

private:
    uint32_t m_buffer_frames;
    std::atomic<uint64_t> m_render_cycle{0};
    std::atomic<bool> m_limiter_enabled{true};
    RingBuffer<protocol::MixerCommand> m_command_queue{512};
    std::array<std::unique_ptr<Track>, kMaxTracks> m_tracks;
    std::array<std::unique_ptr<AudioBus>, kMaxBuses> m_buses;
    AudioBus m_master_bus;

    std::array<size_t, kMaxBuses> m_bus_render_order{};
    size_t m_bus_render_order_count{0};
    std::array<std::unique_ptr<sampling::SampleTap>, kMaxSampleTaps> m_taps;
    clock::TimelineClock m_clock{48000, 120.0};
    threading::AudioWorkerPool m_worker_pool;
};

} // namespace audio_core
