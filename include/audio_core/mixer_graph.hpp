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
#include "audio_core/dsp/multichannel_bus.hpp"
#include "audio_core/routing/universal_routing_matrix.hpp"
#include <string>
#include <vector>
#include <array>
#include <memory>
#include <cmath>
#include <numbers>
#include <algorithm>
#include <atomic>

namespace audio_core {

inline constexpr int32_t kStereoMasterBusId = -1;
inline constexpr int32_t kSpatialMasterBusId = -2;

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
        m_azimuth.store(0.0f, std::memory_order_relaxed);
        m_custom_azimuth.store(false, std::memory_order_relaxed);
        m_mute.store(false, std::memory_order_relaxed);
        m_solo.store(false, std::memory_order_relaxed);
        m_solo_safe.store(false, std::memory_order_relaxed);
        m_dca_mask.store(0, std::memory_order_relaxed);
        m_mute_group_mask.store(0, std::memory_order_relaxed);
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
        m_azimuth.store(0.0f, std::memory_order_relaxed);
        m_custom_azimuth.store(false, std::memory_order_relaxed);
        m_solo_safe.store(false, std::memory_order_relaxed);
        m_dca_mask.store(0, std::memory_order_relaxed);
        m_mute_group_mask.store(0, std::memory_order_relaxed);
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

    void set_pan(float pan) noexcept {
        float p = std::clamp(pan, -1.0f, 1.0f);
        m_pan.store(p, std::memory_order_relaxed);
        if (!m_custom_azimuth.load(std::memory_order_relaxed)) {
            m_azimuth.store(p * std::numbers::pi_v<float> * 0.5f, std::memory_order_relaxed);
        }
    }
    [[nodiscard]] float pan() const noexcept { return m_pan.load(std::memory_order_relaxed); }

    void set_azimuth(float azimuth_rad) noexcept {
        m_azimuth.store(azimuth_rad, std::memory_order_relaxed);
        m_custom_azimuth.store(true, std::memory_order_relaxed);
    }
    [[nodiscard]] float azimuth() const noexcept { return m_azimuth.load(std::memory_order_relaxed); }

    void set_mute(bool mute) noexcept { m_mute.store(mute, std::memory_order_relaxed); }
    [[nodiscard]] bool is_muted() const noexcept { return m_mute.load(std::memory_order_relaxed); }

    void set_solo(bool solo) noexcept { m_solo.store(solo, std::memory_order_relaxed); }
    [[nodiscard]] bool is_solo() const noexcept { return m_solo.load(std::memory_order_relaxed); }

    void set_solo_safe(bool safe) noexcept { m_solo_safe.store(safe, std::memory_order_relaxed); }
    [[nodiscard]] bool is_solo_safe() const noexcept { return m_solo_safe.load(std::memory_order_relaxed); }

    void set_dca_mask(uint8_t mask) noexcept { m_dca_mask.store(mask, std::memory_order_relaxed); }
    [[nodiscard]] uint8_t dca_mask() const noexcept { return m_dca_mask.load(std::memory_order_relaxed); }
    void assign_dca(uint8_t dca_idx, bool enable) noexcept {
        if (dca_idx < 8) {
            uint8_t mask = m_dca_mask.load(std::memory_order_relaxed);
            if (enable) mask |= static_cast<uint8_t>(1 << dca_idx);
            else mask &= static_cast<uint8_t>(~(1 << dca_idx));
            m_dca_mask.store(mask, std::memory_order_relaxed);
        }
    }

    void set_mute_group_mask(uint8_t mask) noexcept { m_mute_group_mask.store(mask, std::memory_order_relaxed); }
    [[nodiscard]] uint8_t mute_group_mask() const noexcept { return m_mute_group_mask.load(std::memory_order_relaxed); }
    void assign_mute_group(uint8_t group_idx, bool enable) noexcept {
        if (group_idx < 8) {
            uint8_t mask = m_mute_group_mask.load(std::memory_order_relaxed);
            if (enable) mask |= static_cast<uint8_t>(1 << group_idx);
            else mask &= static_cast<uint8_t>(~(1 << group_idx));
            m_mute_group_mask.store(mask, std::memory_order_relaxed);
        }
    }

    void set_target_bus(int32_t bus_id) noexcept { m_target_bus.store(bus_id, std::memory_order_relaxed); }
    [[nodiscard]] int32_t target_bus() const noexcept { return m_target_bus.load(std::memory_order_relaxed); }
    void route_to_master() noexcept { set_target_bus(kStereoMasterBusId); }
    void route_to_spatial_bus() noexcept { set_target_bus(kSpatialMasterBusId); }
    void route_to_submix_bus(uint32_t bus_id) noexcept { set_target_bus(static_cast<int32_t>(bus_id)); }

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
    void process_channel_strip(uint32_t frames, const routing::UniversalRoutingMatrix* matrix = nullptr) noexcept {
        Sample* left = m_buffer.view().channel(0);
        Sample* right = m_buffer.view().channel(1);

        // 1. Process Modular Insert Slots (Baxandall EQ, ButterComp2, MultiHeadOde, PurestDrive, WASM)
        for (size_t s = 0; s < m_slots.size(); ++s) {
            const Sample* sc_l = nullptr;
            const Sample* sc_r = nullptr;
            if (matrix && matrix->has_track_sidechain(m_id, static_cast<uint32_t>(s))) {
                sc_l = matrix->track_sidechain_l(m_id, static_cast<uint32_t>(s));
                sc_r = matrix->track_sidechain_r(m_id, static_cast<uint32_t>(s));
            }
            m_slots[s].process_stereo(left, right, frames, sc_l, sc_r);
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
    std::atomic<float> m_azimuth{0.0f};
    std::atomic<bool> m_custom_azimuth{false};
    std::atomic<bool> m_mute{false};
    std::atomic<bool> m_solo{false};
    std::atomic<bool> m_solo_safe{false};
    std::atomic<uint8_t> m_dca_mask{0};
    std::atomic<uint8_t> m_mute_group_mask{0};
    std::atomic<int32_t> m_target_bus{-1}; // -1 = Direct to Master, -2 = Spatial Bus

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
        m_mute.store(false, std::memory_order_relaxed);
        m_solo.store(false, std::memory_order_relaxed);
        m_solo_safe.store(false, std::memory_order_relaxed);
        m_target_bus.store(-1, std::memory_order_relaxed);
        m_buffer.clear();
        reset_meters();
        m_active.store(true, std::memory_order_release);
    }

    void deactivate() noexcept {
        m_active.store(false, std::memory_order_release);
        m_mute.store(false, std::memory_order_relaxed);
        m_solo.store(false, std::memory_order_relaxed);
        m_solo_safe.store(false, std::memory_order_relaxed);
        m_target_bus.store(-1, std::memory_order_relaxed);
        m_buffer.clear();
        reset_meters();
    }

    void set_gain(float gain) noexcept { m_gain.store(std::max(0.0f, gain), std::memory_order_relaxed); }
    [[nodiscard]] float gain() const noexcept { return m_gain.load(std::memory_order_relaxed); }

    void set_mute(bool mute) noexcept { m_mute.store(mute, std::memory_order_relaxed); }
    [[nodiscard]] bool is_muted() const noexcept { return m_mute.load(std::memory_order_relaxed); }

    void set_solo(bool solo) noexcept { m_solo.store(solo, std::memory_order_relaxed); }
    [[nodiscard]] bool is_solo() const noexcept { return m_solo.load(std::memory_order_relaxed); }

    void set_solo_safe(bool safe) noexcept { m_solo_safe.store(safe, std::memory_order_relaxed); }
    [[nodiscard]] bool is_solo_safe() const noexcept { return m_solo_safe.load(std::memory_order_relaxed); }

    void set_target_bus(int32_t bus_id) noexcept { m_target_bus.store(bus_id, std::memory_order_relaxed); }
    [[nodiscard]] int32_t target_bus() const noexcept { return m_target_bus.load(std::memory_order_relaxed); }
    void route_to_master() noexcept { set_target_bus(kStereoMasterBusId); }
    void route_to_spatial_bus() noexcept { set_target_bus(kSpatialMasterBusId); }
    void route_to_submix_bus(uint32_t bus_id) noexcept { set_target_bus(static_cast<int32_t>(bus_id)); }

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

    void process_buss_strip(uint32_t frames, const routing::UniversalRoutingMatrix* matrix = nullptr) noexcept {
        Sample* left = m_buffer.view().channel(0);
        Sample* right = m_buffer.view().channel(1);

        // 1. In-line Console Decode (Reciprocal Airwindows expansion)
        m_console.process_stereo(left, right, frames);

        // 2. Process Bus Insert Slots (Bus Glue Comp, Master EQ, etc.)
        for (size_t s = 0; s < m_slots.size(); ++s) {
            const Sample* sc_l = nullptr;
            const Sample* sc_r = nullptr;
            if (matrix && matrix->has_bus_sidechain(m_id, static_cast<uint32_t>(s))) {
                sc_l = matrix->bus_sidechain_l(m_id, static_cast<uint32_t>(s));
                sc_r = matrix->bus_sidechain_r(m_id, static_cast<uint32_t>(s));
            }
            m_slots[s].process_stereo(left, right, frames, sc_l, sc_r);
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
    std::atomic<bool> m_mute{false};
    std::atomic<bool> m_solo{false};
    std::atomic<bool> m_solo_safe{false};
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
    static constexpr size_t kMaxDcaGroups = 8;
    static constexpr size_t kMaxMuteGroups = 8;

    explicit MixerGraph(uint32_t buffer_frames = 1024, bool enable_multithreading = true, uint32_t sample_rate = 48000)
        : m_buffer_frames(buffer_frames), m_master_bus(0, "Master", buffer_frames),
          m_spatial_master_bus(16, buffer_frames, "Spatial Master Bus"),
          m_scratch_stereo_buffer(2, buffer_frames),
          m_clock(sample_rate, 120.0),
          m_worker_pool(enable_multithreading ? threading::AudioWorkerPool::kAutoDetect : 0) {
        m_master_bus.set_active(true);
        for (auto& g : m_dca_gains) g.store(1.0f, std::memory_order_relaxed);
        for (auto& m : m_dca_mutes) m.store(false, std::memory_order_relaxed);
        for (auto& s : m_dca_solos) s.store(false, std::memory_order_relaxed);
        for (auto& mg : m_mute_groups) mg.store(false, std::memory_order_relaxed);
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
        m_routing_matrix.set_sample_rate(sample_rate);
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

    // ------------------------------------------------------------------------
    // Universal Routing Matrix API
    // ------------------------------------------------------------------------
    [[nodiscard]] routing::UniversalRoutingMatrix& routing_matrix() noexcept { return m_routing_matrix; }
    [[nodiscard]] const routing::UniversalRoutingMatrix& routing_matrix() const noexcept { return m_routing_matrix; }

    int32_t add_route(const routing::RoutingPatch& patch) noexcept {
        return m_routing_matrix.add_patch(patch);
    }

    bool remove_route(uint32_t patch_id) noexcept {
        return m_routing_matrix.remove_patch(patch_id);
    }

    void clear_routes() noexcept {
        m_routing_matrix.clear_all_patches();
    }

    int32_t connect_sidechain(uint32_t src_track_id, uint32_t dst_track_id,
                              uint32_t dst_slot_idx = 0, float lowpass_hz = 0.0f,
                              routing::TapPoint tap = routing::TapPoint::Input) noexcept {
        routing::RoutingPatch p{};
        p.source_type = routing::RoutingSourceType::TrackAudio;
        p.source_id = src_track_id;
        p.tap_point = tap;
        p.source_channel = routing::RouteChannel::MonoSum;
        p.dest_type = routing::RoutingDestType::TrackSidechain;
        p.dest_id = dst_track_id;
        p.dest_slot = dst_slot_idx;
        p.dest_channel = routing::RouteChannel::StereoBoth;
        if (lowpass_hz > 0.0f) {
            p.conditioning.filter_mode = routing::ConditionerFilterMode::Lowpass;
            p.conditioning.cutoff_hz = lowpass_hz;
        } else {
            p.conditioning.filter_mode = routing::ConditionerFilterMode::Bypass;
        }
        return m_routing_matrix.add_patch(p);
    }

    int32_t connect_network_sidechain(uint16_t dante_ch, uint32_t dst_track_id,
                                      uint32_t dst_slot_idx = 0, float gain = 1.0f) noexcept {
        routing::RoutingPatch p{};
        p.source_type = routing::RoutingSourceType::NetworkAoip;
        p.source_id = dante_ch;
        p.source_channel = routing::RouteChannel::Left;
        p.dest_type = routing::RoutingDestType::TrackSidechain;
        p.dest_id = dst_track_id;
        p.dest_slot = dst_slot_idx;
        p.dest_channel = routing::RouteChannel::StereoBoth;
        p.conditioning.filter_mode = routing::ConditionerFilterMode::Bypass;
        p.conditioning.gain = gain;
        return m_routing_matrix.add_patch(p);
    }

    void feed_dante_channel(uint16_t channel, const float* samples, uint32_t frames) noexcept {
        m_routing_matrix.feed_network_channel(channel, samples, frames);
    }

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

    [[nodiscard]] dsp::MultiChannelBus& spatial_master_bus() noexcept { return m_spatial_master_bus; }
    [[nodiscard]] const dsp::MultiChannelBus& spatial_master_bus() const noexcept { return m_spatial_master_bus; }

    void configure_spatial_bus(uint32_t channels) noexcept {
        m_spatial_master_bus.set_channel_count(channels);
    }

    void set_dca_gain(uint8_t idx, float gain) noexcept {
        if (idx < kMaxDcaGroups) m_dca_gains[idx].store(std::max(0.0f, gain), std::memory_order_relaxed);
    }
    [[nodiscard]] float dca_gain(uint8_t idx) const noexcept {
        return (idx < kMaxDcaGroups) ? m_dca_gains[idx].load(std::memory_order_relaxed) : 1.0f;
    }

    void set_dca_mute(uint8_t idx, bool mute) noexcept {
        if (idx < kMaxDcaGroups) m_dca_mutes[idx].store(mute, std::memory_order_relaxed);
    }
    [[nodiscard]] bool is_dca_muted(uint8_t idx) const noexcept {
        return (idx < kMaxDcaGroups) ? m_dca_mutes[idx].load(std::memory_order_relaxed) : false;
    }

    void set_dca_solo(uint8_t idx, bool solo) noexcept {
        if (idx < kMaxDcaGroups) m_dca_solos[idx].store(solo, std::memory_order_relaxed);
    }
    [[nodiscard]] bool is_dca_solo(uint8_t idx) const noexcept {
        return (idx < kMaxDcaGroups) ? m_dca_solos[idx].load(std::memory_order_relaxed) : false;
    }

    void set_mute_group_active(uint8_t idx, bool active) noexcept {
        if (idx < kMaxMuteGroups) m_mute_groups[idx].store(active, std::memory_order_relaxed);
    }
    [[nodiscard]] bool is_mute_group_active(uint8_t idx) const noexcept {
        return (idx < kMaxMuteGroups) ? m_mute_groups[idx].load(std::memory_order_relaxed) : false;
    }

    [[nodiscard]] bool is_track_effectively_muted(const Track* track) const noexcept {
        if (!track || track->is_muted()) return true;
        uint8_t mg = track->mute_group_mask();
        for (size_t g = 0; g < kMaxMuteGroups; ++g) {
            if ((mg & (1 << g)) && m_mute_groups[g].load(std::memory_order_relaxed)) return true;
        }
        uint8_t dca = track->dca_mask();
        for (size_t d = 0; d < kMaxDcaGroups; ++d) {
            if ((dca & (1 << d)) && m_dca_mutes[d].load(std::memory_order_relaxed)) return true;
        }
        return false;
    }

    [[nodiscard]] bool is_track_effectively_solo(const Track* track) const noexcept {
        if (!track) return false;
        if (track->is_solo()) return true;
        uint8_t dca = track->dca_mask();
        for (size_t d = 0; d < kMaxDcaGroups; ++d) {
            if ((dca & (1 << d)) && m_dca_solos[d].load(std::memory_order_relaxed)) return true;
        }
        return false;
    }

    [[nodiscard]] float compute_track_effective_gain(const Track* track) const noexcept {
        if (!track) return 0.0f;
        float g = track->gain();
        uint8_t dca = track->dca_mask();
        for (size_t d = 0; d < kMaxDcaGroups; ++d) {
            if (dca & (1 << d)) {
                g *= m_dca_gains[d].load(std::memory_order_relaxed);
            }
        }
        return g;
    }

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

    // Multichannel spatial rendering (Planar / Poly-WAV destination with optional stereo monitor output)
    void render_multichannel(dsp::MultiChannelBus& out_spatial, AudioBufferView* out_stereo_master = nullptr) noexcept {
        if (out_stereo_master) {
            render(*out_stereo_master);
        } else {
            auto scratch_view = m_scratch_stereo_buffer.view_frames(m_buffer_frames);
            render(scratch_view);
        }

        const uint32_t frames = std::min(out_spatial.num_frames(), m_buffer_frames);
        const uint32_t channels = std::min(out_spatial.num_channels(), m_spatial_master_bus.num_channels());
        for (uint32_t ch = 0; ch < channels; ++ch) {
            std::memcpy(out_spatial.channel(ch), m_spatial_master_bus.channel(ch), frames * sizeof(float));
        }
        for (uint32_t ch = channels; ch < out_spatial.num_channels(); ++ch) {
            std::memset(out_spatial.channel(ch), 0, frames * sizeof(float));
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

        // 1. Clear Master, Spatial Master, and Active Submix Buses
        m_master_bus.clear();
        m_spatial_master_bus.clear_frames(frames);
        for (auto& bus : m_buses) {
            if (bus->is_active()) {
                bus->clear();
            }
        }

        // 2. Evaluate Solo state across active tracks and DCAs
        bool any_solo = false;
        for (size_t d = 0; d < kMaxDcaGroups; ++d) {
            if (m_dca_solos[d].load(std::memory_order_relaxed)) {
                any_solo = true;
                break;
            }
        }
        if (!any_solo) {
            for (const auto& track : m_tracks) {
                if (track->is_active() && track->is_solo()) {
                    any_solo = true;
                    break;
                }
            }
        }

        // 2b. Universal Routing Matrix: Prepare destination buffers and evaluate Network AoIP routes
        m_routing_matrix.prepare_block(frames);

        for (size_t r = 0; r < routing::UniversalRoutingMatrix::kMaxRoutes; ++r) {
            const auto& patch = m_routing_matrix.patches()[r];
            if (!patch.active) continue;
            if (patch.source_type == routing::RoutingSourceType::NetworkAoip) {
                m_routing_matrix.process_route(r, nullptr, nullptr, frames);
            }
        }

        // 3. Track Processing & Universal Routing Matrix Passes
        // Phase 3a: Track input rendering (Clips / Sequencers) & Pre-FX Taps
        for (auto& track : m_tracks) {
            if (!track->is_active()) continue;
            bool trk_muted = is_track_effectively_muted(track.get());
            bool trk_solo = is_track_effectively_solo(track.get());
            if (trk_muted || (any_solo && !trk_solo && !track->is_solo_safe())) {
                track->reset_meters();
                continue;
            }

            // Fill from active clip or step-sequencer
            track->render_input(frames, m_clock, boundary_events);

            const Sample* raw_l = track->buffer().view().channel(0);
            const Sample* raw_r = track->buffer().view().channel(1);

            // Pre-FX Tap for Sampler
            for (auto& tap : m_taps) {
                if (tap && tap->is_active()) {
                    auto src = tap->source();
                    if (src.type == sampling::TapSourceType::TrackInput && src.source_id == track->id()) {
                        tap->record(raw_l, raw_r, frames, &boundary_events);
                    }
                }
            }
        }

        // Phase 3b: Evaluate Pre-Insert / Input tap routes (e.g. Track 2 Lowpass Sidechain)
        for (size_t r = 0; r < routing::UniversalRoutingMatrix::kMaxRoutes; ++r) {
            const auto& patch = m_routing_matrix.patches()[r];
            if (!patch.active) continue;
            if (patch.source_type == routing::RoutingSourceType::TrackAudio &&
                (patch.tap_point == routing::TapPoint::Input || patch.tap_point == routing::TapPoint::PreInsert)) {
                Track* src_trk = get_track(patch.source_id);
                if (src_trk && src_trk->is_active()) {
                    const Sample* in_l = src_trk->buffer().view().channel(0);
                    const Sample* in_r = src_trk->buffer().view().channel(1);
                    m_routing_matrix.process_route(r, in_l, in_r, frames);
                }
            }
        }

        // Phase 3c: Parallel Track Channel Strip Processing (Inserts with Routing Matrix Sidechains + Console)
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

            bool trk_muted = ctx->self->is_track_effectively_muted(track);
            bool trk_solo = ctx->self->is_track_effectively_solo(track);
            if (trk_muted || (ctx->any_solo && !trk_solo && !track->is_solo_safe())) {
                return;
            }

            // In-line Channel Strip processing with Routing Matrix sidechain access!
            track->process_channel_strip(ctx->frames, &ctx->self->m_routing_matrix);

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

        // Phase 3d: Evaluate Post-Insert & Post-Fader routes
        for (size_t r = 0; r < routing::UniversalRoutingMatrix::kMaxRoutes; ++r) {
            const auto& patch = m_routing_matrix.patches()[r];
            if (!patch.active) continue;
            if (patch.source_type == routing::RoutingSourceType::TrackAudio &&
                (patch.tap_point == routing::TapPoint::PostInsert || patch.tap_point == routing::TapPoint::PostFader)) {
                Track* src_trk = get_track(patch.source_id);
                if (src_trk && src_trk->is_active()) {
                    const Sample* post_l = src_trk->buffer().view().channel(0);
                    const Sample* post_r = src_trk->buffer().view().channel(1);
                    m_routing_matrix.process_route(r, post_l, post_r, frames);
                }
            }
        }

        // 3b. Vectorized Bus & Master Accumulation (SIMD linear reduction)
        for (auto& track : m_tracks) {
            if (!track->is_active()) continue;
            bool trk_muted = is_track_effectively_muted(track.get());
            bool trk_solo = is_track_effectively_solo(track.get());
            if (trk_muted || (any_solo && !trk_solo && !track->is_solo_safe())) continue;

            const Sample* trk_l = track->buffer().view().channel(0);
            const Sample* trk_r = track->buffer().view().channel(1);

            const float gain = compute_track_effective_gain(track.get());
            const auto [pan_l, pan_r] = calculate_pan_gains(track->pan());
            const float left_gain = gain * pan_l;
            const float right_gain = gain * pan_r;

            // Routing destination: Spatial Master Bus, Submix bus or Master
            const int32_t target_id = track->target_bus();
            if (target_id == kSpatialMasterBusId) {
                // Vector-Base Spatial Panning into Spatial Master Bus
                // Downmix track stereo output to mono point source in scratch buffer
                Sample* scratch_mono = m_scratch_stereo_buffer.channel(0);
                for (uint32_t i = 0; i < frames; ++i) {
                    scratch_mono[i] = 0.5f * (trk_l[i] + trk_r[i]);
                }
                m_spatial_master_bus.pan_mono_circular(scratch_mono, gain, track->azimuth(), frames);
            } else {
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

        // Check if any submix bus has solo
        bool any_bus_solo = false;
        for (const auto& b : m_buses) {
            if (b->is_active() && b->is_solo()) {
                any_bus_solo = true;
                break;
            }
        }

        // 4. Process Active Submix Buses in Topological DAG Order
        for (size_t k = 0; k < m_bus_render_order_count; ++k) {
            size_t idx = m_bus_render_order[k];
            auto& bus = m_buses[idx];
            if (!bus->is_active()) {
                continue;
            }

            if (bus->is_muted() || (any_bus_solo && !bus->is_solo() && !bus->is_solo_safe())) {
                bus->reset_meters();
                continue;
            }

            bus->process_buss_strip(frames, &m_routing_matrix);

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
            if (tgt_id == kSpatialMasterBusId) {
                // Route stereo submix bus into front channels of spatial bus
                m_spatial_master_bus.mix_stereo_pair(0, b_l, b_r, bus_gain, frames);
            } else {
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
        }

        // 4b. Telemetry & Stereo Folddown of Spatial Master Bus (ITU-R BS.775 / Equal-Power)
        m_spatial_master_bus.update_meters(frames);
        Sample* down_l = m_scratch_stereo_buffer.channel(0);
        Sample* down_r = m_scratch_stereo_buffer.channel(1);
        m_spatial_master_bus.downmix_to_stereo(down_l, down_r, frames);
        Sample* mst_l = m_master_bus.buffer().view().channel(0);
        Sample* mst_r = m_master_bus.buffer().view().channel(1);
        for (uint32_t i = 0; i < frames; ++i) {
            mst_l[i] += down_l[i];
            mst_r[i] += down_r[i];
        }

        // 5. Process Master Bus Strip (Console Decode + Master Inserts + Peak Safety)
        m_master_bus.process_buss_strip(frames, &m_routing_matrix);

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
            case protocol::MixerCommandType::SetTrackAzimuth: {
                if (auto* trk = get_track(cmd.target_id)) {
                    trk->set_azimuth(cmd.value1);
                }
                break;
            }
            case protocol::MixerCommandType::ConfigureSpatialBus: {
                uint32_t ch = (cmd.target_id > 0) ? cmd.target_id : cmd.flags;
                if (ch > 0) {
                    configure_spatial_bus(ch);
                }
                break;
            }
            case protocol::MixerCommandType::SetTrackSoloSafe: {
                if (auto* trk = get_track(cmd.target_id)) {
                    trk->set_solo_safe((cmd.flags & 1) != 0);
                }
                break;
            }
            case protocol::MixerCommandType::SetTrackDcaMask: {
                if (auto* trk = get_track(cmd.target_id)) {
                    trk->set_dca_mask(static_cast<uint8_t>(cmd.flags));
                }
                break;
            }
            case protocol::MixerCommandType::SetTrackMuteGroupMask: {
                if (auto* trk = get_track(cmd.target_id)) {
                    trk->set_mute_group_mask(static_cast<uint8_t>(cmd.flags));
                }
                break;
            }
            case protocol::MixerCommandType::SetBusMute: {
                if (auto* bus = get_bus(cmd.target_id)) {
                    bus->set_mute((cmd.flags & 1) != 0);
                }
                break;
            }
            case protocol::MixerCommandType::SetBusSolo: {
                if (auto* bus = get_bus(cmd.target_id)) {
                    bus->set_solo((cmd.flags & 1) != 0);
                }
                break;
            }
            case protocol::MixerCommandType::SetBusSoloSafe: {
                if (auto* bus = get_bus(cmd.target_id)) {
                    bus->set_solo_safe((cmd.flags & 1) != 0);
                }
                break;
            }
            case protocol::MixerCommandType::SetDcaGain: {
                set_dca_gain(static_cast<uint8_t>(cmd.target_id), cmd.value1);
                break;
            }
            case protocol::MixerCommandType::SetDcaMute: {
                set_dca_mute(static_cast<uint8_t>(cmd.target_id), (cmd.flags & 1) != 0);
                break;
            }
            case protocol::MixerCommandType::SetDcaSolo: {
                set_dca_solo(static_cast<uint8_t>(cmd.target_id), (cmd.flags & 1) != 0);
                break;
            }
            case protocol::MixerCommandType::SetMuteGroupActive: {
                set_mute_group_active(static_cast<uint8_t>(cmd.target_id), (cmd.flags & 1) != 0);
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
    dsp::MultiChannelBus m_spatial_master_bus;
    AudioBuffer m_scratch_stereo_buffer;

    std::array<std::atomic<float>, kMaxDcaGroups> m_dca_gains;
    std::array<std::atomic<bool>, kMaxDcaGroups> m_dca_mutes;
    std::array<std::atomic<bool>, kMaxDcaGroups> m_dca_solos;
    std::array<std::atomic<bool>, kMaxMuteGroups> m_mute_groups;

    std::array<size_t, kMaxBuses> m_bus_render_order{};
    size_t m_bus_render_order_count{0};
    std::array<std::unique_ptr<sampling::SampleTap>, kMaxSampleTaps> m_taps;
    clock::TimelineClock m_clock{48000, 120.0};
    threading::AudioWorkerPool m_worker_pool;
    routing::UniversalRoutingMatrix m_routing_matrix;
};

} // namespace audio_core
