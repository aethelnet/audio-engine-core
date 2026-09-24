#pragma once

#include "audio_core/types.hpp"
#include "audio_core/ring_buffer.hpp"
#include "audio_core/insert_slot.hpp"
#include "audio_core/dsp/console_processor.hpp"
#include "audio_core/protocol/command_packet.hpp"
#include "audio_core/protocol/telemetry_packet.hpp"
#include "audio_core/sampling/sample_tap.hpp"
#include "audio_core/sampling/vari_speed_streamer.hpp"
#include "audio_core/network/aoip_receiver.hpp"
#include "audio_core/network/aoip_transmitter.hpp"
#include "audio_core/clock/timeline_clock.hpp"
#include "audio_core/sequencer/step_sequencer.hpp"
#include "audio_core/sequencer/clip_launcher.hpp"
#include "audio_core/threading/audio_worker_pool.hpp"
#include "audio_core/dsp/multichannel_bus.hpp"
#include "audio_core/dsp/kinetic_meter.hpp"
#include "audio_core/routing/universal_routing_matrix.hpp"
#include "audio_core/routing/automation_curve.hpp"
#include "audio_core/modulation/polyphonic_synth.hpp"
#include "audio_core/modulation/modulation_matrix.hpp"
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
// Track Input Mode (Zähl AM1 Routing / Bridge to PipeWire & Network)
// ============================================================================
enum class TrackInputMode : uint8_t {
    InternalClip = 0,    // Pure internal clip / sequencer playback
    PipeWireStream = 1,  // External PipeWire audio input replaces clip
    MergeAll = 2,        // Sum internal clip + external PipeWire stream
    NetworkAoip = 3,     // Stream from AoIP / Dante network channel
    PolySynth = 4        // Live Polyphonic MSEG Synthesizer generator
};

// ============================================================================
// Track: A single audio/instrument channel in the Mixer Graph
// Pre-allocated in fixed pool; zero allocations during playback
// ============================================================================
class Track {
public:
    static constexpr size_t kMaxSlotParams = 4;
    static constexpr size_t kMaxTrackInsertSlots = audio_core::kMaxTrackInsertSlots;

    Track(uint32_t id, std::string name, uint32_t buffer_frames = 1024)
        : m_id(id), m_name(std::move(name)), m_buffer(2, buffer_frames) {
        m_console.set_mode(dsp::ConsoleMode::Channel);
        m_gain_curve.clear(1.0f);
        m_pan_curve.clear(0.0f);
        m_aux1_curve.clear(0.0f);
        m_aux2_curve.clear(0.0f);
        for (size_t s = 0; s < kMaxTrackInsertSlots; ++s) {
            for (size_t p = 0; p < kMaxSlotParams; ++p) {
                m_slot_curves[s][p].clear(0.0f);
                m_slot_automation_enabled[s][p].store(false, std::memory_order_relaxed);
            }
        }
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
        m_seq_send_a_bus.store(-1, std::memory_order_relaxed);
        m_seq_send_b_bus.store(-1, std::memory_order_relaxed);
        m_input_gain.store(1.0f, std::memory_order_relaxed);
        m_input_phase_invert.store(false, std::memory_order_relaxed);
        m_input_meter_peak_l.store(0.0f, std::memory_order_relaxed);
        m_input_meter_peak_r.store(0.0f, std::memory_order_relaxed);
        for (auto& s : m_sends) {
            s.active = false;
        }
        m_clip.reset();
        m_clip_playhead.store(0.0, std::memory_order_relaxed);
        m_sync_to_transport.store(false, std::memory_order_relaxed);
        m_streamer.reset();
        m_launcher.stop_immediate();
        m_buffer.clear();
        m_pdc_buffer.clear();
        m_pdc_write_pos = 0;
        m_pdc_delay_samples.store(0, std::memory_order_relaxed);
        m_has_previous_gain = false;
        m_poly_synth = nullptr;
        m_mod_matrix = nullptr;
        m_gain_automation_enabled.store(false, std::memory_order_relaxed);
        m_pan_automation_enabled.store(false, std::memory_order_relaxed);
        m_aux1_automation_enabled.store(false, std::memory_order_relaxed);
        m_aux2_automation_enabled.store(false, std::memory_order_relaxed);
        m_active.store(true, std::memory_order_release);
    }

    void deactivate() noexcept {
        m_active.store(false, std::memory_order_release);
        m_poly_synth = nullptr;
        m_mod_matrix = nullptr;
        m_gain_automation_enabled.store(false, std::memory_order_relaxed);
        m_pan_automation_enabled.store(false, std::memory_order_relaxed);
        m_aux1_automation_enabled.store(false, std::memory_order_relaxed);
        m_aux2_automation_enabled.store(false, std::memory_order_relaxed);
        m_azimuth.store(0.0f, std::memory_order_relaxed);
        m_custom_azimuth.store(false, std::memory_order_relaxed);
        m_solo_safe.store(false, std::memory_order_relaxed);
        m_dca_mask.store(0, std::memory_order_relaxed);
        m_mute_group_mask.store(0, std::memory_order_relaxed);
        m_seq_send_a_bus.store(-1, std::memory_order_relaxed);
        m_seq_send_b_bus.store(-1, std::memory_order_relaxed);
        m_input_gain.store(1.0f, std::memory_order_relaxed);
        m_input_phase_invert.store(false, std::memory_order_relaxed);
        m_input_meter_peak_l.store(0.0f, std::memory_order_relaxed);
        m_input_meter_peak_r.store(0.0f, std::memory_order_relaxed);
        for (auto& s : m_sends) {
            s.active = false;
        }
        m_clip.reset();
        m_clip_playhead.store(0.0, std::memory_order_relaxed);
        m_sync_to_transport.store(false, std::memory_order_relaxed);
        m_streamer.reset();
        m_launcher.stop_immediate();
        m_buffer.clear();
        reset_meters();
        m_pdc_buffer.clear();
        m_pdc_write_pos = 0;
        m_pdc_delay_samples.store(0, std::memory_order_relaxed);
        m_has_previous_gain = false;
    }

    void snap_parameters() noexcept {
        m_has_previous_gain = false;
    }

    void set_gain(float gain, bool snap = false) noexcept {
        m_gain.store(std::max(0.0f, gain), std::memory_order_relaxed);
        if (snap) snap_parameters();
    }
    [[nodiscard]] float gain() const noexcept { return m_gain.load(std::memory_order_relaxed); }

    [[nodiscard]] routing::AutomationCurve& gain_curve() noexcept { return m_gain_curve; }
    [[nodiscard]] const routing::AutomationCurve& gain_curve() const noexcept { return m_gain_curve; }

    void set_gain_automation_enabled(bool enabled) noexcept {
        m_gain_automation_enabled.store(enabled, std::memory_order_relaxed);
    }
    [[nodiscard]] bool is_gain_automation_enabled() const noexcept {
        return m_gain_automation_enabled.load(std::memory_order_relaxed);
    }

    [[nodiscard]] routing::AutomationCurve& pan_curve() noexcept { return m_pan_curve; }
    [[nodiscard]] const routing::AutomationCurve& pan_curve() const noexcept { return m_pan_curve; }

    void set_pan_automation_enabled(bool enabled) noexcept {
        m_pan_automation_enabled.store(enabled, std::memory_order_relaxed);
    }
    [[nodiscard]] bool is_pan_automation_enabled() const noexcept {
        return m_pan_automation_enabled.load(std::memory_order_relaxed);
    }

    void ensure_send_active(uint32_t bus_id) noexcept {
        for (const auto& s : m_sends) {
            if (s.active && s.bus_id == bus_id) return;
        }
        for (auto& s : m_sends) {
            if (!s.active) {
                s.bus_id = bus_id;
                s.amount = 1.0f;
                s.pre_fader = false;
                s.active = true;
                return;
            }
        }
    }

    [[nodiscard]] routing::AutomationCurve& aux1_curve() noexcept { return m_aux1_curve; }
    [[nodiscard]] const routing::AutomationCurve& aux1_curve() const noexcept { return m_aux1_curve; }

    void set_aux1_automation_enabled(bool enabled) noexcept {
        m_aux1_automation_enabled.store(enabled, std::memory_order_relaxed);
        if (enabled) ensure_send_active(1);
    }
    [[nodiscard]] bool is_aux1_automation_enabled() const noexcept {
        return m_aux1_automation_enabled.load(std::memory_order_relaxed);
    }

    [[nodiscard]] routing::AutomationCurve& aux2_curve() noexcept { return m_aux2_curve; }
    [[nodiscard]] const routing::AutomationCurve& aux2_curve() const noexcept { return m_aux2_curve; }

    void set_aux2_automation_enabled(bool enabled) noexcept {
        m_aux2_automation_enabled.store(enabled, std::memory_order_relaxed);
        if (enabled) ensure_send_active(2);
    }
    [[nodiscard]] bool is_aux2_automation_enabled() const noexcept {
        return m_aux2_automation_enabled.load(std::memory_order_relaxed);
    }

    [[nodiscard]] routing::AutomationCurve& automation_curve(routing::AutomationTarget target) noexcept {
        switch (target) {
            case routing::AutomationTarget::Gain: return m_gain_curve;
            case routing::AutomationTarget::Pan:  return m_pan_curve;
            case routing::AutomationTarget::Aux1: return m_aux1_curve;
            case routing::AutomationTarget::Aux2: return m_aux2_curve;
            case routing::AutomationTarget::Pitch: break;
            case routing::AutomationTarget::PluginParam: return m_slot_curves[0][0];
        }
        return m_gain_curve;
    }

    [[nodiscard]] const routing::AutomationCurve& automation_curve(routing::AutomationTarget target) const noexcept {
        switch (target) {
            case routing::AutomationTarget::Gain: return m_gain_curve;
            case routing::AutomationTarget::Pan:  return m_pan_curve;
            case routing::AutomationTarget::Aux1: return m_aux1_curve;
            case routing::AutomationTarget::Aux2: return m_aux2_curve;
            case routing::AutomationTarget::Pitch: break;
            case routing::AutomationTarget::PluginParam: return m_slot_curves[0][0];
        }
        return m_gain_curve;
    }

    [[nodiscard]] routing::AutomationCurve& slot_automation_curve(size_t slot_idx, size_t param_idx) noexcept {
        if (slot_idx < kMaxTrackInsertSlots && param_idx < kMaxSlotParams) {
            return m_slot_curves[slot_idx][param_idx];
        }
        return m_slot_curves[0][0];
    }

    [[nodiscard]] const routing::AutomationCurve& slot_automation_curve(size_t slot_idx, size_t param_idx) const noexcept {
        if (slot_idx < kMaxTrackInsertSlots && param_idx < kMaxSlotParams) {
            return m_slot_curves[slot_idx][param_idx];
        }
        return m_slot_curves[0][0];
    }

    void set_slot_automation_enabled(size_t slot_idx, size_t param_idx, bool enabled) noexcept {
        if (slot_idx < kMaxTrackInsertSlots && param_idx < kMaxSlotParams) {
            m_slot_automation_enabled[slot_idx][param_idx].store(enabled, std::memory_order_relaxed);
        }
    }

    [[nodiscard]] bool is_slot_automation_enabled(size_t slot_idx, size_t param_idx) const noexcept {
        if (slot_idx < kMaxTrackInsertSlots && param_idx < kMaxSlotParams) {
            return m_slot_automation_enabled[slot_idx][param_idx].load(std::memory_order_relaxed);
        }
        return false;
    }

    void set_automation_enabled(routing::AutomationTarget target, bool enabled) noexcept {
        switch (target) {
            case routing::AutomationTarget::Gain: m_gain_automation_enabled.store(enabled, std::memory_order_relaxed); break;
            case routing::AutomationTarget::Pan:  m_pan_automation_enabled.store(enabled, std::memory_order_relaxed); break;
            case routing::AutomationTarget::Aux1: {
                m_aux1_automation_enabled.store(enabled, std::memory_order_relaxed);
                if (enabled) ensure_send_active(1);
                break;
            }
            case routing::AutomationTarget::Aux2: {
                m_aux2_automation_enabled.store(enabled, std::memory_order_relaxed);
                if (enabled) ensure_send_active(2);
                break;
            }
            case routing::AutomationTarget::Pitch: break;
            case routing::AutomationTarget::PluginParam: break;
        }
    }

    [[nodiscard]] bool is_automation_enabled(routing::AutomationTarget target) const noexcept {
        switch (target) {
            case routing::AutomationTarget::Gain: return m_gain_automation_enabled.load(std::memory_order_relaxed);
            case routing::AutomationTarget::Pan:  return m_pan_automation_enabled.load(std::memory_order_relaxed);
            case routing::AutomationTarget::Aux1: return m_aux1_automation_enabled.load(std::memory_order_relaxed);
            case routing::AutomationTarget::Aux2: return m_aux2_automation_enabled.load(std::memory_order_relaxed);
            case routing::AutomationTarget::Pitch: return false;
            case routing::AutomationTarget::PluginParam: return false;
        }
        return false;
    }

    void set_pan(float pan, bool snap = false) noexcept {
        float p = std::clamp(pan, -1.0f, 1.0f);
        m_pan.store(p, std::memory_order_relaxed);
        if (!m_custom_azimuth.load(std::memory_order_relaxed)) {
            m_azimuth.store(p * std::numbers::pi_v<float> * 0.5f, std::memory_order_relaxed);
        }
        if (snap) snap_parameters();
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
        m_input_meter_peak_l.store(0.0f, std::memory_order_relaxed);
        m_input_meter_peak_r.store(0.0f, std::memory_order_relaxed);
    }

    void set_input_gain(float gain) noexcept { m_input_gain.store(std::max(0.0f, gain), std::memory_order_relaxed); }
    [[nodiscard]] float input_gain() const noexcept { return m_input_gain.load(std::memory_order_relaxed); }

    void set_input_phase_invert(bool invert) noexcept { m_input_phase_invert.store(invert, std::memory_order_relaxed); }
    [[nodiscard]] bool input_phase_invert() const noexcept { return m_input_phase_invert.load(std::memory_order_relaxed); }

    [[nodiscard]] std::pair<float, float> input_meter() const noexcept {
        return { m_input_meter_peak_l.load(std::memory_order_relaxed), m_input_meter_peak_r.load(std::memory_order_relaxed) };
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
        m_streamer.set_clip(m_clip);
        m_streamer.set_loop(loop);
        m_streamer.set_playhead(0.0);
    }

    void clear_clip() noexcept {
        m_clip.reset();
        m_clip_playhead.store(0.0, std::memory_order_relaxed);
        m_streamer.set_clip(nullptr);
    }

    [[nodiscard]] bool has_clip() const noexcept {
        return m_clip != nullptr;
    }

    [[nodiscard]] std::shared_ptr<sampling::AudioClip> clip() const noexcept {
        return m_clip;
    }

    [[nodiscard]] uint64_t clip_playhead() const noexcept {
        return static_cast<uint64_t>(std::round(m_streamer.playhead()));
    }

    [[nodiscard]] double clip_playhead_f() const noexcept {
        return m_streamer.playhead();
    }

    void set_clip_playhead(double playhead) noexcept {
        m_clip_playhead.store(playhead, std::memory_order_relaxed);
        m_streamer.set_playhead(playhead);
    }

    void reset_playback_state(double playhead = 0.0) noexcept {
        m_clip_playhead.store(playhead, std::memory_order_relaxed);
        m_streamer.set_playhead(playhead);
        m_buffer.clear();
        reset_meters();
        m_pdc_buffer.clear();
        m_pdc_write_pos = 0;
        for (auto& s : m_slots) {
            s.reset();
        }
        if (m_sequencer) {
            m_sequencer->stop();
        }
    }

    void set_sync_to_transport(bool sync) noexcept {
        m_sync_to_transport.store(sync, std::memory_order_relaxed);
    }

    [[nodiscard]] bool sync_to_transport() const noexcept {
        return m_sync_to_transport.load(std::memory_order_relaxed);
    }

    // Vari-Speed Streamer Control APIs
    [[nodiscard]] sampling::VariSpeedStreamer& streamer() noexcept { return m_streamer; }
    [[nodiscard]] const sampling::VariSpeedStreamer& streamer() const noexcept { return m_streamer; }

    void set_playback_mode(sampling::PlaybackMode mode) noexcept { m_streamer.set_playback_mode(mode); }
    [[nodiscard]] sampling::PlaybackMode playback_mode() const noexcept { return m_streamer.playback_mode(); }

    void set_pitch_semitones(float semitones) noexcept { m_streamer.set_pitch_semitones(semitones); }
    [[nodiscard]] float pitch_semitones() const noexcept { return m_streamer.pitch_semitones(); }

    void set_speed_ratio(float ratio) noexcept { m_streamer.set_speed_ratio(ratio); }
    [[nodiscard]] float speed_ratio() const noexcept { return m_streamer.speed_ratio(); }

    void set_reverse(bool rev) noexcept { m_streamer.set_reverse(rev); }
    [[nodiscard]] bool is_reverse() const noexcept { return m_streamer.is_reverse(); }

    void set_capstan_inertia_ms(float ms) noexcept { m_streamer.set_capstan_inertia_ms(ms); }
    [[nodiscard]] float capstan_inertia_ms() const noexcept { return m_streamer.capstan_inertia_ms(); }

    void set_clip_bar_length(float bars) noexcept { m_streamer.set_bar_length(bars); }
    [[nodiscard]] float clip_bar_length() const noexcept { return m_streamer.bar_length(); }

    void trigger_tape_stop(float duration_sec = 0.5f) noexcept { m_streamer.trigger_tape_stop(duration_sec); }
    void trigger_tape_start(float duration_sec = 0.3f) noexcept { m_streamer.trigger_tape_start(duration_sec); }

    void set_sequencer(std::shared_ptr<sequencer::StepSequencer> seq) noexcept {
        m_sequencer = std::move(seq);
        m_sequencer_enabled.store(m_sequencer != nullptr, std::memory_order_relaxed);
        m_launcher.set_associated_sequencer(m_sequencer.get());
    }

    void enable_sequencer(bool enable) noexcept {
        m_sequencer_enabled.store(enable, std::memory_order_relaxed);
    }

    [[nodiscard]] sequencer::StepSequencer* sequencer() noexcept { return m_sequencer.get(); }
    [[nodiscard]] const sequencer::StepSequencer* sequencer() const noexcept { return m_sequencer.get(); }
    [[nodiscard]] bool is_sequencer_enabled() const noexcept {
        return m_sequencer_enabled.load(std::memory_order_relaxed) && (m_sequencer != nullptr);
    }

    void set_sequencer_send_a_bus(int32_t bus_id) noexcept { m_seq_send_a_bus.store(bus_id, std::memory_order_relaxed); }
    [[nodiscard]] int32_t sequencer_send_a_bus() const noexcept { return m_seq_send_a_bus.load(std::memory_order_relaxed); }

    void set_sequencer_send_b_bus(int32_t bus_id) noexcept { m_seq_send_b_bus.store(bus_id, std::memory_order_relaxed); }
    [[nodiscard]] int32_t sequencer_send_b_bus() const noexcept { return m_seq_send_b_bus.load(std::memory_order_relaxed); }

    // Clip Launcher Integration
    [[nodiscard]] sequencer::ClipLauncher& clip_launcher() noexcept { return m_launcher; }
    [[nodiscard]] const sequencer::ClipLauncher& clip_launcher() const noexcept { return m_launcher; }

    void launch_clip(size_t slot_idx, sequencer::LaunchQuantize q = sequencer::LaunchQuantize::Bar, bool legato = false) noexcept {
        m_launcher.launch_slot(slot_idx, q, legato);
    }

    void stop_clip(sequencer::LaunchQuantize q = sequencer::LaunchQuantize::Bar) noexcept {
        m_launcher.stop(q);
    }

    [[nodiscard]] bool is_clip_launcher_active() const noexcept {
        return m_launcher.is_active();
    }

    void set_input_mode(TrackInputMode mode) noexcept { m_input_mode.store(mode, std::memory_order_relaxed); }
    [[nodiscard]] TrackInputMode input_mode() const noexcept { return m_input_mode.load(std::memory_order_relaxed); }

    void set_poly_synth(modulation::PolyphonicSynth* synth, modulation::ModulationMatrix* matrix = nullptr) noexcept {
        m_poly_synth = synth;
        m_mod_matrix = matrix;
    }
    [[nodiscard]] modulation::PolyphonicSynth* poly_synth() const noexcept { return m_poly_synth; }
    [[nodiscard]] modulation::ModulationMatrix* modulation_matrix() const noexcept { return m_mod_matrix; }

    // Called inside RT render loop before channel strip processing
    void render_input(uint32_t frames, const clock::TimelineClock& clock,
                      const clock::BlockBoundaryEvents& boundary_events) noexcept {
        Sample* left = m_buffer.view().channel(0);
        Sample* right = m_buffer.view().channel(1);

        auto mode = m_input_mode.load(std::memory_order_relaxed);
        if (mode == TrackInputMode::PipeWireStream || mode == TrackInputMode::NetworkAoip) {
            // Buffer already populated by external stream before mixer render
            return;
        }

        if (mode == TrackInputMode::PolySynth) {
            if (m_poly_synth) {
                if (m_mod_matrix) {
                    m_mod_matrix->evaluate_block(frames, clock.bpm());
                }
                const float mod_c = m_mod_matrix ? m_mod_matrix->get_destination_value(modulation::ModulationDestination::SynthCutoff) : 0.0f;
                const float mod_p = m_mod_matrix ? m_mod_matrix->get_destination_value(modulation::ModulationDestination::SynthPitch) : 0.0f;
                const float mod_a = m_mod_matrix ? m_mod_matrix->get_destination_value(modulation::ModulationDestination::SynthAmp) : 0.0f;
                m_poly_synth->process_block(left, right, frames, clock.bpm(), mod_c, mod_p, mod_a);
            } else {
                std::memset(left, 0, frames * sizeof(Sample));
                std::memset(right, 0, frames * sizeof(Sample));
            }
            return;
        }

        if (mode == TrackInputMode::MergeAll) {
            // Additive summing: render internal clip/launcher/sequencer/poly_synth and sum into existing external audio
            Sample tmp_l[2048];
            Sample tmp_r[2048];
            const uint32_t f_proc = std::min(frames, 2048u);
            if (m_poly_synth) {
                if (m_mod_matrix) {
                    m_mod_matrix->evaluate_block(f_proc, clock.bpm());
                }
                const float mod_c = m_mod_matrix ? m_mod_matrix->get_destination_value(modulation::ModulationDestination::SynthCutoff) : 0.0f;
                const float mod_p = m_mod_matrix ? m_mod_matrix->get_destination_value(modulation::ModulationDestination::SynthPitch) : 0.0f;
                const float mod_a = m_mod_matrix ? m_mod_matrix->get_destination_value(modulation::ModulationDestination::SynthAmp) : 0.0f;
                m_poly_synth->process_block(tmp_l, tmp_r, f_proc, clock.bpm(), mod_c, mod_p, mod_a);
                for (uint32_t f = 0; f < f_proc; ++f) {
                    left[f] += tmp_l[f];
                    right[f] += tmp_r[f];
                }
            } else if (m_launcher.is_active()) {
                m_launcher.render(tmp_l, tmp_r, f_proc, clock, boundary_events);
                m_clip_playhead.store(m_launcher.playhead(), std::memory_order_relaxed);
                for (uint32_t f = 0; f < f_proc; ++f) {
                    left[f] += tmp_l[f];
                    right[f] += tmp_r[f];
                }
            } else if (is_sequencer_enabled()) {
                m_sequencer->render(tmp_l, tmp_r, f_proc, clock, boundary_events);
                for (uint32_t f = 0; f < f_proc; ++f) {
                    left[f] += tmp_l[f];
                    right[f] += tmp_r[f];
                }
            } else if (m_clip) {
                if (m_sync_to_transport.load(std::memory_order_relaxed) && !clock.is_playing()) {
                    if (clock.is_scrubbing()) {
                        m_streamer.render(tmp_l, tmp_r, f_proc, clock);
                        m_clip_playhead.store(m_streamer.playhead(), std::memory_order_relaxed);
                        for (uint32_t f = 0; f < f_proc; ++f) {
                            left[f] += tmp_l[f];
                            right[f] += tmp_r[f];
                        }
                    }
                } else {
                    m_streamer.render(tmp_l, tmp_r, f_proc, clock);
                    m_clip_playhead.store(m_streamer.playhead(), std::memory_order_relaxed);
                    for (uint32_t f = 0; f < f_proc; ++f) {
                        left[f] += tmp_l[f];
                        right[f] += tmp_r[f];
                    }
                }
            }
            return;
        }

        // Dedicated clip launcher active (overrides arranger playback)
        if (m_launcher.is_active()) {
            m_launcher.render(left, right, frames, clock, boundary_events);
            m_clip_playhead.store(m_launcher.playhead(), std::memory_order_relaxed);
            return;
        }

        // Arranger fallback
        if (is_sequencer_enabled()) {
            m_sequencer->render(left, right, frames, clock, boundary_events);
        } else if (m_clip) {
            if (m_sync_to_transport.load(std::memory_order_relaxed) && !clock.is_playing()) {
                if (clock.is_scrubbing()) {
                    m_streamer.render(left, right, frames, clock);
                    m_clip_playhead.store(m_streamer.playhead(), std::memory_order_relaxed);
                } else {
                    std::memset(left, 0, frames * sizeof(Sample));
                    std::memset(right, 0, frames * sizeof(Sample));
                }
            } else {
                m_streamer.render(left, right, frames, clock);
                m_clip_playhead.store(m_streamer.playhead(), std::memory_order_relaxed);
            }
        }

    }

    void render_input(uint32_t frames) noexcept {
        if (m_input_mode.load(std::memory_order_relaxed) == TrackInputMode::PolySynth && m_poly_synth) {
            Sample* left = m_buffer.view().channel(0);
            Sample* right = m_buffer.view().channel(1);
            if (m_mod_matrix) {
                m_mod_matrix->evaluate_block(frames, 120.0);
            }
            const float mod_c = m_mod_matrix ? m_mod_matrix->get_destination_value(modulation::ModulationDestination::SynthCutoff) : 0.0f;
            const float mod_p = m_mod_matrix ? m_mod_matrix->get_destination_value(modulation::ModulationDestination::SynthPitch) : 0.0f;
            const float mod_a = m_mod_matrix ? m_mod_matrix->get_destination_value(modulation::ModulationDestination::SynthAmp) : 0.0f;
            m_poly_synth->process_block(left, right, frames, 120.0, mod_c, mod_p, mod_a);
            return;
        }
        if (m_clip) {
            Sample* left = m_buffer.view().channel(0);
            Sample* right = m_buffer.view().channel(1);
            m_streamer.render(left, right, frames, m_clip->sample_rate(), 120.0, true);
            m_clip_playhead.store(m_streamer.playhead(), std::memory_order_relaxed);
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
    void process_channel_strip(uint32_t frames,
                               const routing::UniversalRoutingMatrix* matrix = nullptr,
                               double start_beat = 0.0,
                               double end_beat = 0.0,
                               bool is_playing = false) noexcept {
        (void)end_beat;
        (void)is_playing;
        Sample* left = m_buffer.view().channel(0);
        Sample* right = m_buffer.view().channel(1);

        // Pre-Insert Input Gain (Trim), Phase Invert & Input Metering
        float in_g = m_input_gain.load(std::memory_order_relaxed);
        bool in_inv = m_input_phase_invert.load(std::memory_order_relaxed);
        float mult = in_inv ? -in_g : in_g;

        float in_peak_l = 0.0f;
        float in_peak_r = 0.0f;

        for (uint32_t i = 0; i < frames; ++i) {
            if (mult != 1.0f) {
                left[i] *= mult;
                right[i] *= mult;
            }
            float al = std::abs(left[i]);
            float ar = std::abs(right[i]);
            if (al > in_peak_l) in_peak_l = al;
            if (ar > in_peak_r) in_peak_r = ar;
        }
        m_input_meter_peak_l.store(in_peak_l, std::memory_order_relaxed);
        m_input_meter_peak_r.store(in_peak_r, std::memory_order_relaxed);

        // 1. Process Modular Insert Slots (Baxandall EQ, ButterComp2, MultiHeadOde, PurestDrive, WASM)
        for (size_t s = 0; s < m_slots.size(); ++s) {
            auto* proc = m_slots[s].processor();
            if (proc) {
                const uint32_t num_p = std::min<uint32_t>(static_cast<uint32_t>(kMaxSlotParams), proc->parameter_count());
                for (uint32_t p = 0; p < num_p; ++p) {
                    if (m_slot_automation_enabled[s][p].load(std::memory_order_relaxed)) {
                        float val = m_slot_curves[s][p].evaluate_audio_sample(start_beat);
                        proc->set_parameter(p, val);
                    }
                }
            }

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

    // Plugin Delay Compensation (PDC) Query & Delay Application
    [[nodiscard]] uint32_t latency_samples() const noexcept {
        uint32_t lat = 0;
        for (const auto& slot : m_slots) {
            lat += slot.latency_samples();
        }
        return lat;
    }

    void set_pdc_delay_samples(uint32_t delay) noexcept {
        m_pdc_delay_samples.store(std::min(delay, static_cast<uint32_t>(m_pdc_buffer.capacity() - 1)), std::memory_order_relaxed);
    }

    [[nodiscard]] uint32_t pdc_delay_samples() const noexcept {
        return m_pdc_delay_samples.load(std::memory_order_relaxed);
    }

    void apply_pdc_delay(uint32_t frames) noexcept {
        const uint32_t delay = m_pdc_delay_samples.load(std::memory_order_relaxed);
        if (delay == 0) return;

        Sample* left = m_buffer.view().channel(0);
        Sample* right = m_buffer.view().channel(1);
        Sample* pdc_l = m_pdc_buffer.channel(0);
        Sample* pdc_r = m_pdc_buffer.channel(1);
        const size_t cap = m_pdc_buffer.capacity();

        for (uint32_t i = 0; i < frames; ++i) {
            const size_t w_pos = m_pdc_write_pos;
            const size_t r_pos = (w_pos + cap - delay) % cap;

            const float in_l = left[i];
            const float in_r = right[i];

            pdc_l[w_pos] = in_l;
            pdc_r[w_pos] = in_r;

            left[i] = pdc_l[r_pos];
            right[i] = pdc_r[r_pos];

            m_pdc_write_pos = (w_pos + 1) % cap;
        }
    }

    // Sample-accurate parameter ramping state
    float m_gain_l_ramp_start{0.7071f};
    float m_gain_r_ramp_start{0.7071f};
    bool m_has_previous_gain{false};

private:
    uint32_t m_id;
    std::string m_name;
    AudioBuffer m_buffer;
    AudioBuffer m_pdc_buffer{2, 16384};
    size_t m_pdc_write_pos{0};
    std::atomic<uint32_t> m_pdc_delay_samples{0};

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
    std::atomic<int32_t> m_seq_send_a_bus{-1}; // Bus ID for Sequencer Send A (e.g. Reverb)
    std::atomic<int32_t> m_seq_send_b_bus{-1}; // Bus ID for Sequencer Send B (e.g. Delay)

    std::array<InsertSlot, kMaxTrackInsertSlots> m_slots;
    std::array<SendInfo, kMaxTrackSends> m_sends{};
    dsp::ConsoleProcessor m_console;

    std::shared_ptr<sampling::AudioClip> m_clip{nullptr};
    std::atomic<bool> m_clip_loop{true};
    std::atomic<double> m_clip_playhead{0.0};
    std::atomic<bool> m_sync_to_transport{false};
    std::atomic<TrackInputMode> m_input_mode{TrackInputMode::InternalClip};
    std::atomic<float> m_input_gain{1.0f};
    std::atomic<bool> m_input_phase_invert{false};
    std::atomic<float> m_input_meter_peak_l{0.0f};
    std::atomic<float> m_input_meter_peak_r{0.0f};
    modulation::PolyphonicSynth* m_poly_synth{nullptr};
    modulation::ModulationMatrix* m_mod_matrix{nullptr};
    sampling::VariSpeedStreamer m_streamer;

    std::shared_ptr<sequencer::StepSequencer> m_sequencer{nullptr};
    std::atomic<bool> m_sequencer_enabled{false};
    sequencer::ClipLauncher m_launcher;

    routing::AutomationCurve m_gain_curve;
    std::atomic<bool> m_gain_automation_enabled{false};
    routing::AutomationCurve m_pan_curve;
    std::atomic<bool> m_pan_automation_enabled{false};
    routing::AutomationCurve m_aux1_curve;
    std::atomic<bool> m_aux1_automation_enabled{false};
    routing::AutomationCurve m_aux2_curve;
    std::atomic<bool> m_aux2_automation_enabled{false};
    std::array<std::array<routing::AutomationCurve, kMaxSlotParams>, kMaxTrackInsertSlots> m_slot_curves;
    std::array<std::array<std::atomic<bool>, kMaxSlotParams>, kMaxTrackInsertSlots> m_slot_automation_enabled{};

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

    [[nodiscard]] uint32_t latency_samples() const noexcept {
        uint32_t lat = 0;
        for (const auto& slot : m_slots) {
            lat += slot.latency_samples();
        }
        return lat;
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

    void reset_meters() noexcept {
        m_meter_peak_l.store(0.0f, std::memory_order_relaxed);
        m_meter_peak_r.store(0.0f, std::memory_order_relaxed);
        m_meter_rms_l.store(0.0f, std::memory_order_relaxed);
        m_meter_rms_r.store(0.0f, std::memory_order_relaxed);
    }

    void clear() noexcept {
        m_buffer.clear();
    }

    void reset_playback_state() noexcept {
        clear();
        reset_meters();
        for (auto& s : m_slots) {
            s.reset();
        }
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
    static constexpr size_t kMaxBlockFrames = routing::UniversalRoutingMatrix::kMaxBlockFrames;

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
        m_kinetic_meter.set_sample_rate(sample_rate);
    }

    [[nodiscard]] uint32_t sample_rate() const noexcept {
        return m_clock.sample_rate();
    }

    [[nodiscard]] uint32_t buffer_frames() const noexcept {
        return m_buffer_frames;
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

    bool update_route_conditioning(uint32_t patch_id, const routing::InlineConditionerConfig& config) noexcept {
        return m_routing_matrix.update_patch_config(patch_id, config);
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

    int32_t connect_aux_send(uint32_t src_track_id, uint32_t dst_bus_id, float send_gain = 1.0f,
                             routing::TapPoint tap = routing::TapPoint::PostInsert,
                             routing::RouteChannel channel = routing::RouteChannel::StereoBoth) noexcept {
        routing::RoutingPatch p{};
        p.source_type = routing::RoutingSourceType::TrackAudio;
        p.source_id = src_track_id;
        p.tap_point = tap;
        p.source_channel = channel;
        p.dest_type = routing::RoutingDestType::BusAuxInput;
        p.dest_id = dst_bus_id;
        p.dest_channel = channel;
        p.conditioning.filter_mode = routing::ConditionerFilterMode::Bypass;
        p.conditioning.gain = send_gain;
        return m_routing_matrix.add_patch(p);
    }

    int32_t connect_track_audio(uint32_t src_track_id, uint32_t dst_track_id, float gain = 1.0f,
                                routing::TapPoint tap = routing::TapPoint::Input,
                                routing::RouteChannel channel = routing::RouteChannel::StereoBoth) noexcept {
        routing::RoutingPatch p{};
        p.source_type = routing::RoutingSourceType::TrackAudio;
        p.source_id = src_track_id;
        p.tap_point = tap;
        p.source_channel = channel;
        p.dest_type = routing::RoutingDestType::TrackAudioInput;
        p.dest_id = dst_track_id;
        p.dest_channel = channel;
        p.conditioning.filter_mode = routing::ConditionerFilterMode::Bypass;
        p.conditioning.gain = gain;
        return m_routing_matrix.add_patch(p);
    }

    int32_t connect_aoip_transmit(uint32_t src_track_or_bus_id, bool is_bus, uint16_t dst_tx_channel,
                                  routing::RouteChannel channel = routing::RouteChannel::StereoBoth) noexcept {
        routing::RoutingPatch p{};
        p.source_type = is_bus ? routing::RoutingSourceType::BusAudio : routing::RoutingSourceType::TrackAudio;
        p.source_id = src_track_or_bus_id;
        p.tap_point = routing::TapPoint::PostInsert;
        p.source_channel = channel;
        p.dest_type = routing::RoutingDestType::NetworkAoipSink;
        p.dest_id = dst_tx_channel;
        p.dest_channel = channel;
        p.conditioning.filter_mode = routing::ConditionerFilterMode::Bypass;
        p.conditioning.gain = 1.0f;
        return m_routing_matrix.add_patch(p);
    }

    void set_aoip_transmitter(network::AoipTransmitter* transmitter) noexcept {
        m_aoip_transmitter = transmitter;
    }
    [[nodiscard]] network::AoipTransmitter* aoip_transmitter() noexcept { return m_aoip_transmitter; }
    [[nodiscard]] const network::AoipTransmitter* aoip_transmitter() const noexcept { return m_aoip_transmitter; }

    void feed_dante_channel(uint16_t channel, const float* samples, uint32_t frames) noexcept {
        m_routing_matrix.feed_network_channel(channel, samples, frames);
    }

    // Ingest incoming AoIP network frames into all mapped active tracks
    void ingest_aoip(network::AoipReceiver& receiver, uint32_t frames) noexcept {
        for (auto& track : m_tracks) {
            if (!track->is_active()) continue;
            auto mode = track->input_mode();
            if (mode == TrackInputMode::NetworkAoip || (!track->has_clip() && mode != TrackInputMode::PipeWireStream)) {
                Sample* l = track->buffer().view().channel(0);
                Sample* r = track->buffer().view().channel(1);
                receiver.read_track_frames(track->id(), l, r, frames);
            } else if (mode == TrackInputMode::MergeAll) {
                Sample tmp_l[2048];
                Sample tmp_r[2048];
                const uint32_t f_to_read = std::min(frames, 2048u);
                uint32_t popped = receiver.read_track_frames(track->id(), tmp_l, tmp_r, f_to_read);
                if (popped > 0) {
                    Sample* l = track->buffer().view().channel(0);
                    Sample* r = track->buffer().view().channel(1);
                    for (uint32_t f = 0; f < popped; ++f) {
                        l[f] += tmp_l[f];
                        r[f] += tmp_r[f];
                    }
                }
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

    void set_parameter_ramping_enabled(bool enabled) noexcept {
        m_parameter_ramping_enabled.store(enabled, std::memory_order_relaxed);
    }
    [[nodiscard]] bool is_parameter_ramping_enabled() const noexcept {
        return m_parameter_ramping_enabled.load(std::memory_order_relaxed);
    }

    // ========================================================================
    // Plugin Delay Compensation (PDC) Architecture
    // ========================================================================
    void update_pdc_delay_compensation() noexcept {
        uint32_t max_master_lat = 0;
        std::array<uint32_t, kMaxBuses + 1> max_bus_lat{};

        for (const auto& track : m_tracks) {
            if (!track->is_active()) continue;
            const uint32_t lat = track->latency_samples();
            const int32_t tgt = track->target_bus();
            if (tgt <= 0) {
                if (lat > max_master_lat) max_master_lat = lat;
            } else if (static_cast<size_t>(tgt) < max_bus_lat.size()) {
                if (lat > max_bus_lat[tgt]) max_bus_lat[tgt] = lat;
            }
        }

        for (auto& track : m_tracks) {
            if (!track->is_active()) continue;
            const uint32_t lat = track->latency_samples();
            const int32_t tgt = track->target_bus();
            const uint32_t target_max = (tgt <= 0) ? max_master_lat :
                (static_cast<size_t>(tgt) < max_bus_lat.size() ? max_bus_lat[tgt] : 0);
            track->set_pdc_delay_samples(target_max >= lat ? (target_max - lat) : 0);
        }
    }

    [[nodiscard]] uint32_t total_latency_samples() noexcept {
        update_pdc_delay_compensation();
        uint32_t max_track_lat = 0;
        for (const auto& track : m_tracks) {
            if (!track->is_active()) continue;
            uint32_t t_lat = track->latency_samples() + track->pdc_delay_samples();
            if (t_lat > max_track_lat) max_track_lat = t_lat;
        }
        return max_track_lat + m_master_bus.latency_samples();
    }

    [[nodiscard]] uint64_t auto_detect_project_frames() const noexcept {
        uint64_t max_frames = 0;
        for (const auto& track : m_tracks) {
            if (!track->is_active()) continue;
            if (track->has_clip() && track->clip()) {
                max_frames = std::max(max_frames, static_cast<uint64_t>(track->clip()->num_frames()));
            }
            if (track->is_sequencer_enabled() && track->sequencer()) {
                uint64_t pat_frames = static_cast<uint64_t>(m_clock.samples_per_bar() * 1);
                max_frames = std::max(max_frames, pat_frames);
            }
        }
        return max_frames;
    }

    void reset_playback_state(double playhead = 0.0) noexcept {
        for (auto& track : m_tracks) {
            if (track) {
                track->reset_playback_state(playhead);
            }
        }
        for (auto& bus : m_buses) {
            if (bus) {
                bus->reset_playback_state();
            }
        }
        m_master_bus.reset_playback_state();
    }

    // ========================================================================
    // Unified Timeline Scrubbing & Seek Architecture
    // ========================================================================
    void seek(uint64_t target_sample, bool reset_dsp = false) noexcept {
        m_clock.seek(target_sample);
        for (auto& track : m_tracks) {
            if (track) {
                if (reset_dsp) {
                    track->reset_playback_state(static_cast<double>(target_sample));
                } else {
                    track->set_clip_playhead(static_cast<double>(target_sample));
                }
            }
        }
    }

    void start_scrub(uint64_t target_sample) noexcept {
        m_clock.start_scrub(target_sample);
        for (auto& track : m_tracks) {
            if (track) {
                track->set_clip_playhead(static_cast<double>(target_sample));
            }
        }
    }

    void update_scrub(uint64_t target_sample, double velocity = 1.0) noexcept {
        m_clock.update_scrub(target_sample, velocity);
        for (auto& track : m_tracks) {
            if (track) {
                track->set_clip_playhead(static_cast<double>(target_sample));
            }
        }
    }

    void end_scrub(uint64_t target_sample) noexcept {
        m_clock.end_scrub(target_sample);
        for (auto& track : m_tracks) {
            if (track) {
                track->set_clip_playhead(static_cast<double>(target_sample));
            }
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

    [[nodiscard]] const Track* get_track(uint32_t id) const noexcept {
        if (id >= 1 && id <= kMaxTracks) {
            auto* t = m_tracks[id - 1].get();
            return t->is_active() ? t : nullptr;
        }
        return nullptr;
    }

    bool assign_track_poly_synth(uint32_t track_id, modulation::PolyphonicSynth* synth, modulation::ModulationMatrix* matrix = nullptr) noexcept {
        Track* trk = get_track(track_id);
        if (!trk) return false;
        trk->set_poly_synth(synth, matrix);
        trk->set_input_mode(TrackInputMode::PolySynth);
        return true;
    }

    [[nodiscard]] Track* track_by_index(size_t idx) noexcept {
        return (idx < kMaxTracks) ? m_tracks[idx].get() : nullptr;
    }

    [[nodiscard]] const Track* track_by_index(size_t idx) const noexcept {
        return (idx < kMaxTracks) ? m_tracks[idx].get() : nullptr;
    }

    [[nodiscard]] float master_volume() const noexcept { return m_master_bus.gain(); }
    void set_master_volume(float vol) noexcept { m_master_bus.set_gain(vol); }

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

    void launch_track_clip(uint32_t track_id, uint32_t slot_id,
                           sequencer::LaunchQuantize quantize = sequencer::LaunchQuantize::Bar,
                           bool legato = false) noexcept {
        protocol::MixerCommand cmd;
        cmd.type = protocol::MixerCommandType::LaunchTrackClip;
        cmd.target_id = track_id;
        cmd.secondary_id = slot_id;
        cmd.flags = (static_cast<uint32_t>(quantize) & 0xFF) | ((legato ? 1u : 0u) << 8);
        post_command(cmd);
    }

    void stop_track_clip(uint32_t track_id,
                         sequencer::LaunchQuantize quantize = sequencer::LaunchQuantize::Bar) noexcept {
        protocol::MixerCommand cmd;
        cmd.type = protocol::MixerCommandType::StopTrackClip;
        cmd.target_id = track_id;
        cmd.flags = static_cast<uint32_t>(quantize) & 0xFF;
        post_command(cmd);
    }

    void launch_scene(uint32_t scene_id,
                      sequencer::LaunchQuantize quantize = sequencer::LaunchQuantize::Bar,
                      bool legato = false) noexcept {
        protocol::MixerCommand cmd;
        cmd.type = protocol::MixerCommandType::LaunchScene;
        cmd.target_id = scene_id;
        cmd.flags = (static_cast<uint32_t>(quantize) & 0xFF) | ((legato ? 1u : 0u) << 8);
        post_command(cmd);
    }

    void stop_all_clips(sequencer::LaunchQuantize quantize = sequencer::LaunchQuantize::Bar) noexcept {
        protocol::MixerCommand cmd;
        cmd.type = protocol::MixerCommandType::StopAllClips;
        cmd.flags = static_cast<uint32_t>(quantize) & 0xFF;
        post_command(cmd);
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

        size_t active_b_idx = 0;
        for (size_t i = 0; i < kMaxBuses && active_b_idx < protocol::kMaxTelemetryBuses; ++i) {
            if (m_buses[i]->is_active()) {
                auto bm = m_buses[i]->meter();
                out_frame.bus_meters[active_b_idx++] = {bm.peak_l, bm.peak_r, bm.rms_l, bm.rms_r};
            }
        }

        m_kinetic_meter.capture_telemetry(out_frame.kinetic_meter);
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

        // Phase 3b: Evaluate Pre-Insert / Input tap routes (e.g. Track 2 Lowpass Sidechain / Track Audio In)
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

        // Phase 3b2: Accumulate matrix-routed audio input (TrackAudioInput)
        for (auto& track : m_tracks) {
            if (!track->is_active()) continue;
            if (m_routing_matrix.has_track_input(track->id())) {
                const float* in_l = m_routing_matrix.track_input_l(track->id());
                const float* in_r = m_routing_matrix.track_input_r(track->id());
                Sample* raw_l = track->buffer().view().channel(0);
                Sample* raw_r = track->buffer().view().channel(1);
                if (!track->has_clip() && !track->is_sequencer_enabled() && track->input_mode() != TrackInputMode::MergeAll) {
                    std::copy_n(in_l, frames, raw_l);
                    std::copy_n(in_r, frames, raw_r);
                } else {
                    for (uint32_t i = 0; i < frames; ++i) {
                        raw_l[i] += in_l[i];
                        raw_r[i] += in_r[i];
                    }
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
            double start_beat;
            double end_beat;
            bool is_playing;
        };

        // Evaluate Plugin Delay Compensation (PDC) across active buses
        update_pdc_delay_compensation();

        const double spb = std::max(1.0, m_clock.samples_per_beat());
        const double block_start_beat = static_cast<double>(m_clock.sample_position()) / spb;
        const double block_end_beat   = static_cast<double>(m_clock.sample_position() + frames) / spb;
        const bool block_is_playing   = m_clock.is_playing();

        TrackRenderCtx trk_ctx{
            .self = this,
            .frames = frames,
            .clock = &m_clock,
            .events = &boundary_events,
            .any_solo = any_solo,
            .start_beat = block_start_beat,
            .end_beat = block_end_beat,
            .is_playing = block_is_playing
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

            // In-line Channel Strip processing with Routing Matrix sidechain access & slot automation!
            track->process_channel_strip(ctx->frames, &ctx->self->m_routing_matrix, ctx->start_beat, ctx->end_beat, ctx->is_playing);

            // Apply Plugin Delay Compensation (PDC) sample-exact alignment
            track->apply_pdc_delay(ctx->frames);

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

        // 3b. Vectorized Bus & Master Accumulation with Sample-Accurate Parameter Ramping
        for (auto& track : m_tracks) {
            if (!track->is_active()) continue;
            bool trk_muted = is_track_effectively_muted(track.get());
            bool trk_solo = is_track_effectively_solo(track.get());
            if (trk_muted || (any_solo && !trk_solo && !track->is_solo_safe())) continue;

            const Sample* trk_l = track->buffer().view().channel(0);
            const Sample* trk_r = track->buffer().view().channel(1);

            const float target_gain = compute_track_effective_gain(track.get());
            const auto [pan_l, pan_r] = calculate_pan_gains(track->pan());
            const float target_left_gain = target_gain * pan_l;
            const float target_right_gain = target_gain * pan_r;

            const bool auto_gain_active = track->is_gain_automation_enabled();
            const bool auto_pan_active  = track->is_pan_automation_enabled();
            const bool auto_aux1_active = track->is_aux1_automation_enabled();
            const bool auto_aux2_active = track->is_aux2_automation_enabled();

            alignas(64) float auto_curve[kMaxBlockFrames];
            alignas(64) float auto_pan[kMaxBlockFrames];
            alignas(64) float auto_aux1[kMaxBlockFrames];
            alignas(64) float auto_aux2[kMaxBlockFrames];

            if (auto_gain_active || auto_pan_active || auto_aux1_active || auto_aux2_active) {
                const double spb = std::max(1.0, m_clock.samples_per_beat());
                const double start_beat = static_cast<double>(m_clock.sample_position()) / spb;
                const double end_beat   = static_cast<double>(m_clock.sample_position() + frames) / spb;

                if (auto_gain_active) {
                    track->gain_curve().evaluate_audio_block(start_beat, end_beat, auto_curve, frames);
                }
                if (auto_pan_active) {
                    track->pan_curve().evaluate_audio_block(start_beat, end_beat, auto_pan, frames);
                }
                if (auto_aux1_active) {
                    track->aux1_curve().evaluate_audio_block(start_beat, end_beat, auto_aux1, frames);
                }
                if (auto_aux2_active) {
                    track->aux2_curve().evaluate_audio_block(start_beat, end_beat, auto_aux2, frames);
                }
            }

            if (!track->m_has_previous_gain) {
                track->m_gain_l_ramp_start = target_left_gain;
                track->m_gain_r_ramp_start = target_right_gain;
                track->m_has_previous_gain = true;
            }

            const float start_l = track->m_gain_l_ramp_start;
            const float start_r = track->m_gain_r_ramp_start;
            const bool ramping_enabled = m_parameter_ramping_enabled.load(std::memory_order_relaxed);
            const bool is_ramping = ramping_enabled &&
                                    ((std::abs(target_left_gain - start_l) > 1e-5f) ||
                                     (std::abs(target_right_gain - start_r) > 1e-5f));

            // Routing destination: Spatial Master Bus, Submix bus or Master
            const int32_t target_id = track->target_bus();
            if (target_id == kSpatialMasterBusId) {
                // Vector-Base Spatial Panning into Spatial Master Bus
                // Downmix track stereo output to mono point source in scratch buffer
                Sample* scratch_mono = m_scratch_stereo_buffer.channel(0);
                for (uint32_t i = 0; i < frames; ++i) {
                    scratch_mono[i] = 0.5f * (trk_l[i] + trk_r[i]);
                }
                m_spatial_master_bus.pan_mono_circular(scratch_mono, target_gain, track->azimuth(), frames);
            } else {
                AudioBus* target_bus = (target_id > 0) ? get_bus(static_cast<uint32_t>(target_id)) : &m_master_bus;
                if (!target_bus) target_bus = &m_master_bus;

                Sample* dst_l = target_bus->buffer().view().channel(0);
                Sample* dst_r = target_bus->buffer().view().channel(1);

                if (auto_gain_active || auto_pan_active) {
                    #if defined(__GNUC__) || defined(__clang__)
                    #pragma GCC ivdep
                    #endif
                    for (uint32_t i = 0; i < frames; ++i) {
                        const float cur_g = auto_gain_active ? (target_gain * auto_curve[i]) : target_gain;
                        float cur_pan_l = pan_l;
                        float cur_pan_r = pan_r;
                        if (auto_pan_active) {
                            const auto [p_l, p_r] = calculate_pan_gains(auto_pan[i]);
                            cur_pan_l = p_l;
                            cur_pan_r = p_r;
                        }
                        dst_l[i] += trk_l[i] * (cur_g * cur_pan_l);
                        dst_r[i] += trk_r[i] * (cur_g * cur_pan_r);
                    }
                } else if (is_ramping) {
                    const float step_l = (target_left_gain - start_l) / static_cast<float>(frames);
                    const float step_r = (target_right_gain - start_r) / static_cast<float>(frames);
                    #if defined(__GNUC__) || defined(__clang__)
                    #pragma GCC ivdep
                    #endif
                    for (uint32_t i = 0; i < frames; ++i) {
                        const float cur_l = start_l + step_l * static_cast<float>(i + 1);
                        const float cur_r = start_r + step_r * static_cast<float>(i + 1);
                        dst_l[i] += trk_l[i] * cur_l;
                        dst_r[i] += trk_r[i] * cur_r;
                    }
                } else {
                    #if defined(__GNUC__) || defined(__clang__)
                    #pragma GCC ivdep
                    #endif
                    for (uint32_t i = 0; i < frames; ++i) {
                        dst_l[i] += trk_l[i] * target_left_gain;
                        dst_r[i] += trk_r[i] * target_right_gain;
                    }
                }
            }

            track->m_gain_l_ramp_start = target_left_gain;
            track->m_gain_r_ramp_start = target_right_gain;

            // Auxiliary Sends (e.g. Reverb / Delay Busses)
            for (const auto& send : track->sends()) {
                const bool send_auto = (send.bus_id == 1 && auto_aux1_active) || (send.bus_id == 2 && auto_aux2_active);
                if (!send.active || (!send_auto && send.amount <= 0.0f)) continue;
                AudioBus* send_bus = get_bus(send.bus_id);
                if (send_bus) {
                    Sample* s_l = send_bus->buffer().view().channel(0);
                    Sample* s_r = send_bus->buffer().view().channel(1);
                    const float base_send = send.amount;
                    const float* send_auto_buf = (send.bus_id == 1) ? auto_aux1 : auto_aux2;

                    #if defined(__GNUC__) || defined(__clang__)
                    #pragma GCC ivdep
                    #endif
                    for (uint32_t i = 0; i < frames; ++i) {
                        const float s_amt = send_auto ? std::clamp(send_auto_buf[i], 0.0f, 1.0f) : base_send;
                        const float cur_trk_g = auto_gain_active ? (target_gain * auto_curve[i]) : target_gain;
                        const float eff_s = send.pre_fader ? s_amt : (cur_trk_g * s_amt);
                        float cur_pan_l = pan_l;
                        float cur_pan_r = pan_r;
                        if (auto_pan_active) {
                            const auto [p_l, p_r] = calculate_pan_gains(auto_pan[i]);
                            cur_pan_l = p_l;
                            cur_pan_r = p_r;
                        }
                        s_l[i] += trk_l[i] * (eff_s * cur_pan_l);
                        s_r[i] += trk_r[i] * (eff_s * cur_pan_r);
                    }
                }
            }

            // Sequencer Per-Voice Aux Sends (Send A & Send B)
            if (track->is_sequencer_enabled()) {
                auto* seq = track->sequencer();
                if (seq) {
                    const int32_t s_a_id = track->sequencer_send_a_bus();
                    if (s_a_id >= 0) {
                        AudioBus* s_bus_a = get_bus(static_cast<uint32_t>(s_a_id));
                        if (s_bus_a) {
                            Sample* b_l = s_bus_a->buffer().view().channel(0);
                            Sample* b_r = s_bus_a->buffer().view().channel(1);
                            const Sample* sa_l = seq->send_a_buffer().channel(0);
                            const Sample* sa_r = seq->send_a_buffer().channel(1);
                            #if defined(__GNUC__) || defined(__clang__)
                            #pragma GCC ivdep
                            #endif
                            for (uint32_t i = 0; i < frames; ++i) {
                                b_l[i] += sa_l[i] * target_gain;
                                b_r[i] += sa_r[i] * target_gain;
                            }
                        }
                    }

                    const int32_t s_b_id = track->sequencer_send_b_bus();
                    if (s_b_id >= 0) {
                        AudioBus* s_bus_b = get_bus(static_cast<uint32_t>(s_b_id));
                        if (s_bus_b) {
                            Sample* b_l = s_bus_b->buffer().view().channel(0);
                            Sample* b_r = s_bus_b->buffer().view().channel(1);
                            const Sample* sb_l = seq->send_b_buffer().channel(0);
                            const Sample* sb_r = seq->send_b_buffer().channel(1);
                            #if defined(__GNUC__) || defined(__clang__)
                            #pragma GCC ivdep
                            #endif
                            for (uint32_t i = 0; i < frames; ++i) {
                                b_l[i] += sb_l[i] * target_gain;
                                b_r[i] += sb_r[i] * target_gain;
                            }
                        }
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

            // Accumulate matrix-routed aux input (BusAuxInput)
            if (m_routing_matrix.has_bus_aux(bus->id())) {
                const float* aux_l = m_routing_matrix.bus_aux_l(bus->id());
                const float* aux_r = m_routing_matrix.bus_aux_r(bus->id());
                Sample* dst_l = bus->buffer().view().channel(0);
                Sample* dst_r = bus->buffer().view().channel(1);
                for (uint32_t i = 0; i < frames; ++i) {
                    dst_l[i] += aux_l[i];
                    dst_r[i] += aux_r[i];
                }
            }

            bus->process_buss_strip(frames, &m_routing_matrix);

            const float bus_gain = bus->gain();
            const Sample* b_l = bus->buffer().view().channel(0);
            const Sample* b_r = bus->buffer().view().channel(1);

            // Evaluate routes with BusAudio as source (Submix Bus)
            for (size_t r = 0; r < routing::UniversalRoutingMatrix::kMaxRoutes; ++r) {
                const auto& patch = m_routing_matrix.patches()[r];
                if (!patch.active) continue;
                if (patch.source_type == routing::RoutingSourceType::BusAudio && patch.source_id == bus->id()) {
                    m_routing_matrix.process_route(r, b_l, b_r, frames);
                }
            }

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

        // Evaluate routes with BusAudio as source for Master Bus
        for (size_t r = 0; r < routing::UniversalRoutingMatrix::kMaxRoutes; ++r) {
            const auto& patch = m_routing_matrix.patches()[r];
            if (!patch.active) continue;
            if (patch.source_type == routing::RoutingSourceType::BusAudio &&
                (patch.source_id == 0 || static_cast<int32_t>(patch.source_id) == kStereoMasterBusId)) {
                m_routing_matrix.process_route(r, final_l, final_r, frames);
            }
        }

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

        // Update Kinetic ODE & Airwindows Hit Record Meter on Master Output
        m_kinetic_meter.process_block(out_l, (out_master.num_channels() > 1 ? out_r : out_l), frames);

        // 6. Network AoIP Transmitter Egress (Broadcast active matrix channels to network)
        if (m_aoip_transmitter && m_aoip_transmitter->is_open()) {
            uint16_t highest_ch = 0;
            bool has_tx = false;
            for (uint16_t ch = 0; ch < routing::UniversalRoutingMatrix::kMaxNetworkChannels; ++ch) {
                if (m_routing_matrix.has_network_tx(ch)) {
                    highest_ch = ch + 1;
                    has_tx = true;
                }
            }
            if (has_tx && highest_ch > 0) {
                const float* channel_ptrs[routing::UniversalRoutingMatrix::kMaxNetworkChannels]{};
                for (uint16_t ch = 0; ch < highest_ch; ++ch) {
                    channel_ptrs[ch] = m_routing_matrix.network_tx_channel(ch);
                }
                m_aoip_transmitter->send_multichannel(channel_ptrs, highest_ch, static_cast<uint16_t>(frames), m_clock.sample_rate(), true);
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
            case protocol::MixerCommandType::SetTrackPitchSemitones: {
                if (auto* trk = get_track(cmd.target_id)) {
                    trk->set_pitch_semitones(cmd.value1);
                }
                break;
            }
            case protocol::MixerCommandType::SetTrackPlaybackMode: {
                if (auto* trk = get_track(cmd.target_id)) {
                    trk->set_playback_mode(static_cast<sampling::PlaybackMode>(cmd.flags));
                }
                break;
            }
            case protocol::MixerCommandType::SetTrackReverse: {
                if (auto* trk = get_track(cmd.target_id)) {
                    trk->set_reverse((cmd.flags & 1) != 0);
                }
                break;
            }
            case protocol::MixerCommandType::TriggerTrackTapeStop: {
                if (auto* trk = get_track(cmd.target_id)) {
                    trk->trigger_tape_stop(cmd.value1 > 0.0f ? cmd.value1 : 0.5f);
                }
                break;
            }
            case protocol::MixerCommandType::TriggerTrackTapeStart: {
                if (auto* trk = get_track(cmd.target_id)) {
                    trk->trigger_tape_start(cmd.value1 > 0.0f ? cmd.value1 : 0.3f);
                }
                break;
            }
            case protocol::MixerCommandType::LaunchTrackClip: {
                if (auto* trk = get_track(cmd.target_id)) {
                    auto q = static_cast<sequencer::LaunchQuantize>(cmd.flags & 0xFF);
                    bool legato = ((cmd.flags >> 8) & 1) != 0;
                    trk->launch_clip(cmd.secondary_id, q, legato);
                }
                break;
            }
            case protocol::MixerCommandType::StopTrackClip: {
                if (auto* trk = get_track(cmd.target_id)) {
                    auto q = static_cast<sequencer::LaunchQuantize>(cmd.flags & 0xFF);
                    trk->stop_clip(q);
                }
                break;
            }
            case protocol::MixerCommandType::LaunchScene: {
                auto q = static_cast<sequencer::LaunchQuantize>(cmd.flags & 0xFF);
                bool legato = ((cmd.flags >> 8) & 1) != 0;
                for (auto& trk : m_tracks) {
                    if (trk && trk->is_active()) {
                        trk->launch_clip(cmd.target_id, q, legato);
                    }
                }
                break;
            }
            case protocol::MixerCommandType::StopAllClips: {
                auto q = static_cast<sequencer::LaunchQuantize>(cmd.flags & 0xFF);
                for (auto& trk : m_tracks) {
                    if (trk && trk->is_active()) {
                        trk->stop_clip(q);
                    }
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
    std::atomic<bool> m_parameter_ramping_enabled{false};
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
    dsp::KineticMeter m_kinetic_meter{48000};
    network::AoipTransmitter* m_aoip_transmitter{nullptr};
};

} // namespace audio_core
