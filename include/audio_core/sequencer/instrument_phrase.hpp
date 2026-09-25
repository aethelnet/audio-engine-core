#pragma once

#include "audio_core/types.hpp"
#include "audio_core/sequencer/tracker_timing.hpp"

#include <array>
#include <vector>
#include <string>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <memory>

namespace audio_core::sequencer {

// ============================================================================
// PhraseArpMode / Playback Mode
// ============================================================================
enum class PhraseArpMode : uint8_t {
    Transposed = 0, // Transpose whole phrase relative to base_note: note = base_phrase_note + (pressed_key - base_note)
    Direct     = 1, // Absolute pitch: phrase plays exact semitone notes recorded in pattern
    Keymap     = 2, // Renoise Drum Kit / Sliced Instrument mode: each key triggers a distinct phrase or drum slice
    ArpUp      = 3, // Arpeggiates held chord notes sorted ascending (lowest to highest)
    ArpDown    = 4, // Arpeggiates held chord notes sorted descending (highest to lowest)
    ArpUpDown  = 5, // Arpeggiates up then down without repeating boundary notes
    ArpRandom  = 6, // Arpeggiates held chord notes pseudorandomly
    ArpChord   = 7  // Triggers all held chord notes simultaneously on every active step
};

// ============================================================================
// PhraseNote: Single Note / Slice Event within a Tracker Step
// ============================================================================
struct PhraseNote {
    bool active{false};
    int8_t note_offset{0};      // Semitone offset [-60 .. +60] (or absolute MIDI note in Direct mode)
    uint16_t slice_id{0};       // Drum / sample slice ID [0 .. 65535]
    bool is_slice{false};       // True: triggers sample slice; False: triggers melodic synth note
    uint8_t velocity{100};      // MIDI Velocity [0 .. 127]
    int8_t pan{0};              // Stereo panning [-64 (L) .. +63 (R)], 0 = Center
    uint8_t delay_ticks{0};     // Micro-timing delay in ticks [0 .. TPL-1]
    uint16_t gate_ticks{12};    // Gate duration in ticks [1 .. 256]
    uint8_t probability{100};   // Step trigger probability [0 .. 100%]
    uint8_t retrigger_ticks{0}; // 0 = off, >0 = sub-line ratcheting / roll / drill every R ticks
    float cutoff_offset{0.0f};  // Filter cutoff parameter lock [-1.0 .. +1.0]
    float res_offset{0.0f};     // Filter resonance parameter lock [-1.0 .. +1.0]
};

// ============================================================================
// PhraseLine: Polyphonic tracker line with up to 4 note columns
// Allows layered drum hits (e.g. Kick + Closed Hat on line 0) or polyphonic chords.
// ============================================================================
struct PhraseLine {
    static constexpr size_t kMaxColumns = 4;
    std::array<PhraseNote, kMaxColumns> columns{};

    void clear() noexcept {
        for (auto& col : columns) {
            col = PhraseNote{};
        }
    }

    void set_note(size_t col, int8_t note_offset, uint8_t velocity = 100,
                  uint8_t delay_ticks = 0, uint16_t gate_ticks = 12,
                  uint8_t probability = 100, uint8_t retrigger_ticks = 0,
                  int8_t pan = 0, float cutoff = 0.0f, float res = 0.0f) noexcept {
        if (col < kMaxColumns) {
            columns[col] = PhraseNote{
                .active = true,
                .note_offset = note_offset,
                .slice_id = 0,
                .is_slice = false,
                .velocity = std::min<uint8_t>(velocity, 127),
                .pan = std::clamp<int8_t>(pan, -64, 63),
                .delay_ticks = delay_ticks,
                .gate_ticks = std::max<uint16_t>(1, gate_ticks),
                .probability = std::min<uint8_t>(probability, 100),
                .retrigger_ticks = retrigger_ticks,
                .cutoff_offset = std::clamp(cutoff, -1.0f, 1.0f),
                .res_offset = std::clamp(res, -1.0f, 1.0f)
            };
        }
    }

    void set_slice(size_t col, uint16_t slice_id, uint8_t velocity = 100,
                   uint8_t delay_ticks = 0, uint16_t gate_ticks = 12,
                   uint8_t probability = 100, uint8_t retrigger_ticks = 0,
                   int8_t pan = 0, float cutoff = 0.0f, float res = 0.0f) noexcept {
        if (col < kMaxColumns) {
            columns[col] = PhraseNote{
                .active = true,
                .note_offset = 0,
                .slice_id = slice_id,
                .is_slice = true,
                .velocity = std::min<uint8_t>(velocity, 127),
                .pan = std::clamp<int8_t>(pan, -64, 63),
                .delay_ticks = delay_ticks,
                .gate_ticks = std::max<uint16_t>(1, gate_ticks),
                .probability = std::min<uint8_t>(probability, 100),
                .retrigger_ticks = retrigger_ticks,
                .cutoff_offset = std::clamp(cutoff, -1.0f, 1.0f),
                .res_offset = std::clamp(res, -1.0f, 1.0f)
            };
        }
    }

    [[nodiscard]] bool has_active_notes() const noexcept {
        for (const auto& col : columns) {
            if (col.active) return true;
        }
        return false;
    }
};

// ============================================================================
// InstrumentPhrase: Renoise-Style Instrument Pattern
// Owned directly by an instrument; can act as an arpeggiator or drum variation pattern.
// ============================================================================
class InstrumentPhrase {
public:
    static constexpr uint32_t kMaxLines = 512;

    uint32_t id{0};
    std::string name{"Phrase"};
    uint32_t num_lines{16};             // Number of lines in this phrase [1 .. 512]
    uint16_t lpb{0};                    // Lines Per Beat override (0 = inherit host/instrument LPB)
    uint16_t tpl{0};                    // Ticks Per Line override (0 = inherit host/instrument TPL)
    uint8_t base_note{60};              // Root note for transposition (default 60 = C-4)
    bool loop{true};                    // Loop playback while note held
    uint32_t loop_start{0};             // Loop start line
    uint32_t loop_end{16};              // Loop end line
    PhraseArpMode playback_mode{PhraseArpMode::Transposed};

    // Keymap trigger range: in Keymap mode, hitting keys in this range triggers this phrase
    uint8_t key_trigger_min{0};
    uint8_t key_trigger_max{127};

    std::vector<PhraseLine> lines;

    explicit InstrumentPhrase(uint32_t phrase_id = 0, std::string phrase_name = "Phrase", uint32_t lines_count = 16)
        : id(phrase_id), name(std::move(phrase_name)), num_lines(std::clamp(lines_count, 1u, kMaxLines)),
          loop_end(std::clamp(lines_count, 1u, kMaxLines)), lines(std::clamp(lines_count, 1u, kMaxLines)) {}

    void resize(uint32_t count) {
        num_lines = std::clamp(count, 1u, kMaxLines);
        lines.resize(num_lines);
        loop_end = std::min(loop_end, num_lines);
    }

    void clear() noexcept {
        for (auto& l : lines) {
            l.clear();
        }
    }

    void set_note(uint32_t line, size_t col, int8_t note_offset, uint8_t velocity = 100,
                  uint8_t delay_ticks = 0, uint16_t gate_ticks = 12,
                  uint8_t probability = 100, uint8_t retrigger_ticks = 0,
                  int8_t pan = 0, float cutoff = 0.0f, float res = 0.0f) noexcept {
        if (line < lines.size()) {
            lines[line].set_note(col, note_offset, velocity, delay_ticks, gate_ticks,
                                 probability, retrigger_ticks, pan, cutoff, res);
        }
    }

    void set_slice(uint32_t line, size_t col, uint16_t slice_id, uint8_t velocity = 100,
                   uint8_t delay_ticks = 0, uint16_t gate_ticks = 12,
                   uint8_t probability = 100, uint8_t retrigger_ticks = 0,
                   int8_t pan = 0, float cutoff = 0.0f, float res = 0.0f) noexcept {
        if (line < lines.size()) {
            lines[line].set_slice(col, slice_id, velocity, delay_ticks, gate_ticks,
                                  probability, retrigger_ticks, pan, cutoff, res);
        }
    }

    // Helper: Build a classic melodic arpeggiator phrase (e.g. Major triad [0, 4, 7, 12])
    void build_arpeggiator_pattern(const std::vector<int8_t>& semitones, uint8_t velocity = 100) {
        clear();
        if (semitones.empty()) return;
        for (uint32_t i = 0; i < num_lines; ++i) {
            int8_t note = semitones[i % semitones.size()];
            set_note(i, 0, note, velocity);
        }
    }

    // Helper: Build a classic drum break variation with kick, snare, hi-hat & ratchets
    void build_drum_break_variation(uint16_t kick_slice = 0, uint16_t snare_slice = 1,
                                    uint16_t hat_slice = 2, bool add_drill_ratchet = true) {
        clear();
        if (num_lines < 16) resize(16);

        // Standard 16-step tracker break:
        // Line 0: Kick + Hat
        set_slice(0, 0, kick_slice, 115);
        set_slice(0, 1, hat_slice, 80);

        // Line 2: Hat
        set_slice(2, 0, hat_slice, 75);

        // Line 4: Snare + Hat
        set_slice(4, 0, snare_slice, 110);
        set_slice(4, 1, hat_slice, 85);

        // Line 6: Hat
        set_slice(6, 0, hat_slice, 80);

        // Line 7: Ghost Kick
        set_slice(7, 0, kick_slice, 70, 6); // Micro-timed delay (tick 6)

        // Line 8: Kick + Hat
        set_slice(8, 0, kick_slice, 105);
        set_slice(8, 1, hat_slice, 85);

        // Line 10: Kick
        set_slice(10, 0, kick_slice, 95);

        // Line 12: Snare
        set_slice(12, 0, snare_slice, 115);

        // Line 14: Drum fill / ratchet drill on 14 & 15
        if (add_drill_ratchet) {
            // Snare roll ratcheting every 3 ticks!
            set_slice(14, 0, snare_slice, 90, 0, 6, 100, 3);
            set_slice(15, 0, snare_slice, 105, 0, 6, 100, 2);
        } else {
            set_slice(14, 0, hat_slice, 90);
            set_slice(15, 0, kick_slice, 85);
        }
    }
};

// ============================================================================
// InstrumentPhraseBank: Container of Phrases for an Instrument
// ============================================================================
class InstrumentPhraseBank {
public:
    static constexpr size_t kMaxPhrases = 64;

    std::vector<InstrumentPhrase> phrases;
    size_t active_phrase_index{0};

    InstrumentPhraseBank() {
        phrases.emplace_back(0, "Phrase 01", 16);
    }

    [[nodiscard]] size_t size() const noexcept { return phrases.size(); }
    [[nodiscard]] bool empty() const noexcept { return phrases.empty(); }

    InstrumentPhrase& add_phrase(std::string name = "New Phrase", uint32_t lines = 16) {
        if (phrases.size() < kMaxPhrases) {
            uint32_t new_id = static_cast<uint32_t>(phrases.size());
            phrases.emplace_back(new_id, std::move(name), lines);
            return phrases.back();
        }
        return phrases.back();
    }

    [[nodiscard]] InstrumentPhrase* get_phrase(size_t index) noexcept {
        if (index < phrases.size()) return &phrases[index];
        return nullptr;
    }

    [[nodiscard]] const InstrumentPhrase* get_phrase(size_t index) const noexcept {
        if (index < phrases.size()) return &phrases[index];
        return nullptr;
    }

    // Renoise Keymap lookup: find phrase mapped to MIDI key
    [[nodiscard]] int32_t find_phrase_for_key(uint8_t midi_note) const noexcept {
        for (size_t i = 0; i < phrases.size(); ++i) {
            const auto& p = phrases[i];
            if (p.playback_mode == PhraseArpMode::Keymap &&
                midi_note >= p.key_trigger_min && midi_note <= p.key_trigger_max) {
                return static_cast<int32_t>(i);
            }
        }
        return -1;
    }
};

// ============================================================================
// PhraseOutputEvent: Dispatched to Synths, Samplers or DAW Arranger
// Real-time safe event payload.
// ============================================================================
struct PhraseOutputEvent {
    uint32_t frame_offset{0};
    bool is_note_on{true};
    uint8_t note{60};
    float velocity{0.8f};
    float pan{0.0f};           // [-1.0 .. +1.0]
    uint32_t slice_id{0};
    bool is_slice{false};
    float cutoff_offset{0.0f};
    float res_offset{0.0f};
    uint32_t duration_samples{0};
};

// ============================================================================
// InstrumentPhrasePlayer: Real-Time Phrase & Arpeggiator Playback Engine
// Zero heap allocations in process(); sub-line micro-tick accuracy and ratcheting.
// ============================================================================
class InstrumentPhrasePlayer {
public:
    static constexpr size_t kMaxHeldNotes = 16;
    static constexpr size_t kMaxActiveVoices = 32;

    struct HeldKey {
        uint8_t note{0};
        uint8_t velocity{0};
        uint64_t timestamp{0};
    };

    struct ScheduledNoteOff {
        bool active{false};
        uint8_t note{0};
        uint32_t slice_id{0};
        bool is_slice{false};
        uint64_t sample_time{0};
    };

    InstrumentPhrasePlayer() = default;

    void set_phrase(const InstrumentPhrase* phrase) noexcept {
        m_phrase = phrase;
    }

    [[nodiscard]] const InstrumentPhrase* phrase() const noexcept {
        return m_phrase;
    }

    [[nodiscard]] bool is_playing() const noexcept {
        return m_is_playing;
    }

    [[nodiscard]] uint32_t current_line() const noexcept {
        return static_cast<uint32_t>(m_current_line);
    }

    [[nodiscard]] size_t held_note_count() const noexcept {
        return m_num_held;
    }

    // ------------------------------------------------------------------------
    // MIDI Note Ingestion (Real-Time Safe)
    // ------------------------------------------------------------------------
    void note_on(uint8_t note, uint8_t velocity, uint32_t frame_offset = 0) noexcept {
        if (velocity == 0) {
            note_off(note, frame_offset);
            return;
        }

        // Add to held keys
        bool found = false;
        for (size_t i = 0; i < m_num_held; ++i) {
            if (m_held_keys[i].note == note) {
                m_held_keys[i].velocity = velocity;
                m_held_keys[i].timestamp = ++m_key_timestamp_counter;
                found = true;
                break;
            }
        }
        if (!found && m_num_held < kMaxHeldNotes) {
            m_held_keys[m_num_held++] = HeldKey{
                .note = note,
                .velocity = velocity,
                .timestamp = ++m_key_timestamp_counter
            };
        }

        // Start phrase playback if this is the first held key
        if (!m_is_playing) {
            m_is_playing = true;
            m_current_line = 0.0;
            m_line_sample_accumulator = 0.0;
            m_arp_note_index = 0;
            m_arp_direction_up = true;
            m_line_triggered = false;
        }
        m_root_key = note;
        m_root_velocity = velocity;
    }

    void note_off(uint8_t note, uint32_t /*frame_offset*/ = 0) noexcept {
        // Remove from held keys
        for (size_t i = 0; i < m_num_held; ++i) {
            if (m_held_keys[i].note == note) {
                for (size_t j = i; j + 1 < m_num_held; ++j) {
                    m_held_keys[j] = m_held_keys[j + 1];
                }
                --m_num_held;
                break;
            }
        }

        if (m_num_held == 0) {
            m_is_playing = false;
            m_current_line = 0.0;
            m_line_sample_accumulator = 0.0;
        } else {
            // Update root key to most recently pressed held note
            m_root_key = m_held_keys[m_num_held - 1].note;
            m_root_velocity = m_held_keys[m_num_held - 1].velocity;
        }
    }

    void all_notes_off() noexcept {
        m_num_held = 0;
        m_is_playing = false;
        m_current_line = 0.0;
        m_line_sample_accumulator = 0.0;
        for (auto& noff : m_scheduled_note_offs) {
            noff.active = false;
        }
    }

    // ------------------------------------------------------------------------
    // Real-Time Block Processing: Emits PhraseOutputEvents
    // ------------------------------------------------------------------------
    void process_block(uint32_t frames, const TrackerTiming& timing, double sample_rate,
                       PhraseOutputEvent* out_events, size_t max_events, size_t& num_events) noexcept {
        num_events = 0;
        if (!out_events || max_events == 0 || frames == 0) return;

        // Process scheduled Note-Offs for this block
        const uint64_t block_start_sample = m_total_samples_processed;
        const uint64_t block_end_sample = block_start_sample + frames;

        for (auto& noff : m_scheduled_note_offs) {
            if (noff.active && noff.sample_time >= block_start_sample && noff.sample_time < block_end_sample) {
                if (num_events < max_events) {
                    uint32_t offset = static_cast<uint32_t>(noff.sample_time - block_start_sample);
                    out_events[num_events++] = PhraseOutputEvent{
                        .frame_offset = std::min(offset, frames - 1),
                        .is_note_on = false,
                        .note = noff.note,
                        .velocity = 0.0f,
                        .pan = 0.0f,
                        .slice_id = noff.slice_id,
                        .is_slice = noff.is_slice,
                        .cutoff_offset = 0.0f,
                        .res_offset = 0.0f,
                        .duration_samples = 0
                    };
                }
                noff.active = false;
            }
        }

        if (!m_is_playing || !m_phrase || m_phrase->lines.empty() || m_num_held == 0) {
            m_total_samples_processed += frames;
            return;
        }

        // Effective LPB and TPL (phrase override or host timing)
        const uint16_t eff_lpb = (m_phrase->lpb > 0) ? m_phrase->lpb : timing.lpb;
        const uint16_t eff_tpl = (m_phrase->tpl > 0) ? m_phrase->tpl : timing.tpl;

        TrackerTiming phrase_timing = timing;
        phrase_timing.lpb = eff_lpb;
        phrase_timing.tpl = eff_tpl;

        const double samples_per_line = phrase_timing.samples_per_line(sample_rate);

        if (samples_per_line <= 1.0) {
            m_total_samples_processed += frames;
            return;
        }

        const uint32_t phrase_len = std::min<uint32_t>(m_phrase->num_lines, static_cast<uint32_t>(m_phrase->lines.size()));
        if (phrase_len == 0) {
            m_total_samples_processed += frames;
            return;
        }

        // Advance sample by sample across the audio block
        for (uint32_t frame = 0; frame < frames; ++frame) {
            const uint64_t current_abs_sample = m_total_samples_processed + frame;

            // Check if we hit a line boundary
            if (!m_line_triggered) {
                uint32_t line_idx = static_cast<uint32_t>(m_current_line) % phrase_len;
                trigger_line(line_idx, frame, phrase_timing, sample_rate, current_abs_sample,
                             out_events, max_events, num_events, frames);
                m_line_triggered = true;
            }

            m_line_sample_accumulator += 1.0;
            if (m_line_sample_accumulator >= samples_per_line) {
                m_line_sample_accumulator -= samples_per_line;
                m_current_line += 1.0;
                m_line_triggered = false;

                // Loop handling
                if (m_phrase->loop) {
                    const uint32_t l_end = (m_phrase->loop_end > m_phrase->loop_start && m_phrase->loop_end <= phrase_len)
                                           ? m_phrase->loop_end : phrase_len;
                    if (m_current_line >= static_cast<double>(l_end)) {
                        m_current_line = static_cast<double>(m_phrase->loop_start);
                    }
                } else {
                    if (m_current_line >= static_cast<double>(phrase_len)) {
                        m_is_playing = false;
                        break;
                    }
                }
            }
        }

        m_total_samples_processed += frames;
    }

private:
    const InstrumentPhrase* m_phrase{nullptr};
    bool m_is_playing{false};
    double m_current_line{0.0};
    double m_line_sample_accumulator{0.0};
    bool m_line_triggered{false};

    uint8_t m_root_key{60};
    uint8_t m_root_velocity{100};

    std::array<HeldKey, kMaxHeldNotes> m_held_keys{};
    size_t m_num_held{0};
    uint64_t m_key_timestamp_counter{0};

    std::array<ScheduledNoteOff, kMaxActiveVoices> m_scheduled_note_offs{};
    uint64_t m_total_samples_processed{0};

    size_t m_arp_note_index{0};
    bool m_arp_direction_up{true};
    uint32_t m_prng_state{0x12345678};

    uint32_t next_random() noexcept {
        m_prng_state = m_prng_state * 1664525u + 1013904223u;
        return m_prng_state;
    }

    void trigger_line(uint32_t line_idx, uint32_t frame_offset, const TrackerTiming& phrase_timing,
                      double sample_rate, uint64_t current_abs_sample,
                      PhraseOutputEvent* out_events, size_t max_events, size_t& num_events,
                      uint32_t frames) noexcept {
        if (!m_phrase || line_idx >= m_phrase->lines.size()) return;
        const auto& line = m_phrase->lines[line_idx];

        const double samples_per_tick = phrase_timing.samples_per_tick(sample_rate);
        const double samples_per_line = phrase_timing.samples_per_line(sample_rate);

        // Gather sorted held note pitches for arpeggiator modes
        const size_t held_count = std::min(m_num_held, kMaxHeldNotes);
        std::array<uint8_t, kMaxHeldNotes> sorted_notes{};
        for (size_t i = 0; i < held_count; ++i) {
            sorted_notes[i] = m_held_keys[i].note;
        }
        if (held_count > 1) {
            for (size_t i = 1; i < held_count; ++i) {
                uint8_t key = sorted_notes[i];
                size_t j = i;
                while (j > 0 && sorted_notes[j - 1] > key) {
                    sorted_notes[j] = sorted_notes[j - 1];
                    --j;
                }
                sorted_notes[j] = key;
            }
        }

        for (const auto& note_col : line.columns) {
            if (!note_col.active) continue;

            // Probability check
            if (note_col.probability < 100) {
                uint32_t r = next_random() % 100;
                if (r >= note_col.probability) continue;
            }

            // Micro-timing delay in frames
            uint32_t delay_samples = phrase_timing.delay_ticks_to_samples(note_col.delay_ticks, sample_rate);
            uint32_t trig_frame = frame_offset + delay_samples;

            // Gate duration in samples
            uint32_t gate_samples = static_cast<uint32_t>(std::round(static_cast<double>(note_col.gate_ticks) * samples_per_tick));
            gate_samples = std::max(1u, gate_samples);

            // Compute effective note pitch based on PhraseArpMode
            if (note_col.is_slice) {
                // Sliced drum kit trigger
                emit_single_event(trig_frame, true, 0, note_col.slice_id, true,
                                  note_col.velocity, note_col.pan, note_col.cutoff_offset,
                                  note_col.res_offset, gate_samples, current_abs_sample + delay_samples,
                                  out_events, max_events, num_events, frames);

                // Ratchet / retrigger handling for drum rolls / fills
                if (note_col.retrigger_ticks > 0) {
                    uint32_t retrigger_step_samples = static_cast<uint32_t>(std::round(static_cast<double>(note_col.retrigger_ticks) * samples_per_tick));
                    if (retrigger_step_samples > 0) {
                        uint32_t current_ratchet_offset = trig_frame + retrigger_step_samples;
                        while (current_ratchet_offset < frame_offset + static_cast<uint32_t>(samples_per_line) &&
                               current_ratchet_offset < frames + delay_samples) {
                            emit_single_event(current_ratchet_offset, true, 0, note_col.slice_id, true,
                                              note_col.velocity, note_col.pan, note_col.cutoff_offset,
                                              note_col.res_offset, gate_samples,
                                              current_abs_sample + (current_ratchet_offset - frame_offset),
                                              out_events, max_events, num_events, frames);
                            current_ratchet_offset += retrigger_step_samples;
                        }
                    }
                }
            } else {
                // Melodic note trigger
                switch (m_phrase->playback_mode) {
                    case PhraseArpMode::Transposed: {
                        int32_t semitones = static_cast<int32_t>(m_root_key) - static_cast<int32_t>(m_phrase->base_note);
                        int32_t final_note = std::clamp<int32_t>(static_cast<int32_t>(m_phrase->base_note) + note_col.note_offset + semitones, 0, 127);
                        emit_single_event(trig_frame, true, static_cast<uint8_t>(final_note), 0, false,
                                          note_col.velocity, note_col.pan, note_col.cutoff_offset,
                                          note_col.res_offset, gate_samples, current_abs_sample + delay_samples,
                                          out_events, max_events, num_events, frames);
                        break;
                    }
                    case PhraseArpMode::Direct: {
                        int32_t final_note = std::clamp<int32_t>(note_col.note_offset, 0, 127);
                        emit_single_event(trig_frame, true, static_cast<uint8_t>(final_note), 0, false,
                                          note_col.velocity, note_col.pan, note_col.cutoff_offset,
                                          note_col.res_offset, gate_samples, current_abs_sample + delay_samples,
                                          out_events, max_events, num_events, frames);
                        break;
                    }
                    case PhraseArpMode::Keymap: {
                        // Keymap mode directly plays transposed note relative to root
                        int32_t final_note = std::clamp<int32_t>(static_cast<int32_t>(m_root_key) + note_col.note_offset, 0, 127);
                        emit_single_event(trig_frame, true, static_cast<uint8_t>(final_note), 0, false,
                                          note_col.velocity, note_col.pan, note_col.cutoff_offset,
                                          note_col.res_offset, gate_samples, current_abs_sample + delay_samples,
                                          out_events, max_events, num_events, frames);
                        break;
                    }
                    case PhraseArpMode::ArpUp: {
                        if (held_count > 0) {
                            uint8_t arp_note = sorted_notes[m_arp_note_index % held_count];
                            m_arp_note_index = (m_arp_note_index + 1) % held_count;
                            int32_t final_note = std::clamp<int32_t>(static_cast<int32_t>(arp_note) + note_col.note_offset, 0, 127);
                            emit_single_event(trig_frame, true, static_cast<uint8_t>(final_note), 0, false,
                                              note_col.velocity, note_col.pan, note_col.cutoff_offset,
                                              note_col.res_offset, gate_samples, current_abs_sample + delay_samples,
                                              out_events, max_events, num_events, frames);
                        }
                        break;
                    }
                    case PhraseArpMode::ArpDown: {
                        if (held_count > 0) {
                            size_t rev_idx = (held_count - 1) - (m_arp_note_index % held_count);
                            uint8_t arp_note = sorted_notes[rev_idx];
                            m_arp_note_index = (m_arp_note_index + 1) % held_count;
                            int32_t final_note = std::clamp<int32_t>(static_cast<int32_t>(arp_note) + note_col.note_offset, 0, 127);
                            emit_single_event(trig_frame, true, static_cast<uint8_t>(final_note), 0, false,
                                              note_col.velocity, note_col.pan, note_col.cutoff_offset,
                                              note_col.res_offset, gate_samples, current_abs_sample + delay_samples,
                                              out_events, max_events, num_events, frames);
                        }
                        break;
                    }
                    case PhraseArpMode::ArpUpDown: {
                        if (held_count > 1) {
                            uint8_t arp_note = sorted_notes[m_arp_note_index % held_count];
                            if (m_arp_direction_up) {
                                if (m_arp_note_index + 1 >= held_count) {
                                    m_arp_direction_up = false;
                                    m_arp_note_index = (held_count >= 2) ? (held_count - 2) : 0;
                                } else {
                                    ++m_arp_note_index;
                                }
                            } else {
                                if (m_arp_note_index == 0) {
                                    m_arp_direction_up = true;
                                    m_arp_note_index = (held_count > 1) ? 1 : 0;
                                } else {
                                    --m_arp_note_index;
                                }
                            }
                            int32_t final_note = std::clamp<int32_t>(static_cast<int32_t>(arp_note) + note_col.note_offset, 0, 127);
                            emit_single_event(trig_frame, true, static_cast<uint8_t>(final_note), 0, false,
                                              note_col.velocity, note_col.pan, note_col.cutoff_offset,
                                              note_col.res_offset, gate_samples, current_abs_sample + delay_samples,
                                              out_events, max_events, num_events, frames);
                        } else if (held_count == 1) {
                            int32_t final_note = std::clamp<int32_t>(static_cast<int32_t>(sorted_notes[0]) + note_col.note_offset, 0, 127);
                            emit_single_event(trig_frame, true, static_cast<uint8_t>(final_note), 0, false,
                                              note_col.velocity, note_col.pan, note_col.cutoff_offset,
                                              note_col.res_offset, gate_samples, current_abs_sample + delay_samples,
                                              out_events, max_events, num_events, frames);
                        }
                        break;
                    }
                    case PhraseArpMode::ArpRandom: {
                        if (held_count > 0) {
                            size_t rnd_idx = next_random() % held_count;
                            uint8_t arp_note = sorted_notes[rnd_idx];
                            int32_t final_note = std::clamp<int32_t>(static_cast<int32_t>(arp_note) + note_col.note_offset, 0, 127);
                            emit_single_event(trig_frame, true, static_cast<uint8_t>(final_note), 0, false,
                                              note_col.velocity, note_col.pan, note_col.cutoff_offset,
                                              note_col.res_offset, gate_samples, current_abs_sample + delay_samples,
                                              out_events, max_events, num_events, frames);
                        }
                        break;
                    }
                    case PhraseArpMode::ArpChord: {
                        for (size_t c = 0; c < held_count; ++c) {
                            int32_t final_note = std::clamp<int32_t>(static_cast<int32_t>(sorted_notes[c]) + note_col.note_offset, 0, 127);
                            emit_single_event(trig_frame, true, static_cast<uint8_t>(final_note), 0, false,
                                              note_col.velocity, note_col.pan, note_col.cutoff_offset,
                                              note_col.res_offset, gate_samples, current_abs_sample + delay_samples,
                                              out_events, max_events, num_events, frames);
                        }
                        break;
                    }
                }
            }
        }
    }

    void emit_single_event(uint32_t frame_offset, bool is_on, uint8_t note, uint32_t slice_id,
                           bool is_slice, uint8_t velocity, int8_t pan, float cutoff, float res,
                           uint32_t duration_samples, uint64_t abs_trig_sample,
                           PhraseOutputEvent* out_events, size_t max_events, size_t& num_events,
                           uint32_t block_frames) noexcept {
        if (num_events >= max_events) return;

        float norm_vel = static_cast<float>(velocity) / 127.0f;
        float norm_pan = static_cast<float>(pan) / 64.0f;

        out_events[num_events++] = PhraseOutputEvent{
            .frame_offset = std::min(frame_offset, block_frames > 0 ? (block_frames - 1) : 0),
            .is_note_on = is_on,
            .note = note,
            .velocity = norm_vel,
            .pan = std::clamp(norm_pan, -1.0f, 1.0f),
            .slice_id = slice_id,
            .is_slice = is_slice,
            .cutoff_offset = cutoff,
            .res_offset = res,
            .duration_samples = duration_samples
        };

        // Schedule Note-Off
        if (is_on && duration_samples > 0) {
            for (auto& noff : m_scheduled_note_offs) {
                if (!noff.active) {
                    noff = ScheduledNoteOff{
                        .active = true,
                        .note = note,
                        .slice_id = slice_id,
                        .is_slice = is_slice,
                        .sample_time = abs_trig_sample + duration_samples
                    };
                    break;
                }
            }
        }
    }
};

} // namespace audio_core::sequencer
