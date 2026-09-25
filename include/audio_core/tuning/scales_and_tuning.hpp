#pragma once

#include "audio_core/types.hpp"

#include <array>
#include <vector>
#include <string>
#include <string_view>
#include <sstream>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <charconv>

namespace audio_core::tuning {

// ============================================================================
// ScaleType: Musical Scale & Mode Catalog
// ============================================================================
enum class ScaleType : uint8_t {
    Chromatic = 0,
    Major,              // Ionian [0, 2, 4, 5, 7, 9, 11]
    NaturalMinor,       // Aeolian [0, 2, 3, 5, 7, 8, 10]
    HarmonicMinor,      // [0, 2, 3, 5, 7, 8, 11]
    MelodicMinor,       // Jazz Minor [0, 2, 3, 5, 7, 9, 11]
    Dorian,             // [0, 2, 3, 5, 7, 9, 10]
    Phrygian,           // [0, 1, 3, 5, 7, 8, 10]
    Lydian,             // [0, 2, 4, 6, 7, 9, 11]
    Mixolydian,         // [0, 2, 4, 5, 7, 9, 10]
    Locrian,            // [0, 1, 3, 5, 6, 8, 10]
    PentatonicMajor,    // [0, 2, 4, 7, 9]
    PentatonicMinor,    // [0, 3, 5, 7, 10]
    Blues,              // [0, 3, 5, 6, 7, 10]
    ArabicHijaz,        // [0, 1, 4, 5, 7, 8, 10]
    JapaneseInsen,      // [0, 1, 5, 7, 10]
    WholeTone,          // [0, 2, 4, 6, 8, 10]
    DiminishedWholeHalf,// [0, 2, 3, 5, 6, 8, 9, 11]
    DiminishedHalfWhole,// [0, 1, 3, 4, 6, 7, 9, 10]
    HungarianMinor,     // [0, 2, 3, 6, 7, 8, 11]
    Bhairav             // Indian Raga [0, 1, 4, 5, 7, 8, 11]
};

// ============================================================================
// SnapMode: Directional Scale Snapping Strategy
// ============================================================================
enum class SnapMode : uint8_t {
    Nearest = 0,    // Snap to nearest scale pitch (downward if equidistant)
    Floor   = 1,    // Snap down: largest scale pitch <= note
    Ceil    = 2     // Snap up: smallest scale pitch >= note
};

// ============================================================================
// ScaleDictionary: Real-Time Lock-Free Scale Validation and Quantization
// Uses 12-bit bitmasks for instantaneous O(1) membership checks.
// ============================================================================
class ScaleDictionary {
public:
    // Returns 12-bit mask where bit (k) is set if semitone k is in scale
    [[nodiscard]] static constexpr uint16_t scale_mask(ScaleType type) noexcept {
        switch (type) {
            case ScaleType::Chromatic:           return 0b111111111111; // All 12 notes
            case ScaleType::Major:               return (1<<0) | (1<<2) | (1<<4) | (1<<5) | (1<<7) | (1<<9) | (1<<11);
            case ScaleType::NaturalMinor:        return (1<<0) | (1<<2) | (1<<3) | (1<<5) | (1<<7) | (1<<8) | (1<<10);
            case ScaleType::HarmonicMinor:       return (1<<0) | (1<<2) | (1<<3) | (1<<5) | (1<<7) | (1<<8) | (1<<11);
            case ScaleType::MelodicMinor:        return (1<<0) | (1<<2) | (1<<3) | (1<<5) | (1<<7) | (1<<9) | (1<<11);
            case ScaleType::Dorian:              return (1<<0) | (1<<2) | (1<<3) | (1<<5) | (1<<7) | (1<<9) | (1<<10);
            case ScaleType::Phrygian:            return (1<<0) | (1<<1) | (1<<3) | (1<<5) | (1<<7) | (1<<8) | (1<<10);
            case ScaleType::Lydian:              return (1<<0) | (1<<2) | (1<<4) | (1<<6) | (1<<7) | (1<<9) | (1<<11);
            case ScaleType::Mixolydian:          return (1<<0) | (1<<2) | (1<<4) | (1<<5) | (1<<7) | (1<<9) | (1<<10);
            case ScaleType::Locrian:             return (1<<0) | (1<<1) | (1<<3) | (1<<5) | (1<<6) | (1<<8) | (1<<10);
            case ScaleType::PentatonicMajor:     return (1<<0) | (1<<2) | (1<<4) | (1<<7) | (1<<9);
            case ScaleType::PentatonicMinor:     return (1<<0) | (1<<3) | (1<<5) | (1<<7) | (1<<10);
            case ScaleType::Blues:               return (1<<0) | (1<<3) | (1<<5) | (1<<6) | (1<<7) | (1<<10);
            case ScaleType::ArabicHijaz:         return (1<<0) | (1<<1) | (1<<4) | (1<<5) | (1<<7) | (1<<8) | (1<<10);
            case ScaleType::JapaneseInsen:       return (1<<0) | (1<<1) | (1<<5) | (1<<7) | (1<<10);
            case ScaleType::WholeTone:           return (1<<0) | (1<<2) | (1<<4) | (1<<6) | (1<<8) | (1<<10);
            case ScaleType::DiminishedWholeHalf: return (1<<0) | (1<<2) | (1<<3) | (1<<5) | (1<<6) | (1<<8) | (1<<9) | (1<<11);
            case ScaleType::DiminishedHalfWhole: return (1<<0) | (1<<1) | (1<<3) | (1<<4) | (1<<6) | (1<<7) | (1<<9) | (1<<10);
            case ScaleType::HungarianMinor:      return (1<<0) | (1<<2) | (1<<3) | (1<<6) | (1<<7) | (1<<8) | (1<<11);
            case ScaleType::Bhairav:             return (1<<0) | (1<<1) | (1<<4) | (1<<5) | (1<<7) | (1<<8) | (1<<11);
        }
        return 0b111111111111;
    }

    [[nodiscard]] static const char* scale_name(ScaleType type) noexcept {
        switch (type) {
            case ScaleType::Chromatic:           return "Chromatic";
            case ScaleType::Major:               return "Major (Ionian)";
            case ScaleType::NaturalMinor:        return "Natural Minor (Aeolian)";
            case ScaleType::HarmonicMinor:       return "Harmonic Minor";
            case ScaleType::MelodicMinor:        return "Melodic Minor";
            case ScaleType::Dorian:              return "Dorian";
            case ScaleType::Phrygian:            return "Phrygian";
            case ScaleType::Lydian:              return "Lydian";
            case ScaleType::Mixolydian:          return "Mixolydian";
            case ScaleType::Locrian:             return "Locrian";
            case ScaleType::PentatonicMajor:     return "Pentatonic Major";
            case ScaleType::PentatonicMinor:     return "Pentatonic Minor";
            case ScaleType::Blues:               return "Blues";
            case ScaleType::ArabicHijaz:         return "Arabic Hijaz";
            case ScaleType::JapaneseInsen:       return "Japanese Insen";
            case ScaleType::WholeTone:           return "Whole Tone";
            case ScaleType::DiminishedWholeHalf: return "Diminished (Whole-Half)";
            case ScaleType::DiminishedHalfWhole: return "Diminished (Half-Whole)";
            case ScaleType::HungarianMinor:      return "Hungarian Minor";
            case ScaleType::Bhairav:             return "Bhairav (Indian)";
        }
        return "Unknown";
    }

    [[nodiscard]] static std::vector<uint8_t> scale_intervals(ScaleType type) {
        std::vector<uint8_t> intervals;
        const uint16_t mask = scale_mask(type);
        for (uint8_t i = 0; i < 12; ++i) {
            if ((mask >> i) & 1) {
                intervals.push_back(i);
            }
        }
        return intervals;
    }

    [[nodiscard]] static inline bool is_note_in_scale(uint8_t note, uint8_t root_note, ScaleType scale) noexcept {
        int rel = (static_cast<int>(note) - static_cast<int>(root_note)) % 12;
        if (rel < 0) rel += 12;
        return (scale_mask(scale) >> rel) & 1;
    }

    [[nodiscard]] static inline uint8_t snap_to_scale(uint8_t note, uint8_t root_note, ScaleType scale,
                                                      SnapMode mode = SnapMode::Nearest) noexcept {
        if (scale == ScaleType::Chromatic || is_note_in_scale(note, root_note, scale)) {
            return note;
        }

        const uint16_t mask = scale_mask(scale);
        int rel = (static_cast<int>(note) - static_cast<int>(root_note)) % 12;
        if (rel < 0) rel += 12;

        int floor_dist = -1;
        for (int d = 1; d <= 11; ++d) {
            int cand = (rel - d + 12) % 12;
            if ((mask >> cand) & 1) {
                floor_dist = d;
                break;
            }
        }

        int ceil_dist = -1;
        for (int d = 1; d <= 11; ++d) {
            int cand = (rel + d) % 12;
            if ((mask >> cand) & 1) {
                ceil_dist = d;
                break;
            }
        }

        if (mode == SnapMode::Floor) {
            return static_cast<uint8_t>(std::clamp(static_cast<int>(note) - floor_dist, 0, 127));
        }
        if (mode == SnapMode::Ceil) {
            return static_cast<uint8_t>(std::clamp(static_cast<int>(note) + ceil_dist, 0, 127));
        }

        // SnapMode::Nearest
        if (floor_dist <= ceil_dist) {
            return static_cast<uint8_t>(std::clamp(static_cast<int>(note) - floor_dist, 0, 127));
        } else {
            return static_cast<uint8_t>(std::clamp(static_cast<int>(note) + ceil_dist, 0, 127));
        }
    }

    [[nodiscard]] static inline int32_t get_degree(uint8_t note, uint8_t root_note, ScaleType scale) noexcept {
        int rel = (static_cast<int>(note) - static_cast<int>(root_note)) % 12;
        if (rel < 0) rel += 12;
        const uint16_t mask = scale_mask(scale);
        if (!((mask >> rel) & 1)) return -1; // Not in scale

        int32_t degree = 0;
        for (int i = 0; i < rel; ++i) {
            if ((mask >> i) & 1) ++degree;
        }
        return degree;
    }

    [[nodiscard]] static inline uint8_t degree_to_note(int32_t degree, uint8_t root_note, ScaleType scale,
                                                       int32_t octave_offset = 0) noexcept {
        const auto intervals = scale_intervals(scale);
        if (intervals.empty()) return root_note;

        const int num_degrees = static_cast<int>(intervals.size());
        int oct = degree / num_degrees;
        int deg = degree % num_degrees;
        if (deg < 0) {
            deg += num_degrees;
            oct -= 1;
        }
        oct += octave_offset;

        int final_pitch = static_cast<int>(root_note) + (oct * 12) + static_cast<int>(intervals[deg]);
        return static_cast<uint8_t>(std::clamp(final_pitch, 0, 127));
    }

    [[nodiscard]] static std::string note_name(uint8_t note, bool flat = false) {
        static constexpr const char* kSharps[] = { "C-", "C#", "D-", "D#", "E-", "F-", "F#", "G-", "G#", "A-", "A#", "B-" };
        static constexpr const char* kFlats[]  = { "C-", "Db", "D-", "Eb", "E-", "F-", "Gb", "G-", "Ab", "A-", "Bb", "B-" };

        int pitch = note % 12;
        int octave = (note / 12) - 1; // MIDI standard: 60 = C-4

        std::string res = flat ? kFlats[pitch] : kSharps[pitch];
        res += std::to_string(octave);
        return res;
    }
};

// ============================================================================
// TuningTable: Microtonal Tuning Engine & Huygens-Fokker Scala (.scl / .kbm)
// Pre-computes all 128 MIDI frequencies for zero-overhead RT synthesis.
// ============================================================================
class TuningTable {
public:
    std::string name{"12-TET Standard Concert A4=440Hz"};
    std::string description{"12-Tone Equal Temperament"};
    std::array<double, 128> frequencies{};
    uint8_t reference_note{69};          // A4 (69)
    double reference_frequency{440.0};   // 440.0 Hz
    double formal_period_ratio{2.0};     // Formal interval ratio (2.0 for octave, 3.0 for Bohlen-Pierce)

    TuningTable() {
        make_12tet(440.0);
    }

    [[nodiscard]] inline double frequency(uint8_t note) const noexcept {
        return frequencies[note & 0x7F];
    }

    [[nodiscard]] inline float pitch_ratio(uint8_t note, uint8_t base_note = 60) const noexcept {
        const double f_base = frequency(base_note);
        if (f_base <= 1e-4) return 1.0f;
        return static_cast<float>(frequency(note) / f_base);
    }

    void set_note_frequency(uint8_t note, double freq) noexcept {
        frequencies[note & 0x7F] = std::max(1.0, freq);
    }

    // ------------------------------------------------------------------------
    // Classical Temperament & Microtonal Presets
    // ------------------------------------------------------------------------
    void make_12tet(double a4_freq = 440.0) noexcept {
        name = "12-TET Standard";
        description = "12-Tone Equal Temperament";
        reference_note = 69;
        reference_frequency = a4_freq;
        formal_period_ratio = 2.0;

        for (int i = 0; i < 128; ++i) {
            frequencies[i] = a4_freq * std::pow(2.0, (static_cast<double>(i) - 69.0) / 12.0);
        }
    }

    void make_just_intonation(uint8_t root_note = 60, double a4_freq = 440.0) noexcept {
        name = "Just Intonation (5-Limit)";
        description = "Pure harmonic ratios derived from 5-limit overtone series";
        reference_note = 69;
        reference_frequency = a4_freq;
        formal_period_ratio = 2.0;

        // Classical 5-limit Just Intonation ratios relative to tonic
        constexpr double ratios[12] = {
            1.0 / 1.0,    // Unison
            16.0 / 15.0,  // Minor second
            9.0 / 8.0,    // Major second
            6.0 / 5.0,    // Minor third
            5.0 / 4.0,    // Major third
            4.0 / 3.0,    // Perfect fourth
            45.0 / 32.0,  // Augmented fourth / Tritone
            3.0 / 2.0,    // Perfect fifth
            8.0 / 5.0,    // Minor sixth
            5.0 / 3.0,    // Major sixth
            9.0 / 5.0,    // Minor seventh
            15.0 / 8.0    // Major seventh
        };

        // Determine root frequency so that A4 aligns with a4_freq
        int a4_offset = (69 - static_cast<int>(root_note)) % 12;
        if (a4_offset < 0) a4_offset += 12;
        int a4_oct = (69 - static_cast<int>(root_note)) / 12;
        if ((69 - static_cast<int>(root_note)) < 0 && a4_offset != 0) a4_oct -= 1;

        double a4_ratio = ratios[a4_offset] * std::pow(2.0, static_cast<double>(a4_oct));
        double root_freq = a4_freq / a4_ratio;

        for (int i = 0; i < 128; ++i) {
            int rel = (i - static_cast<int>(root_note)) % 12;
            int oct = (i - static_cast<int>(root_note)) / 12;
            if (rel < 0) {
                rel += 12;
                oct -= 1;
            }
            frequencies[i] = root_freq * ratios[rel] * std::pow(2.0, static_cast<double>(oct));
        }
    }

    void make_pythagorean(uint8_t root_note = 60, double a4_freq = 440.0) noexcept {
        name = "Pythagorean Tuning";
        description = "Tuning based on pure 3:2 fifths";
        reference_note = 69;
        reference_frequency = a4_freq;
        formal_period_ratio = 2.0;

        constexpr double ratios[12] = {
            1.0 / 1.0,
            256.0 / 243.0,
            9.0 / 8.0,
            32.0 / 27.0,
            81.0 / 64.0,
            4.0 / 3.0,
            729.0 / 512.0,
            3.0 / 2.0,
            128.0 / 81.0,
            27.0 / 16.0,
            16.0 / 9.0,
            243.0 / 128.0
        };

        int a4_offset = (69 - static_cast<int>(root_note)) % 12;
        if (a4_offset < 0) a4_offset += 12;
        int a4_oct = (69 - static_cast<int>(root_note)) / 12;
        if ((69 - static_cast<int>(root_note)) < 0 && a4_offset != 0) a4_oct -= 1;

        double a4_ratio = ratios[a4_offset] * std::pow(2.0, static_cast<double>(a4_oct));
        double root_freq = a4_freq / a4_ratio;

        for (int i = 0; i < 128; ++i) {
            int rel = (i - static_cast<int>(root_note)) % 12;
            int oct = (i - static_cast<int>(root_note)) / 12;
            if (rel < 0) {
                rel += 12;
                oct -= 1;
            }
            frequencies[i] = root_freq * ratios[rel] * std::pow(2.0, static_cast<double>(oct));
        }
    }

    void make_24edo(double a4_freq = 440.0) noexcept {
        name = "24-EDO Quarter-Tone";
        description = "24 Equal Divisions of the Octave (Quarter-Tone Scale, 50 cents/step)";
        reference_note = 69;
        reference_frequency = a4_freq;
        formal_period_ratio = 2.0;

        // Maps MIDI note keys 0..127 across 24-EDO temperament
        // One semitone on keyboard = 1 quartertone (50 cents), spanning over 5.3 octaves!
        for (int i = 0; i < 128; ++i) {
            frequencies[i] = a4_freq * std::pow(2.0, (static_cast<double>(i) - 69.0) / 24.0);
        }
    }

    void make_werckmeister_iii(uint8_t root_note = 60, double a4_freq = 440.0) noexcept {
        name = "Werckmeister III";
        description = "Baroque Well-Temperament (Andreas Werckmeister 1691)";
        reference_note = 69;
        reference_frequency = a4_freq;
        formal_period_ratio = 2.0;

        // Cents for Werckmeister III
        constexpr double cents[12] = {
            0.0, 90.22, 192.18, 294.13, 390.22, 498.04, 588.27, 696.09, 792.18, 888.27, 996.09, 1092.18
        };

        int a4_offset = (69 - static_cast<int>(root_note)) % 12;
        if (a4_offset < 0) a4_offset += 12;
        int a4_oct = (69 - static_cast<int>(root_note)) / 12;
        if ((69 - static_cast<int>(root_note)) < 0 && a4_offset != 0) a4_oct -= 1;

        double a4_ratio = std::pow(2.0, cents[a4_offset] / 1200.0) * std::pow(2.0, static_cast<double>(a4_oct));
        double root_freq = a4_freq / a4_ratio;

        for (int i = 0; i < 128; ++i) {
            int rel = (i - static_cast<int>(root_note)) % 12;
            int oct = (i - static_cast<int>(root_note)) / 12;
            if (rel < 0) {
                rel += 12;
                oct -= 1;
            }
            frequencies[i] = root_freq * std::pow(2.0, cents[rel] / 1200.0) * std::pow(2.0, static_cast<double>(oct));
        }
    }

    void make_bohlen_pierce(double base_freq = 220.0) noexcept {
        name = "Bohlen-Pierce";
        description = "Alternative non-octave scale dividing the 3:1 tritave into 13 steps";
        reference_note = 60;
        reference_frequency = base_freq;
        formal_period_ratio = 3.0; // 3:1 tritave

        for (int i = 0; i < 128; ++i) {
            frequencies[i] = base_freq * std::pow(3.0, (static_cast<double>(i) - 60.0) / 13.0);
        }
    }

    void make_arabic_rast(uint8_t root_note = 60, double a4_freq = 440.0) noexcept {
        name = "Arabic Maqam Rast";
        description = "Authentic Middle-Eastern quarter-tone Rast scale (Sikah & Rast neutral intervals)";
        reference_note = 69;
        reference_frequency = a4_freq;
        formal_period_ratio = 2.0;

        // Cents for Maqam Rast with neutral 3rd (350 cents) and neutral 7th (1050 cents)
        constexpr double cents[12] = {
            0.0, 100.0, 200.0, 350.0, 400.0, 500.0, 600.0, 700.0, 800.0, 900.0, 1050.0, 1100.0
        };

        int a4_offset = (69 - static_cast<int>(root_note)) % 12;
        if (a4_offset < 0) a4_offset += 12;
        int a4_oct = (69 - static_cast<int>(root_note)) / 12;
        if ((69 - static_cast<int>(root_note)) < 0 && a4_offset != 0) a4_oct -= 1;

        double a4_ratio = std::pow(2.0, cents[a4_offset] / 1200.0) * std::pow(2.0, static_cast<double>(a4_oct));
        double root_freq = a4_freq / a4_ratio;

        for (int i = 0; i < 128; ++i) {
            int rel = (i - static_cast<int>(root_note)) % 12;
            int oct = (i - static_cast<int>(root_note)) / 12;
            if (rel < 0) {
                rel += 12;
                oct -= 1;
            }
            frequencies[i] = root_freq * std::pow(2.0, cents[rel] / 1200.0) * std::pow(2.0, static_cast<double>(oct));
        }
    }

    // ------------------------------------------------------------------------
    // Huygens-Fokker Scala (.scl) Specification Parser
    // ------------------------------------------------------------------------
    bool parse_scl(std::string_view scl_text, uint8_t root_note = 60, double root_freq = 261.625565) {
        std::istringstream stream{std::string(scl_text)};
        std::string line;

        // 1. Read description line (skip initial comment lines)
        std::string desc;
        while (std::getline(stream, line)) {
            line = trim(line);
            if (line.empty() || line.front() == '!') continue;
            desc = line;
            break;
        }
        if (desc.empty()) return false;

        // 2. Read number of notes
        int num_notes = 0;
        while (std::getline(stream, line)) {
            line = trim(line);
            if (line.empty() || line.front() == '!') continue;
            try {
                num_notes = std::stoi(line);
            } catch (...) {
                return false;
            }
            break;
        }
        if (num_notes <= 0 || num_notes > 1200) return false;

        // 3. Read interval definitions
        std::vector<double> ratios;
        ratios.reserve(num_notes);

        while (std::getline(stream, line) && static_cast<int>(ratios.size()) < num_notes) {
            line = trim(line);
            if (line.empty() || line.front() == '!') continue;

            // Remove any trailing comments on the line
            size_t comment_pos = line.find('!');
            if (comment_pos != std::string::npos) {
                line = trim(line.substr(0, comment_pos));
            }

            // Also check for spaces in trailing text
            size_t space_pos = line.find_first_of(" \t");
            std::string token = (space_pos != std::string::npos) ? line.substr(0, space_pos) : line;

            if (token.find('.') != std::string::npos) {
                // Cents value
                try {
                    double cents = std::stod(token);
                    ratios.push_back(std::pow(2.0, cents / 1200.0));
                } catch (...) {
                    return false;
                }
            } else if (token.find('/') != std::string::npos) {
                // Rational ratio: num/den
                size_t slash_pos = token.find('/');
                try {
                    double num = std::stod(token.substr(0, slash_pos));
                    double den = std::stod(token.substr(slash_pos + 1));
                    if (den <= 0.0) return false;
                    ratios.push_back(num / den);
                } catch (...) {
                    return false;
                }
            } else {
                // Integer ratio e.g. "2" for octave
                try {
                    double whole = std::stod(token);
                    ratios.push_back(whole);
                } catch (...) {
                    return false;
                }
            }
        }

        if (static_cast<int>(ratios.size()) != num_notes) return false;

        // Last ratio is the formal period (e.g. 2.0 for octave)
        double period = ratios.back();
        if (period <= 1.0) period = 2.0;

        name = desc;
        description = "Scala imported: " + desc;
        reference_note = root_note;
        reference_frequency = root_freq;
        formal_period_ratio = period;

        // Generate full 128 MIDI note table
        for (int i = 0; i < 128; ++i) {
            int delta = i - static_cast<int>(root_note);
            int oct = delta / num_notes;
            int step = delta % num_notes;
            if (step < 0) {
                step += num_notes;
                oct -= 1;
            }

            double mult = std::pow(period, static_cast<double>(oct));
            double note_ratio = (step == 0) ? 1.0 : ratios[step - 1];
            frequencies[i] = root_freq * note_ratio * mult;
        }

        return true;
    }

    // ------------------------------------------------------------------------
    // Keyboard Mapping (.kbm) Specification Parser
    // ------------------------------------------------------------------------
    bool parse_kbm(std::string_view kbm_text) {
        std::istringstream stream{std::string(kbm_text)};
        std::string line;
        std::vector<std::string> lines;

        while (std::getline(stream, line)) {
            line = trim(line);
            if (line.empty() || line.front() == '!') continue;
            lines.push_back(line);
        }

        if (lines.size() < 7) return false;

        try {
            int map_size = std::stoi(lines[0]);
            int first_note = std::stoi(lines[1]);
            int last_note = std::stoi(lines[2]);
            int middle_note = std::stoi(lines[3]);
            int ref_note = std::stoi(lines[4]);
            double ref_freq = std::stod(lines[5]);
            int formal_oct_degree = std::stoi(lines[6]);

            (void)map_size;
            (void)first_note;
            (void)last_note;
            (void)middle_note;
            (void)formal_oct_degree;

            reference_note = static_cast<uint8_t>(std::clamp(ref_note, 0, 127));
            reference_frequency = std::max(1.0, ref_freq);
            return true;
        } catch (...) {
            return false;
        }
    }

private:
    static std::string trim(std::string_view str) {
        size_t first = str.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) return "";
        size_t last = str.find_last_not_of(" \t\r\n");
        return std::string(str.substr(first, (last - first + 1)));
    }
};

} // namespace audio_core::tuning
