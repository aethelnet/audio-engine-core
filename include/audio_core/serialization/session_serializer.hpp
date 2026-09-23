#pragma once

#include "audio_core/types.hpp"
#include "audio_core/mixer_graph.hpp"
#include "audio_core/clock/timeline_clock.hpp"
#include "audio_core/dsp/processor_factory.hpp"
#include "audio_core/modulation/modulation_matrix.hpp"

#include <string>
#include <string_view>
#include <vector>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <cctype>
#include <cmath>
#include <optional>
#include <unordered_map>
#include <iostream>

namespace audio_core::serialization {

// ============================================================================
// Lightweight Zero-Dependency JSON Tokenizer & Parser
// ============================================================================
namespace json {

enum class Type { Null, Bool, Number, String, Array, Object };

struct Value {
    Type type{Type::Null};
    bool bool_val{false};
    double num_val{0.0};
    std::string str_val{};
    std::vector<Value> arr_val{};
    std::unordered_map<std::string, Value> obj_val{};

    [[nodiscard]] bool is_null() const noexcept { return type == Type::Null; }
    [[nodiscard]] bool is_bool() const noexcept { return type == Type::Bool; }
    [[nodiscard]] bool is_number() const noexcept { return type == Type::Number; }
    [[nodiscard]] bool is_string() const noexcept { return type == Type::String; }
    [[nodiscard]] bool is_array() const noexcept { return type == Type::Array; }
    [[nodiscard]] bool is_object() const noexcept { return type == Type::Object; }

    [[nodiscard]] double as_double(double def = 0.0) const noexcept {
        return is_number() ? num_val : def;
    }
    [[nodiscard]] float as_float(float def = 0.0f) const noexcept {
        return is_number() ? static_cast<float>(num_val) : def;
    }
    [[nodiscard]] int32_t as_int(int32_t def = 0) const noexcept {
        return is_number() ? static_cast<int32_t>(num_val) : def;
    }
    [[nodiscard]] uint32_t as_uint(uint32_t def = 0) const noexcept {
        return is_number() ? static_cast<uint32_t>(num_val) : def;
    }
    [[nodiscard]] bool as_bool(bool def = false) const noexcept {
        return is_bool() ? bool_val : def;
    }
    [[nodiscard]] std::string as_string(const std::string& def = "") const noexcept {
        return is_string() ? str_val : def;
    }

    [[nodiscard]] const Value* get(const std::string& key) const noexcept {
        if (!is_object()) return nullptr;
        auto it = obj_val.find(key);
        return (it != obj_val.end()) ? &it->second : nullptr;
    }

    [[nodiscard]] bool has(const std::string& key) const noexcept {
        return get(key) != nullptr;
    }
};

class Parser {
public:
    static std::optional<Value> parse(std::string_view text) {
        Parser p(text);
        p.skip_whitespace();
        if (p.is_eof()) return std::nullopt;
        auto val = p.parse_value();
        p.skip_whitespace();
        return val;
    }

private:
    explicit Parser(std::string_view text) : m_text(text), m_pos(0) {}

    [[nodiscard]] bool is_eof() const noexcept { return m_pos >= m_text.size(); }
    [[nodiscard]] char peek() const noexcept { return is_eof() ? '\0' : m_text[m_pos]; }
    char get() noexcept { return is_eof() ? '\0' : m_text[m_pos++]; }

    void skip_whitespace() noexcept {
        while (!is_eof()) {
            char c = peek();
            if (std::isspace(static_cast<unsigned char>(c))) {
                m_pos++;
            } else if (c == '/' && m_pos + 1 < m_text.size() && m_text[m_pos + 1] == '/') {
                // Line comment
                while (!is_eof() && peek() != '\n') m_pos++;
            } else {
                break;
            }
        }
    }

    std::optional<Value> parse_value() {
        skip_whitespace();
        if (is_eof()) return std::nullopt;

        char c = peek();
        if (c == '{') return parse_object();
        if (c == '[') return parse_array();
        if (c == '"') return parse_string();
        if (c == 't' || c == 'f') return parse_bool();
        if (c == 'n') return parse_null();
        if (c == '-' || std::isdigit(static_cast<unsigned char>(c))) return parse_number();

        return std::nullopt;
    }

    std::optional<Value> parse_object() {
        if (get() != '{') return std::nullopt;
        Value val;
        val.type = Type::Object;

        while (true) {
            skip_whitespace();
            if (peek() == '}') {
                get();
                return val;
            }

            auto key_opt = parse_string();
            if (!key_opt || !key_opt->is_string()) return std::nullopt;

            skip_whitespace();
            if (get() != ':') return std::nullopt;

            auto field_val = parse_value();
            if (!field_val) return std::nullopt;

            val.obj_val[key_opt->str_val] = std::move(*field_val);

            skip_whitespace();
            if (peek() == ',') {
                get();
            } else if (peek() == '}') {
                get();
                return val;
            } else {
                return std::nullopt;
            }
        }
    }

    std::optional<Value> parse_array() {
        if (get() != '[') return std::nullopt;
        Value val;
        val.type = Type::Array;

        while (true) {
            skip_whitespace();
            if (peek() == ']') {
                get();
                return val;
            }

            auto item = parse_value();
            if (!item) return std::nullopt;
            val.arr_val.push_back(std::move(*item));

            skip_whitespace();
            if (peek() == ',') {
                get();
            } else if (peek() == ']') {
                get();
                return val;
            } else {
                return std::nullopt;
            }
        }
    }

    std::optional<Value> parse_string() {
        if (get() != '"') return std::nullopt;
        std::string s;
        while (!is_eof()) {
            char c = get();
            if (c == '"') {
                Value val;
                val.type = Type::String;
                val.str_val = std::move(s);
                return val;
            }
            if (c == '\\') {
                if (is_eof()) return std::nullopt;
                char esc = get();
                switch (esc) {
                    case '"': s += '"'; break;
                    case '\\': s += '\\'; break;
                    case '/': s += '/'; break;
                    case 'b': s += '\b'; break;
                    case 'f': s += '\f'; break;
                    case 'n': s += '\n'; break;
                    case 'r': s += '\r'; break;
                    case 't': s += '\t'; break;
                    default: s += esc; break;
                }
            } else {
                s += c;
            }
        }
        return std::nullopt;
    }

    std::optional<Value> parse_number() {
        size_t start = m_pos;
        if (peek() == '-') m_pos++;
        while (!is_eof() && (std::isdigit(static_cast<unsigned char>(peek())) || peek() == '.' || peek() == 'e' || peek() == 'E' || peek() == '+' || peek() == '-')) {
            m_pos++;
        }
        std::string num_str(m_text.substr(start, m_pos - start));
        try {
            double d = std::stod(num_str);
            Value val;
            val.type = Type::Number;
            val.num_val = d;
            return val;
        } catch (...) {
            return std::nullopt;
        }
    }

    std::optional<Value> parse_bool() {
        if (m_text.substr(m_pos, 4) == "true") {
            m_pos += 4;
            Value v; v.type = Type::Bool; v.bool_val = true; return v;
        }
        if (m_text.substr(m_pos, 5) == "false") {
            m_pos += 5;
            Value v; v.type = Type::Bool; v.bool_val = false; return v;
        }
        return std::nullopt;
    }

    std::optional<Value> parse_null() {
        if (m_text.substr(m_pos, 4) == "null") {
            m_pos += 4;
            Value v; v.type = Type::Null; return v;
        }
        return std::nullopt;
    }

    std::string_view m_text;
    size_t m_pos{0};
};

} // namespace json

// ============================================================================
// Data Structs for Rack Presets and Full Projects
// ============================================================================

struct SlotPresetData {
    uint32_t slot_idx{0};
    std::string processor_name{};
    bool bypassed{false};
    std::vector<float> parameters{};
};

struct RackPresetData {
    std::string name{"Default Rack"};
    std::string category{"General"};
    std::vector<SlotPresetData> slots{};

    [[nodiscard]] std::string to_json(int indent = 2) const {
        std::ostringstream ss;
        std::string ind(indent, ' ');
        std::string ind2(indent + 2, ' ');
        std::string ind3(indent + 4, ' ');

        ss << "{\n";
        ss << ind2 << "\"name\": \"" << name << "\",\n";
        ss << ind2 << "\"category\": \"" << category << "\",\n";
        ss << ind2 << "\"slots\": [\n";
        for (size_t i = 0; i < slots.size(); ++i) {
            const auto& s = slots[i];
            ss << ind3 << "{\n";
            ss << ind3 << "  \"slot_idx\": " << s.slot_idx << ",\n";
            ss << ind3 << "  \"processor\": \"" << s.processor_name << "\",\n";
            ss << ind3 << "  \"bypassed\": " << (s.bypassed ? "true" : "false") << ",\n";
            ss << ind3 << "  \"parameters\": [";
            for (size_t p = 0; p < s.parameters.size(); ++p) {
                ss << s.parameters[p] << (p + 1 < s.parameters.size() ? ", " : "");
            }
            ss << "]\n";
            ss << ind3 << "}" << (i + 1 < slots.size() ? ",\n" : "\n");
        }
        ss << ind2 << "]\n";
        ss << ind << "}";
        return ss.str();
    }

    static std::optional<RackPresetData> from_json_val(const json::Value& val) {
        if (!val.is_object()) return std::nullopt;
        RackPresetData preset;
        if (auto* n = val.get("name")) preset.name = n->as_string("Default Rack");
        if (auto* c = val.get("category")) preset.category = c->as_string("General");

        if (auto* slots_arr = val.get("slots")) {
            if (slots_arr->is_array()) {
                for (const auto& item : slots_arr->arr_val) {
                    if (!item.is_object()) continue;
                    SlotPresetData sdata;
                    if (auto* sidx = item.get("slot_idx")) sdata.slot_idx = sidx->as_uint();
                    if (auto* proc = item.get("processor")) sdata.processor_name = proc->as_string();
                    if (auto* byp = item.get("bypassed")) sdata.bypassed = byp->as_bool();
                    if (auto* p_arr = item.get("parameters")) {
                        if (p_arr->is_array()) {
                            for (const auto& pv : p_arr->arr_val) {
                                sdata.parameters.push_back(pv.as_float());
                            }
                        }
                    }
                    preset.slots.push_back(std::move(sdata));
                }
            }
        }
        return preset;
    }
};

struct StepTriggerData {
    bool active{false};
    uint32_t slice_id{0};
    float velocity{1.0f};
    float pitch_ratio{1.0f};
    uint8_t probability{100};
    bool reverse{false};
    float pan{0.0f};
    float micro_timing{0.0f};
    float quantize_pct{0.0f};
    uint8_t choke_group{0};
    float filter_cutoff{20000.0f};
    float filter_res{0.707f};
    float decay_ms{0.0f};
    float drive{0.0f};
    float send_a{0.0f};
    float send_b{0.0f};
};

struct AutomationPointData {
    double t{0.0};
    float v{1.0f};
    uint8_t m{0};
    float tau{0.0f};
};

struct SlotAutomationData {
    uint32_t slot{0};
    uint32_t param{0};
    bool enabled{false};
    std::vector<AutomationPointData> points{};
};

struct TrackAutomationData {
    bool gain_enabled{false};
    std::vector<AutomationPointData> gain_points{};
    bool pan_enabled{false};
    std::vector<AutomationPointData> pan_points{};
    bool aux1_enabled{false};
    std::vector<AutomationPointData> aux1_points{};
    bool aux2_enabled{false};
    std::vector<AutomationPointData> aux2_points{};
    std::vector<SlotAutomationData> slot_automation{};
};

struct ClipEnvelopeData {
    bool gain_enabled{false};
    std::vector<AutomationPointData> gain_points{};
    bool pan_enabled{false};
    std::vector<AutomationPointData> pan_points{};
    bool pitch_enabled{false};
    std::vector<AutomationPointData> pitch_points{};
    double length_beats{0.0};
};

struct TrackPresetData {
    uint32_t id{0};
    std::string name{"Track"};
    bool active{true};
    float gain{1.0f};
    float pan{0.0f};
    bool mute{false};
    bool solo{false};
    bool solo_safe{false};
    uint32_t dca_mask{0};
    int32_t target_bus{-1};
    uint8_t input_mode{0};
    float input_gain{1.0f};
    bool input_phase_invert{false};
    RackPresetData rack{};
    TrackAutomationData automation{};
    ClipEnvelopeData clip_envelopes{};
    std::vector<StepTriggerData> sequencer_steps{};
};

// ============================================================================
// Modulation Matrix & Polyphonic Synthesizer Session Serialization
// ============================================================================

struct ModulationRouteData {
    uint8_t source{0};
    uint8_t destination{0};
    float amount{0.0f};
    bool active{false};
    bool bipolar{true};
};

struct LfoData {
    uint8_t waveform{0};
    bool bipolar{true};
    float frequency_hz{1.0f};
    bool beat_sync{false};
    double beats_per_cycle{1.0};
    float depth{1.0f};
};

struct MsegPointData {
    double time{0.0};
    float value{0.0f};
    uint8_t node_mode{0};
    float tension{0.0f};
};

struct MsegData {
    uint8_t time_mode{0};
    uint8_t loop_mode{0};
    int32_t sustain_idx{-1};
    float base_time_scale{1.0f};
    float base_attack_scale{1.0f};
    float base_decay_scale{1.0f};
    float base_level_scale{1.0f};
    float base_tension_offset{0.0f};
    std::vector<MsegPointData> points{};
};

struct PolySynthData {
    size_t polyphony_limit{16};
    uint8_t play_mode{0};
    uint8_t osc1_waveform{0};
    uint8_t osc2_waveform{0};
    float osc_mix{0.5f};
    float osc2_detune_cents{0.0f};
    int osc2_octave_offset{0};
    uint8_t filter_type{0};
    float base_cutoff{2000.0f};
    float resonance_q{0.707f};
    float filter_env_amount{0.0f};
    float keytrack_amount{0.0f};
    float velocity_to_filter{0.0f};
    float velocity_to_amp{1.0f};
    float voice_pan_spread{0.0f};
    float glide_time_ms{0.0f};
    float master_level{1.0f};
};

struct ModulationMatrixData {
    std::vector<ModulationRouteData> routes{};
    LfoData lfo1{};
    LfoData lfo2{};
    MsegData mseg1{};
    MsegData mseg2{};
    PolySynthData poly_synth{};
    bool has_data{false};

    [[nodiscard]] std::string to_json(int indent = 2) const {
        std::ostringstream ss;
        std::string ind(indent, ' ');
        std::string ind2(indent + 2, ' ');
        std::string ind3(indent + 4, ' ');

        ss << "{\n";
        // Routes
        ss << ind2 << "\"routes\": [\n";
        for (size_t i = 0; i < routes.size(); ++i) {
            const auto& r = routes[i];
            ss << ind3 << "{\"src\": " << static_cast<int>(r.source)
               << ", \"dst\": " << static_cast<int>(r.destination)
               << ", \"amount\": " << r.amount
               << ", \"active\": " << (r.active ? "true" : "false")
               << ", \"bipolar\": " << (r.bipolar ? "true" : "false")
               << "}" << (i + 1 < routes.size() ? ",\n" : "\n");
        }
        ss << ind2 << "],\n";

        // LFO1
        ss << ind2 << "\"lfo1\": {\n";
        ss << ind3 << "\"waveform\": " << static_cast<int>(lfo1.waveform) << ",\n";
        ss << ind3 << "\"bipolar\": " << (lfo1.bipolar ? "true" : "false") << ",\n";
        ss << ind3 << "\"freq_hz\": " << lfo1.frequency_hz << ",\n";
        ss << ind3 << "\"beat_sync\": " << (lfo1.beat_sync ? "true" : "false") << ",\n";
        ss << ind3 << "\"beats_per_cycle\": " << lfo1.beats_per_cycle << ",\n";
        ss << ind3 << "\"depth\": " << lfo1.depth << "\n";
        ss << ind2 << "},\n";

        // LFO2
        ss << ind2 << "\"lfo2\": {\n";
        ss << ind3 << "\"waveform\": " << static_cast<int>(lfo2.waveform) << ",\n";
        ss << ind3 << "\"bipolar\": " << (lfo2.bipolar ? "true" : "false") << ",\n";
        ss << ind3 << "\"freq_hz\": " << lfo2.frequency_hz << ",\n";
        ss << ind3 << "\"beat_sync\": " << (lfo2.beat_sync ? "true" : "false") << ",\n";
        ss << ind3 << "\"beats_per_cycle\": " << lfo2.beats_per_cycle << ",\n";
        ss << ind3 << "\"depth\": " << lfo2.depth << "\n";
        ss << ind2 << "},\n";

        // Helper lambda for MSEG JSON
        auto dump_mseg = [&](const std::string& name, const MsegData& m, bool has_comma) {
            ss << ind2 << "\"" << name << "\": {\n";
            ss << ind3 << "\"time_mode\": " << static_cast<int>(m.time_mode) << ",\n";
            ss << ind3 << "\"loop_mode\": " << static_cast<int>(m.loop_mode) << ",\n";
            ss << ind3 << "\"sustain_idx\": " << m.sustain_idx << ",\n";
            ss << ind3 << "\"base_time_scale\": " << m.base_time_scale << ",\n";
            ss << ind3 << "\"base_attack_scale\": " << m.base_attack_scale << ",\n";
            ss << ind3 << "\"base_decay_scale\": " << m.base_decay_scale << ",\n";
            ss << ind3 << "\"base_level_scale\": " << m.base_level_scale << ",\n";
            ss << ind3 << "\"base_tension_offset\": " << m.base_tension_offset << ",\n";
            ss << ind3 << "\"points\": [";
            for (size_t p = 0; p < m.points.size(); ++p) {
                const auto& pt = m.points[p];
                ss << "{\"t\":" << pt.time << ",\"v\":" << pt.value
                   << ",\"m\":" << static_cast<int>(pt.node_mode) << ",\"tau\":" << pt.tension << "}"
                   << (p + 1 < m.points.size() ? "," : "");
            }
            ss << "]\n";
            ss << ind2 << "}" << (has_comma ? ",\n" : "\n");
        };

        dump_mseg("mseg1", mseg1, true);
        dump_mseg("mseg2", mseg2, true);

        // PolySynth
        ss << ind2 << "\"poly_synth\": {\n";
        ss << ind3 << "\"polyphony_limit\": " << poly_synth.polyphony_limit << ",\n";
        ss << ind3 << "\"play_mode\": " << static_cast<int>(poly_synth.play_mode) << ",\n";
        ss << ind3 << "\"osc1_waveform\": " << static_cast<int>(poly_synth.osc1_waveform) << ",\n";
        ss << ind3 << "\"osc2_waveform\": " << static_cast<int>(poly_synth.osc2_waveform) << ",\n";
        ss << ind3 << "\"osc_mix\": " << poly_synth.osc_mix << ",\n";
        ss << ind3 << "\"osc2_detune_cents\": " << poly_synth.osc2_detune_cents << ",\n";
        ss << ind3 << "\"osc2_octave_offset\": " << poly_synth.osc2_octave_offset << ",\n";
        ss << ind3 << "\"filter_type\": " << static_cast<int>(poly_synth.filter_type) << ",\n";
        ss << ind3 << "\"base_cutoff\": " << poly_synth.base_cutoff << ",\n";
        ss << ind3 << "\"resonance_q\": " << poly_synth.resonance_q << ",\n";
        ss << ind3 << "\"filter_env_amount\": " << poly_synth.filter_env_amount << ",\n";
        ss << ind3 << "\"keytrack_amount\": " << poly_synth.keytrack_amount << ",\n";
        ss << ind3 << "\"velocity_to_filter\": " << poly_synth.velocity_to_filter << ",\n";
        ss << ind3 << "\"velocity_to_amp\": " << poly_synth.velocity_to_amp << ",\n";
        ss << ind3 << "\"voice_pan_spread\": " << poly_synth.voice_pan_spread << ",\n";
        ss << ind3 << "\"glide_time_ms\": " << poly_synth.glide_time_ms << ",\n";
        ss << ind3 << "\"master_level\": " << poly_synth.master_level << "\n";
        ss << ind2 << "}\n";

        ss << ind << "}";
        return ss.str();
    }

    static std::optional<ModulationMatrixData> from_json_val(const json::Value& val) {
        if (!val.is_object()) return std::nullopt;
        ModulationMatrixData mdata;
        mdata.has_data = true;

        if (auto* r_arr = val.get("routes")) {
            if (r_arr->is_array()) {
                for (const auto& item : r_arr->arr_val) {
                    if (!item.is_object()) continue;
                    ModulationRouteData r;
                    if (auto* s = item.get("src")) r.source = static_cast<uint8_t>(s->as_uint(0));
                    if (auto* d = item.get("dst")) r.destination = static_cast<uint8_t>(d->as_uint(0));
                    if (auto* a = item.get("amount")) r.amount = a->as_float(0.0f);
                    if (auto* act = item.get("active")) r.active = act->as_bool(false);
                    if (auto* bp = item.get("bipolar")) r.bipolar = bp->as_bool(true);
                    mdata.routes.push_back(r);
                }
            }
        }

        auto parse_lfo = [](const json::Value* obj, LfoData& out) {
            if (!obj || !obj->is_object()) return;
            if (auto* w = obj->get("waveform")) out.waveform = static_cast<uint8_t>(w->as_uint(0));
            if (auto* b = obj->get("bipolar")) out.bipolar = b->as_bool(true);
            if (auto* f = obj->get("freq_hz")) out.frequency_hz = f->as_float(1.0f);
            if (auto* bs = obj->get("beat_sync")) out.beat_sync = bs->as_bool(false);
            if (auto* bpc = obj->get("beats_per_cycle")) out.beats_per_cycle = bpc->as_double(1.0);
            if (auto* d = obj->get("depth")) out.depth = d->as_float(1.0f);
        };

        parse_lfo(val.get("lfo1"), mdata.lfo1);
        parse_lfo(val.get("lfo2"), mdata.lfo2);

        auto parse_mseg = [](const json::Value* obj, MsegData& out) {
            if (!obj || !obj->is_object()) return;
            if (auto* tm = obj->get("time_mode")) out.time_mode = static_cast<uint8_t>(tm->as_uint(0));
            if (auto* lm = obj->get("loop_mode")) out.loop_mode = static_cast<uint8_t>(lm->as_uint(0));
            if (auto* si = obj->get("sustain_idx")) out.sustain_idx = si->as_int(-1);
            if (auto* bts = obj->get("base_time_scale")) out.base_time_scale = bts->as_float(1.0f);
            if (auto* bas = obj->get("base_attack_scale")) out.base_attack_scale = bas->as_float(1.0f);
            if (auto* bds = obj->get("base_decay_scale")) out.base_decay_scale = bds->as_float(1.0f);
            if (auto* bls = obj->get("base_level_scale")) out.base_level_scale = bls->as_float(1.0f);
            if (auto* bto = obj->get("base_tension_offset")) out.base_tension_offset = bto->as_float(0.0f);

            if (auto* pa = obj->get("points")) {
                if (pa->is_array()) {
                    for (const auto& pv : pa->arr_val) {
                        if (!pv.is_object()) continue;
                        MsegPointData ptd;
                        if (auto* t = pv.get("t")) ptd.time = t->as_double(0.0);
                        if (auto* v = pv.get("v")) ptd.value = v->as_float(0.0f);
                        if (auto* m = pv.get("m")) ptd.node_mode = static_cast<uint8_t>(m->as_uint(0));
                        if (auto* tau = pv.get("tau")) ptd.tension = tau->as_float(0.0f);
                        out.points.push_back(ptd);
                    }
                }
            }
        };

        parse_mseg(val.get("mseg1"), mdata.mseg1);
        parse_mseg(val.get("mseg2"), mdata.mseg2);

        if (auto* ps = val.get("poly_synth")) {
            if (ps->is_object()) {
                if (auto* pl = ps->get("polyphony_limit")) mdata.poly_synth.polyphony_limit = pl->as_uint(16);
                if (auto* pm = ps->get("play_mode")) mdata.poly_synth.play_mode = static_cast<uint8_t>(pm->as_uint(0));
                if (auto* o1 = ps->get("osc1_waveform")) mdata.poly_synth.osc1_waveform = static_cast<uint8_t>(o1->as_uint(0));
                if (auto* o2 = ps->get("osc2_waveform")) mdata.poly_synth.osc2_waveform = static_cast<uint8_t>(o2->as_uint(0));
                if (auto* om = ps->get("osc_mix")) mdata.poly_synth.osc_mix = om->as_float(0.5f);
                if (auto* od = ps->get("osc2_detune_cents")) mdata.poly_synth.osc2_detune_cents = od->as_float(0.0f);
                if (auto* oo = ps->get("osc2_octave_offset")) mdata.poly_synth.osc2_octave_offset = oo->as_int(0);
                if (auto* ft = ps->get("filter_type")) mdata.poly_synth.filter_type = static_cast<uint8_t>(ft->as_uint(0));
                if (auto* bc = ps->get("base_cutoff")) mdata.poly_synth.base_cutoff = bc->as_float(2000.0f);
                if (auto* rq = ps->get("resonance_q")) mdata.poly_synth.resonance_q = rq->as_float(0.707f);
                if (auto* fea = ps->get("filter_env_amount")) mdata.poly_synth.filter_env_amount = fea->as_float(0.0f);
                if (auto* kta = ps->get("keytrack_amount")) mdata.poly_synth.keytrack_amount = kta->as_float(0.0f);
                if (auto* vtf = ps->get("velocity_to_filter")) mdata.poly_synth.velocity_to_filter = vtf->as_float(0.0f);
                if (auto* vta = ps->get("velocity_to_amp")) mdata.poly_synth.velocity_to_amp = vta->as_float(1.0f);
                if (auto* vps = ps->get("voice_pan_spread")) mdata.poly_synth.voice_pan_spread = vps->as_float(0.0f);
                if (auto* gt = ps->get("glide_time_ms")) mdata.poly_synth.glide_time_ms = gt->as_float(0.0f);
                if (auto* ml = ps->get("master_level")) mdata.poly_synth.master_level = ml->as_float(1.0f);
            }
        }

        return mdata;
    }
};

struct ProjectSessionData {
    std::string project_name{"Untitled Project"};
    uint32_t sample_rate{48000};
    double bpm{120.0};
    float master_volume{1.0f};
    bool master_limiter_enabled{true};
    RackPresetData master_rack{};
    std::vector<TrackPresetData> tracks{};
    std::optional<ModulationMatrixData> modulation_matrix{std::nullopt};

    [[nodiscard]] std::string to_json() const {
        std::ostringstream ss;
        ss << "{\n";
        ss << "  \"project_name\": \"" << project_name << "\",\n";
        ss << "  \"sample_rate\": " << sample_rate << ",\n";
        ss << "  \"bpm\": " << bpm << ",\n";
        ss << "  \"master_volume\": " << master_volume << ",\n";
        ss << "  \"master_limiter\": " << (master_limiter_enabled ? "true" : "false") << ",\n";
        ss << "  \"master_rack\": " << master_rack.to_json(4) << ",\n";
        ss << "  \"tracks\": [\n";
        for (size_t t = 0; t < tracks.size(); ++t) {
            const auto& trk = tracks[t];
            ss << "    {\n";
            ss << "      \"id\": " << trk.id << ",\n";
            ss << "      \"name\": \"" << trk.name << "\",\n";
            ss << "      \"active\": " << (trk.active ? "true" : "false") << ",\n";
            ss << "      \"gain\": " << trk.gain << ",\n";
            ss << "      \"pan\": " << trk.pan << ",\n";
            ss << "      \"mute\": " << (trk.mute ? "true" : "false") << ",\n";
            ss << "      \"solo\": " << (trk.solo ? "true" : "false") << ",\n";
            ss << "      \"solo_safe\": " << (trk.solo_safe ? "true" : "false") << ",\n";
            ss << "      \"dca_mask\": " << trk.dca_mask << ",\n";
            ss << "      \"target_bus\": " << trk.target_bus << ",\n";
            ss << "      \"input_mode\": " << static_cast<int>(trk.input_mode) << ",\n";
            ss << "      \"input_gain\": " << trk.input_gain << ",\n";
            ss << "      \"input_phase_invert\": " << (trk.input_phase_invert ? "true" : "false") << ",\n";
            ss << "      \"rack\": " << trk.rack.to_json(6) << ",\n";
            ss << "      \"automation\": {\n";
            ss << "        \"gain_enabled\": " << (trk.automation.gain_enabled ? "true" : "false") << ",\n";
            ss << "        \"gain_points\": [";
            for (size_t p = 0; p < trk.automation.gain_points.size(); ++p) {
                const auto& pt = trk.automation.gain_points[p];
                ss << "{\"t\":" << pt.t << ",\"v\":" << pt.v << ",\"m\":" << static_cast<int>(pt.m) << ",\"tau\":" << pt.tau << "}"
                   << (p + 1 < trk.automation.gain_points.size() ? "," : "");
            }
            ss << "],\n";
            ss << "        \"pan_enabled\": " << (trk.automation.pan_enabled ? "true" : "false") << ",\n";
            ss << "        \"pan_points\": [";
            for (size_t p = 0; p < trk.automation.pan_points.size(); ++p) {
                const auto& pt = trk.automation.pan_points[p];
                ss << "{\"t\":" << pt.t << ",\"v\":" << pt.v << ",\"m\":" << static_cast<int>(pt.m) << ",\"tau\":" << pt.tau << "}"
                   << (p + 1 < trk.automation.pan_points.size() ? "," : "");
            }
            ss << "],\n";
            ss << "        \"aux1_enabled\": " << (trk.automation.aux1_enabled ? "true" : "false") << ",\n";
            ss << "        \"aux1_points\": [";
            for (size_t p = 0; p < trk.automation.aux1_points.size(); ++p) {
                const auto& pt = trk.automation.aux1_points[p];
                ss << "{\"t\":" << pt.t << ",\"v\":" << pt.v << ",\"m\":" << static_cast<int>(pt.m) << ",\"tau\":" << pt.tau << "}"
                   << (p + 1 < trk.automation.aux1_points.size() ? "," : "");
            }
            ss << "],\n";
            ss << "        \"aux2_enabled\": " << (trk.automation.aux2_enabled ? "true" : "false") << ",\n";
            ss << "        \"aux2_points\": [";
            for (size_t p = 0; p < trk.automation.aux2_points.size(); ++p) {
                const auto& pt = trk.automation.aux2_points[p];
                ss << "{\"t\":" << pt.t << ",\"v\":" << pt.v << ",\"m\":" << static_cast<int>(pt.m) << ",\"tau\":" << pt.tau << "}"
                   << (p + 1 < trk.automation.aux2_points.size() ? "," : "");
            }
            ss << "],\n";
            ss << "        \"slot_automation\": [";
            for (size_t sa = 0; sa < trk.automation.slot_automation.size(); ++sa) {
                const auto& sdata = trk.automation.slot_automation[sa];
                ss << "{\"slot\":" << sdata.slot << ",\"param\":" << sdata.param
                   << ",\"enabled\":" << (sdata.enabled ? "true" : "false") << ",\"points\":[";
                for (size_t p = 0; p < sdata.points.size(); ++p) {
                    const auto& pt = sdata.points[p];
                    ss << "{\"t\":" << pt.t << ",\"v\":" << pt.v << ",\"m\":" << static_cast<int>(pt.m) << ",\"tau\":" << pt.tau << "}"
                       << (p + 1 < sdata.points.size() ? "," : "");
                }
                ss << "]}" << (sa + 1 < trk.automation.slot_automation.size() ? "," : "");
            }
            ss << "]\n";
            ss << "      },\n";
            ss << "      \"clip_envelopes\": {\n";
            ss << "        \"gain_enabled\": " << (trk.clip_envelopes.gain_enabled ? "true" : "false") << ",\n";
            ss << "        \"gain_points\": [";
            for (size_t p = 0; p < trk.clip_envelopes.gain_points.size(); ++p) {
                const auto& pt = trk.clip_envelopes.gain_points[p];
                ss << "{\"t\":" << pt.t << ",\"v\":" << pt.v << ",\"m\":" << static_cast<int>(pt.m) << ",\"tau\":" << pt.tau << "}"
                   << (p + 1 < trk.clip_envelopes.gain_points.size() ? "," : "");
            }
            ss << "],\n";
            ss << "        \"pan_enabled\": " << (trk.clip_envelopes.pan_enabled ? "true" : "false") << ",\n";
            ss << "        \"pan_points\": [";
            for (size_t p = 0; p < trk.clip_envelopes.pan_points.size(); ++p) {
                const auto& pt = trk.clip_envelopes.pan_points[p];
                ss << "{\"t\":" << pt.t << ",\"v\":" << pt.v << ",\"m\":" << static_cast<int>(pt.m) << ",\"tau\":" << pt.tau << "}"
                   << (p + 1 < trk.clip_envelopes.pan_points.size() ? "," : "");
            }
            ss << "],\n";
            ss << "        \"pitch_enabled\": " << (trk.clip_envelopes.pitch_enabled ? "true" : "false") << ",\n";
            ss << "        \"pitch_points\": [";
            for (size_t p = 0; p < trk.clip_envelopes.pitch_points.size(); ++p) {
                const auto& pt = trk.clip_envelopes.pitch_points[p];
                ss << "{\"t\":" << pt.t << ",\"v\":" << pt.v << ",\"m\":" << static_cast<int>(pt.m) << ",\"tau\":" << pt.tau << "}"
                   << (p + 1 < trk.clip_envelopes.pitch_points.size() ? "," : "");
            }
            ss << "],\n";
            ss << "        \"length_beats\": " << trk.clip_envelopes.length_beats << "\n";
            ss << "      },\n";
            ss << "      \"sequencer\": [\n";
            for (size_t s = 0; s < trk.sequencer_steps.size(); ++s) {
                const auto& step = trk.sequencer_steps[s];
                ss << "        { \"active\": " << (step.active ? "true" : "false")
                   << ", \"slice\": " << step.slice_id
                   << ", \"vel\": " << step.velocity
                   << ", \"pitch\": " << step.pitch_ratio
                   << ", \"prob\": " << static_cast<int>(step.probability)
                   << ", \"rev\": " << (step.reverse ? "true" : "false")
                   << ", \"pan\": " << step.pan
                   << ", \"micro\": " << step.micro_timing
                   << ", \"choke\": " << static_cast<int>(step.choke_group)
                   << ", \"cutoff\": " << step.filter_cutoff
                   << ", \"res\": " << step.filter_res
                   << ", \"decay\": " << step.decay_ms
                   << ", \"drive\": " << step.drive
                   << ", \"send_a\": " << step.send_a
                   << ", \"send_b\": " << step.send_b
                   << " }" << (s + 1 < trk.sequencer_steps.size() ? ",\n" : "\n");
            }
            ss << "      ]\n";
            ss << "    }" << (t + 1 < tracks.size() ? ",\n" : "\n");
        }
        ss << "  ]";
        if (modulation_matrix.has_value()) {
            ss << ",\n  \"modulation_matrix\": " << modulation_matrix->to_json(2);
        }
        ss << "\n}";
        return ss.str();
    }

    static std::optional<ProjectSessionData> from_json_val(const json::Value& val) {
        if (!val.is_object()) return std::nullopt;
        ProjectSessionData data;
        if (auto* p = val.get("project_name")) data.project_name = p->as_string("Untitled Project");
        if (auto* sr = val.get("sample_rate")) data.sample_rate = sr->as_uint(48000);
        if (auto* b = val.get("bpm")) data.bpm = b->as_double(120.0);
        if (auto* mv = val.get("master_volume")) data.master_volume = mv->as_float(1.0f);
        if (auto* ml = val.get("master_limiter")) data.master_limiter_enabled = ml->as_bool(true);

        if (auto* mrack = val.get("master_rack")) {
            auto r = RackPresetData::from_json_val(*mrack);
            if (r) data.master_rack = std::move(*r);
        }

        if (auto* trks_arr = val.get("tracks")) {
            if (trks_arr->is_array()) {
                for (const auto& tv : trks_arr->arr_val) {
                    if (!tv.is_object()) continue;
                    TrackPresetData tdata;
                    if (auto* tid = tv.get("id")) tdata.id = tid->as_uint();
                    if (auto* tname = tv.get("name")) tdata.name = tname->as_string("Track");
                    if (auto* tact = tv.get("active")) tdata.active = tact->as_bool(true);
                    if (auto* tgain = tv.get("gain")) tdata.gain = tgain->as_float(1.0f);
                    if (auto* tpan = tv.get("pan")) tdata.pan = tpan->as_float(0.0f);
                    if (auto* tmute = tv.get("mute")) tdata.mute = tmute->as_bool(false);
                    if (auto* tsolo = tv.get("solo")) tdata.solo = tsolo->as_bool(false);
                    if (auto* tsafe = tv.get("solo_safe")) tdata.solo_safe = tsafe->as_bool(false);
                    if (auto* tdca = tv.get("dca_mask")) tdata.dca_mask = tdca->as_uint(0);
                    if (auto* ttgt = tv.get("target_bus")) tdata.target_bus = ttgt->as_int(-1);
                    if (auto* tim = tv.get("input_mode")) tdata.input_mode = static_cast<uint8_t>(tim->as_uint(0));
                    if (auto* tig = tv.get("input_gain")) tdata.input_gain = tig->as_float(1.0f);
                    if (auto* tipi = tv.get("input_phase_invert")) tdata.input_phase_invert = tipi->as_bool(false);

                    if (auto* track_rack = tv.get("rack")) {
                        auto r = RackPresetData::from_json_val(*track_rack);
                        if (r) tdata.rack = std::move(*r);
                    }

                    if (auto* auto_obj = tv.get("automation")) {
                        if (auto_obj->is_object()) {
                            auto parse_lane = [](const json::Value* obj, const std::string& en_key, const std::string& pts_key,
                                                 bool& out_en, std::vector<AutomationPointData>& out_pts) {
                                if (auto* e = obj->get(en_key)) out_en = e->as_bool(false);
                                if (auto* pa = obj->get(pts_key)) {
                                    if (pa->is_array()) {
                                        for (const auto& pv : pa->arr_val) {
                                            if (!pv.is_object()) continue;
                                            AutomationPointData ptd;
                                            if (auto* t = pv.get("t")) ptd.t = t->as_double(0.0);
                                            if (auto* v = pv.get("v")) ptd.v = v->as_float(1.0f);
                                            if (auto* m = pv.get("m")) ptd.m = static_cast<uint8_t>(m->as_uint(0));
                                            if (auto* tau = pv.get("tau")) ptd.tau = tau->as_float(0.0f);
                                            out_pts.push_back(ptd);
                                        }
                                    }
                                }
                            };
                            parse_lane(auto_obj, "gain_enabled", "gain_points", tdata.automation.gain_enabled, tdata.automation.gain_points);
                            parse_lane(auto_obj, "pan_enabled", "pan_points", tdata.automation.pan_enabled, tdata.automation.pan_points);
                            parse_lane(auto_obj, "aux1_enabled", "aux1_points", tdata.automation.aux1_enabled, tdata.automation.aux1_points);
                            parse_lane(auto_obj, "aux2_enabled", "aux2_points", tdata.automation.aux2_enabled, tdata.automation.aux2_points);

                            if (auto* sa_arr = auto_obj->get("slot_automation")) {
                                if (sa_arr->is_array()) {
                                    for (const auto& entry : sa_arr->arr_val) {
                                        if (!entry.is_object()) continue;
                                        SlotAutomationData sdata;
                                        if (auto* s = entry.get("slot")) sdata.slot = s->as_uint(0);
                                        if (auto* p = entry.get("param")) sdata.param = p->as_uint(0);
                                        if (auto* e = entry.get("enabled")) sdata.enabled = e->as_bool(false);
                                        if (auto* pa = entry.get("points")) {
                                            if (pa->is_array()) {
                                                for (const auto& pv : pa->arr_val) {
                                                    if (!pv.is_object()) continue;
                                                    AutomationPointData ptd;
                                                    if (auto* t = pv.get("t")) ptd.t = t->as_double(0.0);
                                                    if (auto* v = pv.get("v")) ptd.v = v->as_float(1.0f);
                                                    if (auto* m = pv.get("m")) ptd.m = static_cast<uint8_t>(m->as_uint(0));
                                                    if (auto* tau = pv.get("tau")) ptd.tau = tau->as_float(0.0f);
                                                    sdata.points.push_back(ptd);
                                                }
                                            }
                                        }
                                        tdata.automation.slot_automation.push_back(std::move(sdata));
                                    }
                                }
                            }
                        }
                    }

                    if (auto* cenv_obj = tv.get("clip_envelopes")) {
                        if (cenv_obj->is_object()) {
                            auto parse_lane = [](const json::Value* parent, const char* en_key, const char* pts_key,
                                                 bool& out_en, std::vector<AutomationPointData>& out_pts) {
                                if (auto* e = parent->get(en_key)) out_en = e->as_bool(false);
                                if (auto* pa = parent->get(pts_key)) {
                                    if (pa->is_array()) {
                                        for (const auto& pv : pa->arr_val) {
                                            if (!pv.is_object()) continue;
                                            AutomationPointData ptd;
                                            if (auto* t = pv.get("t")) ptd.t = t->as_double(0.0);
                                            if (auto* v = pv.get("v")) ptd.v = v->as_float(1.0f);
                                            if (auto* m = pv.get("m")) ptd.m = static_cast<uint8_t>(m->as_uint(0));
                                            if (auto* tau = pv.get("tau")) ptd.tau = tau->as_float(0.0f);
                                            out_pts.push_back(ptd);
                                        }
                                    }
                                }
                            };
                            parse_lane(cenv_obj, "gain_enabled", "gain_points", tdata.clip_envelopes.gain_enabled, tdata.clip_envelopes.gain_points);
                            parse_lane(cenv_obj, "pan_enabled", "pan_points", tdata.clip_envelopes.pan_enabled, tdata.clip_envelopes.pan_points);
                            parse_lane(cenv_obj, "pitch_enabled", "pitch_points", tdata.clip_envelopes.pitch_enabled, tdata.clip_envelopes.pitch_points);
                            if (auto* lb = cenv_obj->get("length_beats")) tdata.clip_envelopes.length_beats = lb->as_double(4.0);
                        }
                    }

                    if (auto* seq_arr = tv.get("sequencer")) {
                        if (seq_arr->is_array()) {
                            for (const auto& sv : seq_arr->arr_val) {
                                if (!sv.is_object()) continue;
                                StepTriggerData st;
                                if (auto* a = sv.get("active")) st.active = a->as_bool();
                                if (auto* s = sv.get("slice")) st.slice_id = s->as_uint();
                                if (auto* v = sv.get("vel")) st.velocity = v->as_float(1.0f);
                                if (auto* p = sv.get("pitch")) st.pitch_ratio = p->as_float(1.0f);
                                if (auto* pb = sv.get("prob")) st.probability = static_cast<uint8_t>(pb->as_uint(100));
                                if (auto* r = sv.get("rev")) st.reverse = r->as_bool(false);
                                if (auto* pn = sv.get("pan")) st.pan = pn->as_float(0.0f);
                                if (auto* m = sv.get("micro")) st.micro_timing = m->as_float(0.0f);
                                if (auto* chk = sv.get("choke")) st.choke_group = static_cast<uint8_t>(chk->as_uint(0));
                                if (auto* c = sv.get("cutoff")) st.filter_cutoff = c->as_float(20000.0f);
                                if (auto* res = sv.get("res")) st.filter_res = res->as_float(0.707f);
                                if (auto* dec = sv.get("decay")) st.decay_ms = dec->as_float(0.0f);
                                if (auto* drv = sv.get("drive")) st.drive = drv->as_float(0.0f);
                                if (auto* sa = sv.get("send_a")) st.send_a = sa->as_float(0.0f);
                                if (auto* sb = sv.get("send_b")) st.send_b = sb->as_float(0.0f);
                                tdata.sequencer_steps.push_back(st);
                            }
                        }
                    }
                    data.tracks.push_back(std::move(tdata));
                }
            }
        }

        if (auto* mm_val = val.get("modulation_matrix")) {
            data.modulation_matrix = ModulationMatrixData::from_json_val(*mm_val);
        }

        return data;
    }
};

// ============================================================================
// SessionSerializer API: Bridges State to/from MixerGraph & Disk
// ============================================================================
class SessionSerializer {
public:
    static RackPresetData extract_rack_preset(const Track& track, std::string name = "Track Rack", std::string category = "User") {
        RackPresetData preset;
        preset.name = std::move(name);
        preset.category = std::move(category);

        for (uint32_t i = 0; i < 4; ++i) {
            const auto& slot = track.slot(i);
            const auto* proc = slot.processor();
            SlotPresetData sdata;
            sdata.slot_idx = i;
            sdata.bypassed = slot.is_bypassed();
            if (proc) {
                sdata.processor_name = proc->name();
                // Extract common parameters
                for (uint32_t p = 0; p < 8; ++p) {
                    sdata.parameters.push_back(proc->get_parameter(p));
                }
            }
            preset.slots.push_back(std::move(sdata));
        }
        return preset;
    }

    static void apply_rack_preset(Track& track, const RackPresetData& preset, uint32_t sample_rate = 48000) {
        for (const auto& sdata : preset.slots) {
            if (sdata.slot_idx >= 4) continue;
            auto& slot = track.slot(sdata.slot_idx);
            slot.set_bypass(sdata.bypassed);

            if (sdata.processor_name.empty()) {
                slot.set_processor(nullptr);
            } else {
                auto proc = create_processor_by_name(sdata.processor_name, sample_rate);
                if (proc) {
                    for (size_t p = 0; p < sdata.parameters.size(); ++p) {
                        proc->set_parameter(static_cast<uint32_t>(p), sdata.parameters[p]);
                    }
                    slot.set_processor(proc);
                }
            }
        }
    }

    static ModulationMatrixData extract_modulation_matrix(const modulation::ModulationMatrix& matrix) {
        ModulationMatrixData mdata;
        mdata.has_data = true;

        for (const auto& r : matrix.routes()) {
            if (r.active || r.source != modulation::ModulationSource::None) {
                mdata.routes.push_back(ModulationRouteData{
                    .source = static_cast<uint8_t>(r.source),
                    .destination = static_cast<uint8_t>(r.destination),
                    .amount = r.amount,
                    .active = r.active,
                    .bipolar = r.bipolar
                });
            }
        }

        const auto& lfo1 = matrix.lfo1();
        mdata.lfo1.waveform = static_cast<uint8_t>(lfo1.waveform());
        mdata.lfo1.bipolar = lfo1.is_bipolar();
        mdata.lfo1.frequency_hz = lfo1.frequency_hz();
        mdata.lfo1.beat_sync = lfo1.is_beat_sync();
        mdata.lfo1.beats_per_cycle = lfo1.beats_per_cycle();
        mdata.lfo1.depth = lfo1.depth();

        const auto& lfo2 = matrix.lfo2();
        mdata.lfo2.waveform = static_cast<uint8_t>(lfo2.waveform());
        mdata.lfo2.bipolar = lfo2.is_bipolar();
        mdata.lfo2.frequency_hz = lfo2.frequency_hz();
        mdata.lfo2.beat_sync = lfo2.is_beat_sync();
        mdata.lfo2.beats_per_cycle = lfo2.beats_per_cycle();
        mdata.lfo2.depth = lfo2.depth();

        auto extract_mseg = [](const modulation::MultiStageEnvelope& mseg, MsegData& out) {
            out.time_mode = static_cast<uint8_t>(mseg.time_mode());
            out.loop_mode = static_cast<uint8_t>(mseg.loop_mode());
            out.sustain_idx = mseg.sustain_index();
            out.base_time_scale = mseg.base_time_scale();
            out.base_attack_scale = mseg.base_attack_scale();
            out.base_decay_scale = mseg.base_decay_scale();
            out.base_level_scale = mseg.base_level_scale();
            out.base_tension_offset = mseg.base_tension_offset();

            for (const auto& pt : mseg.get_points()) {
                out.points.push_back(MsegPointData{
                    .time = pt.time,
                    .value = pt.value,
                    .node_mode = static_cast<uint8_t>(pt.node_mode),
                    .tension = pt.tension
                });
            }
        };

        extract_mseg(matrix.mseg1(), mdata.mseg1);
        extract_mseg(matrix.mseg2(), mdata.mseg2);

        const auto& ps = matrix.poly_synth();
        mdata.poly_synth.polyphony_limit = ps.polyphony_limit();
        mdata.poly_synth.play_mode = static_cast<uint8_t>(ps.play_mode());
        mdata.poly_synth.osc1_waveform = static_cast<uint8_t>(ps.osc1_waveform());
        mdata.poly_synth.osc2_waveform = static_cast<uint8_t>(ps.osc2_waveform());
        mdata.poly_synth.osc_mix = ps.osc_mix();
        mdata.poly_synth.osc2_detune_cents = ps.osc2_detune_cents();
        mdata.poly_synth.osc2_octave_offset = ps.osc2_octave_offset();
        mdata.poly_synth.filter_type = static_cast<uint8_t>(ps.filter_type());
        mdata.poly_synth.base_cutoff = ps.base_cutoff();
        mdata.poly_synth.resonance_q = ps.resonance_q();
        mdata.poly_synth.filter_env_amount = ps.filter_env_amount();
        mdata.poly_synth.keytrack_amount = ps.keytrack_amount();
        mdata.poly_synth.velocity_to_filter = ps.velocity_to_filter();
        mdata.poly_synth.velocity_to_amp = ps.velocity_to_amp();
        mdata.poly_synth.voice_pan_spread = ps.voice_pan_spread();
        mdata.poly_synth.glide_time_ms = ps.glide_time_ms();
        mdata.poly_synth.master_level = ps.master_level();

        return mdata;
    }

    static void apply_modulation_matrix(modulation::ModulationMatrix& matrix, const ModulationMatrixData& data) {
        if (!data.has_data) return;

        for (size_t i = 0; i < modulation::ModulationMatrix::kMaxRoutes; ++i) {
            matrix.clear_route(i);
        }

        for (size_t i = 0; i < data.routes.size() && i < modulation::ModulationMatrix::kMaxRoutes; ++i) {
            const auto& r = data.routes[i];
            matrix.set_route(i,
                             static_cast<modulation::ModulationSource>(r.source),
                             static_cast<modulation::ModulationDestination>(r.destination),
                             r.amount,
                             r.active,
                             r.bipolar);
        }

        auto& lfo1 = matrix.lfo1();
        lfo1.set_waveform(static_cast<modulation::LfoWaveform>(data.lfo1.waveform));
        lfo1.set_bipolar(data.lfo1.bipolar);
        lfo1.set_frequency_hz(data.lfo1.frequency_hz);
        lfo1.set_beat_sync(data.lfo1.beat_sync);
        lfo1.set_beats_per_cycle(data.lfo1.beats_per_cycle);
        lfo1.set_depth(data.lfo1.depth);

        auto& lfo2 = matrix.lfo2();
        lfo2.set_waveform(static_cast<modulation::LfoWaveform>(data.lfo2.waveform));
        lfo2.set_bipolar(data.lfo2.bipolar);
        lfo2.set_frequency_hz(data.lfo2.frequency_hz);
        lfo2.set_beat_sync(data.lfo2.beat_sync);
        lfo2.set_beats_per_cycle(data.lfo2.beats_per_cycle);
        lfo2.set_depth(data.lfo2.depth);

        auto apply_mseg = [](modulation::MultiStageEnvelope& mseg, const MsegData& m) {
            mseg.set_base_time_scale(m.base_time_scale);
            mseg.set_base_attack_scale(m.base_attack_scale);
            mseg.set_base_decay_scale(m.base_decay_scale);
            mseg.set_base_level_scale(m.base_level_scale);
            mseg.set_base_tension_offset(m.base_tension_offset);

            if (!m.points.empty()) {
                std::vector<modulation::MsegPoint> pts;
                pts.reserve(m.points.size());
                for (const auto& pt : m.points) {
                    pts.push_back(modulation::MsegPoint{
                        .time = pt.time,
                        .value = pt.value,
                        .node_mode = static_cast<routing::NodeMode>(pt.node_mode),
                        .tension = pt.tension
                    });
                }
                mseg.set_points(std::move(pts),
                                static_cast<modulation::MsegTimeMode>(m.time_mode),
                                static_cast<modulation::MsegLoopMode>(m.loop_mode),
                                m.sustain_idx);
            }
        };

        apply_mseg(matrix.mseg1(), data.mseg1);
        apply_mseg(matrix.mseg2(), data.mseg2);

        auto& ps = matrix.poly_synth();
        ps.set_polyphony_limit(data.poly_synth.polyphony_limit);
        ps.set_play_mode(static_cast<modulation::PolyphonyPlayMode>(data.poly_synth.play_mode));
        ps.set_osc1_waveform(static_cast<dsp::Waveform>(data.poly_synth.osc1_waveform));
        ps.set_osc2_waveform(static_cast<dsp::Waveform>(data.poly_synth.osc2_waveform));
        ps.set_osc_mix(data.poly_synth.osc_mix);
        ps.set_osc2_detune_cents(data.poly_synth.osc2_detune_cents);
        ps.set_osc2_octave_offset(data.poly_synth.osc2_octave_offset);
        ps.set_filter_type(static_cast<dsp::FilterType>(data.poly_synth.filter_type));
        ps.set_base_cutoff(data.poly_synth.base_cutoff);
        ps.set_resonance_q(data.poly_synth.resonance_q);
        ps.set_filter_env_amount(data.poly_synth.filter_env_amount);
        ps.set_keytrack_amount(data.poly_synth.keytrack_amount);
        ps.set_velocity_to_filter(data.poly_synth.velocity_to_filter);
        ps.set_velocity_to_amp(data.poly_synth.velocity_to_amp);
        ps.set_voice_pan_spread(data.poly_synth.voice_pan_spread);
        ps.set_glide_time_ms(data.poly_synth.glide_time_ms);
        ps.set_master_level(data.poly_synth.master_level);
    }

    static ProjectSessionData extract_session(const MixerGraph& mixer, const clock::TimelineClock& clock,
                                              const std::string& name = "Untitled Project",
                                              const modulation::ModulationMatrix* mod_matrix = nullptr) {
        ProjectSessionData data;
        data.project_name = name;
        data.sample_rate = mixer.sample_rate();
        data.bpm = clock.bpm();
        data.master_volume = mixer.master_volume();
        data.master_limiter_enabled = mixer.is_master_limiter_enabled();

        if (mod_matrix) {
            data.modulation_matrix = extract_modulation_matrix(*mod_matrix);
        }

        // Extract tracks
        for (uint32_t t = 0; t < mixer.track_count(); ++t) {
            const auto* trk = mixer.track_by_index(t);
            if (!trk) continue;

            TrackPresetData tdata;
            tdata.id = trk->id();
            tdata.name = trk->name();
            tdata.active = trk->is_active();
            tdata.gain = trk->gain();
            tdata.pan = trk->pan();
            tdata.mute = trk->is_muted();
            tdata.solo = trk->is_solo();
            tdata.solo_safe = trk->is_solo_safe();
            tdata.dca_mask = trk->dca_mask();
            tdata.target_bus = trk->target_bus();
            tdata.input_mode = static_cast<uint8_t>(trk->input_mode());
            tdata.input_gain = trk->input_gain();
            tdata.input_phase_invert = trk->input_phase_invert();
            tdata.rack = extract_rack_preset(*trk, trk->name() + " Rack");

            auto extract_pts = [](const routing::AutomationCurve& curve, std::vector<AutomationPointData>& out) {
                for (const auto& pt : curve.get_points()) {
                    out.push_back(AutomationPointData{pt.time_beats, pt.value, static_cast<uint8_t>(pt.node_mode), pt.tension});
                }
            };
            tdata.automation.gain_enabled = trk->is_automation_enabled(routing::AutomationTarget::Gain);
            extract_pts(trk->automation_curve(routing::AutomationTarget::Gain), tdata.automation.gain_points);

            tdata.automation.pan_enabled = trk->is_automation_enabled(routing::AutomationTarget::Pan);
            extract_pts(trk->automation_curve(routing::AutomationTarget::Pan), tdata.automation.pan_points);

            tdata.automation.aux1_enabled = trk->is_automation_enabled(routing::AutomationTarget::Aux1);
            extract_pts(trk->automation_curve(routing::AutomationTarget::Aux1), tdata.automation.aux1_points);

            tdata.automation.aux2_enabled = trk->is_automation_enabled(routing::AutomationTarget::Aux2);
            extract_pts(trk->automation_curve(routing::AutomationTarget::Aux2), tdata.automation.aux2_points);

            for (size_t s = 0; s < Track::kMaxTrackInsertSlots; ++s) {
                for (size_t p = 0; p < Track::kMaxSlotParams; ++p) {
                    bool en = trk->is_slot_automation_enabled(s, p);
                    const auto& pts = trk->slot_automation_curve(s, p).get_points();
                    if (en || pts.size() > 1 || (pts.size() == 1 && std::abs(pts[0].value) > 1e-4f)) {
                        SlotAutomationData sdata;
                        sdata.slot = static_cast<uint32_t>(s);
                        sdata.param = static_cast<uint32_t>(p);
                        sdata.enabled = en;
                        extract_pts(trk->slot_automation_curve(s, p), sdata.points);
                        tdata.automation.slot_automation.push_back(std::move(sdata));
                    }
                }
            }

            // Extract clip envelopes if track has an audio clip
            if (trk->clip()) {
                const auto& clp = trk->clip();
                tdata.clip_envelopes.gain_enabled = clp->is_envelope_enabled(sampling::ClipEnvelopeTarget::Gain);
                extract_pts(clp->envelope(sampling::ClipEnvelopeTarget::Gain), tdata.clip_envelopes.gain_points);

                tdata.clip_envelopes.pan_enabled = clp->is_envelope_enabled(sampling::ClipEnvelopeTarget::Pan);
                extract_pts(clp->envelope(sampling::ClipEnvelopeTarget::Pan), tdata.clip_envelopes.pan_points);

                tdata.clip_envelopes.pitch_enabled = clp->is_envelope_enabled(sampling::ClipEnvelopeTarget::Pitch);
                extract_pts(clp->envelope(sampling::ClipEnvelopeTarget::Pitch), tdata.clip_envelopes.pitch_points);

                tdata.clip_envelopes.length_beats = clp->envelope_length_beats();
            }

            // Extract sequencer pattern steps
            const auto* seq = trk->sequencer();
            if (seq) {
                const auto& pat = seq->pattern(0);
                for (size_t s = 0; s < pat.num_steps; ++s) {
                    const auto& step = pat.steps[s];
                    StepTriggerData st;
                    st.active = step.active;
                    st.slice_id = step.slice_id;
                    st.velocity = step.velocity;
                    st.pitch_ratio = step.pitch_ratio;
                    st.probability = step.probability;
                    st.reverse = step.reverse;
                    st.pan = step.pan;
                    st.micro_timing = step.micro_timing;
                    st.quantize_pct = step.quantize_pct;
                    st.choke_group = step.choke_group;
                    st.filter_cutoff = step.filter_cutoff;
                    st.filter_res = step.filter_res;
                    st.decay_ms = step.decay_ms;
                    st.drive = step.drive;
                    st.send_a = step.send_a;
                    st.send_b = step.send_b;
                    tdata.sequencer_steps.push_back(st);
                }
            }

            data.tracks.push_back(std::move(tdata));
        }

        return data;
    }

    static bool apply_session(MixerGraph& mixer, clock::TimelineClock& clock, const ProjectSessionData& data,
                              modulation::ModulationMatrix* mod_matrix = nullptr) {
        clock.set_bpm(data.bpm);
        mixer.set_master_volume(data.master_volume);
        mixer.set_master_limiter_enabled(data.master_limiter_enabled);

        if (mod_matrix && data.modulation_matrix.has_value()) {
            apply_modulation_matrix(*mod_matrix, *data.modulation_matrix);
        }

        for (const auto& tdata : data.tracks) {
            Track* trk = nullptr;
            if (tdata.id >= 1 && tdata.id <= MixerGraph::kMaxTracks) {
                trk = mixer.track_by_index(tdata.id - 1);
            }
            if (!trk) continue;

            trk->activate(tdata.id, tdata.name);
            trk->set_active(tdata.active);
            trk->set_gain(tdata.gain, true);
            trk->set_pan(tdata.pan, true);
            trk->set_mute(tdata.mute);
            trk->set_solo(tdata.solo);
            trk->set_solo_safe(tdata.solo_safe);
            trk->set_dca_mask(tdata.dca_mask);
            trk->set_target_bus(tdata.target_bus);
            trk->set_input_mode(static_cast<TrackInputMode>(tdata.input_mode));
            trk->set_input_gain(tdata.input_gain);
            trk->set_input_phase_invert(tdata.input_phase_invert);

            apply_rack_preset(*trk, tdata.rack, data.sample_rate);

            // Restore multi-parameter automation curves
            auto restore_lane = [](routing::AutomationCurve& curve, const std::vector<AutomationPointData>& pts, float def_val) {
                if (pts.empty()) {
                    curve.clear(def_val);
                } else {
                    std::vector<routing::AutomationPoint> r_pts;
                    r_pts.reserve(pts.size());
                    for (const auto& p : pts) {
                        r_pts.push_back(routing::AutomationPoint{p.t, p.v, static_cast<routing::NodeMode>(p.m), p.tau});
                    }
                    curve.set_points(std::move(r_pts));
                }
            };

            restore_lane(trk->automation_curve(routing::AutomationTarget::Gain), tdata.automation.gain_points, 1.0f);
            trk->set_automation_enabled(routing::AutomationTarget::Gain, tdata.automation.gain_enabled);

            restore_lane(trk->automation_curve(routing::AutomationTarget::Pan), tdata.automation.pan_points, 0.0f);
            trk->set_automation_enabled(routing::AutomationTarget::Pan, tdata.automation.pan_enabled);

            restore_lane(trk->automation_curve(routing::AutomationTarget::Aux1), tdata.automation.aux1_points, 0.0f);
            trk->set_automation_enabled(routing::AutomationTarget::Aux1, tdata.automation.aux1_enabled);

            restore_lane(trk->automation_curve(routing::AutomationTarget::Aux2), tdata.automation.aux2_points, 0.0f);
            trk->set_automation_enabled(routing::AutomationTarget::Aux2, tdata.automation.aux2_enabled);

            for (const auto& sdata : tdata.automation.slot_automation) {
                if (sdata.slot < Track::kMaxTrackInsertSlots && sdata.param < Track::kMaxSlotParams) {
                    restore_lane(trk->slot_automation_curve(sdata.slot, sdata.param), sdata.points, 0.0f);
                    trk->set_slot_automation_enabled(sdata.slot, sdata.param, sdata.enabled);
                }
            }

            // Restore clip envelopes if track has an audio clip (or instantiate clip if envelope data exists)
            if (!trk->clip() && (tdata.clip_envelopes.gain_enabled ||
                                 tdata.clip_envelopes.pan_enabled ||
                                 tdata.clip_envelopes.pitch_enabled ||
                                 !tdata.clip_envelopes.gain_points.empty() ||
                                 !tdata.clip_envelopes.pan_points.empty() ||
                                 !tdata.clip_envelopes.pitch_points.empty())) {
                const uint32_t def_frames = mixer.sample_rate() * 2;
                auto new_clp = std::make_shared<sampling::AudioClip>("RestoredClip", mixer.sample_rate(), 2, def_frames);
                trk->set_clip(new_clp, true);
            }

            if (trk->clip()) {
                auto clp = trk->clip();
                restore_lane(clp->envelope(sampling::ClipEnvelopeTarget::Gain), tdata.clip_envelopes.gain_points, 1.0f);
                clp->set_envelope_enabled(sampling::ClipEnvelopeTarget::Gain, tdata.clip_envelopes.gain_enabled);

                restore_lane(clp->envelope(sampling::ClipEnvelopeTarget::Pan), tdata.clip_envelopes.pan_points, 0.0f);
                clp->set_envelope_enabled(sampling::ClipEnvelopeTarget::Pan, tdata.clip_envelopes.pan_enabled);

                restore_lane(clp->envelope(sampling::ClipEnvelopeTarget::Pitch), tdata.clip_envelopes.pitch_points, 0.0f);
                clp->set_envelope_enabled(sampling::ClipEnvelopeTarget::Pitch, tdata.clip_envelopes.pitch_enabled);

                if (tdata.clip_envelopes.length_beats > 0.0) {
                    clp->set_envelope_length_beats(tdata.clip_envelopes.length_beats);
                }
            }

            // Restore sequencer pattern
            auto* seq = trk->sequencer();
            if (!seq && !tdata.sequencer_steps.empty()) {
                trk->set_sequencer(std::make_shared<sequencer::StepSequencer>());
                seq = trk->sequencer();
            }
            if (seq) {
                auto& pat = seq->pattern(0);
                pat.clear();
                pat.num_steps = static_cast<uint32_t>(std::min<size_t>(tdata.sequencer_steps.size(), 16));
                for (size_t s = 0; s < tdata.sequencer_steps.size() && s < sequencer::Pattern::kMaxSteps; ++s) {
                    const auto& st = tdata.sequencer_steps[s];
                    if (st.active) {
                        pat.set_step(s, st.slice_id, st.velocity, st.pitch_ratio, st.probability,
                                     st.reverse, st.pan, st.micro_timing, st.quantize_pct,
                                     st.choke_group, st.filter_cutoff, st.filter_res,
                                     dsp::FilterType::Lowpass, st.decay_ms, st.drive,
                                     st.send_a, st.send_b);
                    }
                }
            }
        }

        return true;
    }

    // High-Level File Persistence
    static bool save_session_file(const std::string& filepath, const MixerGraph& mixer,
                                  const clock::TimelineClock& clock, const std::string& name = "Untitled Project",
                                  const modulation::ModulationMatrix* mod_matrix = nullptr) {
        auto data = extract_session(mixer, clock, name, mod_matrix);
        std::ofstream file(filepath);
        if (!file.is_open()) return false;
        file << data.to_json();
        return true;
    }

    static bool load_session_file(const std::string& filepath, MixerGraph& mixer, clock::TimelineClock& clock,
                                  modulation::ModulationMatrix* mod_matrix = nullptr) {
        std::ifstream file(filepath);
        if (!file.is_open()) return false;
        std::string json_str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        auto parsed = json::Parser::parse(json_str);
        if (!parsed) return false;
        auto data = ProjectSessionData::from_json_val(*parsed);
        if (!data) return false;
        return apply_session(mixer, clock, *data, mod_matrix);
    }

    static bool save_rack_preset_file(const std::string& filepath, const Track& track, const std::string& name = "User Preset") {
        auto data = extract_rack_preset(track, name);
        std::ofstream file(filepath);
        if (!file.is_open()) return false;
        file << data.to_json();
        return true;
    }

    static bool load_rack_preset_file(const std::string& filepath, Track& track, uint32_t sample_rate = 48000) {
        std::ifstream file(filepath);
        if (!file.is_open()) return false;
        std::string json_str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        auto parsed = json::Parser::parse(json_str);
        if (!parsed) return false;
        auto data = RackPresetData::from_json_val(*parsed);
        if (!data) return false;
        apply_rack_preset(track, *data, sample_rate);
        return true;
    }
};

} // namespace audio_core::serialization
