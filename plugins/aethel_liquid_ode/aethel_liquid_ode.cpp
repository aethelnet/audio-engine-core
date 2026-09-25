#include <clap/clap.h>
#include "clapwrapper/vst3.h"
#include "aethel_liquid_ode_entry.h"

#include "audio_core/dsp/liquid_ode.hpp"
#include "audio_core/dsp/liquid_vactrol.hpp"

#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <algorithm>

namespace {

static const char *s_plugin_features[] = {
    CLAP_PLUGIN_FEATURE_AUDIO_EFFECT,
    CLAP_PLUGIN_FEATURE_FILTER,
    CLAP_PLUGIN_FEATURE_DISTORTION,
    CLAP_PLUGIN_FEATURE_STEREO,
    nullptr
};

static const clap_plugin_descriptor_t s_aethel_liquid_ode_desc = {
    CLAP_VERSION_INIT,
    "org.aethel.audio.liquid-ode",
    "Aethel Liquid ODE",
    "Aethel Audio",
    "https://github.com/aethelnet",
    "",
    "",
    "1.0.0",
    "Continuous-Time Liquid ODE Nonlinear Filter & Buchla LPG Saturator",
    s_plugin_features
};

enum ParamIds : clap_id {
    pid_CUTOFF = 0,
    pid_DRIVE = 1,
    pid_MODE = 2,
    pid_VISCOSITY = 3,
    pid_MEMORY = 4,
    pid_MIX = 5,
    pid_OUTPUT_TRIM = 6
};

constexpr uint32_t kParamCount = 7;

enum FilterMode : int32_t {
    MODE_LOWPASS = 0,
    MODE_HIGHPASS = 1,
    MODE_BANDPASS = 2,
    MODE_BUCHLA_LPG = 3
};

struct AethelLiquidOdePlug {
    clap_plugin_t plugin;
    const clap_host_t *host{nullptr};
    const clap_host_log_t *host_log{nullptr};
    const clap_host_params_t *host_params{nullptr};

    double sample_rate{48000.0};
    uint32_t channels{2};

    // Parameters
    float cutoff_hz{1000.0f};
    float drive{1.0f};
    int32_t mode{MODE_LOWPASS};
    float viscosity{1.0f};
    float memory_depth{0.5f};
    float mix{1.0f};
    float output_trim_db{0.0f};

    // DSP Engine
    audio_core::dsp::LiquidMultimodeFilter filter_l;
    audio_core::dsp::LiquidMultimodeFilter filter_r;
    audio_core::dsp::LiquidVactrol vactrol;


    void update_dsp() noexcept {
        const float active_fc = std::clamp(cutoff_hz * viscosity, 10.0f, static_cast<float>(sample_rate * 0.49f));
        filter_l.set_cutoff(active_fc);
        filter_r.set_cutoff(active_fc);
        filter_l.set_drive(drive);
        filter_r.set_drive(drive);

        switch (mode) {
            case MODE_LOWPASS:
                filter_l.set_mode(audio_core::dsp::LiquidMultimodeFilter::Mode::Lowpass);
                filter_r.set_mode(audio_core::dsp::LiquidMultimodeFilter::Mode::Lowpass);
                break;
            case MODE_HIGHPASS:
                filter_l.set_mode(audio_core::dsp::LiquidMultimodeFilter::Mode::Highpass);
                filter_r.set_mode(audio_core::dsp::LiquidMultimodeFilter::Mode::Highpass);
                break;
            case MODE_BANDPASS:
                filter_l.set_mode(audio_core::dsp::LiquidMultimodeFilter::Mode::Bandpass);
                filter_r.set_mode(audio_core::dsp::LiquidMultimodeFilter::Mode::Bandpass);
                filter_l.set_bandpass_corners(std::max(20.0f, active_fc * 0.5f), std::min(active_fc * 2.0f, static_cast<float>(sample_rate * 0.49f)));
                filter_r.set_bandpass_corners(std::max(20.0f, active_fc * 0.5f), std::min(active_fc * 2.0f, static_cast<float>(sample_rate * 0.49f)));
                break;
            case MODE_BUCHLA_LPG:
                vactrol.set_mode(audio_core::dsp::VactrolMode::BuchlaLPG);
                vactrol.set_peak_reduction(std::clamp(drive * 0.15f, 0.0f, 1.0f));
                vactrol.set_memory_depth(memory_depth);
                break;
            default:
                break;
        }
    }
};

// ============================================================================
// Parameter Management
// ============================================================================

uint32_t plug_param_count(const clap_plugin_t *plugin) {
    return kParamCount;
}

bool plug_param_get_info(const clap_plugin_t *plugin, uint32_t param_index, clap_param_info_t *info) {
    if (!info) return false;
    std::memset(info, 0, sizeof(*info));

    switch (param_index) {
        case pid_CUTOFF:
            info->id = pid_CUTOFF;
            std::strncpy(info->name, "Cutoff", sizeof(info->name) - 1);
            std::strncpy(info->module, "Filter", sizeof(info->module) - 1);
            info->min_value = 20.0;
            info->max_value = 20000.0;
            info->default_value = 1000.0;
            info->flags = CLAP_PARAM_IS_AUTOMATABLE | CLAP_PARAM_IS_MODULATABLE;
            return true;

        case pid_DRIVE:
            info->id = pid_DRIVE;
            std::strncpy(info->name, "Drive", sizeof(info->name) - 1);
            std::strncpy(info->module, "Nonlinear", sizeof(info->module) - 1);
            info->min_value = 0.1;
            info->max_value = 10.0;
            info->default_value = 1.0;
            info->flags = CLAP_PARAM_IS_AUTOMATABLE | CLAP_PARAM_IS_MODULATABLE;
            return true;

        case pid_MODE:
            info->id = pid_MODE;
            std::strncpy(info->name, "Mode", sizeof(info->name) - 1);
            std::strncpy(info->module, "Topology", sizeof(info->module) - 1);
            info->min_value = 0.0;
            info->max_value = 3.0;
            info->default_value = 0.0;
            info->flags = CLAP_PARAM_IS_AUTOMATABLE | CLAP_PARAM_IS_STEPPED;
            return true;

        case pid_VISCOSITY:
            info->id = pid_VISCOSITY;
            std::strncpy(info->name, "Viscosity / Tau", sizeof(info->name) - 1);
            std::strncpy(info->module, "Liquid Core", sizeof(info->module) - 1);
            info->min_value = 0.1;
            info->max_value = 5.0;
            info->default_value = 1.0;
            info->flags = CLAP_PARAM_IS_AUTOMATABLE | CLAP_PARAM_IS_MODULATABLE;
            return true;

        case pid_MEMORY:
            info->id = pid_MEMORY;
            std::strncpy(info->name, "Vactrol Trap Memory", sizeof(info->name) - 1);
            std::strncpy(info->module, "Liquid Core", sizeof(info->module) - 1);
            info->min_value = 0.0;
            info->max_value = 1.0;
            info->default_value = 0.5;
            info->flags = CLAP_PARAM_IS_AUTOMATABLE | CLAP_PARAM_IS_MODULATABLE;
            return true;

        case pid_MIX:
            info->id = pid_MIX;
            std::strncpy(info->name, "Dry/Wet Mix", sizeof(info->name) - 1);
            std::strncpy(info->module, "Master", sizeof(info->module) - 1);
            info->min_value = 0.0;
            info->max_value = 1.0;
            info->default_value = 1.0;
            info->flags = CLAP_PARAM_IS_AUTOMATABLE | CLAP_PARAM_IS_MODULATABLE;
            return true;

        case pid_OUTPUT_TRIM:
            info->id = pid_OUTPUT_TRIM;
            std::strncpy(info->name, "Output Trim", sizeof(info->name) - 1);
            std::strncpy(info->module, "Master", sizeof(info->module) - 1);
            info->min_value = -24.0;
            info->max_value = 24.0;
            info->default_value = 0.0;
            info->flags = CLAP_PARAM_IS_AUTOMATABLE | CLAP_PARAM_IS_MODULATABLE;
            return true;

        default:
            return false;
    }
}

bool plug_param_get_value(const clap_plugin_t *plugin, clap_id param_id, double *value) {
    if (!plugin || !value) return false;
    auto *plug = static_cast<AethelLiquidOdePlug*>(plugin->plugin_data);

    switch (param_id) {
        case pid_CUTOFF:
            *value = plug->cutoff_hz;
            return true;
        case pid_DRIVE:
            *value = plug->drive;
            return true;
        case pid_MODE:
            *value = static_cast<double>(plug->mode);
            return true;
        case pid_VISCOSITY:
            *value = plug->viscosity;
            return true;
        case pid_MEMORY:
            *value = plug->memory_depth;
            return true;
        case pid_MIX:
            *value = plug->mix;
            return true;
        case pid_OUTPUT_TRIM:
            *value = plug->output_trim_db;
            return true;
        default:
            return false;
    }
}

bool plug_param_value_to_text(const clap_plugin_t *plugin, clap_id param_id, double value,
                              char *display, uint32_t size) {
    if (!display || size == 0) return false;

    switch (param_id) {
        case pid_CUTOFF:
            std::snprintf(display, size, "%.1f Hz", value);
            return true;
        case pid_DRIVE:
            std::snprintf(display, size, "%.2fx", value);
            return true;
        case pid_MODE: {
            const int m = static_cast<int>(std::round(value));
            switch (m) {
                case MODE_LOWPASS:
                    std::snprintf(display, size, "Lowpass (ODE)");
                    break;
                case MODE_HIGHPASS:
                    std::snprintf(display, size, "Highpass (Complement)");
                    break;
                case MODE_BANDPASS:
                    std::snprintf(display, size, "Bandpass (Subtractive)");
                    break;
                case MODE_BUCHLA_LPG:
                    std::snprintf(display, size, "Buchla LPG (Vactrol)");
                    break;
                default:
                    std::snprintf(display, size, "Mode %d", m);
                    break;
            }
            return true;
        }
        case pid_VISCOSITY:
            std::snprintf(display, size, "%.2fx", value);
            return true;
        case pid_MEMORY:
            std::snprintf(display, size, "%.1f %%", value * 100.0);
            return true;
        case pid_MIX:
            std::snprintf(display, size, "%.1f %%", value * 100.0);
            return true;
        case pid_OUTPUT_TRIM:
            std::snprintf(display, size, "%+.1f dB", value);
            return true;
        default:
            return false;
    }
}

bool plug_param_text_to_value(const clap_plugin_t *plugin, clap_id param_id, const char *display, double *value) {
    if (!display || !value) return false;
    char *endptr = nullptr;
    const double val = std::strtod(display, &endptr);
    if (endptr == display) return false;
    *value = val;
    return true;
}

static void plug_process_event(AethelLiquidOdePlug *plug, const clap_event_header_t *hdr) {
    if (hdr->space_id != CLAP_CORE_EVENT_SPACE_ID) return;

    if (hdr->type == CLAP_EVENT_PARAM_VALUE) {
        const auto *ev = reinterpret_cast<const clap_event_param_value_t*>(hdr);
        switch (ev->param_id) {
            case pid_CUTOFF:
                plug->cutoff_hz = static_cast<float>(ev->value);
                break;
            case pid_DRIVE:
                plug->drive = static_cast<float>(ev->value);
                break;
            case pid_MODE:
                plug->mode = static_cast<int32_t>(std::round(ev->value));
                break;
            case pid_VISCOSITY:
                plug->viscosity = static_cast<float>(ev->value);
                break;
            case pid_MEMORY:
                plug->memory_depth = static_cast<float>(ev->value);
                break;
            case pid_MIX:
                plug->mix = static_cast<float>(ev->value);
                break;
            case pid_OUTPUT_TRIM:
                plug->output_trim_db = static_cast<float>(ev->value);
                break;
            default:
                break;
        }
        plug->update_dsp();
    }
}

void plug_flush(const clap_plugin_t *plugin, const clap_input_events_t *in, const clap_output_events_t *out) {
    if (!plugin || !in) return;
    auto *plug = static_cast<AethelLiquidOdePlug*>(plugin->plugin_data);
    const uint32_t size = in->size(in);
    for (uint32_t i = 0; i < size; ++i) {
        const clap_event_header_t *hdr = in->get(in, i);
        plug_process_event(plug, hdr);
    }
}

static const clap_plugin_params_t s_plug_params = {
    plug_param_count,
    plug_param_get_info,
    plug_param_get_value,
    plug_param_value_to_text,
    plug_param_text_to_value,
    plug_flush
};

// ============================================================================
// Audio Ports
// ============================================================================

uint32_t plug_audio_ports_count(const clap_plugin_t *plugin, bool is_input) {
    return 1;
}

bool plug_audio_ports_get(const clap_plugin_t *plugin, uint32_t index, bool is_input, clap_audio_port_info_t *info) {
    if (index > 0 || !info) return false;
    auto *plug = static_cast<AethelLiquidOdePlug*>(plugin->plugin_data);

    info->id = is_input ? 0 : 1;
    std::strncpy(info->name, is_input ? "Stereo In" : "Stereo Out", sizeof(info->name) - 1);
    info->flags = CLAP_AUDIO_PORT_IS_MAIN;
    info->channel_count = plug->channels;
    info->port_type = CLAP_PORT_STEREO;
    info->in_place_pair = is_input ? 1 : 0;
    return true;
}

static const clap_plugin_audio_ports_t s_plug_audio_ports = {
    plug_audio_ports_count,
    plug_audio_ports_get
};

// ============================================================================
// State Save / Load
// ============================================================================

#pragma pack(push, 1)
struct AethelPluginState {
    uint32_t magic;      // 'AODE'
    uint32_t version;    // 1
    float cutoff_hz;
    float drive;
    int32_t mode;
    float viscosity;
    float memory_depth;
    float mix;
    float output_trim_db;
};
#pragma pack(pop)

bool plug_state_save(const clap_plugin_t *plugin, const clap_ostream_t *stream) {
    if (!plugin || !stream) return false;
    auto *plug = static_cast<AethelLiquidOdePlug*>(plugin->plugin_data);

    AethelPluginState state{};
    state.magic = 0x414F4445; // 'AODE'
    state.version = 1;
    state.cutoff_hz = plug->cutoff_hz;
    state.drive = plug->drive;
    state.mode = plug->mode;
    state.viscosity = plug->viscosity;
    state.memory_depth = plug->memory_depth;
    state.mix = plug->mix;
    state.output_trim_db = plug->output_trim_db;

    const char *ptr = reinterpret_cast<const char*>(&state);
    int64_t remaining = sizeof(state);
    while (remaining > 0) {
        int64_t written = stream->write(stream, ptr, remaining);
        if (written <= 0) return false;
        ptr += written;
        remaining -= written;
    }
    return true;
}

bool plug_state_load(const clap_plugin_t *plugin, const clap_istream_t *stream) {
    if (!plugin || !stream) return false;
    auto *plug = static_cast<AethelLiquidOdePlug*>(plugin->plugin_data);

    AethelPluginState state{};
    char *ptr = reinterpret_cast<char*>(&state);
    int64_t remaining = sizeof(state);
    while (remaining > 0) {
        int64_t bytes_read = stream->read(stream, ptr, remaining);
        if (bytes_read <= 0) return false;
        ptr += bytes_read;
        remaining -= bytes_read;
    }

    if (state.magic != 0x414F4445 || state.version != 1) {
        return false;
    }

    plug->cutoff_hz = state.cutoff_hz;
    plug->drive = state.drive;
    plug->mode = state.mode;
    plug->viscosity = state.viscosity;
    plug->memory_depth = state.memory_depth;
    plug->mix = state.mix;
    plug->output_trim_db = state.output_trim_db;
    plug->update_dsp();

    if (plug->host && plug->host_params) {
        plug->host_params->rescan(plug->host, CLAP_PARAM_RESCAN_VALUES | CLAP_PARAM_RESCAN_TEXT);
        plug->host_params->request_flush(plug->host);
    }
    return true;
}

static const clap_plugin_state_t s_plug_state = {
    plug_state_save,
    plug_state_load
};

// ============================================================================
// Latency: Zero Latency Continuous-Time Dynamical System
// ============================================================================

uint32_t plug_latency_get(const clap_plugin_t *plugin) {
    return 0; // Pure zero-latency continuous ODE
}

static const clap_plugin_latency_t s_plug_latency = {
    plug_latency_get
};

// ============================================================================
// Lifecycle & Audio Processing
// ============================================================================

bool plug_init(const struct clap_plugin *plugin) {
    if (!plugin) return false;
    auto *plug = static_cast<AethelLiquidOdePlug*>(plugin->plugin_data);
    if (!plug || !plug->host) return false;

    plug->host_log = static_cast<const clap_host_log_t*>(plug->host->get_extension(plug->host, CLAP_EXT_LOG));
    plug->host_params = static_cast<const clap_host_params_t*>(plug->host->get_extension(plug->host, CLAP_EXT_PARAMS));
    plug->channels = 2;

    if (plug->host_log) {
        plug->host_log->log(plug->host, CLAP_LOG_INFO, "Aethel Liquid ODE plugin initialized successfully.");
    }
    return true;
}

void plug_destroy(const struct clap_plugin *plugin) {
    if (!plugin) return;
    auto *plug = static_cast<AethelLiquidOdePlug*>(plugin->plugin_data);
    delete plug;
}

bool plug_activate(const struct clap_plugin *plugin, double sample_rate,
                   uint32_t min_frames_count, uint32_t max_frames_count) {
    if (!plugin) return false;
    auto *plug = static_cast<AethelLiquidOdePlug*>(plugin->plugin_data);
    plug->sample_rate = sample_rate > 0.0 ? sample_rate : 48000.0;
    plug->filter_l.set_sample_rate(static_cast<float>(plug->sample_rate));
    plug->filter_r.set_sample_rate(static_cast<float>(plug->sample_rate));
    plug->vactrol.set_sample_rate(static_cast<uint32_t>(plug->sample_rate));
    plug->vactrol.reset();
    plug->update_dsp();
    return true;
}

void plug_deactivate(const struct clap_plugin *plugin) {
}

bool plug_start_processing(const struct clap_plugin *plugin) {
    return true;
}

void plug_stop_processing(const struct clap_plugin *plugin) {
}

void plug_reset(const struct clap_plugin *plugin) {
    if (!plugin) return;
    auto *plug = static_cast<AethelLiquidOdePlug*>(plugin->plugin_data);
    plug->filter_l.reset();
    plug->filter_r.reset();
    plug->vactrol.reset();
}

clap_process_status plug_process(const struct clap_plugin *plugin, const clap_process_t *process) {
    if (!plugin || !process) return CLAP_PROCESS_ERROR;
    auto *plug = static_cast<AethelLiquidOdePlug*>(plugin->plugin_data);

    const uint32_t nframes = process->frames_count;
    if (nframes == 0) return CLAP_PROCESS_CONTINUE;

    // Check audio port validity
    if (process->audio_inputs_count == 0 || process->audio_outputs_count == 0) {
        return CLAP_PROCESS_CONTINUE;
    }

    const uint32_t in_chans = process->audio_inputs[0].channel_count;
    const uint32_t out_chans = process->audio_outputs[0].channel_count;
    if (in_chans == 0 || out_chans == 0) return CLAP_PROCESS_CONTINUE;

    const float *in_l = process->audio_inputs[0].data32[0];
    const float *in_r = (in_chans > 1) ? process->audio_inputs[0].data32[1] : in_l;
    float *out_l = process->audio_outputs[0].data32[0];
    float *out_r = (out_chans > 1) ? process->audio_outputs[0].data32[1] : out_l;

    const uint32_t nev = process->in_events ? process->in_events->size(process->in_events) : 0;
    uint32_t ev_index = 0;
    uint32_t next_ev_frame = (nev > 0) ? 0 : nframes;

    for (uint32_t i = 0; i < nframes;) {
        // Dispatch automation events at current frame
        while (ev_index < nev && next_ev_frame == i) {
            const clap_event_header_t *hdr = process->in_events->get(process->in_events, ev_index);
            if (hdr->time != i) {
                next_ev_frame = hdr->time;
                break;
            }
            plug_process_event(plug, hdr);
            ++ev_index;
            if (ev_index == nev) {
                next_ev_frame = nframes;
                break;
            }
        }

        // Process audio up to next automation frame
        const uint32_t end_frame = std::min(next_ev_frame, nframes);
        const float trim_gain = std::pow(10.0f, plug->output_trim_db / 20.0f);
        const float wet = plug->mix;
        const float dry = 1.0f - wet;

        for (; i < end_frame; ++i) {
            const float s_in_l = in_l[i];
            const float s_in_r = in_r[i];

            float proc_l = s_in_l;
            float proc_r = s_in_r;

            if (plug->mode == MODE_BUCHLA_LPG) {
                float b_l[1] = {s_in_l};
                float b_r[1] = {s_in_r};
                plug->vactrol.process_stereo(b_l, b_r, 1);
                proc_l = b_l[0];
                proc_r = b_r[0];
            } else {
                proc_l = plug->filter_l.process_sample(s_in_l);
                proc_r = plug->filter_r.process_sample(s_in_r);
            }

            float s_out_l = ((dry * s_in_l) + (wet * proc_l)) * trim_gain;
            float s_out_r = ((dry * s_in_r) + (wet * proc_r)) * trim_gain;

            // Denormal protection
            if (std::abs(s_out_l) < 1e-18f) s_out_l = 0.0f;
            if (std::abs(s_out_r) < 1e-18f) s_out_r = 0.0f;

            out_l[i] = s_out_l;
            if (out_chans > 1) {
                out_r[i] = s_out_r;
            }
        }
    }

    return CLAP_PROCESS_CONTINUE;
}

const void *plug_get_extension(const struct clap_plugin *plugin, const char *id) {
    if (!id) return nullptr;
    if (!std::strcmp(id, CLAP_EXT_AUDIO_PORTS)) return &s_plug_audio_ports;
    if (!std::strcmp(id, CLAP_EXT_PARAMS)) return &s_plug_params;
    if (!std::strcmp(id, CLAP_EXT_STATE)) return &s_plug_state;
    if (!std::strcmp(id, CLAP_EXT_LATENCY)) return &s_plug_latency;
    return nullptr;
}

void plug_on_main_thread(const struct clap_plugin *plugin) {
}

const clap_plugin_t *aethel_create_plugin(const clap_host_t *host) {
    auto *plug = new AethelLiquidOdePlug();
    plug->host = host;
    plug->plugin.desc = &s_aethel_liquid_ode_desc;
    plug->plugin.plugin_data = plug;
    plug->plugin.init = plug_init;
    plug->plugin.destroy = plug_destroy;
    plug->plugin.activate = plug_activate;
    plug->plugin.deactivate = plug_deactivate;
    plug->plugin.start_processing = plug_start_processing;
    plug->plugin.stop_processing = plug_stop_processing;
    plug->plugin.reset = plug_reset;
    plug->plugin.process = plug_process;
    plug->plugin.get_extension = plug_get_extension;
    plug->plugin.on_main_thread = plug_on_main_thread;
    return &plug->plugin;
}

// ============================================================================
// Factory Implementation
// ============================================================================

uint32_t factory_get_plugin_count(const struct clap_plugin_factory *factory) {
    return 1;
}

const clap_plugin_descriptor_t *factory_get_plugin_descriptor(const struct clap_plugin_factory *factory, uint32_t index) {
    return (index == 0) ? &s_aethel_liquid_ode_desc : nullptr;
}

const clap_plugin_t *factory_create_plugin(const struct clap_plugin_factory *factory, const clap_host_t *host, const char *plugin_id) {
    if (!clap_version_is_compatible(host->clap_version)) return nullptr;
    if (!std::strcmp(plugin_id, s_aethel_liquid_ode_desc.id)) {
        return aethel_create_plugin(host);
    }
    return nullptr;
}

static const clap_plugin_factory_t s_plugin_factory = {
    factory_get_plugin_count,
    factory_get_plugin_descriptor,
    factory_create_plugin
};

// VST3 Compatibility JSON
const char *factory_get_vst3_compatibility(const clap_plugin_factory_as_vst3 *factory) {
    return "[]";
}

static const clap_plugin_factory_as_vst3_t s_plugin_factory_as_vst3 = {
    nullptr, // vendor
    nullptr, // vendor_url
    nullptr, // email_contact
    nullptr, // get_vst3_info
    factory_get_vst3_compatibility
};

} // anonymous namespace

// ============================================================================
// Exported Entry Functions
// ============================================================================

bool aethel_liquid_ode_init(const char *plugin_path) {
    return true;
}

void aethel_liquid_ode_deinit(void) {
}

const void *aethel_liquid_ode_get_factory(const char *factory_id) {
    if (!std::strcmp(factory_id, CLAP_PLUGIN_FACTORY_ID)) return &s_plugin_factory;
    if (!std::strcmp(factory_id, CLAP_PLUGIN_FACTORY_INFO_VST3)) return &s_plugin_factory_as_vst3;
    if (!std::strcmp(factory_id, CLAP_PLUGIN_FACTORY_INFO_VST3_V1)) return &s_plugin_factory_as_vst3;
    return nullptr;
}
