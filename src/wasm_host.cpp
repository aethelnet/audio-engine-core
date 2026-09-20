#include "audio_core/wasm_host.hpp"

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wextra"
#endif

#include "third_party/wasm3/source/wasm3.h"

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include <fstream>
#include <iostream>
#include <cstring>

namespace audio_core {

struct WasmDspPlugin::Impl {
    IM3Environment env{nullptr};
    IM3Runtime runtime{nullptr};
    IM3Module module{nullptr};

    std::vector<uint8_t> wasm_bytecode;

    IM3Function fn_init{nullptr};
    IM3Function fn_process{nullptr};
    IM3Function fn_set_param{nullptr};
    IM3Function fn_get_param{nullptr};
    IM3Function fn_get_in_buf{nullptr};
    IM3Function fn_get_out_buf{nullptr};

    uint32_t in_buf_offset{0};
    uint32_t out_buf_offset{0};
    bool loaded{false};

    ~Impl() {
        cleanup();
    }

    void cleanup() {
        if (runtime) {
            m3_FreeRuntime(runtime);
            runtime = nullptr;
            module = nullptr;
        }
        if (env) {
            m3_FreeEnvironment(env);
            env = nullptr;
        }
        loaded = false;
        in_buf_offset = 0;
        out_buf_offset = 0;
    }
};

WasmDspPlugin::WasmDspPlugin() : m_impl(std::make_unique<Impl>()) {}
WasmDspPlugin::~WasmDspPlugin() = default;

WasmDspPlugin::WasmDspPlugin(WasmDspPlugin&&) noexcept = default;
WasmDspPlugin& WasmDspPlugin::operator=(WasmDspPlugin&&) noexcept = default;

bool WasmDspPlugin::load_from_memory(std::span<const uint8_t> wasm_bytes) {
    m_impl->cleanup();

    if (wasm_bytes.empty()) {
        return false;
    }

    m_impl->wasm_bytecode.assign(wasm_bytes.begin(), wasm_bytes.end());

    m_impl->env = m3_NewEnvironment();
    if (!m_impl->env) {
        std::cerr << "[WasmHost] Failed to allocate WASM environment" << std::endl;
        return false;
    }

    // 64 KB stack size
    m_impl->runtime = m3_NewRuntime(m_impl->env, 64 * 1024, nullptr);
    if (!m_impl->runtime) {
        std::cerr << "[WasmHost] Failed to allocate WASM runtime" << std::endl;
        return false;
    }

    M3Result res = m3_ParseModule(m_impl->env, &m_impl->module, m_impl->wasm_bytecode.data(), static_cast<uint32_t>(m_impl->wasm_bytecode.size()));
    if (res) {
        std::cerr << "[WasmHost] ParseModule error: " << res << std::endl;
        return false;
    }

    res = m3_LoadModule(m_impl->runtime, m_impl->module);
    if (res) {
        std::cerr << "[WasmHost] LoadModule error: " << res << std::endl;
        return false;
    }

    m3_FindFunction(&m_impl->fn_init, m_impl->runtime, "dsp_init");
    m3_FindFunction(&m_impl->fn_process, m_impl->runtime, "dsp_process");
    m3_FindFunction(&m_impl->fn_set_param, m_impl->runtime, "dsp_set_param");
    m3_FindFunction(&m_impl->fn_get_param, m_impl->runtime, "dsp_get_param");
    m3_FindFunction(&m_impl->fn_get_in_buf, m_impl->runtime, "dsp_get_input_buffer");
    m3_FindFunction(&m_impl->fn_get_out_buf, m_impl->runtime, "dsp_get_output_buffer");

    if (!m_impl->fn_process || !m_impl->fn_get_in_buf || !m_impl->fn_get_out_buf) {
        std::cerr << "[WasmHost] Missing required DSP exports (dsp_process, dsp_get_input_buffer, dsp_get_output_buffer)" << std::endl;
        return false;
    }

    // Retrieve linear buffer offsets
    if (m3_CallV(m_impl->fn_get_in_buf) == m3Err_none) {
        m3_GetResultsV(m_impl->fn_get_in_buf, &m_impl->in_buf_offset);
    }
    if (m3_CallV(m_impl->fn_get_out_buf) == m3Err_none) {
        m3_GetResultsV(m_impl->fn_get_out_buf, &m_impl->out_buf_offset);
    }

    m_impl->loaded = true;
    return true;
}

bool WasmDspPlugin::load_from_file(const std::string& filepath) {
    std::ifstream file(filepath, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        std::cerr << "[WasmHost] Failed to open file: " << filepath << std::endl;
        return false;
    }

    const auto size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<uint8_t> buffer(static_cast<size_t>(size));
    if (!file.read(reinterpret_cast<char*>(buffer.data()), size)) {
        std::cerr << "[WasmHost] Failed to read file bytes: " << filepath << std::endl;
        return false;
    }

    return load_from_memory(buffer);
}

bool WasmDspPlugin::init(uint32_t sample_rate) {
    if (!m_impl->loaded) return false;
    if (m_impl->fn_init) {
        M3Result r = m3_CallV(m_impl->fn_init, sample_rate);
        return r == m3Err_none;
    }
    return true;
}

void WasmDspPlugin::process_stereo(const Sample* in_left, const Sample* in_right,
                                   Sample* out_left, Sample* out_right,
                                   uint32_t num_frames) noexcept {
    if (!m_impl->loaded || !m_impl->fn_process) {
        // Safe bypass
        if (in_left != out_left) std::memcpy(out_left, in_left, num_frames * sizeof(Sample));
        if (in_right != out_right) std::memcpy(out_right, in_right, num_frames * sizeof(Sample));
        return;
    }

    size_t mem_size = 0;
    uint8_t* mem = m3_GetMemory(m_impl->module, &mem_size, 0);
    if (!mem) return;

    constexpr uint32_t kMaxPluginFrames = 1024;
    const uint32_t frames = (num_frames < kMaxPluginFrames) ? num_frames : kMaxPluginFrames;

    auto* wasm_in = reinterpret_cast<Sample*>(mem + m_impl->in_buf_offset);
    const auto* wasm_out = reinterpret_cast<const Sample*>(mem + m_impl->out_buf_offset);

    // Copy planar channels into WASM memory
    std::memcpy(wasm_in, in_left, frames * sizeof(Sample));
    std::memcpy(wasm_in + kMaxPluginFrames, in_right, frames * sizeof(Sample));

    // Execute sandboxed DSP
    m3_CallV(m_impl->fn_process, frames);

    // Copy processed samples back
    std::memcpy(out_left, wasm_out, frames * sizeof(Sample));
    std::memcpy(out_right, wasm_out + kMaxPluginFrames, frames * sizeof(Sample));
}

void WasmDspPlugin::set_parameter(uint32_t param_id, float value) noexcept {
    if (!m_impl->loaded || !m_impl->fn_set_param) return;
    m3_CallV(m_impl->fn_set_param, param_id, value);
}

float WasmDspPlugin::get_parameter(uint32_t param_id) noexcept {
    if (!m_impl->loaded || !m_impl->fn_get_param) return 0.0f;
    M3Result r = m3_CallV(m_impl->fn_get_param, param_id);
    if (r != m3Err_none) return 0.0f;

    float val = 0.0f;
    m3_GetResultsV(m_impl->fn_get_param, &val);
    return val;
}

bool WasmDspPlugin::is_loaded() const noexcept {
    return m_impl->loaded;
}

} // namespace audio_core
