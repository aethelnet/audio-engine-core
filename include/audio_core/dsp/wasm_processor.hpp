#pragma once

#include "audio_core/insert_slot.hpp"
#include "audio_core/wasm_host.hpp"
#include <memory>
#include <string>

namespace audio_core::dsp {

// ============================================================================
// WasmProcessor: Adapter wrapping WasmDspPlugin inside IProcessor
// Enables sandboxed WASM plugins to sit directly inside Track & Bus Insert Slots
// ============================================================================
class WasmProcessor : public IProcessor {
public:
    explicit WasmProcessor(std::unique_ptr<WasmDspPlugin> plugin, std::string name = "WASM Plugin")
        : m_plugin(std::move(plugin)), m_name(std::move(name)) {}

    void init(uint32_t sample_rate) noexcept override {
        if (m_plugin) {
            m_plugin->init(sample_rate);
        }
    }

    void reset() noexcept override {
        if (m_plugin) {
            m_plugin->clear_fault();
        }
    }

    [[nodiscard]] const char* name() const noexcept override {
        return m_name.c_str();
    }

    void set_parameter(uint32_t index, float value) noexcept override {
        if (m_plugin) {
            m_plugin->set_parameter(index, value);
        }
    }

    [[nodiscard]] float get_parameter(uint32_t index) const noexcept override {
        if (m_plugin) {
            return m_plugin->get_parameter(index);
        }
        return 0.0f;
    }

    [[nodiscard]] uint32_t get_num_parameters() const noexcept {
        if (m_plugin) {
            return m_plugin->get_num_parameters();
        }
        return 0;
    }

    [[nodiscard]] std::string get_parameter_name(uint32_t index) const {
        if (m_plugin) {
            return m_plugin->get_parameter_name(index);
        }
        return "";
    }

    void process_stereo(Sample* left, Sample* right, uint32_t frames) noexcept override {
        if (m_plugin && m_plugin->is_loaded()) {
            m_plugin->process_stereo(left, right, left, right, frames);
        }
    }

    [[nodiscard]] bool supports_sidechain() const noexcept override {
        return m_plugin && m_plugin->supports_sidechain();
    }

    void process_stereo_sidechain(Sample* left, Sample* right,
                                  const Sample* sc_left, const Sample* sc_right,
                                  uint32_t frames) noexcept override {
        if (m_plugin && m_plugin->is_loaded()) {
            m_plugin->process_stereo_sidechain(left, right, sc_left, sc_right, left, right, frames);
        }
    }

    [[nodiscard]] bool has_fault() const noexcept override {
        return m_plugin && m_plugin->has_fault();
    }

    [[nodiscard]] const char* fault_reason() const noexcept override {
        return m_plugin ? m_plugin->last_error() : nullptr;
    }

    void clear_fault() noexcept override {
        if (m_plugin) {
            m_plugin->clear_fault();
        }
    }

    [[nodiscard]] WasmDspPlugin* plugin() noexcept {
        return m_plugin.get();
    }

    [[nodiscard]] const WasmDspPlugin* plugin() const noexcept {
        return m_plugin.get();
    }

private:
    std::unique_ptr<WasmDspPlugin> m_plugin;
    std::string m_name;
};

} // namespace audio_core::dsp
