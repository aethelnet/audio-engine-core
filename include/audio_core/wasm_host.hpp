#pragma once

#include "audio_core/types.hpp"
#include <string>
#include <vector>
#include <memory>
#include <span>

namespace audio_core {

class WasmDspPlugin {
public:
    WasmDspPlugin();
    ~WasmDspPlugin();

    // Move-only
    WasmDspPlugin(const WasmDspPlugin&) = delete;
    WasmDspPlugin& operator=(const WasmDspPlugin&) = delete;
    WasmDspPlugin(WasmDspPlugin&&) noexcept;
    WasmDspPlugin& operator=(WasmDspPlugin&&) noexcept;

    // Loading
    bool load_from_memory(std::span<const uint8_t> wasm_bytes);
    bool load_from_file(const std::string& filepath);

    bool init(uint32_t sample_rate);

    // Real-Time Audio Processing (Planar Stereo)
    // Non-allocating, safe for real-time audio thread
    void process_stereo(const Sample* in_left, const Sample* in_right,
                        Sample* out_left, Sample* out_right,
                        uint32_t num_frames) noexcept;

    // Parameter Control & Sovereign WASM ABI
    void set_parameter(uint32_t param_id, float value) noexcept;
    [[nodiscard]] float get_parameter(uint32_t param_id) noexcept;
    [[nodiscard]] uint32_t get_num_parameters() const noexcept;
    [[nodiscard]] std::string get_parameter_name(uint32_t param_id) const;

    // Watchdog & Fault Inspection
    [[nodiscard]] bool is_loaded() const noexcept;
    [[nodiscard]] bool has_fault() const noexcept;
    [[nodiscard]] const char* last_error() const noexcept;
    [[nodiscard]] uint64_t trap_count() const noexcept;
    void clear_fault() noexcept;

    // Gas Limit Watchdog Configuration
    void set_gas_limit_per_frame(double gas) noexcept;
    [[nodiscard]] double gas_limit_per_frame() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace audio_core
