#pragma once

#include "audio_core/types.hpp"
#include <atomic>
#include <cmath>
#include <algorithm>
#include <string>
#include <string_view>
#include <cstdint>

#if defined(__AVX2__)
#include <immintrin.h>
#elif defined(__SSE__)
#include <xmmintrin.h>
#endif

namespace audio_core::routing {

// ============================================================================
// ModulatableParameter<float>: Zero-Converter Audio-Rate Modulatable Parameter
// Solves the "Bitwig Dilemma" in the Sovereign Audio Engine.
// Unified Signal Model: Audio == CV == Automation == Modulation.
// Features:
// 1. Thread-safe lock-free atomic parameter adjustments (base, depth, bounds).
// 2. High-performance sample-accurate audio-rate evaluation via Vectorized FMA.
// 3. Audio-rate compound modulation without intermediary converter nodes.
// ============================================================================
class ModulatableParameter {
public:
    ModulatableParameter(std::string name, float default_val, float min_val, float max_val) noexcept
        : m_name(std::move(name))
        , m_base(default_val)
        , m_min(min_val)
        , m_max(max_val)
        , m_depth(1.0f)
        , m_mod_active(false) {}

    ModulatableParameter() noexcept
        : ModulatableParameter("Param", 0.0f, 0.0f, 1.0f) {}

    [[nodiscard]] const std::string& name() const noexcept { return m_name; }
    void set_name(std::string name) noexcept { m_name = std::move(name); }

    void set_base(float val) noexcept {
        m_base.store(std::clamp(val, m_min.load(std::memory_order_relaxed), m_max.load(std::memory_order_relaxed)), std::memory_order_relaxed);
    }
    [[nodiscard]] float base() const noexcept { return m_base.load(std::memory_order_relaxed); }

    void set_depth(float depth) noexcept { m_depth.store(depth, std::memory_order_relaxed); }
    [[nodiscard]] float depth() const noexcept { return m_depth.load(std::memory_order_relaxed); }

    void set_bounds(float min_val, float max_val) noexcept {
        m_min.store(min_val, std::memory_order_relaxed);
        m_max.store(max_val, std::memory_order_relaxed);
        set_base(base());
    }
    [[nodiscard]] float min_val() const noexcept { return m_min.load(std::memory_order_relaxed); }
    [[nodiscard]] float max_val() const noexcept { return m_max.load(std::memory_order_relaxed); }

    void set_modulation_active(bool active) noexcept { m_mod_active.store(active, std::memory_order_relaxed); }
    [[nodiscard]] bool is_modulation_active() const noexcept { return m_mod_active.load(std::memory_order_relaxed); }

    // Instantaneous sample evaluation
    [[nodiscard]] inline float evaluate_sample(float mod_sample) const noexcept {
        const float b = m_base.load(std::memory_order_relaxed);
        if (!m_mod_active.load(std::memory_order_relaxed)) return b;

        const float d = m_depth.load(std::memory_order_relaxed);
        const float val = b + (d * mod_sample);
        return std::clamp(val, m_min.load(std::memory_order_relaxed), m_max.load(std::memory_order_relaxed));
    }

    // Audio-rate block evaluation: computes out[i] for all frames in the block
    void evaluate_block(const float* mod_buffer, float* out_buffer, uint32_t frames) const noexcept {
        if (!out_buffer || frames == 0) return;

        const float b = m_base.load(std::memory_order_relaxed);
        const float min_v = m_min.load(std::memory_order_relaxed);
        const float max_v = m_max.load(std::memory_order_relaxed);

        if (!mod_buffer || !m_mod_active.load(std::memory_order_relaxed)) {
            // Constant base value throughout block
            std::fill_n(out_buffer, frames, b);
            return;
        }

        const float d = m_depth.load(std::memory_order_relaxed);

        #if defined(__AVX2__)
        uint32_t i = 0;
        const __m256 v_base = _mm256_set1_ps(b);
        const __m256 v_depth = _mm256_set1_ps(d);
        const __m256 v_min = _mm256_set1_ps(min_v);
        const __m256 v_max = _mm256_set1_ps(max_v);

        for (; i + 8 <= frames; i += 8) {
            __m256 v_mod = _mm256_loadu_ps(mod_buffer + i);
            #if defined(__FMA__)
            __m256 v_res = _mm256_fmadd_ps(v_depth, v_mod, v_base);
            #else
            __m256 v_res = _mm256_add_ps(v_base, _mm256_mul_ps(v_depth, v_mod));
            #endif
            v_res = _mm256_max_ps(v_min, _mm256_min_ps(v_max, v_res));
            _mm256_storeu_ps(out_buffer + i, v_res);
        }

        for (; i < frames; ++i) {
            out_buffer[i] = std::clamp(b + (d * mod_buffer[i]), min_v, max_v);
        }
        #elif defined(__SSE__)
        uint32_t i = 0;
        const __m128 v_base = _mm_set1_ps(b);
        const __m128 v_depth = _mm_set1_ps(d);
        const __m128 v_min = _mm_set1_ps(min_v);
        const __m128 v_max = _mm_set1_ps(max_v);

        for (; i + 4 <= frames; i += 4) {
            __m128 v_mod = _mm_loadu_ps(mod_buffer + i);
            __m128 v_res = _mm_add_ps(v_base, _mm_mul_ps(v_depth, v_mod));
            v_res = _mm_max_ps(v_min, _mm_min_ps(v_max, v_res));
            _mm_storeu_ps(out_buffer + i, v_res);
        }

        for (; i < frames; ++i) {
            out_buffer[i] = std::clamp(b + (d * mod_buffer[i]), min_v, max_v);
        }
        #else
        #if defined(__GNUC__) || defined(__clang__)
        #pragma GCC ivdep
        #endif
        for (uint32_t i = 0; i < frames; ++i) {
            out_buffer[i] = std::clamp(b + (d * mod_buffer[i]), min_v, max_v);
        }
        #endif
    }

private:
    std::string m_name;
    std::atomic<float> m_base{0.0f};
    std::atomic<float> m_min{0.0f};
    std::atomic<float> m_max{1.0f};
    std::atomic<float> m_depth{1.0f};
    std::atomic<bool> m_mod_active{false};
};

} // namespace audio_core::routing
