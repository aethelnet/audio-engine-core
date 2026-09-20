#pragma once

#include "audio_core/types.hpp"
#include <cmath>
#include <numbers>
#include <algorithm>
#include <cstdint>

namespace audio_core::dsp {

enum class ConsoleType : uint8_t {
    Bypass = 0,
    Console1,      // Original cubic polynomial (warm, vintage character)
    Purest,        // Console 5/8 (mathematically pure sin/asin, crystal depth)
    InvSquare,     // Console 6 (inverse square/root, punchy VCA transient snap)
    Spiral,        // Console 7 (density/spiral blend with golden ratio phi)
    BShifty,       // PurestConsole 3 (polynomial sine/asin without transcendental calls)
    ConsoleZero    // Rational fractional / Padé function (tube console curve)
};

enum class ConsoleMode : uint8_t {
    Channel = 0,   // Encode stage (placed per-track, post-fader)
    Buss           // Decode stage (placed on mix bus / master input)
};

class ConsoleProcessor {
public:
    ConsoleProcessor() = default;

    void set_type(ConsoleType type) noexcept {
        m_type = type;
    }

    void set_mode(ConsoleMode mode) noexcept {
        m_mode = mode;
    }

    void set_in_trim(float trim) noexcept {
        m_in_trim = std::clamp(trim, 0.0f, 4.0f);
    }

    void set_out_trim(float trim) noexcept {
        m_out_trim = std::clamp(trim, 0.0f, 4.0f);
    }

    [[nodiscard]] ConsoleType type() const noexcept { return m_type; }
    [[nodiscard]] ConsoleMode mode() const noexcept { return m_mode; }
    [[nodiscard]] float in_trim() const noexcept { return m_in_trim; }
    [[nodiscard]] float out_trim() const noexcept { return m_out_trim; }

    inline void process_sample(Sample& left, Sample& right) noexcept {
        if (m_type == ConsoleType::Bypass) {
            return;
        }

        if (m_in_trim != 1.0f) {
            left *= m_in_trim;
            right *= m_in_trim;
        }

        if (m_mode == ConsoleMode::Channel) {
            left = encode_sample(left);
            right = encode_sample(right);
        } else {
            left = decode_sample(left);
            right = decode_sample(right);
        }

        if (m_out_trim != 1.0f) {
            left *= m_out_trim;
            right *= m_out_trim;
        }
    }

    void process_stereo(Sample* left, Sample* right, uint32_t num_frames) noexcept {
        if (m_type == ConsoleType::Bypass || !left) {
            return;
        }

        for (uint32_t i = 0; i < num_frames; ++i) {
            Sample l = left[i];
            Sample r = right ? right[i] : l;
            process_sample(l, r);
            left[i] = l;
            if (right) {
                right[i] = r;
            }
        }
    }

private:
    [[nodiscard]] inline float encode_sample(float x) const noexcept {
        switch (m_type) {
            case ConsoleType::Console1: {
                float half = x * 0.83f;
                float falf = std::abs(half);
                return x - (half * falf * falf);
            }

            case ConsoleType::Purest: {
                constexpr float kMaxSine = std::numbers::pi_v<float> * 0.5f;
                float clamped = std::clamp(x, -kMaxSine, kMaxSine);
                return std::sin(clamped);
            }

            case ConsoleType::InvSquare: {
                if (x > 1.0f) return 1.0f;
                if (x > 0.0f) return 1.0f - (1.0f - x) * (1.0f - x);
                if (x < -1.0f) return -1.0f;
                return -1.0f + (1.0f + x) * (1.0f + x);
            }

            case ConsoleType::Spiral: {
                float clamped = std::clamp(x, -1.097f, 1.097f);
                float ax = std::abs(clamped);
                float denom = (ax == 0.0f) ? 1.0f : ax;
                float spiral = std::sin(clamped * ax) / denom;
                return (spiral * 0.8f) + (std::sin(clamped) * 0.2f);
            }

            case ConsoleType::BShifty: {
                float x2 = x * x;
                float x3 = x2 * x;
                float x5 = x3 * x2;
                float x7 = x5 * x2;
                float x9 = x7 * x2;
                return x + ((x5 / 128.0f) + (x9 / 262144.0f)) - ((x3 / 8.0f) + (x7 / 4096.0f));
            }

            case ConsoleType::ConsoleZero: {
                constexpr float kLimit = 1.4137166941154f;
                float clamped = std::clamp(x, -kLimit, kLimit);
                if (clamped > 0.0f) {
                    return (clamped * 0.5f) * (2.8274333882308f - clamped);
                } else {
                    return -(clamped * -0.5f) * (2.8274333882308f + clamped);
                }
            }

            default:
                return x;
        }
    }

    [[nodiscard]] inline float decode_sample(float y) const noexcept {
        switch (m_type) {
            case ConsoleType::Console1: {
                float half = y * 0.885f;
                float falf = std::abs(half);
                return y + (half * falf * falf);
            }

            case ConsoleType::Purest: {
                float clamped = std::clamp(y, -1.0f, 1.0f);
                return std::asin(clamped);
            }

            case ConsoleType::InvSquare: {
                if (y > 1.0f) return 1.0f;
                if (y > 0.0f) return 1.0f - std::sqrt(1.0f - y);
                if (y < -1.0f) return -1.0f;
                return -1.0f + std::sqrt(1.0f + y);
            }

            case ConsoleType::Spiral: {
                float clamped = std::clamp(y, -1.0f, 1.0f);
                float ay = std::abs(clamped);
                float denom = (ay == 0.0f) ? 1.0f : ay;
                float spiral = std::asin(clamped * ay) / denom;
                constexpr float phi1 = 0.61803398874989484820f;
                constexpr float phi2 = 0.38196601125010515180f;
                return (spiral * phi1) + (std::asin(clamped) * phi2);
            }

            case ConsoleType::BShifty: {
                float x2 = y * y;
                float x3 = x2 * y;
                float x5 = x3 * x2;
                float x7 = x5 * x2;
                float x9 = x7 * x2;
                return y + (x3 / 4.0f) + (x5 / 8.0f) + (x7 / 16.0f) + (x9 / 32.0f);
            }

            case ConsoleType::ConsoleZero: {
                constexpr float kLimit = 2.8f;
                float clamped = std::clamp(y, -kLimit, kLimit);
                if (clamped > 0.0f) {
                    return (clamped * 2.0f) / (3.0f - clamped);
                } else {
                    return -(clamped * -2.0f) / (3.0f + clamped);
                }
            }

            default:
                return y;
        }
    }

    ConsoleType m_type{ConsoleType::Bypass};
    ConsoleMode m_mode{ConsoleMode::Channel};
    float m_in_trim{1.0f};
    float m_out_trim{1.0f};
};

} // namespace audio_core::dsp
