#pragma once

#include "audio_core/types.hpp"
#include <complex>
#include <vector>
#include <cmath>
#include <numbers>
#include <cstdint>
#include <algorithm>
#include <span>

namespace audio_core::dsp {

// ============================================================================
// FastFourierTransform: High-Performance In-Place Radix-2 Cooley-Tukey FFT
// Header-only, zero external dependencies, broadcast-grade numerical precision.
// Used for:
// - Farina ESS Impulse Response Deconvolution
// - Acoustic Room Mode Spectral Analysis
// - Fast Frequency-Domain Convolution
// ============================================================================
class FastFourierTransform {
public:
    using Complex = std::complex<float>;

    // Next power of 2 >= n
    [[nodiscard]] static constexpr size_t next_power_of_two(size_t n) noexcept {
        if (n <= 1) return 1;
        --n;
        n |= n >> 1;
        n |= n >> 2;
        n |= n >> 4;
        n |= n >> 8;
        n |= n >> 16;
#if UINTPTR_MAX > 0xFFFFFFFF
        n |= n >> 32;
#endif
        return n + 1;
    }

    [[nodiscard]] static constexpr bool is_power_of_two(size_t n) noexcept {
        return (n > 0) && ((n & (n - 1)) == 0);
    }

    // In-place Forward FFT
    static void forward(std::vector<Complex>& x) noexcept {
        transform(x, false);
    }

    // In-place Inverse FFT (scales by 1/N)
    static void inverse(std::vector<Complex>& x) noexcept {
        transform(x, true);
        const float inv_n = 1.0f / static_cast<float>(x.size());
        for (auto& val : x) {
            val *= inv_n;
        }
    }

    // Direct real-to-complex forward transform
    static std::vector<Complex> forward_real(const float* data, size_t size) {
        const size_t n = next_power_of_two(size);
        std::vector<Complex> x(n, Complex(0.0f, 0.0f));
        for (size_t i = 0; i < size; ++i) {
            x[i] = Complex(data[i], 0.0f);
        }
        forward(x);
        return x;
    }

    // Fast linear convolution using FFT: y = x * h
    static std::vector<float> convolve(const float* x, size_t x_len,
                                       const float* h, size_t h_len) {
        if (!x || !h || x_len == 0 || h_len == 0) return {};

        const size_t conv_len = x_len + h_len - 1;
        const size_t n = next_power_of_two(conv_len);

        std::vector<Complex> X(n, Complex(0.0f, 0.0f));
        std::vector<Complex> H(n, Complex(0.0f, 0.0f));

        for (size_t i = 0; i < x_len; ++i) X[i] = Complex(x[i], 0.0f);
        for (size_t i = 0; i < h_len; ++i) H[i] = Complex(h[i], 0.0f);

        forward(X);
        forward(H);

        for (size_t i = 0; i < n; ++i) {
            X[i] *= H[i];
        }

        inverse(X);

        std::vector<float> result(conv_len);
        for (size_t i = 0; i < conv_len; ++i) {
            result[i] = X[i].real();
        }
        return result;
    }

    // Compute magnitude spectrum in dB: 20 * log10(|X[k]|)
    static std::vector<float> magnitude_spectrum_db(const std::vector<Complex>& X) {
        const size_t half_n = X.size() / 2 + 1;
        std::vector<float> mag_db(half_n);
        for (size_t k = 0; k < half_n; ++k) {
            float mag = std::abs(X[k]);
            mag_db[k] = 20.0f * std::log10(std::max(mag, 1e-12f));
        }
        return mag_db;
    }

private:
    static void bit_reverse(std::vector<Complex>& x) noexcept {
        const size_t n = x.size();
        size_t j = 0;
        for (size_t i = 0; i < n; ++i) {
            if (i < j) {
                std::swap(x[i], x[j]);
            }
            size_t bit = n >> 1;
            while (j & bit) {
                j ^= bit;
                bit >>= 1;
            }
            j ^= bit;
        }
    }

    static void transform(std::vector<Complex>& x, bool invert) noexcept {
        const size_t n = x.size();
        if (n <= 1 || !is_power_of_two(n)) return;

        bit_reverse(x);

        for (size_t len = 2; len <= n; len <<= 1) {
            const float angle = (invert ? 2.0f : -2.0f) * std::numbers::pi_v<float> / static_cast<float>(len);
            const Complex wlen(std::cos(angle), std::sin(angle));

            for (size_t i = 0; i < n; i += len) {
                Complex w(1.0f, 0.0f);
                const size_t half_len = len >> 1;
                for (size_t j = 0; j < half_len; ++j) {
                    Complex u = x[i + j];
                    Complex v = x[i + j + half_len] * w;
                    x[i + j] = u + v;
                    x[i + j + half_len] = u - v;
                    w *= wlen;
                }
            }
        }
    }
};

} // namespace audio_core::dsp
