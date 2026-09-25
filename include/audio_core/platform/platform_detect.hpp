#pragma once

// ============================================================================
// Sovereign Audio Core: Unified Platform, OS, CPU Architecture & SIMD Matrix
// Single-source-of-truth for multi-platform compilation across:
// - Linux Desktop (x86_64, aarch64 / Raspberry Pi / Asahi)
// - Android (aarch64, armv7-a via Android NDK & AAudio)
// - Microsoft Windows (x86_64 via MSVC / MinGW-w64 & WASAPI)
// - Apple macOS / iOS (Apple Silicon arm64, x86_64 & CoreAudio)
// - WebAssembly / Emscripten (wasm32 / wasm64 & WebAudio)
// ============================================================================

// ----------------------------------------------------------------------------
// 1. Operating System Identification
// ----------------------------------------------------------------------------
#if defined(__ANDROID__)
    #define AETHEL_OS_ANDROID 1
    #define AETHEL_OS_MOBILE 1
    #define AETHEL_PLATFORM_NAME "Android"
#elif defined(__linux__)
    #define AETHEL_OS_LINUX 1
    #define AETHEL_OS_DESKTOP 1
    #define AETHEL_PLATFORM_NAME "Linux Desktop"
#elif defined(_WIN32) || defined(_WIN64)
    #define AETHEL_OS_WINDOWS 1
    #define AETHEL_OS_DESKTOP 1
    #define AETHEL_PLATFORM_NAME "Windows"
#elif defined(__APPLE__)
    #include <TargetConditionals.h>
    #if TARGET_OS_IPHONE || TARGET_IPHONE_SIMULATOR
        #define AETHEL_OS_IOS 1
        #define AETHEL_OS_MOBILE 1
        #define AETHEL_PLATFORM_NAME "iOS"
    #else
        #define AETHEL_OS_MACOS 1
        #define AETHEL_OS_DESKTOP 1
        #define AETHEL_PLATFORM_NAME "macOS"
    #endif
#elif defined(__EMSCRIPTEN__)
    #define AETHEL_OS_WEB 1
    #define AETHEL_PLATFORM_NAME "WebAssembly / Emscripten"
#else
    #define AETHEL_OS_UNKNOWN 1
    #define AETHEL_PLATFORM_NAME "Generic POSIX / Unknown"
#endif

// ----------------------------------------------------------------------------
// 2. CPU Architecture Identification
// ----------------------------------------------------------------------------
#if defined(__x86_64__) || defined(_M_X64)
    #define AETHEL_ARCH_X86_64 1
    #define AETHEL_ARCH_NAME "x86_64"
#elif defined(__aarch64__) || defined(_M_ARM64)
    #define AETHEL_ARCH_ARM64 1
    #define AETHEL_ARCH_NAME "ARM64 (aarch64)"
#elif defined(__arm__) || defined(_M_ARM)
    #define AETHEL_ARCH_ARM32 1
    #define AETHEL_ARCH_NAME "ARM32"
#elif defined(__wasm32__) || defined(__wasm64__)
    #define AETHEL_ARCH_WASM 1
    #define AETHEL_ARCH_NAME "WebAssembly Bytecode"
#else
    #define AETHEL_ARCH_UNKNOWN 1
    #define AETHEL_ARCH_NAME "Unknown Architecture"
#endif

// ----------------------------------------------------------------------------
// 3. Hardware SIMD Vectorization Flags
// ----------------------------------------------------------------------------
#if defined(__AVX2__)
    #define AETHEL_SIMD_AVX2 1
#endif
#if defined(__AVX__)
    #define AETHEL_SIMD_AVX 1
#endif
#if defined(__SSE4_2__)
    #define AETHEL_SIMD_SSE4_2 1
#endif
#if defined(__SSE2__) || (defined(_M_IX86_FP) && _M_IX86_FP >= 2)
    #define AETHEL_SIMD_SSE2 1
#endif
#if defined(__ARM_NEON) || defined(__ARM_NEON__)
    #define AETHEL_SIMD_NEON 1
#endif
#if defined(__wasm_simd128__)
    #define AETHEL_SIMD_WASM128 1
#endif

// ----------------------------------------------------------------------------
// 4. Compiler Performance & Inlining Attributes
// ----------------------------------------------------------------------------
#if defined(_MSC_VER)
    #define AETHEL_FORCE_INLINE __forceinline
    #define AETHEL_RESTRICT __restrict
#elif defined(__GNUC__) || defined(__clang__)
    #define AETHEL_FORCE_INLINE inline __attribute__((always_inline))
    #define AETHEL_RESTRICT __restrict__
    #define AETHEL_HOT_PATH __attribute__((hot))
    #define AETHEL_COLD_PATH __attribute__((cold))
#else
    #define AETHEL_FORCE_INLINE inline
    #define AETHEL_RESTRICT
    #define AETHEL_HOT_PATH
    #define AETHEL_COLD_PATH
#endif

// Cache line size for cache-friendly zero-allocation POD structures
#ifndef AETHEL_CACHE_LINE_SIZE
    #define AETHEL_CACHE_LINE_SIZE 64
#endif

namespace audio_core::platform {

// ----------------------------------------------------------------------------
// Runtime Platform Query Helper
// ----------------------------------------------------------------------------
struct PlatformInfo {
    static constexpr const char* os_name = AETHEL_PLATFORM_NAME;
    static constexpr const char* arch_name = AETHEL_ARCH_NAME;

    [[nodiscard]] static constexpr bool is_mobile() noexcept {
#if defined(AETHEL_OS_MOBILE)
        return true;
#else
        return false;
#endif
    }

    [[nodiscard]] static constexpr bool is_desktop() noexcept {
#if defined(AETHEL_OS_DESKTOP)
        return true;
#else
        return false;
#endif
    }

    [[nodiscard]] static constexpr bool is_android() noexcept {
#if defined(AETHEL_OS_ANDROID)
        return true;
#else
        return false;
#endif
    }

    [[nodiscard]] static constexpr bool is_linux() noexcept {
#if defined(AETHEL_OS_LINUX)
        return true;
#else
        return false;
#endif
    }

    [[nodiscard]] static constexpr bool is_windows() noexcept {
#if defined(AETHEL_OS_WINDOWS)
        return true;
#else
        return false;
#endif
    }

    [[nodiscard]] static constexpr bool is_macos() noexcept {
#if defined(AETHEL_OS_MACOS)
        return true;
#else
        return false;
#endif
    }

    [[nodiscard]] static constexpr bool is_arm64() noexcept {
#if defined(AETHEL_ARCH_ARM64)
        return true;
#else
        return false;
#endif
    }

    [[nodiscard]] static constexpr bool is_x86_64() noexcept {
#if defined(AETHEL_ARCH_X86_64)
        return true;
#else
        return false;
#endif
    }

    [[nodiscard]] static constexpr bool has_simd() noexcept {
#if defined(AETHEL_SIMD_AVX2) || defined(AETHEL_SIMD_AVX) || \
    defined(AETHEL_SIMD_SSE2) || defined(AETHEL_SIMD_NEON) || \
    defined(AETHEL_SIMD_WASM128)
        return true;
#else
        return false;
#endif
    }
};

} // namespace audio_core::platform
