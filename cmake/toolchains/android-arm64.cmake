# ============================================================================
# Android ARM64 Cross-Compilation Toolchain
# Usage:
#   cmake -B build-android \
#         -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/android-arm64.cmake \
#         -DANDROID_NDK=/path/to/android-ndk
# ============================================================================

set(CMAKE_SYSTEM_NAME Android)
set(CMAKE_SYSTEM_VERSION 26) # Android 8.0 Oreo (Minimum API for AAudio MMAP Low-Latency)
set(CMAKE_ANDROID_ARCH_ABI arm64-v8a)
set(CMAKE_ANDROID_NDK_TOOLCHAIN_VERSION clang)
set(CMAKE_ANDROID_STL_TYPE c++_static)

# Compiler optimization for mobile NEON & Cortex-A series
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -flto -fno-math-errno -fvectorize" CACHE STRING "" FORCE)
set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG -flto" CACHE STRING "" FORCE)

# Audio Core Platform Flags
set(AUDIO_CORE_ENABLE_PIPEWIRE OFF CACHE BOOL "Disable Linux Desktop PipeWire on Android" FORCE)
set(AUDIO_CORE_ENABLE_AAUDIO ON CACHE BOOL "Enable Native AAudio on Android" FORCE)
set(AUDIO_CORE_BUILD_TESTS OFF CACHE BOOL "Disable Desktop CLI tests on Android device" FORCE)
