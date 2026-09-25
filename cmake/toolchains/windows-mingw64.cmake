# ============================================================================
# Windows x86_64 MinGW Cross-Compilation Toolchain
# Usage:
#   cmake -B build-win64 -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/windows-mingw64.cmake
# ============================================================================

set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR x86_64)

# MinGW-w64 Cross Compilers
set(CMAKE_C_COMPILER x86_64-w64-mingw32-gcc)
set(CMAKE_CXX_COMPILER x86_64-w64-mingw32-g++)
set(CMAKE_RC_COMPILER x86_64-w64-mingw32-windres)

set(CMAKE_FIND_ROOT_PATH /usr/x86_64-w64-mingw32)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# Disable Linux-specific audio backends
set(AUDIO_CORE_ENABLE_PIPEWIRE OFF CACHE BOOL "Disable PipeWire on Windows" FORCE)
set(AUDIO_CORE_ENABLE_WASAPI ON CACHE BOOL "Enable WASAPI on Windows" FORCE)
