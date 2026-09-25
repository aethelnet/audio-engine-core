# ============================================================================
# Linux ARM64 (aarch64) Cross-Compilation Toolchain
# Targets: Raspberry Pi 4/5, Asahi Linux, Rockchip RK3588, Odroid
# Usage:
#   cmake -B build-arm64 -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/linux-arm64.cmake
# ============================================================================

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# GNU Cross Compilers
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Optimization for ARM Cortex-A53 / A72 / A76 (Neon vectorization)
set(CMAKE_CXX_FLAGS "-march=armv8-a+crc -O3 -fPIC" CACHE STRING "" FORCE)
