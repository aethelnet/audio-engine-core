cmake_minimum_required(VERSION 3.20)
project(audio_engine_core VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 99)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


# Compiler warnings and optimizations
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wno-unused-parameter -O3)
endif()

# Embedded Wasm3 Runtime Library
set(WASM3_SOURCES
    third_party/wasm3/source/m3_api_libc.c
    third_party/wasm3/source/m3_bind.c
    third_party/wasm3/source/m3_code.c
    third_party/wasm3/source/m3_compile.c
    third_party/wasm3/source/m3_core.c
    third_party/wasm3/source/m3_env.c
    third_party/wasm3/source/m3_exec.c
    third_party/wasm3/source/m3_function.c
    third_party/wasm3/source/m3_info.c
    third_party/wasm3/source/m3_module.c
    third_party/wasm3/source/m3_parse.c
    third_party/wasm3/source/m3_validate.c
)
add_library(wasm3_core STATIC ${WASM3_SOURCES})
target_include_directories(wasm3_core PUBLIC third_party/wasm3/source)
target_compile_definitions(wasm3_core PUBLIC d_m3HasGasMetering=1)
target_compile_options(wasm3_core PRIVATE -w) # Suppress warnings in vendored C library

# PipeWire Integration
set(PIPEWIRE_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/pipewire/usr/include/pipewire-0.3
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/pipewire/usr/include/spa-0.2
)
set(PIPEWIRE_LIB
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/pipewire/usr/lib64/libpipewire-0.3.so
)

# Ableton Link Integration
set(LINK_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/link/include
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/link/modules/asio-standalone/asio/include
)

# Core Audio Engine Library
add_library(audio_core STATIC
    src/engine.cpp
    src/wasm_host.cpp
    backends/desktop/desktop_backend.cpp
    backends/pipewire/pipewire_backend.cpp
    backends/android/aaudio_backend.cpp
)

target_include_directories(audio_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/wasm3/source>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/pipewire/usr/include/pipewire-0.3>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/pipewire/usr/include/spa-0.2>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/link/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/link/modules/asio-standalone/asio/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_definitions(audio_core PUBLIC LINK_PLATFORM_LINUX=1)
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(audio_core PUBLIC -Wno-multichar)
endif()

find_package(Threads REQUIRED)

# RTKit / systemd D-Bus Integration for Linux Real-Time Scheduling
if (UNIX AND NOT APPLE AND NOT ANDROID)
    find_package(PkgConfig QUIET)
    if (PKG_CONFIG_FOUND)
        pkg_check_modules(LIBSYSTEMD QUIET libsystemd)
    endif()

    if (LIBSYSTEMD_FOUND)
        target_link_libraries(audio_core PUBLIC ${LIBSYSTEMD_LIBRARIES})
        target_include_directories(audio_core PUBLIC ${LIBSYSTEMD_INCLUDE_DIRS})
        target_compile_definitions(audio_core PUBLIC AUDIO_CORE_HAS_SYSTEMD=1)
    else()
        find_library(SYSTEMD_LIB systemd)
        if (SYSTEMD_LIB)
            target_link_libraries(audio_core PUBLIC ${SYSTEMD_LIB})
            target_compile_definitions(audio_core PUBLIC AUDIO_CORE_HAS_SYSTEMD=1)
        endif()
    endif()
endif()

# Platform Audio Driver Options
if (UNIX AND NOT APPLE AND NOT ANDROID)
    option(AUDIO_CORE_ENABLE_PIPEWIRE "Enable Linux PipeWire pro-audio backend" ON)
else()
    option(AUDIO_CORE_ENABLE_PIPEWIRE "Enable Linux PipeWire pro-audio backend" OFF)
endif()

# Dependencies (threads, math, dl, wasm3)
target_link_libraries(audio_core PUBLIC
    wasm3_core
    Threads::Threads
    ${CMAKE_DL_LIBS}
    m
)

if (AUDIO_CORE_ENABLE_PIPEWIRE)
    target_link_libraries(audio_core PUBLIC ${PIPEWIRE_LIB})
    target_compile_definitions(audio_core PUBLIC AUDIO_CORE_HAS_PIPEWIRE=1)
endif()

if (ANDROID)
    target_sources(audio_core PRIVATE android/jni/aethel_audio_jni.cpp)
    target_link_libraries(audio_core PUBLIC aaudio log)
    target_compile_definitions(audio_core PUBLIC AUDIO_CORE_HAS_AAUDIO=1)
endif()

# Test Suite Executable & Automated Verification Harness
enable_testing()
add_executable(audio_tests
    tests/test_main.cpp
)
target_link_libraries(audio_tests PRIVATE audio_core)
target_compile_options(audio_tests PRIVATE -UNDEBUG)

add_test(NAME AudioCoreTestSuite COMMAND audio_tests)

# Automatic Post-Build Verification: Compiling automatically runs the full 20-test harness!
# If ANY regression or audio bug is introduced, the build halts with an error.
add_custom_command(TARGET audio_tests POST_BUILD
    COMMAND audio_tests
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Running audio-engine-core automated verification suite..."
)

# Interactive CLI Synth Executable
add_executable(cli_synth
    examples/cli_synth.cpp
)
target_link_libraries(cli_synth PRIVATE audio_core)

# Standalone PipeWire Desk Daemon
add_executable(pipewire_desk_daemon
    examples/pipewire_desk_daemon.cpp
)
target_link_libraries(pipewire_desk_daemon PRIVATE audio_core)

# A/B Summing Shootout & Audio Renderer
add_executable(ab_summing_shootout
    examples/ab_summing_shootout.cpp
)
target_link_libraries(ab_summing_shootout PRIVATE audio_core)

# Standalone PTPv2 Boundary Clock / Master Sync Daemon
add_executable(ptp_boundary_daemon
    examples/ptp_boundary_daemon.cpp
)
target_link_libraries(ptp_boundary_daemon PRIVATE audio_core)

# Automated Golden Master Checksum Tool & Generator
add_executable(golden_master_tool
    examples/golden_master_tool.cpp
)
target_link_libraries(golden_master_tool PRIVATE audio_core)

# Standalone WebMixer Daemon & Remote WebSocket Bridge
add_executable(webmixer_daemon
    examples/webmixer_daemon.cpp
)
target_link_libraries(webmixer_daemon PRIVATE audio_core)

# ALSA Hardware Loopback & Real-Time Engine Stress Benchmark
add_executable(alsa_hardware_stress_bench
    examples/alsa_hardware_stress_bench.cpp
)
target_link_libraries(alsa_hardware_stress_bench PRIVATE audio_core)

# ============================================================================
# Dear ImGui & Desktop UI Workstation (aethel_desk)
# ============================================================================
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

set(IMGUI_SOURCES
    third_party/imgui/imgui.cpp
    third_party/imgui/imgui_draw.cpp
    third_party/imgui/imgui_tables.cpp
    third_party/imgui/imgui_widgets.cpp
    third_party/imgui/backends/imgui_impl_glfw.cpp
    third_party/imgui/backends/imgui_impl_opengl3.cpp
)
add_library(imgui_core STATIC ${IMGUI_SOURCES})
target_include_directories(imgui_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui/backends
)
target_link_libraries(imgui_core PUBLIC
    glfw
    OpenGL::GL
)

# Desktop Workstation Executable
add_executable(aethel_desk
    examples/aethel_desk.cpp
)
target_include_directories(aethel_desk PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui/backends
)
target_link_libraries(aethel_desk PRIVATE
    audio_core
    imgui_core
    glfw
    OpenGL::GL
)

# ============================================================================
# Standalone Audio Plugins (CLAP & VST3 for Renoise / Bitwig / Reaper)
# ============================================================================
option(BUILD_STANDALONE_PLUGINS "Build Standalone Audio Plugins (CLAP & VST3)" ON)
if (BUILD_STANDALONE_PLUGINS)
    set(CLAP_WRAPPER_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(CLAP_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/clap" CACHE PATH "" FORCE)
    set(VST3_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/vst3sdk" CACHE PATH "" FORCE)
    set(CLAP_WRAPPER_DOWNLOAD_DEPENDENCIES FALSE CACHE BOOL "" FORCE)
    add_subdirectory(third_party/clap-wrapper third_party/clap-wrapper-build EXCLUDE_FROM_ALL)
    add_subdirectory(plugins/aethel_liquid_ode)
endif()



