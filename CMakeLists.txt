cmake_minimum_required(VERSION 3.20)
project(audio_engine_core VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 99)

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

# Dependencies (threads, math, dl, wasm3, pipewire)
target_link_libraries(audio_core PUBLIC
    wasm3_core
    Threads::Threads
    ${CMAKE_DL_LIBS}
    ${PIPEWIRE_LIB}
    m
)

if (ANDROID)
    target_link_libraries(audio_core PUBLIC aaudio)
endif()

# Test Suite Executable & Automated Verification Harness
enable_testing()
add_executable(audio_tests
    tests/test_main.cpp
)
target_link_libraries(audio_tests PRIVATE audio_core)

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
