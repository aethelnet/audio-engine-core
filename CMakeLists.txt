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

# Core Audio Engine Library
add_library(audio_core STATIC
    src/engine.cpp
    src/wasm_host.cpp
    backends/desktop/desktop_backend.cpp
)

target_include_directories(audio_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/wasm3/source>
    $<INSTALL_INTERFACE:include>
)

find_package(Threads REQUIRED)

# Dependencies (threads, math, dl, wasm3)
target_link_libraries(audio_core PUBLIC
    wasm3_core
    Threads::Threads
    ${CMAKE_DL_LIBS}
    m
)

# Test Suite Executable
add_executable(audio_tests
    tests/test_main.cpp
)
target_link_libraries(audio_tests PRIVATE audio_core)

# Interactive CLI Synth Executable
add_executable(cli_synth
    examples/cli_synth.cpp
)
target_link_libraries(cli_synth PRIVATE audio_core)
