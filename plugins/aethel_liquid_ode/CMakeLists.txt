cmake_minimum_required(VERSION 3.20)
project(aethel_liquid_ode VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(PRODUCT_NAME "Aethel Liquid ODE")


# Step 1: Implementation Static Library
add_library(aethel_liquid_ode-impl STATIC aethel_liquid_ode.cpp)

target_include_directories(aethel_liquid_ode-impl PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/clap/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/clap-wrapper/include
)

target_link_libraries(aethel_liquid_ode-impl PUBLIC clap clap-wrapper-extensions)
set_target_properties(aethel_liquid_ode-impl PROPERTIES POSITION_INDEPENDENT_CODE ON)


# Step 2: Assemble CLAP and VST3 using make_clapfirst_plugins
make_clapfirst_plugins(
    TARGET_NAME aethel_liquid_ode
    IMPL_TARGET aethel_liquid_ode-impl
    OUTPUT_NAME "${PRODUCT_NAME}"
    ENTRY_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/aethel_liquid_ode_entry.cpp"
    BUNDLE_IDENTIFIER "org.aethel.audio.liquid-ode"
    BUNDLE_VERSION ${PROJECT_VERSION}
    COPY_AFTER_BUILD FALSE
    PLUGIN_FORMATS CLAP VST3
    ASSET_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/plugins/assets"
)

# Step 3: Install target to ~/.vst3 and ~/.clap for immediate use in Renoise / Bitwig / Reaper
add_custom_target(install_aethel_plugins
    COMMAND ${CMAKE_COMMAND} -E make_directory $ENV{HOME}/.vst3
    COMMAND ${CMAKE_COMMAND} -E make_directory $ENV{HOME}/.clap
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_BINARY_DIR}/plugins/assets/${PRODUCT_NAME}.vst3"
        "$ENV{HOME}/.vst3/${PRODUCT_NAME}.vst3"
    COMMAND ${CMAKE_COMMAND} -E copy
        "${CMAKE_BINARY_DIR}/plugins/assets/${PRODUCT_NAME}.clap"
        "$ENV{HOME}/.clap/${PRODUCT_NAME}.clap"
    DEPENDS aethel_liquid_ode_all
    COMMENT "Installing Aethel Liquid ODE to ~/.vst3 and ~/.clap"
)
