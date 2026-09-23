#pragma once

#include "audio_core/types.hpp"
#include "audio_core/insert_slot.hpp"
#include "audio_core/dsp/baxandall.hpp"
#include "audio_core/dsp/buttercomp2.hpp"
#include "audio_core/dsp/purest_drive.hpp"
#include "audio_core/dsp/derez.hpp"
#include "audio_core/dsp/clip_only2.hpp"
#include "audio_core/dsp/interstage.hpp"
#include "audio_core/dsp/liquid_vactrol.hpp"
#include "audio_core/dsp/multihead_ode_compressor.hpp"

#include <string_view>
#include <memory>

namespace audio_core {

inline std::shared_ptr<IProcessor> create_processor_by_name(std::string_view name, uint32_t sample_rate = kDefaultSampleRate) {
    std::shared_ptr<IProcessor> p;
    if (name == "Baxandall" || name == "Baxandall EQ" || name == "Airwindows Baxandall EQ") {
        p = std::make_shared<dsp::Baxandall>();
    } else if (name == "ButterComp2" || name == "Airwindows ButterComp2") {
        p = std::make_shared<dsp::ButterComp2>();
    } else if (name == "PurestDrive" || name == "Airwindows PurestDrive") {
        p = std::make_shared<dsp::PurestDrive>();
    } else if (name == "DeRez" || name == "DeRez2" || name == "Airwindows DeRez2") {
        p = std::make_shared<dsp::DeRez>();
    } else if (name == "ClipOnly2" || name == "Airwindows ClipOnly2") {
        p = std::make_shared<dsp::ClipOnly2>();
    } else if (name == "Interstage" || name == "Airwindows Interstage") {
        p = std::make_shared<dsp::Interstage>();
    } else if (name == "LiquidVactrol" || name == "LiquidVactrolProcessor" || name == "Liquid Vactrol Leveler") {
        p = std::make_shared<dsp::LiquidVactrolProcessor>(sample_rate);
    } else if (name == "MultiHeadOde" || name == "MultiHeadOdeProcessor" || name == "Sovereign MultiHead ODE Compressor") {
        p = std::make_shared<dsp::MultiHeadOdeProcessor>(sample_rate);
    }

    if (p) {
        p->init(sample_rate);
    }
    return p;
}

} // namespace audio_core
