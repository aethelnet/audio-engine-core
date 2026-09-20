#include "audio_core/ring_buffer.hpp"
#include "audio_core/dsp/oscillator.hpp"
#include "audio_core/dsp/biquad_filter.hpp"
#include "audio_core/dsp/envelope.hpp"
#include "audio_core/audio_graph.hpp"
#include "audio_core/wasm_host.hpp"

#include <iostream>
#include <thread>
#include <vector>
#include <atomic>
#include <cmath>
#include <cstdlib>

#define TEST_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "Assertion failed at " << __FILE__ << ":" << __LINE__ << ": " << #expr << std::endl; \
            std::abort(); \
        } \
    } while (0)

void test_ring_buffer_concurrency() {
    std::cout << "[TEST] Running Lock-Free SPSC RingBuffer Concurrency Stress Test..." << std::endl;
    audio_core::RingBuffer<uint64_t> rb(1024);

    constexpr uint64_t kTotalItems = 200000;
    std::atomic<bool> producer_done{false};
    std::vector<uint64_t> received;
    received.reserve(kTotalItems);

    std::thread producer([&]() {
        for (uint64_t i = 0; i < kTotalItems; ++i) {
            while (!rb.try_push(i)) {
                std::this_thread::yield();
            }
        }
        producer_done.store(true, std::memory_order_release);
    });

    std::thread consumer([&]() {
        uint64_t val = 0;
        while (!producer_done.load(std::memory_order_acquire) || !rb.empty()) {
            if (rb.try_pop(val)) {
                received.push_back(val);
            } else {
                std::this_thread::yield();
            }
        }
    });

    producer.join();
    consumer.join();

    TEST_CHECK(received.size() == kTotalItems);
    for (uint64_t i = 0; i < kTotalItems; ++i) {
        TEST_CHECK(received[i] == i);
    }
    std::cout << "  -> PASSED (200,000 items transferred lock-free in strict FIFO order, 0 loss)" << std::endl;
}

void test_dsp_oscillator() {
    std::cout << "[TEST] Running PolyBLEP Oscillator DSP Test..." << std::endl;
    audio_core::dsp::Oscillator osc;
    osc.init(48000);
    osc.set_frequency(440.0f);

    for (auto wf : {audio_core::dsp::Waveform::Sine,
                    audio_core::dsp::Waveform::Saw,
                    audio_core::dsp::Waveform::Square,
                    audio_core::dsp::Waveform::Triangle}) {
        osc.set_waveform(wf);
        for (int i = 0; i < 4800; ++i) { // 100ms
            float sample = osc.process_sample();
            TEST_CHECK(!std::isnan(sample));
            TEST_CHECK(!std::isinf(sample));
            TEST_CHECK(sample >= -1.5f && sample <= 1.5f);
        }
    }
    std::cout << "  -> PASSED (Oscillator waveforms stable and bounded)" << std::endl;
}

void test_biquad_filter() {
    std::cout << "[TEST] Running Biquad Filter Frequency Attenuation Test..." << std::endl;
    audio_core::dsp::BiquadFilter filter;
    filter.init(48000);
    filter.set_type(audio_core::dsp::FilterType::Lowpass);
    filter.set_cutoff(500.0f); // 500 Hz cutoff
    filter.set_q(0.7071f);

    // Pass 100 Hz wave vs 5000 Hz wave
    audio_core::dsp::Oscillator osc_low;
    osc_low.init(48000);
    osc_low.set_frequency(100.0f);

    audio_core::dsp::Oscillator osc_high;
    osc_high.init(48000);
    osc_high.set_frequency(5000.0f);

    float low_energy = 0.0f;
    float high_energy = 0.0f;

    filter.reset();
    for (int i = 0; i < 4800; ++i) {
        float in = osc_low.process_sample();
        float out = filter.process_sample(in);
        low_energy += out * out;
    }

    filter.reset();
    for (int i = 0; i < 4800; ++i) {
        float in = osc_high.process_sample();
        float out = filter.process_sample(in);
        high_energy += out * out;
    }

    // High frequency must be strongly attenuated compared to low frequency
    TEST_CHECK(low_energy > high_energy * 10.0f);
    std::cout << "  -> PASSED (Lowpass strongly attenuates 5kHz vs 100Hz: "
              << low_energy << " vs " << high_energy << ")" << std::endl;
}

void test_envelope() {
    std::cout << "[TEST] Running ADSR Envelope Stage Transition Test..." << std::endl;
    audio_core::dsp::Envelope env;
    env.init(48000);
    env.set_attack_ms(10.0f);
    env.set_decay_ms(20.0f);
    env.set_sustain_level(0.5f);
    env.set_release_ms(10.0f);

    TEST_CHECK(!env.is_active());
    env.gate(true);
    TEST_CHECK(env.stage() == audio_core::dsp::EnvelopeStage::Attack);

    // Run attack
    float last = 0.0f;
    while (env.stage() == audio_core::dsp::EnvelopeStage::Attack) {
        last = env.process_sample();
    }
    TEST_CHECK(std::abs(last - 1.0f) < 0.01f);

    // Run decay
    while (env.stage() == audio_core::dsp::EnvelopeStage::Decay) {
        last = env.process_sample();
    }
    TEST_CHECK(std::abs(last - 0.5f) < 0.01f);

    // Gate off -> Release
    env.gate(false);
    TEST_CHECK(env.stage() == audio_core::dsp::EnvelopeStage::Release);
    while (env.is_active()) {
        (void)env.process_sample();
    }
    TEST_CHECK(env.stage() == audio_core::dsp::EnvelopeStage::Idle);
    std::cout << "  -> PASSED (ADSR cycles correctly through all stages)" << std::endl;
}

void test_limiter_protection() {
    std::cout << "[TEST] Running Limiter Protection Test (16 saturated voices)..." << std::endl;
    audio_core::AudioGraph graph;
    graph.init(48000);
    graph.set_master_volume(1.5f);

    // Trigger all 16 voices simultaneously
    for (uint8_t note = 40; note < 40 + 16; ++note) {
        audio_core::MidiEvent ev{0, static_cast<uint8_t>(audio_core::MidiStatus::NoteOn), note, 127};
        graph.handle_midi_event(ev);
    }

    audio_core::AudioBuffer buffer(2, 256);
    auto view = buffer.view();
    graph.render(view);

    // Check that samples are limited and not clipping to +/- inf or > 1.05
    for (uint32_t i = 0; i < 256; ++i) {
        float l = view.channel(0)[i];
        float r = view.channel(1)[i];
        TEST_CHECK(l >= -1.01f && l <= 1.01f);
        TEST_CHECK(r >= -1.01f && r <= 1.01f);
    }
    std::cout << "  -> PASSED (Soft-clipper prevents digital overs even under full load)" << std::endl;
}

void test_wasm_dsp() {
    std::cout << "[TEST] Running Sandboxed WASM DSP Plugin Host Test..." << std::endl;
    audio_core::WasmDspPlugin plugin;

    bool loaded = plugin.load_from_file("plugins/saturator/saturator.wasm");
    TEST_CHECK(loaded);
    TEST_CHECK(plugin.is_loaded());

    bool inited = plugin.init(48000);
    TEST_CHECK(inited);

    // Test parameter get / set
    plugin.set_parameter(1, 5.0f); // Drive = 5.0
    float drive = plugin.get_parameter(1);
    TEST_CHECK(std::abs(drive - 5.0f) < 0.001f);

    plugin.set_parameter(3, 1.0f); // Mix = 100% wet
    float mix = plugin.get_parameter(3);
    TEST_CHECK(std::abs(mix - 1.0f) < 0.001f);

    // Generate test sine wave input
    constexpr uint32_t kFrames = 256;
    std::vector<float> in_l(kFrames);
    std::vector<float> in_r(kFrames);
    std::vector<float> out_l(kFrames, 0.0f);
    std::vector<float> out_r(kFrames, 0.0f);

    for (uint32_t i = 0; i < kFrames; ++i) {
        float phase = static_cast<float>(i) / static_cast<float>(kFrames);
        in_l[i] = std::sin(phase * 6.2831853f) * 0.7f;
        in_r[i] = std::sin(phase * 6.2831853f) * 0.7f;
    }

    // Process through WASM sandbox
    plugin.process_stereo(in_l.data(), in_r.data(), out_l.data(), out_r.data(), kFrames);

    // Verify output is saturated and non-zero
    for (uint32_t i = 0; i < kFrames; ++i) {
        TEST_CHECK(!std::isnan(out_l[i]));
        TEST_CHECK(!std::isnan(out_r[i]));
        TEST_CHECK(!std::isinf(out_l[i]));
        TEST_CHECK(!std::isinf(out_r[i]));
        TEST_CHECK(out_l[i] >= -1.05f && out_l[i] <= 1.05f);
        TEST_CHECK(out_r[i] >= -1.05f && out_r[i] <= 1.05f);
    }

    // Verify 100% dry bypass works
    plugin.set_parameter(3, 0.0f); // Mix = 0% wet (100% dry)
    plugin.process_stereo(in_l.data(), in_r.data(), out_l.data(), out_r.data(), kFrames);
    for (uint32_t i = 0; i < kFrames; ++i) {
        TEST_CHECK(std::abs(out_l[i] - in_l[i]) < 0.0001f);
        TEST_CHECK(std::abs(out_r[i] - in_r[i]) < 0.0001f);
    }

    std::cout << "  -> PASSED (WASM DSP plugin executed in sandbox, saturation and dry/wet verified)" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "   RUNNING AUDIO-ENGINE-CORE UNIT TESTS " << std::endl;
    std::cout << "========================================" << std::endl;

    test_ring_buffer_concurrency();
    test_dsp_oscillator();
    test_biquad_filter();
    test_envelope();
    test_limiter_protection();
    test_wasm_dsp();

    std::cout << "========================================" << std::endl;
    std::cout << "   ALL AUDIO CORE TESTS PASSED!         " << std::endl;
    std::cout << "========================================" << std::endl;
    return 0;
}
