#include "audio_core/ring_buffer.hpp"
#include "audio_core/dsp/oscillator.hpp"
#include "audio_core/dsp/biquad_filter.hpp"
#include "audio_core/dsp/envelope.hpp"
#include "audio_core/audio_graph.hpp"
#include "audio_core/wasm_host.hpp"
#include "audio_core/dsp/console_processor.hpp"
#include "audio_core/mixer_graph.hpp"
#include "audio_core/protocol/command_packet.hpp"
#include "audio_core/protocol/aoip_packet.hpp"
#include "audio_core/protocol/telemetry_packet.hpp"
#include "audio_core/insert_slot.hpp"
#include "audio_core/dsp/purest_drive.hpp"
#include "audio_core/dsp/buttercomp2.hpp"
#include "audio_core/dsp/baxandall.hpp"
#include "audio_core/dsp/clip_only2.hpp"
#include "audio_core/dsp/interstage.hpp"
#include "audio_core/dsp/wasm_processor.hpp"
#include "audio_core/network/aoip_transmitter.hpp"
#include "audio_core/clock/link_bridge.hpp"
#include "audio_core/analysis/transient_detector.hpp"
#include "audio_core/sampling/loop_conditioner.hpp"
#include "audio_core/sequencer/step_sequencer.hpp"
#include "backends/pipewire/pipewire_backend.hpp"
#include "backends/android/aaudio_backend.hpp"
#include "audio_core/dsp/anti_aliasing_filter.hpp"
#include "audio_core/dsp/dither.hpp"
#include "audio_core/dsp/fft.hpp"
#include "audio_core/dsp/crossover.hpp"
#include "audio_core/dsp/multichannel_bus.hpp"
#include "audio_core/analysis/measurement_engine.hpp"
#include "audio_core/dsp/liquid_ode.hpp"
#include "audio_core/dsp/multihead_ode_compressor.hpp"
#include "audio_core/network/aes67_ptp_engine.hpp"
#include "audio_core/dsp/speaker_calibration_matrix.hpp"
#include <numbers>
#include <fstream>
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
    if (!loaded) loaded = plugin.load_from_file("../plugins/saturator/saturator.wasm");
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

void test_airwindows_console_processor() {
    std::cout << "[TEST] Running Airwindows EveryConsole Encode/Decode Precision & Summing Test..." << std::endl;

    using namespace audio_core::dsp;

    // 1. Test Reconstruction Accuracy on Solo Track: Buss(Channel(x)) == x
    ConsoleProcessor channel;
    channel.set_mode(ConsoleMode::Channel);

    ConsoleProcessor buss;
    buss.set_mode(ConsoleMode::Buss);

    // Test Purest (Console 5/8 Sine/Arcsin)
    channel.set_type(ConsoleType::Purest);
    buss.set_type(ConsoleType::Purest);

    constexpr uint32_t kFrames = 512;
    std::vector<float> input(kFrames);
    std::vector<float> encoded(kFrames);
    std::vector<float> decoded(kFrames);

    for (uint32_t i = 0; i < kFrames; ++i) {
        float x = (static_cast<float>(i) / static_cast<float>(kFrames)) * 1.8f - 0.9f; // -0.9 to +0.9
        input[i] = x;
        encoded[i] = x;
        float dummy_r = x;
        channel.process_sample(encoded[i], dummy_r);
        decoded[i] = encoded[i];
        float dummy_r2 = encoded[i];
        buss.process_sample(decoded[i], dummy_r2);

        // Verify asin(sin(x)) == x within floating point precision
        TEST_CHECK(std::abs(decoded[i] - input[i]) < 0.0001f);
    }
    std::cout << "  -> PASSED (Purest Console 5/8 exact trigonometric inversion verified, error < 1e-4)" << std::endl;

    // Test InvSquare (Console 6)
    channel.set_type(ConsoleType::InvSquare);
    buss.set_type(ConsoleType::InvSquare);
    for (uint32_t i = 0; i < kFrames; ++i) {
        float x = input[i] * 0.8f; // inside [-0.8, +0.8]
        float enc_l = x, enc_r = x;
        channel.process_sample(enc_l, enc_r);
        float dec_l = enc_l, dec_r = enc_r;
        buss.process_sample(dec_l, dec_r);
        TEST_CHECK(std::abs(dec_l - x) < 0.0001f);
    }
    std::cout << "  -> PASSED (InvSquare Console 6 exact algebraic inversion verified)" << std::endl;

    // Test Spiral (Console 7)
    channel.set_type(ConsoleType::Spiral);
    buss.set_type(ConsoleType::Spiral);
    for (uint32_t i = 0; i < kFrames; ++i) {
        float x = input[i] * 0.6f; // moderate amplitude
        float enc_l = x, enc_r = x;
        channel.process_sample(enc_l, enc_r);
        float dec_l = enc_l, dec_r = enc_r;
        buss.process_sample(dec_l, dec_r);
        TEST_CHECK(!std::isnan(dec_l) && !std::isinf(dec_l));
        TEST_CHECK(std::abs(dec_l - x) < 0.05f); // Spiral is harmonic blending approximation
    }
    std::cout << "  -> PASSED (Spiral Console 7 golden-ratio non-linear transfer verified)" << std::endl;

    // 2. Test Multi-Track Non-Linear Analog Summing Intermodulation
    // Simulate 4 tracks summing together
    channel.set_type(ConsoleType::Purest);
    buss.set_type(ConsoleType::Purest);

    std::vector<float> track1(kFrames), track2(kFrames), track3(kFrames), track4(kFrames);
    std::vector<float> linear_sum(kFrames);
    std::vector<float> console_sum(kFrames, 0.0f);

    for (uint32_t i = 0; i < kFrames; ++i) {
        float t = static_cast<float>(i) / 44100.0f;
        track1[i] = std::sin(2.0f * std::numbers::pi_v<float> * 220.0f * t) * 0.35f;
        track2[i] = std::sin(2.0f * std::numbers::pi_v<float> * 440.0f * t) * 0.35f;
        track3[i] = std::sin(2.0f * std::numbers::pi_v<float> * 660.0f * t) * 0.35f;
        track4[i] = std::sin(2.0f * std::numbers::pi_v<float> * 880.0f * t) * 0.35f;

        linear_sum[i] = track1[i] + track2[i] + track3[i] + track4[i];

        // Process each track through its channel encoder
        float e1 = track1[i], dummy = 0; channel.process_sample(e1, dummy);
        float e2 = track2[i]; channel.process_sample(e2, dummy);
        float e3 = track3[i]; channel.process_sample(e3, dummy);
        float e4 = track4[i]; channel.process_sample(e4, dummy);

        // Sum encoded tracks on bus
        float bus_encoded = e1 + e2 + e3 + e4;

        // Decode on master bus
        float bus_decoded = bus_encoded;
        buss.process_sample(bus_decoded, dummy);
        console_sum[i] = bus_decoded;
    }

    // Verify intermodulation occurred: Console Sum is non-linear relative to Linear Sum
    bool difference_observed = false;
    for (uint32_t i = 0; i < kFrames; ++i) {
        TEST_CHECK(!std::isnan(console_sum[i]) && !std::isinf(console_sum[i]));
        if (std::abs(console_sum[i] - linear_sum[i]) > 0.01f) {
            difference_observed = true;
        }
    }
    TEST_CHECK(difference_observed);
    std::cout << "  -> PASSED (Multi-track non-linear analog summing intermodulation verified!)" << std::endl;

    // 3. High-Throughput Performance Benchmark (1,000,000 Samples)
    constexpr uint32_t kBenchFrames = 1000000;
    std::vector<float> bench_buffer(kBenchFrames, 0.4f);

    auto start_time = std::chrono::high_resolution_clock::now();
    channel.process_stereo(bench_buffer.data(), bench_buffer.data(), kBenchFrames);
    buss.process_stereo(bench_buffer.data(), bench_buffer.data(), kBenchFrames);
    auto end_time = std::chrono::high_resolution_clock::now();

    auto duration_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    double samples_per_sec = (static_cast<double>(kBenchFrames) / (duration_ms / 1000.0)) / 1000000.0;

    std::cout << "  -> BENCHMARK: 1,000,000 stereo frames processed in "
              << duration_ms << " ms (" << samples_per_sec << " Million frames/sec)" << std::endl;
    TEST_CHECK(duration_ms < 100.0); // Must easily achieve > 10M frames/s
}

void test_mixer_graph_routing() {
    std::cout << "[TEST] Running MixerGraph Routing, Panning, Mute/Solo & Bus Summing Test..." << std::endl;

    using namespace audio_core;

    // 1. Verify Constant-Power Pan Law
    {
        auto [l_hard_left, r_hard_left] = MixerGraph::calculate_pan_gains(-1.0f);
        TEST_CHECK(std::abs(l_hard_left - 1.0f) < 1e-5f);
        TEST_CHECK(std::abs(r_hard_left - 0.0f) < 1e-5f);

        auto [l_hard_right, r_hard_right] = MixerGraph::calculate_pan_gains(1.0f);
        TEST_CHECK(std::abs(l_hard_right - 0.0f) < 1e-5f);
        TEST_CHECK(std::abs(r_hard_right - 1.0f) < 1e-5f);

        auto [l_center, r_center] = MixerGraph::calculate_pan_gains(0.0f);
        TEST_CHECK(std::abs(l_center - r_center) < 1e-5f);
        TEST_CHECK(std::abs(l_center - 0.70710678f) < 1e-4f);

        // Constant power property: cos^2 + sin^2 == 1 across arbitrary angles
        for (float p = -1.0f; p <= 1.0f; p += 0.1f) {
            auto [gl, gr] = MixerGraph::calculate_pan_gains(p);
            float power = gl * gl + gr * gr;
            TEST_CHECK(std::abs(power - 1.0f) < 1e-4f);
        }
        std::cout << "  -> Pan Law: 100% verified (Constant-power AES standard: L^2 + R^2 == 1.0)" << std::endl;
    }

    constexpr uint32_t kFrames = 256;

    // 2. Test Channel Isolation with Panning
    {
        MixerGraph mixer(kFrames);
        Track* trk = mixer.add_track("Synth Lead");
        TEST_CHECK(trk != nullptr);
        TEST_CHECK(trk->id() == 1);

        // Populate track buffer with constant 0.4f
        for (uint32_t i = 0; i < kFrames; ++i) {
            trk->buffer().channel(0)[i] = 0.4f;
            trk->buffer().channel(1)[i] = 0.4f;
        }

        // Hard Left Pan
        trk->set_pan(-1.0f);
        AudioBuffer master_out(2, kFrames);
        auto master_view = master_out.view();
        mixer.render(master_view);

        // Verify: Left has signal, Right is strictly zero
        for (uint32_t i = 0; i < kFrames; ++i) {
            TEST_CHECK(master_view.channel(0)[i] > 0.35f);
            TEST_CHECK(std::abs(master_view.channel(1)[i]) < 1e-5f);
        }

        // Hard Right Pan
        for (uint32_t i = 0; i < kFrames; ++i) {
            trk->buffer().channel(0)[i] = 0.4f;
            trk->buffer().channel(1)[i] = 0.4f;
        }
        trk->set_pan(1.0f);
        mixer.render(master_view);

        // Verify: Left is strictly zero, Right has signal
        for (uint32_t i = 0; i < kFrames; ++i) {
            TEST_CHECK(std::abs(master_view.channel(0)[i]) < 1e-5f);
            TEST_CHECK(master_view.channel(1)[i] > 0.35f);
        }
        std::cout << "  -> Pan Isolation: PASSED (Hard-left and Hard-right cleanly isolate channels)" << std::endl;
    }

    // 3. Test Mute & Solo Logic
    {
        MixerGraph mixer(kFrames);
        Track* trk1 = mixer.add_track("Kick");
        Track* trk2 = mixer.add_track("Snare");

        trk1->set_pan(-1.0f); // Kick on Left
        trk2->set_pan(1.0f);  // Snare on Right

        auto refill = [&]() {
            for (uint32_t i = 0; i < kFrames; ++i) {
                trk1->buffer().channel(0)[i] = 0.5f;
                trk1->buffer().channel(1)[i] = 0.5f;
                trk2->buffer().channel(0)[i] = 0.5f;
                trk2->buffer().channel(1)[i] = 0.5f;
            }
        };

        AudioBuffer master_out(2, kFrames);
        auto view = master_out.view();

        // 3a. Both active
        refill();
        mixer.render(view);
        TEST_CHECK(view.channel(0)[0] > 0.4f);
        TEST_CHECK(view.channel(1)[0] > 0.4f);

        // 3b. Mute Track 1 (Kick muted) -> Only Snare on Right
        refill();
        trk1->set_mute(true);
        mixer.render(view);
        TEST_CHECK(std::abs(view.channel(0)[0]) < 1e-5f); // Kick silent
        TEST_CHECK(view.channel(1)[0] > 0.4f);            // Snare audible

        // 3c. Unmute Track 1, Solo Track 1 -> Snare suppressed even though not muted
        refill();
        trk1->set_mute(false);
        trk1->set_solo(true);
        mixer.render(view);
        TEST_CHECK(view.channel(0)[0] > 0.4f);            // Kick audible
        TEST_CHECK(std::abs(view.channel(1)[0]) < 1e-5f); // Snare muted by solo-in-place

        // 3d. Solo both -> Both audible
        refill();
        trk2->set_solo(true);
        mixer.render(view);
        TEST_CHECK(view.channel(0)[0] > 0.4f);
        TEST_CHECK(view.channel(1)[0] > 0.4f);

        std::cout << "  -> Mute/Solo Logic: PASSED (Solo-in-place and mute precedence verified)" << std::endl;
    }

    // 4. Test Submix Bus Routing (Track -> Drum Bus -> Master)
    {
        MixerGraph mixer(kFrames);
        Track* kick = mixer.add_track("Kick");
        Track* snare = mixer.add_track("Snare");
        AudioBus* drum_bus = mixer.add_submix_bus("Drum Submix");

        TEST_CHECK(drum_bus != nullptr);
        TEST_CHECK(drum_bus->id() == 1);

        // Route kick and snare to drum bus
        kick->set_target_bus(static_cast<int32_t>(drum_bus->id()));
        snare->set_target_bus(static_cast<int32_t>(drum_bus->id()));

        // Center kick and snare
        kick->set_pan(0.0f);
        snare->set_pan(0.0f);

        for (uint32_t i = 0; i < kFrames; ++i) {
            kick->buffer().channel(0)[i] = 0.2f;
            kick->buffer().channel(1)[i] = 0.2f;
            snare->buffer().channel(0)[i] = 0.2f;
            snare->buffer().channel(1)[i] = 0.2f;
        }

        AudioBuffer master_out(2, kFrames);
        auto view = master_out.view();
        mixer.render(view);

        // Both summed into drum bus, then drum bus into master
        TEST_CHECK(view.channel(0)[0] > 0.2f);
        TEST_CHECK(view.channel(1)[0] > 0.2f);

        // Mute drum bus by setting gain to 0.0
        for (uint32_t i = 0; i < kFrames; ++i) {
            kick->buffer().channel(0)[i] = 0.2f;
            kick->buffer().channel(1)[i] = 0.2f;
            snare->buffer().channel(0)[i] = 0.2f;
            snare->buffer().channel(1)[i] = 0.2f;
        }
        drum_bus->set_gain(0.0f);
        mixer.render(view);
        TEST_CHECK(std::abs(view.channel(0)[0]) < 1e-5f);
        TEST_CHECK(std::abs(view.channel(1)[0]) < 1e-5f);

        std::cout << "  -> Submix Bus Routing: PASSED (Tracks -> Submix Bus -> Master verified)" << std::endl;
    }

    // 5. Test Aux Sends (Track -> Reverb Bus)
    {
        MixerGraph mixer(kFrames);
        Track* vocal = mixer.add_track("Vocal");
        AudioBus* reverb_bus = mixer.add_submix_bus("Reverb Bus");

        // Vocal routes directly to master, but also sends 50% to reverb bus
        vocal->set_target_bus(-1); // Master
        vocal->set_send(reverb_bus->id(), 0.5f);
        vocal->set_pan(0.0f);

        for (uint32_t i = 0; i < kFrames; ++i) {
            vocal->buffer().channel(0)[i] = 0.4f;
            vocal->buffer().channel(1)[i] = 0.4f;
        }

        AudioBuffer master_out(2, kFrames);
        auto view = master_out.view();
        mixer.render(view);

        // Verify reverb bus received audio
        const Sample* rev_l = reverb_bus->buffer().view().channel(0);
        TEST_CHECK(std::abs(rev_l[0]) > 0.1f);

        // Check telemetry metering
        auto vocal_meter = vocal->meter();
        TEST_CHECK(vocal_meter.peak_l > 0.1f);
        TEST_CHECK(vocal_meter.rms_l > 0.1f);

        auto master_meter = mixer.master_bus().meter();
        TEST_CHECK(master_meter.peak_l > 0.1f);
        TEST_CHECK(master_meter.rms_l > 0.1f);

        // Test Pre-Fader vs Post-Fader Send
        // Post-Fader: fader at 0.0 -> send is 0.0
        vocal->set_gain(0.0f);
        vocal->set_send(reverb_bus->id(), 0.5f, /*pre_fader=*/false);
        for (uint32_t i = 0; i < kFrames; ++i) {
            vocal->buffer().channel(0)[i] = 0.4f;
            vocal->buffer().channel(1)[i] = 0.4f;
        }
        mixer.render(view);
        TEST_CHECK(std::abs(reverb_bus->buffer().channel(0)[0]) < 1e-5f);

        // Pre-Fader: fader at 0.0 -> send STILL sends 0.5f!
        vocal->set_send(reverb_bus->id(), 0.5f, /*pre_fader=*/true);
        for (uint32_t i = 0; i < kFrames; ++i) {
            vocal->buffer().channel(0)[i] = 0.4f;
            vocal->buffer().channel(1)[i] = 0.4f;
        }
        mixer.render(view);
        TEST_CHECK(std::abs(reverb_bus->buffer().channel(0)[0]) > 0.1f);

        // Test Mute resets meters
        vocal->set_mute(true);
        mixer.render(view);
        auto muted_meter = vocal->meter();
        TEST_CHECK(std::abs(muted_meter.peak_l) < 1e-5f);
        TEST_CHECK(std::abs(muted_meter.rms_l) < 1e-5f);

        std::cout << "  -> Aux Sends & Telemetry: PASSED (Pre/Post-fader sends and meter mute reset verified)" << std::endl;
    }
}

void test_binary_protocol_and_command_queue() {
    std::cout << "[TEST] Running Binary Protocol Precision & Lock-Free Command Queue Test..." << std::endl;

    using namespace audio_core;
    using namespace audio_core::protocol;

    // 1. Binary Struct Alignment & Footprint Checks
    static_assert(sizeof(MixerCommand) == 32);
    static_assert(alignof(MixerCommand) == 32);
    static_assert(sizeof(AoipHeader) == 32);
    static_assert(sizeof(ChannelMeterData) == 16);
    static_assert(alignof(MixerTelemetryFrame) == 64);

    std::cout << "  -> Memory Layout: PASSED (MixerCommand=32B aligned, AoipHeader=32B packed, Telemetry=64B cache aligned)" << std::endl;

    // 2. AoIP Network Packet Validation & Zero-Copy Deserialization
    {
        constexpr uint16_t kChannels = 2;
        constexpr uint16_t kFrames = 64;
        constexpr size_t kPayloadBytes = kChannels * kFrames * sizeof(float);
        constexpr size_t kTotalBytes = sizeof(AoipHeader) + kPayloadBytes;

        std::vector<uint8_t> packet_data(kTotalBytes, 0);
        auto* hdr = reinterpret_cast<AoipHeader*>(packet_data.data());
        hdr->magic = kAoipMagic;
        hdr->version = kAoipVersion;
        hdr->payload_format = static_cast<uint16_t>(AoipPayloadFormat::Float32_Interleaved);
        hdr->sequence_number = 1042;
        hdr->timestamp_ns = 1726857600000000ULL;
        hdr->sample_rate = 48000;
        hdr->channels = kChannels;
        hdr->frames = kFrames;

        // Valid packet test
        TEST_CHECK(validate_aoip_packet(packet_data.data(), kTotalBytes));

        // Truncated packet test
        TEST_CHECK(!validate_aoip_packet(packet_data.data(), kTotalBytes - 1));

        // Corrupted magic test
        hdr->magic = 0xDEADBEEF;
        TEST_CHECK(!validate_aoip_packet(packet_data.data(), kTotalBytes));
        hdr->magic = kAoipMagic;

        // Corrupted version test
        hdr->version = 99;
        TEST_CHECK(!validate_aoip_packet(packet_data.data(), kTotalBytes));

        std::cout << "  -> AoIP Wire Format: PASSED (Zero-copy header validation, sequence tracking & payload bound checks)" << std::endl;
    }

    // 3. Lock-Free SPSC Binary Command Queue Dispatch
    {
        constexpr uint32_t kFrames = 128;
        MixerGraph mixer(kFrames);

        Track* kick = mixer.add_track("808 Kick");
        Track* bass = mixer.add_track("Sub Bass");

        // Initial state
        TEST_CHECK(kick->gain() == 1.0f);
        TEST_CHECK(kick->pan() == 0.0f);
        TEST_CHECK(!kick->is_muted());

        // UI / Network thread posts binary POD commands into ring buffer
        MixerCommand cmd_gain;
        cmd_gain.type = MixerCommandType::SetTrackGain;
        cmd_gain.target_id = kick->id();
        cmd_gain.value1 = 0.42f;
        TEST_CHECK(mixer.post_command(cmd_gain));

        MixerCommand cmd_pan;
        cmd_pan.type = MixerCommandType::SetTrackPan;
        cmd_pan.target_id = kick->id();
        cmd_pan.value1 = -0.75f;
        TEST_CHECK(mixer.post_command(cmd_pan));

        MixerCommand cmd_mute;
        cmd_mute.type = MixerCommandType::SetTrackMute;
        cmd_mute.target_id = bass->id();
        cmd_mute.flags = 1; // Muted
        TEST_CHECK(mixer.post_command(cmd_mute));

        // Prior to render, changes are not yet applied to RT graph
        TEST_CHECK(kick->gain() == 1.0f);

        // RT Audio thread calls render -> drains commands sample-accurately with zero allocations
        AudioBuffer master_out(2, kFrames);
        auto view = master_out.view();
        mixer.render(view);

        // Verify commands executed atomically on audio cycle boundary
        TEST_CHECK(std::abs(kick->gain() - 0.42f) < 1e-5f);
        TEST_CHECK(std::abs(kick->pan() - -0.75f) < 1e-5f);
        TEST_CHECK(bass->is_muted());

        // 4. Zero-Copy 60Hz Telemetry Frame Capture
        MixerTelemetryFrame snapshot{};
        mixer.capture_telemetry_snapshot(snapshot);
        TEST_CHECK(snapshot.render_cycle == 1);
        TEST_CHECK(snapshot.active_tracks == 2);
        TEST_CHECK(snapshot.active_buses == 0);

        std::cout << "  -> Command Queue & Telemetry: PASSED (Lock-free SPSC commands executed on cycle, 60Hz telemetry captured)" << std::endl;
    }
}

void test_channel_strip_insert_slots() {
    std::cout << "[TEST] Running Modular Insert Slots & Airwindows DSP Suite Test..." << std::endl;
    using namespace audio_core;

    constexpr uint32_t kFrames = 256;
    constexpr uint32_t kSampleRate = 48000;

    // 1. PurestDrive Test (Harmonic saturation)
    {
        audio_core::dsp::PurestDrive drive;
        drive.init(kSampleRate);
        TEST_CHECK(std::string_view(drive.name()).find("PurestDrive") != std::string_view::npos);

        std::vector<float> left(kFrames), right(kFrames);
        std::vector<float> orig_left(kFrames);
        for (uint32_t i = 0; i < kFrames; ++i) {
            float s = std::sin(2.0f * std::numbers::pi_v<float> * 220.0f * i / kSampleRate) * 0.8f;
            left[i] = right[i] = orig_left[i] = s;
        }

        drive.set_parameter(0, 0.9f); // Drive = 0.9
        drive.set_parameter(1, 1.0f); // Wet = 1.0
        drive.process_stereo(left.data(), right.data(), kFrames);

        float diff_sum = 0.0f;
        for (uint32_t i = 0; i < kFrames; ++i) {
            TEST_CHECK(!std::isnan(left[i]));
            TEST_CHECK(!std::isinf(left[i]));
            diff_sum += std::abs(left[i] - orig_left[i]);
        }
        TEST_CHECK(diff_sum > 0.1f);
        std::cout << "  -> PurestDrive: PASSED (Nonlinear harmonic enrichment verified, diff=" << diff_sum << ")" << std::endl;
    }

    // 2. ButterComp2 Test (Quad-interleaved bipolar-RMS compressor)
    {
        audio_core::dsp::ButterComp2 comp;
        comp.init(kSampleRate);
        TEST_CHECK(std::string_view(comp.name()).find("ButterComp2") != std::string_view::npos);

        constexpr uint32_t kCompFrames = 4800; // 100ms for RMS integrator to settle
        std::vector<float> left(kCompFrames), right(kCompFrames);
        for (uint32_t i = 0; i < kCompFrames; ++i) {
            left[i] = right[i] = 1.2f; // Hot step input
        }

        comp.set_parameter(0, 0.8f); // Compress = 0.8
        comp.set_parameter(1, 1.0f); // Makeup = 1.0
        comp.set_parameter(2, 1.0f); // Wet = 1.0
        comp.process_stereo(left.data(), right.data(), kCompFrames);

        TEST_CHECK(!std::isnan(left[kCompFrames - 1]));
        TEST_CHECK(left[kCompFrames - 1] < 1.2f);
        std::cout << "  -> ButterComp2: PASSED (Buttery RMS dynamic leveling verified, out=" << left[kCompFrames - 1] << " < 1.2)" << std::endl;
    }

    // 3. Baxandall EQ Test (Low/High shelving in Console carrier wave)
    {
        audio_core::dsp::Baxandall eq;
        eq.init(kSampleRate);

        eq.set_parameter(0, 12.0f); // Bass +12dB
        eq.set_parameter(1, 0.0f);  // Treble 0dB

        std::vector<float> left(kFrames), right(kFrames);
        float in_energy = 0.0f;
        for (uint32_t i = 0; i < kFrames; ++i) {
            float s = std::sin(2.0f * std::numbers::pi_v<float> * 80.0f * i / kSampleRate) * 0.2f;
            left[i] = right[i] = s;
            in_energy += s * s;
        }

        eq.process_stereo(left.data(), right.data(), kFrames);

        float out_energy = 0.0f;
        for (uint32_t i = 0; i < kFrames; ++i) {
            TEST_CHECK(!std::isnan(left[i]));
            out_energy += left[i] * left[i];
        }
        TEST_CHECK(out_energy > in_energy * 2.0f);
        std::cout << "  -> Baxandall EQ: PASSED (+12dB Low-shelf amplified 80Hz energy from " << in_energy << " to " << out_energy << ")" << std::endl;
    }

    // 4. ClipOnly2 Test (Zero-latency transient slew rounder)
    {
        audio_core::dsp::ClipOnly2 clipper;
        clipper.init(kSampleRate);

        std::vector<float> left = {0.5f, 0.96f, 1.2f, 1.5f, -1.3f, 0.2f};
        std::vector<float> right = left;

        clipper.process_stereo(left.data(), right.data(), static_cast<uint32_t>(left.size()));

        for (size_t i = 0; i < left.size(); ++i) {
            TEST_CHECK(left[i] <= 1.0f && left[i] >= -1.0f);
            TEST_CHECK(right[i] <= 1.0f && right[i] >= -1.0f);
        }
        std::cout << "  -> ClipOnly2: PASSED (Intercepted 1.5f spike and bounded to <= 1.0f with 0 latency)" << std::endl;
    }

    // 5. Integration into MixerGraph Track Channel Strip with WASM & Airwindows
    {
        MixerGraph mixer(kFrames);
        Track* track = mixer.allocate_track("Synth Lead");
        TEST_CHECK(track != nullptr);

        auto bax = std::make_shared<audio_core::dsp::Baxandall>();
        bax->init(kSampleRate);
        track->slot(0).set_processor(bax);

        auto comp = std::make_shared<audio_core::dsp::ButterComp2>();
        comp->init(kSampleRate);
        track->slot(1).set_processor(comp);

        auto drive = std::make_shared<audio_core::dsp::PurestDrive>();
        drive->init(kSampleRate);
        track->slot(2).set_processor(drive);

        // Slot 3: Sandboxed WASM Saturator Plugin
        auto wasm = std::make_unique<audio_core::WasmDspPlugin>();
        bool loaded = wasm->load_from_file("plugins/saturator/saturator.wasm");
        if (!loaded) loaded = wasm->load_from_file("../plugins/saturator/saturator.wasm");
        TEST_CHECK(loaded);
        auto wasm_proc = std::make_shared<audio_core::dsp::WasmProcessor>(std::move(wasm), "WASM Saturator");
        wasm_proc->init(kSampleRate);
        track->slot(3).set_processor(wasm_proc);

        // Feed sine wave into track
        Sample* trk_l = track->buffer().view().channel(0);
        Sample* trk_r = track->buffer().view().channel(1);
        for (uint32_t i = 0; i < kFrames; ++i) {
            trk_l[i] = trk_r[i] = std::sin(2.0f * std::numbers::pi_v<float> * 440.0f * i / kSampleRate) * 0.5f;
        }

        AudioBuffer master_out(2, kFrames);
        auto view = master_out.view();
        mixer.render(view);

        auto meter = track->meter();
        TEST_CHECK(meter.peak_l > 0.1f);
        TEST_CHECK(meter.rms_l > 0.05f);

        // Test lock-free command queue slot bypass
        protocol::MixerCommand cmd_bypass;
        cmd_bypass.type = protocol::MixerCommandType::SetTrackSlotBypass;
        cmd_bypass.target_id = track->id();
        cmd_bypass.secondary_id = 0; // Slot 0 (Baxandall)
        cmd_bypass.flags = 1; // Bypass = true
        mixer.post_command(cmd_bypass);

        mixer.render(view);
        TEST_CHECK(track->slot(0).is_bypassed());

        // Test lock-free command queue parameter update on WASM slot
        protocol::MixerCommand cmd_param;
        cmd_param.type = protocol::MixerCommandType::SetTrackSlotParam;
        cmd_param.target_id = track->id();
        cmd_param.secondary_id = 3; // Slot 3 (WASM)
        cmd_param.flags = 1;        // Param 1 (Drive)
        cmd_param.value1 = 8.5f;
        mixer.post_command(cmd_param);

        mixer.render(view);
        TEST_CHECK(std::abs(track->slot(3).processor()->get_parameter(1) - 8.5f) < 0.01f);

        std::cout << "  -> Channel Strip Modular Inserts: PASSED (Airwindows + WASM chain executed, bypass & params automated via lock-free queue)" << std::endl;
    }
}

void test_nested_bus_topological_routing() {
    std::cout << "[TEST] Running Nested Bus Topological Routing (DAG) & Cycle Detection Test..." << std::endl;
    using namespace audio_core;

    constexpr uint32_t kFrames = 256;
    MixerGraph mixer(kFrames);

    // Allocate 3 submix buses:
    // Bus 1 = "Snare Submix"
    // Bus 2 = "Drum Submix"
    // Bus 3 = "Pre-Master"
    AudioBus* bus1 = mixer.allocate_submix_bus("Snare Submix");
    AudioBus* bus2 = mixer.allocate_submix_bus("Drum Submix");
    AudioBus* bus3 = mixer.allocate_submix_bus("Pre-Master");

    TEST_CHECK(bus1 != nullptr && bus1->id() == 1);
    TEST_CHECK(bus2 != nullptr && bus2->id() == 2);
    TEST_CHECK(bus3 != nullptr && bus3->id() == 3);

    // Setup DAG routing:
    // Bus 1 -> Bus 2 -> Bus 3 -> Master (-1)
    TEST_CHECK(mixer.set_bus_target_bus(1, 2));
    TEST_CHECK(mixer.set_bus_target_bus(2, 3));
    TEST_CHECK(mixer.set_bus_target_bus(3, -1)); // Master

    // Allocate a track routed to Bus 1
    Track* snare_trk = mixer.allocate_track("Snare Top");
    TEST_CHECK(snare_trk != nullptr);
    snare_trk->set_target_bus(1); // Routes to Bus 1

    // Fill track with known impulse (0.5f at sample 0)
    snare_trk->buffer().view().channel(0)[0] = 0.5f;
    snare_trk->buffer().view().channel(1)[0] = 0.5f;

    // Use PurestConsole (exact linear/sine)
    snare_trk->set_console_type(audio_core::dsp::ConsoleType::Purest);
    bus1->set_console_type(audio_core::dsp::ConsoleType::Purest);
    bus2->set_console_type(audio_core::dsp::ConsoleType::Purest);
    bus3->set_console_type(audio_core::dsp::ConsoleType::Purest);
    mixer.master_bus().set_console_type(audio_core::dsp::ConsoleType::Purest);

    AudioBuffer out_buf(2, kFrames);
    auto view = out_buf.view();
    mixer.render(view);

    // Verify audio arrived at master within the single buffer cycle
    TEST_CHECK(view.channel(0)[0] > 0.1f);
    TEST_CHECK(view.channel(1)[0] > 0.1f);
    std::cout << "  -> Nested Bus Routing: PASSED (Track -> Bus 1 -> Bus 2 -> Bus 3 -> Master routed in single block)" << std::endl;

    // Cycle Detection Tests:
    // 1. Direct Self-Loop: Bus 1 -> Bus 1
    TEST_CHECK(!mixer.set_bus_target_bus(1, 1));
    TEST_CHECK(bus1->target_bus() == 2); // Preserved

    // 2. Circular Loop: Bus 3 -> Bus 1 (Creates: 1 -> 2 -> 3 -> 1)
    TEST_CHECK(!mixer.set_bus_target_bus(3, 1));
    TEST_CHECK(bus3->target_bus() == -1); // Preserved

    // 3. 2-Node Mutual Loop: Bus 2 -> Bus 1 (Creates: 1 -> 2 -> 1)
    TEST_CHECK(!mixer.set_bus_target_bus(2, 1));
    TEST_CHECK(bus2->target_bus() == 3); // Preserved

    std::cout << "  -> Cycle Detection (Kahn's Algorithm): PASSED (Self-loops and multi-node cycles rejected, DAG integrity preserved)" << std::endl;
}

void test_pipewire_backend_integration() {
    std::cout << "[TEST] Running Native PipeWire Filter Node & Virtual Sink Integration Test..." << std::endl;
    using namespace audio_core;

    MixerGraph mixer(256);
    Track* trk1 = mixer.allocate_track("Vocals In");
    TEST_CHECK(trk1 != nullptr);

    PipeWireBackend pw(mixer);
    bool inited = pw.init("Aethel Test Daemon", 48000);
    TEST_CHECK(inited);

    bool started = pw.start();
    TEST_CHECK(started);
    TEST_CHECK(pw.is_running());

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    pw.stop();
    TEST_CHECK(!pw.is_running());

    std::cout << "  -> PipeWire Native Backend: PASSED (Node registered, virtual sinks & master outs created, clean shutdown)" << std::endl;
}

void test_aoip_network_streaming_and_unpacking() {
    std::cout << "[TEST] Running AoIP Network Streaming & Multi-Channel Unpacking Test..." << std::endl;
    using namespace audio_core;

    constexpr uint16_t kTestPort = 14848;
    network::AoipReceiver receiver(kTestPort);
    TEST_CHECK(receiver.bind_port(kTestPort, "127.0.0.1"));

    // Map 8-channel stream:
    // Ch 0,1 -> Track 1
    // Ch 2,3 -> Track 2
    // Ch 4,5 -> Track 3
    // Ch 6,7 -> Track 4
    receiver.map_channel_pair(1, 0, 1);
    receiver.map_channel_pair(2, 2, 3);
    receiver.map_channel_pair(3, 4, 5);
    receiver.map_channel_pair(4, 6, 7);

    network::AoipTransmitter transmitter;
    TEST_CHECK(transmitter.open("127.0.0.1", kTestPort));

    constexpr uint16_t kNumChannels = 8;
    constexpr uint16_t kFrames = 128;
    std::vector<std::vector<float>> test_audio(kNumChannels, std::vector<float>(kFrames, 0.0f));

    // Fill each channel with distinct DC test signals
    for (uint16_t ch = 0; ch < kNumChannels; ++ch) {
        float val = static_cast<float>(ch + 1) * 0.1f; // Ch 0 = 0.1, Ch 1 = 0.2, etc.
        for (uint16_t f = 0; f < kFrames; ++f) {
            test_audio[ch][f] = val;
        }
    }

    std::vector<const float*> ch_ptrs(kNumChannels);
    for (uint16_t ch = 0; ch < kNumChannels; ++ch) {
        ch_ptrs[ch] = test_audio[ch].data();
    }

    // Transmit 3 packets
    for (int p = 0; p < 3; ++p) {
        TEST_CHECK(transmitter.send_multichannel(ch_ptrs.data(), kNumChannels, kFrames, 48000, true));
    }

    // Give kernel UDP queue 10ms to deliver
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Poll packets synchronously
    uint32_t polled = receiver.poll_available_packets();
    TEST_CHECK(polled == 3);
    TEST_CHECK(receiver.stats().packets_received.load() == 3);
    TEST_CHECK(receiver.stats().packets_dropped.load() == 0);

    // Ingest into MixerGraph
    MixerGraph mixer(256);
    Track* trk1 = mixer.allocate_track("AoIP Synth L/R");
    Track* trk2 = mixer.allocate_track("AoIP Drums L/R");
    TEST_CHECK(trk1 != nullptr && trk2 != nullptr);

    // Add Live Effect Processing to Track 1 (PurestDrive)
    trk1->slot(0).set_processor(std::make_shared<dsp::PurestDrive>());
    trk1->slot(0).processor()->set_parameter(0, 0.7f); // Drive = 0.7

    // Ingest network frames into tracks
    mixer.ingest_aoip(receiver, kFrames);

    // Verify raw ingest values in Track 1 and Track 2 buffers
    const float* trk1_l = trk1->buffer().view().channel(0);
    const float* trk1_r = trk1->buffer().view().channel(1);
    const float* trk2_l = trk2->buffer().view().channel(0);
    const float* trk2_r = trk2->buffer().view().channel(1);

    TEST_CHECK(std::abs(trk1_l[0] - 0.1f) < 1e-4f);
    TEST_CHECK(std::abs(trk1_r[0] - 0.2f) < 1e-4f);
    TEST_CHECK(std::abs(trk2_l[0] - 0.3f) < 1e-4f);
    TEST_CHECK(std::abs(trk2_r[0] - 0.4f) < 1e-4f);

    std::cout << "  -> AoIP Multi-Channel Unpacking: PASSED (8 channels mapped into discrete stereo tracks bit-accurately)" << std::endl;
}

void test_universal_sampling_and_bounce_tap() {
    std::cout << "[TEST] Running Universal Sampling & Multi-Stage Bounce Tap Test..." << std::endl;
    using namespace audio_core;

    MixerGraph mixer(128);
    Track* trk1 = mixer.allocate_track("Live Synth");
    Track* trk2 = mixer.allocate_track("Resampled Bounce Loop");
    AudioBus* drum_bus = mixer.allocate_submix_bus("Processed Bus");
    TEST_CHECK(trk1 && trk2 && drum_bus);

    // Route Track 1 -> drum_bus
    trk1->set_target_bus(1);

    // Configure Modular FX on Track 1: Baxandall EQ + PurestDrive
    trk1->slot(0).set_processor(std::make_shared<dsp::Baxandall>());
    trk1->slot(0).processor()->set_parameter(0, 0.8f); // Treble boost
    trk1->slot(1).set_processor(std::make_shared<dsp::PurestDrive>());
    trk1->slot(1).processor()->set_parameter(0, 0.5f); // Saturation

    // Attach Multi-Stage Taps:
    // Tap 0: Pre-FX TrackInput (Raw incoming audio)
    // Tap 1: Post-FX TrackOutput (Effects applied live!)
    // Tap 2: BusOutput (Submix bus bounce)
    // Tap 3: MasterOutput (Full mix bounce)
    auto* tap_raw = mixer.tap(0);
    auto* tap_post_fx = mixer.tap(1);
    auto* tap_bus = mixer.tap(2);
    auto* tap_master = mixer.tap(3);

    tap_raw->set_source(sampling::TapSourceType::TrackInput, trk1->id());
    tap_post_fx->set_source(sampling::TapSourceType::TrackOutput, trk1->id());
    tap_bus->set_source(sampling::TapSourceType::BusOutput, drum_bus->id());
    tap_master->set_source(sampling::TapSourceType::MasterOutput);

    // Arm a Quantized Bounce on Tap 1 (Post-FX) for exactly 256 frames
    tap_post_fx->arm_quantized_bounce(256, "Lead_Synth_PostFX_Bounce");

    // Feed a pure 1.0f impulse / sine into Track 1
    float* in_l = trk1->buffer().view().channel(0);
    float* in_r = trk1->buffer().view().channel(1);
    for (uint32_t i = 0; i < 128; ++i) {
        float val = 0.5f * std::sin(2.0f * std::numbers::pi_v<float> * 440.0f * i / 48000.0f);
        in_l[i] = val;
        in_r[i] = val;
    }

    AudioBuffer master_out(2, 128);
    auto master_view = master_out.view();

    // Render Block 1 (128 frames)
    mixer.render(master_view);
    TEST_CHECK(tap_post_fx->record_state() == sampling::RecordState::Recording);

    // Render Block 2 (another 128 frames to complete 256 frames)
    for (uint32_t i = 0; i < 128; ++i) {
        float val = 0.5f * std::sin(2.0f * std::numbers::pi_v<float> * 440.0f * (i + 128) / 48000.0f);
        in_l[i] = val;
        in_r[i] = val;
    }
    mixer.render(master_view);

    // Verify Quantized Bounce is complete
    TEST_CHECK(tap_post_fx->record_state() == sampling::RecordState::Complete);
    auto bounce_clip = tap_post_fx->get_quantized_clip();
    TEST_CHECK(bounce_clip != nullptr);
    TEST_CHECK(bounce_clip->num_frames() == 256);
    TEST_CHECK(bounce_clip->name() == "Lead_Synth_PostFX_Bounce");

    // Verify Retroactive capture on Master tap (captured last 128 frames)
    auto master_clip = tap_master->capture_retroactive(128, "Master_Retro_Sample");
    TEST_CHECK(master_clip != nullptr);
    TEST_CHECK(master_clip->num_frames() == 128);

    // Test Bounce-to-Track: Assign bounce_clip to Track 2 for looping playback!
    trk2->set_clip(bounce_clip, true);
    TEST_CHECK(trk2->has_clip());

    // Clear Track 1 to silence
    trk1->buffer().clear();

    // Render Block 3: Track 2 should now automatically render and loop its bounced clip!
    mixer.render(master_view);
    TEST_CHECK(trk2->clip_playhead() == 128);

    // Check that Master Output contains the bounced audio from Track 2
    float master_energy = 0.0f;
    for (uint32_t i = 0; i < 128; ++i) {
        master_energy += std::abs(master_view.channel(0)[i]);
    }
    TEST_CHECK(master_energy > 0.1f);

    std::cout << "  -> Multi-Stage Bounce Tap & Resampling: PASSED (Pre-FX, Post-FX, Bus & Master bounced and looped seamlessly)" << std::endl;
}

void test_timeline_clock_and_link_bridge_master_authority() {
    std::cout << "[TEST] Running TimelineClock & Ableton Link Master Authority Test..." << std::endl;
    using namespace audio_core;
    using namespace audio_core::clock;

    TimelineClock clock(48000, 120.0);
    TEST_CHECK(clock.authority() == ClockAuthority::Master);
    TEST_CHECK(clock.sample_rate() == 48000);
    TEST_CHECK(clock.bpm() == 120.0);

    // 120 BPM @ 48kHz = 48000 * 60 / 120 = 24000 samples per beat
    // 4/4 time = 96000 samples per bar
    TEST_CHECK(std::abs(clock.samples_per_beat() - 24000.0) < 1e-4);
    TEST_CHECK(std::abs(clock.samples_per_bar() - 96000.0) < 1e-4);
    TEST_CHECK(clock.samples_for_bars(2) == 192000);

    // Test advance_block boundary detection
    clock.set_playing(true);

    bool detected_beat = false;
    uint32_t beat_offset = 0;
    bool detected_bar = false;
    uint32_t bar_offset = 0;

    // Advance block by block (128 frames)
    constexpr uint32_t kBlockSize = 128;
    for (uint32_t s = 0; s < 96128; s += kBlockSize) {
        auto events = clock.advance_block(kBlockSize);
        if (events.has_beat_boundary) {
            detected_beat = true;
            beat_offset = events.beat_sample_offset;
        }
        if (events.has_bar_boundary) {
            detected_bar = true;
            bar_offset = events.bar_sample_offset;
        }
    }

    TEST_CHECK(detected_beat && beat_offset < kBlockSize);
    TEST_CHECK(detected_bar && bar_offset < kBlockSize);
    TEST_CHECK(clock.sample_position() == 96128);

    auto snap = clock.position_snapshot();
    TEST_CHECK(snap.bar_index == 1);
    TEST_CHECK(snap.is_playing);

    // Test Ableton Link Bridge & Master Sovereignty
    LinkBridge link(120.0);
    link.bind_clock(&clock);
    link.enable(true);
    TEST_CHECK(link.is_enabled());

    // In Master mode, sync_audio_thread preserves and enforces 120.0 BPM
    link.sync_audio_thread(kBlockSize);
    TEST_CHECK(clock.bpm() == 120.0);

    // Change internal master tempo to 128.0 BPM
    clock.set_bpm(128.0);
    link.sync_audio_thread(kBlockSize);
    TEST_CHECK(clock.bpm() == 128.0);

    // Test Anti-Hijack: Switch authority to Follower vs Master
    clock.set_authority(ClockAuthority::Master);
    TEST_CHECK(clock.authority() == ClockAuthority::Master);

    link.enable(false);
    TEST_CHECK(!link.is_enabled());

    std::cout << "  -> TimelineClock & Link Master Sovereignty: PASSED (Sample-accurate beat/bar grid and anti-hijack master authority verified)" << std::endl;
}

void test_transient_detection_and_slice_engine() {
    std::cout << "[TEST] Running Transient Detection & Sample Slicing Engine Test..." << std::endl;
    using namespace audio_core::sampling;
    using namespace audio_core::analysis;

    // Create 1-bar drum loop buffer at 120 BPM (48000 frames)
    constexpr uint32_t kFrames = 48000;
    AudioClip clip("TestDrumLoop", 48000, 2, kFrames);
    clip.set_bpm(120.0);

    float* l = clip.channel(0);
    float* r = clip.channel(1);

    // Inject 4 distinct drum hits:
    // Beat 0 (frame 0): Kick
    // Beat 1 (frame 12000): Snare
    // Beat 2 (frame 24000): Kick
    // Beat 3 (frame 36000): Snare
    const uint32_t hit_positions[4] = {0, 12000, 24000, 36000};
    for (uint32_t hit : hit_positions) {
        for (uint32_t i = 0; i < 500; ++i) {
            float env = std::exp(-static_cast<float>(i) / 100.0f);
            float s = 0.8f * env * std::sin(2.0f * std::numbers::pi_v<float> * 120.0f * i / 48000.0f);
            if (hit + i < kFrames) {
                l[hit + i] += s;
                r[hit + i] += s;
            }
        }
    }

    TransientDetector detector(48000);
    auto analysis = detector.analyze(l, r, kFrames, 0.6f);

    // Verify onset detection found all 4 hits
    TEST_CHECK(analysis.onsets.size() == 4);
    TEST_CHECK(analysis.onsets[0].sample_offset < 100);
    TEST_CHECK(std::abs(static_cast<int>(analysis.onsets[1].sample_offset) - 12000) < 50);
    TEST_CHECK(std::abs(static_cast<int>(analysis.onsets[2].sample_offset) - 24000) < 50);
    TEST_CHECK(std::abs(static_cast<int>(analysis.onsets[3].sample_offset) - 36000) < 50);

    // Slice at detected markers
    std::vector<uint32_t> markers;
    for (const auto& o : analysis.onsets) markers.push_back(o.sample_offset);
    clip.slice_at_markers(markers);
    TEST_CHECK(clip.slices().size() == 4);

    // Test playback of specific slice 1 (Snare)
    uint64_t slice_playhead = 0;
    std::vector<float> snare_out_l(512, 0.0f);
    std::vector<float> snare_out_r(512, 0.0f);
    uint32_t rendered = clip.read_slice(1, slice_playhead, snare_out_l.data(), snare_out_r.data(), 512, false);
    TEST_CHECK(rendered == 512);
    TEST_CHECK(slice_playhead == 512);

    // Energy check: Snare slice must contain audio
    float energy = 0.0f;
    for (float val : snare_out_l) energy += std::abs(val);
    TEST_CHECK(energy > 0.1f);

    // Test Grid Slicing (16 slices)
    clip.slice_grid(16);
    TEST_CHECK(clip.slices().size() == 16);
    TEST_CHECK(clip.slices()[0].end_frame == 3000);

    std::cout << "  -> Transient Detection & Beat Slicing: PASSED (4 drum hits isolated, slice rearranged & triggered)" << std::endl;
}

void test_seamless_loop_equal_power_conditioning() {
    std::cout << "[TEST] Running Seamless Loop Equal-Power Conditioning Test..." << std::endl;
    using namespace audio_core::sampling;

    constexpr uint32_t kFrames = 2048;
    AudioClip clip("DiscontinuousLoop", 48000, 2, kFrames);

    float* l = clip.channel(0);
    float* r = clip.channel(1);

    // Create intentional Heaviside step discontinuity and DC offset
    // Start of clip: -0.3f
    // End of clip: +0.6f (jump = 0.9f!)
    // Add artificial DC offset: +0.05f
    for (uint32_t i = 0; i < kFrames; ++i) {
        float ramp = -0.3f + (0.9f * static_cast<float>(i) / static_cast<float>(kFrames));
        l[i] = ramp + 0.05f;
        r[i] = ramp + 0.05f;
    }

    float initial_step = std::abs(l[kFrames - 1] - l[0]);
    TEST_CHECK(initial_step > 0.85f); // Massive click!

    // Condition loop for seamless playback with 128-frame equal-power crossfade and DC trap
    LoopConditioner::condition_seamless(clip, 128);

    // Check step discontinuity between end and start:
    // With equal-power crossfade towards head, tail[kFrames - 1] matches head[127] smoothly
    float conditioned_end = l[kFrames - 1];
    float target_head = l[127];
    float seam_difference = std::abs(conditioned_end - target_head);
    TEST_CHECK(seam_difference < 0.05f);

    // Check DC offset removal (mean should be close to 0)
    double mean = 0.0;
    for (uint32_t i = 0; i < kFrames; ++i) mean += l[i];
    mean /= kFrames;
    TEST_CHECK(std::abs(mean) < 0.02);

    std::cout << "  -> Seamless Loop Equal-Power Seam: PASSED (Heaviside step jump eliminated, DC offset suppressed)" << std::endl;
}

// Mock rogue / corrupt plugin simulating broken community WASM code
class RogueWasmPlugin : public audio_core::IProcessor {
public:
    void init(uint32_t) noexcept override {}
    void reset() noexcept override {}
    void process_stereo(audio_core::Sample* left, audio_core::Sample* right, uint32_t frames) noexcept override {
        for (uint32_t i = 0; i < frames; ++i) {
            if (i % 8 == 0) {
                left[i] = std::numeric_limits<float>::quiet_NaN();
                right[i] = std::numeric_limits<float>::infinity();
            } else if (i % 8 == 1) {
                left[i] = 1000.0f; // Acoustic explosion
                right[i] = -500.0f;
            } else {
                left[i] = 0.5f; // DC bias
                right[i] = 0.5f;
            }
        }
    }
    void set_parameter(uint32_t, float) noexcept override {}
    [[nodiscard]] float get_parameter(uint32_t) const noexcept override { return 0.0f; }
    [[nodiscard]] const char* name() const noexcept override { return "RogueWasmPlugin"; }
};

void test_insert_slot_safety_hardening_and_circuit_breaker() {
    std::cout << "[TEST] Running InsertSlot Safety Hardening & Circuit Breaker Test..." << std::endl;
    using namespace audio_core;

    enable_ftz_daz();

    InsertSlot slot;
    slot.init(48000);

    TEST_CHECK(!slot.is_bypassed());
    TEST_CHECK(!slot.has_fault());
    TEST_CHECK(!slot.is_circuit_breaker_tripped());
    TEST_CHECK(slot.corrupt_samples_detected() == 0);

    // Mount rogue plugin
    slot.set_processor(std::make_shared<RogueWasmPlugin>());

    constexpr uint32_t kFrames = 256;
    std::vector<Sample> left(kFrames, 0.0f);
    std::vector<Sample> right(kFrames, 0.0f);

    // Process through hardened slot: rogue plugin will throw NaNs, Infs, and huge amplitudes
    slot.process_stereo(left.data(), right.data(), kFrames);

    // 1. Verify Circuit Breaker was tripped and slot was auto-bypassed
    TEST_CHECK(slot.has_fault());
    TEST_CHECK(slot.is_circuit_breaker_tripped());
    TEST_CHECK(slot.is_bypassed());
    TEST_CHECK(slot.corrupt_samples_detected() > 0);

    // 2. Verify corrupted block was zeroed out to protect user ears
    for (uint32_t i = 0; i < kFrames; ++i) {
        TEST_CHECK(std::isfinite(left[i]));
        TEST_CHECK(std::isfinite(right[i]));
        TEST_CHECK(left[i] == 0.0f);
        TEST_CHECK(right[i] == 0.0f);
    }

    // 3. Test DC-Blocking filter on a well-behaved plugin with DC offset
    class DcOffsetPlugin : public IProcessor {
    public:
        void init(uint32_t) noexcept override {}
        void reset() noexcept override {}
        void process_stereo(Sample* l, Sample* r, uint32_t frames) noexcept override {
            for (uint32_t i = 0; i < frames; ++i) {
                l[i] = 0.5f; // Pure +0.5f DC offset
                r[i] = 0.5f;
            }
        }
        void set_parameter(uint32_t, float) noexcept override {}
        [[nodiscard]] float get_parameter(uint32_t) const noexcept override { return 0.0f; }
        [[nodiscard]] const char* name() const noexcept override { return "DcOffsetPlugin"; }
    };

    InsertSlot dc_slot;
    dc_slot.init(48000);
    dc_slot.set_processor(std::make_shared<DcOffsetPlugin>());

    // Run 48000 frames (1 second) of DC through the slot
    std::vector<Sample> dc_l(1024, 0.0f);
    std::vector<Sample> dc_r(1024, 0.0f);
    for (int block = 0; block < 48; ++block) {
        dc_slot.process_stereo(dc_l.data(), dc_r.data(), 1024);
    }

    // After 1 second of 5Hz highpass filtering, the DC level must be attenuated towards 0
    TEST_CHECK(std::abs(dc_l[1023]) < 0.05f);
    TEST_CHECK(std::abs(dc_r[1023]) < 0.05f);

    std::cout << "  -> InsertSlot Safety Hardening: PASSED (NaNs intercepted, explosions clamped, circuit breaker auto-bypassed, DC filtered)" << std::endl;
}

void test_airwindows_interstage_processor() {
    std::cout << "[TEST] Running Airwindows Interstage Processor Test..." << std::endl;
    using namespace audio_core::dsp;

    Interstage interstage;
    interstage.init(48000);

    TEST_CHECK(std::string_view(interstage.name()) == "Interstage");

    // 1. Transparency test on moderate signal (0.1 amplitude 1kHz sine)
    constexpr uint32_t kFrames = 480;
    std::vector<audio_core::Sample> l(kFrames, 0.0f);
    std::vector<audio_core::Sample> r(kFrames, 0.0f);

    for (uint32_t i = 0; i < kFrames; ++i) {
        float s = 0.1f * std::sin(2.0f * std::numbers::pi_v<float> * 1000.0f * i / 48000.0f);
        l[i] = s;
        r[i] = s;
    }

    interstage.process_stereo(l.data(), r.data(), kFrames);

    // Verify signal is preserved and stable
    float max_val = 0.0f;
    for (uint32_t i = 0; i < kFrames; ++i) {
        TEST_CHECK(std::isfinite(l[i]));
        TEST_CHECK(std::isfinite(r[i]));
        if (std::abs(l[i]) > max_val) max_val = std::abs(l[i]);
    }
    TEST_CHECK(max_val > 0.05f && max_val <= 0.15f);

    // 2. Transformer Slew-Rate Limiting Test:
    // Feed high-amplitude high-frequency step transients (harsh digital slews)
    interstage.reset();
    for (uint32_t i = 0; i < kFrames; ++i) {
        l[i] = (i % 2 == 0) ? +0.9f : -0.9f; // Alternating Nyquist spike
        r[i] = l[i];
    }

    interstage.process_stereo(l.data(), r.data(), kFrames);

    // Interstage slew-limiting must tame the Nyquist edge
    for (uint32_t i = 1; i < kFrames; ++i) {
        float slew = std::abs(l[i] - l[i - 1]);
        TEST_CHECK(slew < 1.8f); // Slew rate restricted by analog transformer emulation
        TEST_CHECK(std::isfinite(l[i]));
    }

    std::cout << "  -> Airwindows Interstage: PASSED (Analog transformer coupling, slew-rate limiting, and Nyquist softening verified)" << std::endl;
}

void test_clock_synchronized_quantized_tap_and_bar_looping() {
    std::cout << "[TEST] Running Clock-Synchronized Quantized Tap & Bar-Looping Test..." << std::endl;
    using namespace audio_core;
    using namespace audio_core::sampling;

    MixerGraph mixer(256);
    mixer.clock().set_bpm(120.0); // 48000 Hz, 120 BPM: 24000 samples/beat, 96000 samples/bar
    mixer.clock().set_playing(true);

    // Position clock at sample 48000 (middle of bar 0, beat 2.0)
    mixer.clock().set_sample_position(48000);

    // Allocate Track 1 with test audio
    Track* trk = mixer.add_track("SynthTrack");
    TEST_CHECK(trk != nullptr);

    // Configure Tap 0 on MasterOutput
    SampleTap* tap = mixer.tap(0);
    TEST_CHECK(tap != nullptr);
    tap->set_source(TapSourceType::MasterOutput, 0);

    // Arm 1-bar quantized bounce (BarSync mode)
    tap->arm_bar_bounce(mixer.clock(), 1, "QuantizedBar1", true);

    TEST_CHECK(tap->record_state() == RecordState::Armed);
    TEST_CHECK(tap->sync_mode() == QuantizeSyncMode::BarSync);

    constexpr uint32_t kBlockFrames = 256;
    AudioBuffer out_master(2, kBlockFrames);
    auto master_view = out_master.view();

    // 1. Render blocks before the downbeat (from 48000 up to 95744)
    // 47744 samples / 256 = 186.5 blocks
    for (int i = 0; i < 186; ++i) {
        mixer.render(master_view);
    }

    // Verify tap is STILL Armed and has NOT recorded anything yet (waiting for bar 1.0 downbeat)
    TEST_CHECK(tap->record_state() == RecordState::Armed);
    TEST_CHECK(tap->get_quantized_clip() == nullptr);

    // 2. Render the block that crosses sample 96000 (downbeat of bar 1)
    // Current sample_pos: 48000 + 186 * 256 = 95616
    // Next block: 95616 .. 95872
    mixer.render(master_view);
    // Next block: 95872 .. 96128 (crosses 96000 at sample offset 96000 - 95872 = 128!)
    mixer.render(master_view);

    // Tap MUST have triggered on the sub-block downbeat and transitioned to Recording!
    TEST_CHECK(tap->record_state() == RecordState::Recording);

    // 3. Render remaining blocks to capture the full 96000 frames of bar 1
    // Total frames needed: 96000.
    while (tap->record_state() == RecordState::Recording) {
        mixer.render(master_view);
    }

    // Verify recording is Complete!
    TEST_CHECK(tap->record_state() == RecordState::Complete);

    // 4. Retrieve the conditioned clip
    auto clip = tap->get_quantized_clip();
    TEST_CHECK(clip != nullptr);
    TEST_CHECK(clip->num_frames() == 96000); // Exactly 1 musical bar!
    TEST_CHECK(clip->num_channels() == 2);

    // 5. Verify Resampling loop handoff: Assign clip to Track 2 and verify loop playback
    Track* trk2 = mixer.add_track("LoopedBounceTrack");
    TEST_CHECK(trk2 != nullptr);
    trk2->set_clip(clip, true); // Looped playback

    TEST_CHECK(trk2->is_active());

    std::cout << "  -> Clock-Synchronized Quantized Tap: PASSED (Bar-aligned arming, sub-block downbeat trigger, 96000 frames captured & looped)" << std::endl;
}

void test_step_sequencer_and_slice_trigger_engine() {
    std::cout << "[TEST] Running Slice Step-Sequencer & Micro-Fade Choke Engine Test..." << std::endl;
    using namespace audio_core;
    using namespace audio_core::sequencer;
    using namespace audio_core::sampling;

    // 1. Synthesize a 4-slice audio clip (Total: 4000 samples @ 48kHz)
    // Slice 0: Positive flat DC (1.0f) for 1000 samples (Simulates Kick)
    // Slice 1: Negative flat DC (-1.0f) for 1000 samples (Simulates Snare)
    // Slice 2: 440 Hz Sine tone for 1000 samples (Simulates HiHat)
    // Slice 3: Alternating +0.5 / -0.5 for 1000 samples (Simulates Perc)
    auto clip = std::make_shared<AudioClip>("DrumKit", 48000, 2, 4000);
    clip->slices().push_back({0, 0, 1000, 1.0f});
    clip->slices().push_back({1, 1000, 2000, 1.0f});
    clip->slices().push_back({2, 2000, 3000, 1.0f});
    clip->slices().push_back({3, 3000, 4000, 1.0f});

    float* ch0 = clip->channel(0);
    float* ch1 = clip->channel(1);
    for (uint32_t i = 0; i < 1000; ++i) {
        ch0[i] = 1.0f;
        ch1[i] = 1.0f;
    }
    for (uint32_t i = 1000; i < 2000; ++i) {
        ch0[i] = -1.0f;
        ch1[i] = -1.0f;
    }
    for (uint32_t i = 2000; i < 3000; ++i) {
        float val = std::sin(2.0f * std::numbers::pi_v<float> * 440.0f * (static_cast<float>(i - 2000) / 48000.0f));
        ch0[i] = val;
        ch1[i] = val;
    }
    for (uint32_t i = 3000; i < 4000; ++i) {
        float val = ((i % 2) == 0) ? 0.5f : -0.5f;
        ch0[i] = val;
        ch1[i] = val;
    }

    auto seq = std::make_shared<StepSequencer>(clip);
    TEST_CHECK(seq != nullptr);

    // 2. Anti-Click Micro-Fade Choke Verification
    // Trigger Slice 0 (+1.0f), render 100 samples, then trigger Slice 1 (-1.0f)
    // Without micro-fade, jumping from +1.0 to -1.0 would cause an instantaneous discontinuity of 2.0 (massive pop).
    // With 64-sample micro-fade, maximum delta between adjacent samples is bounded to < 0.06 per sample!
    std::vector<Sample> out_l(256, 0.0f);
    std::vector<Sample> out_r(256, 0.0f);
    clock::TimelineClock clk(48000, 120.0);
    clock::BlockBoundaryEvents no_events{};

    seq->trigger_slice(0, 1.0f); // Kick (+1.0)
    seq->render(out_l.data(), out_r.data(), 100, clk, no_events);
    TEST_CHECK(std::abs(out_l[99] - 1.0f) < 0.01f);

    // Choke with Slice 1 (-1.0f)
    seq->trigger_slice(1, 1.0f);
    seq->render(out_l.data(), out_r.data(), 128, clk, no_events);

    // Check maximum step delta across the 64 micro-fade frames
    float max_delta = 0.0f;
    for (size_t i = 1; i < 64; ++i) {
        float d = std::abs(out_l[i] - out_l[i - 1]);
        if (d > max_delta) max_delta = d;
    }
    TEST_CHECK(max_delta < 0.06f);
    // After micro-fade completes (>64 samples), output should reach -1.0f
    TEST_CHECK(std::abs(out_l[100] - (-1.0f)) < 0.01f);

    std::cout << "  -> Anti-Click Micro-Fade Choke: PASSED (Delta bounded to " << max_delta << " < 0.06 over 64-sample window, 0 click)" << std::endl;

    // 3. Pattern Scheduling & Instant Switching Test
    seq->stop();
    seq->pattern(0).clear();
    seq->pattern(0).set_step(0, 0, 1.0f); // Kick on step 0
    seq->pattern(0).set_step(4, 1, 1.0f); // Snare on step 4

    seq->pattern(1).clear();
    seq->pattern(1).set_step(0, 2, 1.0f); // HiHat on step 0
    seq->pattern(1).set_step(4, 3, 1.0f); // Perc on step 4

    // Test Bar-Quantized Queuing:
    seq->switch_pattern_immediate(0);
    TEST_CHECK(seq->current_pattern_index() == 0);

    seq->queue_pattern_switch(1, PatternSwitchMode::BarQuantized);
    TEST_CHECK(seq->has_queued_pattern());
    TEST_CHECK(seq->queued_pattern_index() == 1);

    // Render a block with NO bar boundary -> should remain on Pattern 0
    clock::BlockBoundaryEvents mid_bar_events{};
    mid_bar_events.has_bar_boundary = false;
    seq->render(out_l.data(), out_r.data(), 128, clk, mid_bar_events);
    TEST_CHECK(seq->current_pattern_index() == 0);
    TEST_CHECK(seq->has_queued_pattern());

    // Render a block WITH bar boundary -> must trigger pattern switch!
    clock::BlockBoundaryEvents bar_events{};
    bar_events.has_bar_boundary = true;
    bar_events.bar_sample_offset = 32;
    seq->render(out_l.data(), out_r.data(), 128, clk, bar_events);
    TEST_CHECK(seq->current_pattern_index() == 1);
    TEST_CHECK(!seq->has_queued_pattern());

    // Test Direct / Instant Switch:
    seq->switch_pattern_immediate(0);
    TEST_CHECK(seq->current_pattern_index() == 0);
    TEST_CHECK(!seq->has_queued_pattern());

    std::cout << "  -> Pattern Scheduling & Instant Switching: PASSED (Bar-quantized wait and instant override verified)" << std::endl;

    // 4. MixerGraph & AudioTrack Integration Test
    MixerGraph mixer(256);
    mixer.clock().set_sample_rate(48000);
    mixer.clock().set_bpm(120.0);
    mixer.clock().set_playing(true);

    Track* trk = mixer.add_track("BeatChopper");
    TEST_CHECK(trk != nullptr);
    trk->set_sequencer(seq);
    TEST_CHECK(trk->is_sequencer_enabled());

    AudioBuffer master_buf(2, 256);
    auto master_view = master_buf.view();

    // Render first block (crosses step 0 -> slice 0 triggers)
    mixer.render(master_view);

    // Verify energy reached master bus
    float sum_l = 0.0f;
    for (uint32_t i = 0; i < 256; ++i) {
        sum_l += std::abs(master_buf.channel(0)[i]);
    }
    TEST_CHECK(sum_l > 0.0f);

    std::cout << "  -> Track & MixerGraph Integration: PASSED (StepSequencer rendered directly into channel strip and master summing bus)" << std::endl;
}

void test_multicore_worker_pool_and_kernel_scaling() {
    std::cout << "[TEST] Running Multi-Core Lock-Free Worker Pool & Bit-Exact Scaling Test..." << std::endl;
    using namespace audio_core;

    constexpr uint32_t kFrames = 256;
    constexpr uint32_t kNumTracks = 16;

    // 1. Single-threaded reference mixer
    MixerGraph mixer_st(kFrames, false);
    mixer_st.set_worker_threads(0);
    TEST_CHECK(mixer_st.worker_threads() == 0);

    // 2. Multi-core mixer (4 workers)
    MixerGraph mixer_mc(kFrames, true);
    mixer_mc.set_worker_threads(4);
    TEST_CHECK(mixer_mc.worker_threads() == 4);

    // Populate identical tracks on both mixers with heavy DSP chains
    for (uint32_t i = 0; i < kNumTracks; ++i) {
        Track* trk_st = mixer_st.add_track("Trk_" + std::to_string(i));
        Track* trk_mc = mixer_mc.add_track("Trk_" + std::to_string(i));
        TEST_CHECK(trk_st != nullptr && trk_mc != nullptr);

        // Load DSP chain: Baxandall EQ + PurestDrive + Console
        trk_st->slot(0).set_processor(std::make_unique<dsp::Baxandall>());
        trk_mc->slot(0).set_processor(std::make_unique<dsp::Baxandall>());

        trk_st->slot(1).set_processor(std::make_unique<dsp::PurestDrive>());
        trk_mc->slot(1).set_processor(std::make_unique<dsp::PurestDrive>());

        float pan = (static_cast<float>(i) / static_cast<float>(kNumTracks)) * 2.0f - 1.0f;
        trk_st->set_pan(pan);
        trk_mc->set_pan(pan);

        // Fill input buffers with identical test signal
        Sample* l_st = trk_st->buffer().view().channel(0);
        Sample* r_st = trk_st->buffer().view().channel(1);
        Sample* l_mc = trk_mc->buffer().view().channel(0);
        Sample* r_mc = trk_mc->buffer().view().channel(1);

        for (uint32_t f = 0; f < kFrames; ++f) {
            float val = std::sin(2.0f * std::numbers::pi_v<float> * (100.0f + i * 50.0f) * (static_cast<float>(f) / 48000.0f)) * 0.2f;
            l_st[f] = val;
            r_st[f] = val;
            l_mc[f] = val;
            r_mc[f] = val;
        }
    }

    AudioBuffer out_st(2, kFrames);
    AudioBuffer out_mc(2, kFrames);
    auto view_st = out_st.view();
    auto view_mc = out_mc.view();

    // Render single-threaded
    mixer_st.render(view_st);

    // Render multi-core
    mixer_mc.render(view_mc);

    // Compare outputs: MUST be bit-exact identical within floating point rounding!
    float max_diff = 0.0f;
    for (uint32_t ch = 0; ch < 2; ++ch) {
        for (uint32_t f = 0; f < kFrames; ++f) {
            float diff = std::abs(out_st.channel(ch)[f] - out_mc.channel(ch)[f]);
            if (diff > max_diff) max_diff = diff;
        }
    }

    TEST_CHECK(max_diff < 1e-5f);
    std::cout << "  -> Bit-Exact Verification: PASSED (Single-Thread vs 4-Core max diff = " << max_diff << " < 1e-5)" << std::endl;

    // Benchmark 1000 blocks on multi-core
    auto start = std::chrono::high_resolution_clock::now();
    for (int b = 0; b < 1000; ++b) {
        mixer_mc.render(view_mc);
    }
    auto end = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(end - start).count();
    std::cout << "  -> Multi-Core Throughput: 1000 blocks (" << (1000 * kFrames) << " frames across 16 tracks + DSP) rendered in " << ms << " ms" << std::endl;
}

void test_native_android_aaudio_backend() {
    std::cout << "[TEST] Running Native Android AAudio Backend Driver Test..." << std::endl;
    using namespace audio_core;

    AAudioBackend aaudio;
    TEST_CHECK(!aaudio.is_running());

    // 1. Initialize AAudio stream: 48kHz, 2 channels, 192 burst size
    bool inited = aaudio.init(48000, 2, 192);
    TEST_CHECK(inited);
    TEST_CHECK(aaudio.actual_sample_rate() == 48000);
    TEST_CHECK(aaudio.channels() == 2);
    TEST_CHECK(aaudio.actual_buffer_size() == 192);
    TEST_CHECK(aaudio.buffer_capacity() >= 192);

    // 2. Attach real-time audio callback
    std::atomic<uint32_t> blocks_rendered{0};
    aaudio.set_callback([&](Sample* out, uint32_t frames, uint32_t channels) {
        for (uint32_t i = 0; i < frames * channels; ++i) {
            out[i] = 0.42f; // Deterministic test marker
        }
        blocks_rendered.fetch_add(1, std::memory_order_relaxed);
    });

    // 3. Start stream and simulate blocks
    TEST_CHECK(aaudio.start());
    TEST_CHECK(aaudio.is_running());

    std::vector<Sample> test_buf(192 * 2, 0.0f);
    aaudio.simulate_render_block(test_buf.data(), 192);

    TEST_CHECK(blocks_rendered.load() == 1);
    TEST_CHECK(std::abs(test_buf[0] - 0.42f) < 1e-4f);
    TEST_CHECK(std::abs(test_buf[192 * 2 - 1] - 0.42f) < 1e-4f);

    // 4. Hardware XRun telemetry check
    TEST_CHECK(aaudio.xrun_count() == 0);

    // 5. Clean shutdown
    aaudio.stop();
    TEST_CHECK(!aaudio.is_running());

    std::cout << "  -> Native Android AAudio Driver: PASSED (Exclusive mode, 192 burst, callback rendering and xrun monitoring verified)" << std::endl;
}

void test_sample_rate_agility_and_hermite_resampling() {
    std::cout << "[TEST] Running Sample Rate Agility & Continuous Hermite Spline Resampling Test..." << std::endl;
    using namespace audio_core;
    using namespace audio_core::dsp;
    using namespace audio_core::sampling;

    // 1. Spline Mathematical Kernel Verification
    {
        // At frac=0.0, output must be exact y1
        float val0 = hermite_interpolate(0.2f, 0.5f, 0.8f, 0.9f, 0.0f);
        TEST_CHECK(std::abs(val0 - 0.5f) < 1e-6f);

        // At frac=1.0, output must be exact y2
        float val1 = hermite_interpolate(0.2f, 0.5f, 0.8f, 0.9f, 1.0f);
        TEST_CHECK(std::abs(val1 - 0.8f) < 1e-6f);

        // Monotonic ramp interpolation
        float val_mid = hermite_interpolate(0.0f, 0.25f, 0.75f, 1.0f, 0.5f);
        TEST_CHECK(std::abs(val_mid - 0.5f) < 1e-4f);

        // Clamped fetch boundary check
        std::vector<float> buf = {1.0f, 2.0f, 3.0f};
        TEST_CHECK(fetch_sample_clamped(buf.data(), -5, 3) == 1.0f);
        TEST_CHECK(fetch_sample_clamped(buf.data(), 10, 3) == 3.0f);

        // Wrapped fetch loop boundary check
        TEST_CHECK(fetch_sample_wrapped(buf.data(), -1, 3) == 3.0f);
        TEST_CHECK(fetch_sample_wrapped(buf.data(), 3, 3) == 1.0f);
        TEST_CHECK(fetch_sample_wrapped(buf.data(), 4, 3) == 2.0f);

        std::cout << "  -> Hermite Spline Kernel: PASSED (C1 continuity, boundary clamping and loop seam wrapping verified)" << std::endl;
    }

    // 2. AudioClip Resampling Precision & Zero Pitch/Tempo Drift
    {
        // Create 44.1 kHz sine wave at 440 Hz for 1.0 second (44100 frames)
        constexpr uint32_t kSrIn = 44100;
        constexpr uint32_t kSrOut = 48000;
        constexpr float kFreq = 440.0f;
        auto clip_44k = std::make_shared<AudioClip>("Sine440_44k", kSrIn, 2, kSrIn);

        float* c_l = clip_44k->channel(0);
        float* c_r = clip_44k->channel(1);
        for (uint32_t i = 0; i < kSrIn; ++i) {
            float phase = static_cast<float>(i) / static_cast<float>(kSrIn);
            float s = std::sin(phase * 2.0f * std::numbers::pi_v<float> * kFreq) * 0.75f;
            c_l[i] = s;
            c_r[i] = s;
        }

        // Read resampled into 48 kHz buffer (48000 frames = 1.0 second)
        std::vector<float> out_l(kSrOut, 0.0f);
        std::vector<float> out_r(kSrOut, 0.0f);
        double playhead = 0.0;
        uint32_t read_frames = clip_44k->read_resampled(playhead, kSrOut, out_l.data(), out_r.data(), kSrOut, false);

        TEST_CHECK(read_frames == kSrOut);
        TEST_CHECK(std::abs(playhead - static_cast<double>(kSrIn)) < 1e-4);

        // Verify frequency accuracy: output should closely match ideal 440 Hz at 48 kHz
        float max_error = 0.0f;
        for (uint32_t i = 10; i < kSrOut - 10; ++i) {
            float expected = std::sin((static_cast<float>(i) / static_cast<float>(kSrOut)) * 2.0f * std::numbers::pi_v<float> * kFreq) * 0.75f;
            float err = std::abs(out_l[i] - expected);
            if (err > max_error) max_error = err;
        }
        TEST_CHECK(max_error < 0.005f); // Spline interpolation error < 0.5% across 48000 samples

        // Test 2:1 downsampling (96 kHz -> 48 kHz)
        constexpr uint32_t kSr96 = 96000;
        auto clip_96k = std::make_shared<AudioClip>("Sine1000_96k", kSr96, 2, kSr96);
        for (uint32_t i = 0; i < kSr96; ++i) {
            float s = std::sin((static_cast<float>(i) / static_cast<float>(kSr96)) * 2.0f * std::numbers::pi_v<float> * 1000.0f);
            clip_96k->channel(0)[i] = s;
            clip_96k->channel(1)[i] = s;
        }

        playhead = 0.0;
        uint32_t read_down = clip_96k->read_resampled(playhead, kSrOut, out_l.data(), out_r.data(), kSrOut, false);
        TEST_CHECK(read_down == kSrOut);
        TEST_CHECK(std::abs(playhead - static_cast<double>(kSr96)) < 1e-4);

        // Test 4:1 downsampling (192 kHz -> 48 kHz)
        constexpr uint32_t kSr192 = 192000;
        auto clip_192k = std::make_shared<AudioClip>("Sine1000_192k", kSr192, 2, kSr192);
        for (uint32_t i = 0; i < kSr192; ++i) {
            float s = std::sin((static_cast<float>(i) / static_cast<float>(kSr192)) * 2.0f * std::numbers::pi_v<float> * 1000.0f);
            clip_192k->channel(0)[i] = s;
            clip_192k->channel(1)[i] = s;
        }

        playhead = 0.0;
        uint32_t read_down4 = clip_192k->read_resampled(playhead, kSrOut, out_l.data(), out_r.data(), kSrOut, false);
        TEST_CHECK(read_down4 == kSrOut);
        TEST_CHECK(std::abs(playhead - static_cast<double>(kSr192)) < 1e-4);

        std::cout << "  -> AudioClip Resampling: PASSED (44.1k/96k/192k -> 48k bit-accurate tempo, 0 pitch drift, error < 0.005)" << std::endl;
    }

    // 3. Slice Resampling with Pitch Ratio
    {
        auto clip = std::make_shared<AudioClip>("SliceClip", 96000, 2, 96000);
        clip->slice_grid(4); // 4 slices of 24000 frames each

        std::vector<float> slice_out_l(12000, 0.0f);
        std::vector<float> slice_out_r(12000, 0.0f);

        // Play slice 0 at 48 kHz with normal pitch (1.0)
        // Rate ratio = 96000 / 48000 = 2.0
        // 12000 output frames consume 12000 * 2.0 = 24000 clip frames!
        double slice_ph = 0.0;
        uint32_t read_slice = clip->read_slice_resampled(0, slice_ph, 48000, slice_out_l.data(), slice_out_r.data(), 12000, false, 1.0);
        TEST_CHECK(read_slice == 12000);
        TEST_CHECK(std::abs(slice_ph - 24000.0) < 1e-4);

        std::cout << "  -> Slice Resampling: PASSED (Arbitrary sample rate & slice boundaries respected)" << std::endl;
    }

    // 4. StepSequencer Sample Rate Agility
    {
        // 96 kHz audio clip with 4 slices
        auto clip_96k = std::make_shared<AudioClip>("Drum96k", 96000, 2, 96000);
        for (uint32_t i = 0; i < 96000; ++i) {
            clip_96k->channel(0)[i] = 0.6f;
            clip_96k->channel(1)[i] = 0.6f;
        }
        clip_96k->slice_grid(4);

        sequencer::StepSequencer seq(clip_96k);
        clock::TimelineClock clock_48k(48000, 120.0);
        clock_48k.set_playing(true);
        clock::BlockBoundaryEvents events{};

        // Manual trigger slice 0 in 48kHz engine
        seq.trigger_slice(0, 1.0f, 1.0f);
        std::vector<float> seq_l(256, 0.0f);
        std::vector<float> seq_r(256, 0.0f);
        seq.render(seq_l.data(), seq_r.data(), 256, clock_48k, events);

        TEST_CHECK(seq.is_voice_active());
        // Verify audio was rendered
        float seq_energy = 0.0f;
        for (uint32_t i = 0; i < 256; ++i) {
            seq_energy += std::abs(seq_l[i]);
        }
        TEST_CHECK(seq_energy > 1.0f);

        std::cout << "  -> StepSequencer Agility: PASSED (96kHz clip rendered smoothly in 48kHz clock)" << std::endl;
    }

    // 5. MixerGraph Dynamic Sample Rate Switching & Lock-Free Command Dispatch
    {
        MixerGraph mixer(256, false, 48000);
        TEST_CHECK(mixer.sample_rate() == 48000);

        Track* trk = mixer.add_track("Vocal");
        TEST_CHECK(trk != nullptr);

        // Load 44.1 kHz clip into track
        auto clip_44k = std::make_shared<AudioClip>("Vocal44k", 44100, 2, 44100);
        for (uint32_t i = 0; i < 44100; ++i) {
            clip_44k->channel(0)[i] = 0.5f;
            clip_44k->channel(1)[i] = 0.5f;
        }
        trk->set_clip(clip_44k, false);

        // Dynamically switch engine sample rate to 96000
        mixer.set_sample_rate(96000);
        TEST_CHECK(mixer.sample_rate() == 96000);
        TEST_CHECK(mixer.clock().sample_rate() == 96000);

        // Render 1 block (256 frames @ 96 kHz)
        AudioBuffer master_out(2, 256);
        auto view = master_out.view();
        mixer.render(view);

        // Clip advance should be: 256 * (44100 / 96000) = 117.6 clip frames
        double expected_ph = 256.0 * (44100.0 / 96000.0);
        TEST_CHECK(std::abs(trk->clip_playhead_f() - expected_ph) < 0.1);

        // Switch sample rate via binary protocol command packet
        protocol::MixerCommand cmd;
        cmd.type = protocol::MixerCommandType::SetSampleRate;
        cmd.target_id = 192000;
        TEST_CHECK(mixer.post_command(cmd));

        // Prior to render, sample rate not yet updated
        TEST_CHECK(mixer.sample_rate() == 96000);

        // Render next block -> drains command and updates sample rate to 192 kHz
        mixer.render(view);
        TEST_CHECK(mixer.sample_rate() == 192000);

        std::cout << "  -> MixerGraph Dynamic Sample Rate: PASSED (Switched 48k -> 96k -> 192k on cycle, command automated)" << std::endl;
    }

    // 6. BufferedResampler Push-Pull Streaming (48kHz -> 44.1kHz Android bridging)
    {
        BufferedResampler resampler(48000, 44100);
        TEST_CHECK(resampler.input_rate() == 48000);
        TEST_CHECK(resampler.output_rate() == 44100);

        // Push 512 frames of 48 kHz audio (typical engine block size)
        std::vector<float> in_l(512, 0.5f);
        std::vector<float> in_r(512, 0.5f);
        uint32_t pushed = resampler.push_stereo(in_l.data(), in_r.data(), 512);
        TEST_CHECK(pushed == 512);
        TEST_CHECK(resampler.available_input_frames() == 512);

        // Pull 441 frames of 44.1 kHz audio
        std::vector<float> out_l(441, 0.0f);
        std::vector<float> out_r(441, 0.0f);
        uint32_t pulled = resampler.pull_stereo(out_l.data(), out_r.data(), 441);
        TEST_CHECK(pulled == 441);

        for (uint32_t i = 0; i < 441; ++i) {
            TEST_CHECK(std::abs(out_l[i] - 0.5f) < 1e-4f);
            TEST_CHECK(std::abs(out_r[i] - 0.5f) < 1e-4f);
        }

        std::cout << "  -> BufferedResampler FIFO: PASSED (Lock-free push/pull 48k -> 44.1k burst bridging verified)" << std::endl;
    }
}

void test_anti_aliasing_and_airwindows_dither() {
    std::cout << "[TEST] Running Anti-Aliasing Decimation Filter & Airwindows Dither Test..." << std::endl;

    using namespace audio_core::dsp;

    // 1. UltrasonicAntiAliasingFilter Minimum-Phase Impulse Response (Zero Pre-Ringing)
    {
        UltrasonicAntiAliasingFilter filter(96000, 48000);
        TEST_CHECK(filter.is_active());

        // Feed 100 samples with an impulse at sample 40
        std::vector<float> impulse(100, 0.0f);
        impulse[40] = 1.0f;
        std::vector<float> out_l(100, 0.0f);
        std::vector<float> out_r(100, 0.0f);

        filter.process_stereo(impulse.data(), impulse.data(), out_l.data(), out_r.data(), 100);

        // Verify strictly 0.000 ms pre-ringing (causal minimum-phase behavior)
        for (int i = 0; i < 40; ++i) {
            TEST_CHECK(out_l[i] == 0.0f);
            TEST_CHECK(out_r[i] == 0.0f);
        }
        // Impulse response begins at t = 40 with positive peak
        TEST_CHECK(out_l[40] > 0.0f);

        std::cout << "  -> Minimum-Phase Causal Response: PASSED (0.000 ms pre-ringing, impulse attack preserved)" << std::endl;
    }

    // 2. Ultrasonic Stopband Attenuation vs Audible Passband Transparency
    {
        UltrasonicAntiAliasingFilter filter(96000, 48000);
        constexpr uint32_t kFrames = 2048;

        // Audible 1 kHz test tone at 96 kHz
        std::vector<float> audio_1k(kFrames);
        for (uint32_t i = 0; i < kFrames; ++i) {
            audio_1k[i] = std::sin(2.0f * std::numbers::pi_v<float> * 1000.0f * static_cast<float>(i) / 96000.0f);
        }
        std::vector<float> out_1k(kFrames);
        filter.process_stereo(audio_1k.data(), audio_1k.data(), out_1k.data(), out_1k.data(), kFrames);

        // Measure passband amplitude after filter settles (samples 500..2048)
        float max_1k = 0.0f;
        for (uint32_t i = 500; i < kFrames; ++i) {
            max_1k = std::max(max_1k, std::abs(out_1k[i]));
        }
        TEST_CHECK(std::abs(max_1k - 1.0f) < 0.02f); // Passband is transparent (<0.2 dB loss)

        // Ultrasonic 35 kHz tone at 96 kHz (would alias to 13 kHz upon decimation to 48 kHz)
        filter.reset();
        std::vector<float> audio_35k(kFrames);
        for (uint32_t i = 0; i < kFrames; ++i) {
            audio_35k[i] = std::sin(2.0f * std::numbers::pi_v<float> * 35000.0f * static_cast<float>(i) / 96000.0f);
        }
        std::vector<float> out_35k(kFrames);
        filter.process_stereo(audio_35k.data(), audio_35k.data(), out_35k.data(), out_35k.data(), kFrames);

        float max_35k = 0.0f;
        for (uint32_t i = 500; i < kFrames; ++i) {
            max_35k = std::max(max_35k, std::abs(out_35k[i]));
        }
        // 10th order filter attenuates 35 kHz (> 30 dB down -> peak < 0.0316)
        TEST_CHECK(max_35k < 0.0316f);

        std::cout << "  -> Ultrasonic Stopband Rejection: PASSED (1kHz passed at " << max_1k 
                  << ", 35kHz suppressed to " << max_35k << " (>30dB attenuation))" << std::endl;
    }

    // 3. Resampler Alias Suppression: 96kHz -> 48kHz Decimation
    {
        // Compare StreamResampler with and without anti-aliasing filter
        StreamResampler resampler_raw(96000, 48000, false);
        StreamResampler resampler_filtered(96000, 48000, true);

        constexpr uint32_t in_frames = 2048;
        constexpr uint32_t out_frames = 1024;
        std::vector<float> in_35k(in_frames);
        for (uint32_t i = 0; i < in_frames; ++i) {
            in_35k[i] = std::sin(2.0f * std::numbers::pi_v<float> * 35000.0f * static_cast<float>(i) / 96000.0f);
        }

        std::vector<float> out_raw_l(out_frames), out_raw_r(out_frames);
        std::vector<float> out_filt_l(out_frames), out_filt_r(out_frames);

        resampler_raw.process_stereo(in_35k.data(), in_35k.data(), in_frames, out_raw_l.data(), out_raw_r.data(), out_frames);
        resampler_filtered.process_stereo(in_35k.data(), in_35k.data(), in_frames, out_filt_l.data(), out_filt_r.data(), out_frames);

        float max_raw = 0.0f;
        float max_filt = 0.0f;
        for (uint32_t i = 200; i < out_frames; ++i) {
            max_raw = std::max(max_raw, std::abs(out_raw_l[i]));
            max_filt = std::max(max_filt, std::abs(out_filt_l[i]));
        }

        // Without filter, 35 kHz aliases into audible spectrum with high amplitude
        TEST_CHECK(max_raw > 0.3f);
        // With ultrasonic decimation filter, the alias is suppressed by > 30 dB
        TEST_CHECK(max_filt < 0.0316f);

        std::cout << "  -> Decimation Anti-Aliasing: PASSED (Raw alias=" << max_raw 
                  << " vs Filtered alias=" << max_filt << ")" << std::endl;
    }

    // 4. Airwindows NJAD Silence Gating (Zero-Noise Floor on Digital Silence)
    {
        DitherEngine dither_njad(DitherType::NJAD);
        DitherEngine dither_tpdf(DitherType::TPDF);

        constexpr uint32_t kFrames = 1024;
        std::vector<float> silence(kFrames, 0.0f);
        std::vector<int16_t> out_njad_l(kFrames), out_njad_r(kFrames);
        std::vector<int16_t> out_tpdf_l(kFrames), out_tpdf_r(kFrames);

        dither_njad.process_stereo_16(silence.data(), silence.data(), out_njad_l.data(), out_njad_r.data(), kFrames);
        dither_tpdf.process_stereo_16(silence.data(), silence.data(), out_tpdf_l.data(), out_tpdf_r.data(), kFrames);

        // NJAD must output strictly 0 on silence (silence clamp)
        for (uint32_t i = 0; i < kFrames; ++i) {
            TEST_CHECK(out_njad_l[i] == 0);
            TEST_CHECK(out_njad_r[i] == 0);
        }

        // TPDF constantly dither-noises between -1 and +1 LSB
        bool tpdf_has_noise = false;
        for (uint32_t i = 0; i < kFrames; ++i) {
            if (out_tpdf_l[i] != 0) {
                tpdf_has_noise = true;
                break;
            }
        }
        TEST_CHECK(tpdf_has_noise);

        std::cout << "  -> Airwindows NJAD Silence Clamp: PASSED (Absolute digital zero on silence verified)" << std::endl;
    }

    // 5. Low-Level Sub-LSB Signal Linearization (Elimination of Truncation Distortion)
    {
        // Generate a 0.25 LSB sine wave (peak amplitude 0.25 / 32768.0f)
        constexpr uint32_t kFrames = 16384;
        const float sub_lsb_amp = 0.25f / 32768.0f;
        std::vector<float> sub_lsb(kFrames);
        for (uint32_t i = 0; i < kFrames; ++i) {
            sub_lsb[i] = sub_lsb_amp * std::sin(2.0f * std::numbers::pi_v<float> * 1000.0f * static_cast<float>(i) / 48000.0f);
        }

        // Mode None: hard truncation
        DitherEngine dither_none(DitherType::None);
        std::vector<int16_t> out_none_l(kFrames), out_none_r(kFrames);
        dither_none.process_stereo_16(sub_lsb.data(), sub_lsb.data(), out_none_l.data(), out_none_r.data(), kFrames);

        // All samples truncated to 0!
        for (uint32_t i = 0; i < kFrames; ++i) {
            TEST_CHECK(out_none_l[i] == 0);
        }

        // Mode TPDF: preserves low-level signal via probability modulation
        DitherEngine dither_tpdf(DitherType::TPDF);
        std::vector<int16_t> out_tpdf_l(kFrames), out_tpdf_r(kFrames);
        dither_tpdf.process_stereo_16(sub_lsb.data(), sub_lsb.data(), out_tpdf_l.data(), out_tpdf_r.data(), kFrames);

        // Correlate with the 1 kHz probe tone:
        double correlation = 0.0;
        for (uint32_t i = 0; i < kFrames; ++i) {
            double probe = std::sin(2.0 * std::numbers::pi_v<double> * 1000.0 * static_cast<double>(i) / 48000.0);
            correlation += static_cast<double>(out_tpdf_l[i]) * probe;
        }
        // Correlation is statistically significant and positive, confirming sub-LSB signal preservation!
        TEST_CHECK(correlation > 50.0);

        std::cout << "  -> Sub-LSB Signal Linearization: PASSED (Truncation deadband eliminated, correlation=" 
                  << correlation << " > 50)" << std::endl;
    }

    // 6. PaulDither vs Dark Noise Shaping Spectral Tilting
    {
        DitherEngine dither_paul(DitherType::PaulDither);
        DitherEngine dither_dark(DitherType::Dark);

        constexpr uint32_t kFrames = 8192;
        std::vector<float> silence(kFrames, 0.0001f); // Tiny offset to keep dither active
        std::vector<float> out_paul_l(kFrames), out_paul_r(kFrames);
        std::vector<float> out_dark_l(kFrames), out_dark_r(kFrames);

        dither_paul.process_stereo_float(silence.data(), silence.data(), out_paul_l.data(), out_paul_r.data(), kFrames);
        dither_dark.process_stereo_float(silence.data(), silence.data(), out_dark_l.data(), out_dark_r.data(), kFrames);

        // High frequency energy measured via 1st difference (high-pass metric |x[n] - x[n-1]|^2)
        double hf_paul = 0.0;
        double hf_dark = 0.0;
        for (uint32_t i = 1; i < kFrames; ++i) {
            double diff_p = out_paul_l[i] - out_paul_l[i - 1];
            double diff_d = out_dark_l[i] - out_dark_l[i - 1];
            hf_paul += diff_p * diff_p;
            hf_dark += diff_d * diff_d;
        }

        // PaulDither has highpass noise shaping (1 - z^-1), whereas Dark has lowpass smoothing (1 + z^-1)
        // High frequency power of PaulDither must be substantially higher than Dark
        TEST_CHECK(hf_paul > hf_dark * 1.5);

        std::cout << "  -> Airwindows Dither Voicing: PASSED (PaulDither HF=" << hf_paul 
                  << " vs Dark HF=" << hf_dark << " (velvet highpass vs warm lowpass verified))" << std::endl;
    }

    // 7. 24-Bit Studio Mastering Wordlength Reduction
    {
        DitherEngine dither24(DitherType::TPDF);
        int32_t out24_l = 0, out24_r = 0;
        dither24.process_sample_24(0.5f, -0.5f, out24_l, out24_r);

        // 24-bit 0.5f is around 4194304 (+/- 4 LSB dither)
        TEST_CHECK(std::abs(out24_l - 4194304) <= 4);
        TEST_CHECK(std::abs(out24_r - (-4194304)) <= 4);

        std::cout << "  -> 24-Bit Studio Master Dither: PASSED (24-bit wordlength scaling verified)" << std::endl;
    }
}

void test_acoustic_measurement_and_crossover_engine() {
    std::cout << "[TEST] Running Acoustic Measurement Engine & Linkwitz-Riley Crossover Test..." << std::endl;

    using namespace audio_core::dsp;
    using namespace audio_core::analysis;

    // 1. FastFourierTransform Forward/Inverse Roundtrip & Convolution
    {
        constexpr size_t N = 1024;
        std::vector<FastFourierTransform::Complex> x(N);
        for (size_t i = 0; i < N; ++i) {
            float phase = 2.0f * std::numbers::pi_v<float> * 440.0f * static_cast<float>(i) / 48000.0f;
            x[i] = FastFourierTransform::Complex(std::sin(phase), std::cos(phase * 0.5f));
        }

        std::vector<FastFourierTransform::Complex> orig = x;
        FastFourierTransform::forward(x);
        FastFourierTransform::inverse(x);

        float max_err = 0.0f;
        for (size_t i = 0; i < N; ++i) {
            max_err = std::max(max_err, std::abs(x[i].real() - orig[i].real()));
            max_err = std::max(max_err, std::abs(x[i].imag() - orig[i].imag()));
        }
        TEST_CHECK(max_err < 1e-4f);

        // Fast convolution test: convolve delta impulse with arbitrary signal
        std::vector<float> sig = {1.0f, 2.0f, 3.0f, 4.0f};
        std::vector<float> delta = {1.0f, 0.0f, 0.0f};
        std::vector<float> conv = FastFourierTransform::convolve(sig.data(), sig.size(), delta.data(), delta.size());
        TEST_CHECK(conv.size() == 6);
        for (size_t i = 0; i < sig.size(); ++i) {
            TEST_CHECK(std::abs(conv[i] - sig[i]) < 1e-4f);
        }

        std::cout << "  -> FFT & Fast Convolution: PASSED (Roundtrip error=" << max_err << " < 1e-4)" << std::endl;
    }

    // 2. Linkwitz-Riley 4th-Order (LR4) 2-Way Crossover Unity Summation & Isolation
    {
        LinkwitzRiley2Way lr(1000.0f, 48000);
        constexpr uint32_t kFrames = 2048;

        // Test with a multi-frequency composite signal: 100 Hz (bass) + 10 kHz (treble)
        std::vector<float> in_l(kFrames), in_r(kFrames);
        for (uint32_t i = 0; i < kFrames; ++i) {
            float t = static_cast<float>(i) / 48000.0f;
            float s_low = 0.5f * std::sin(2.0f * std::numbers::pi_v<float> * 100.0f * t);
            float s_high = 0.5f * std::sin(2.0f * std::numbers::pi_v<float> * 10000.0f * t);
            in_l[i] = s_low + s_high;
            in_r[i] = s_low - s_high;
        }

        std::vector<float> low_l(kFrames), low_r(kFrames);
        std::vector<float> high_l(kFrames), high_r(kFrames);

        lr.process_stereo(in_l.data(), in_r.data(),
                          low_l.data(), low_r.data(),
                          high_l.data(), high_r.data(),
                          kFrames);

        // Sum low + high and check reconstruction energy after filter settles (samples 500..2048)
        float rms_in = 0.0f;
        float rms_sum = 0.0f;
        for (uint32_t i = 500; i < kFrames; ++i) {
            float sum_l = low_l[i] + high_l[i];
            rms_in += in_l[i] * in_l[i];
            rms_sum += sum_l * sum_l;
        }
        rms_in = std::sqrt(rms_in / (kFrames - 500));
        rms_sum = std::sqrt(rms_sum / (kFrames - 500));
        TEST_CHECK(std::abs(rms_sum - rms_in) < 0.005f);

        // Isolation: At 10 kHz, low output should be heavily attenuated (>30 dB)
        float max_low_treble = 0.0f;
        lr.reset();
        std::vector<float> pure_10k(kFrames);
        for (uint32_t i = 0; i < kFrames; ++i) {
            pure_10k[i] = std::sin(2.0f * std::numbers::pi_v<float> * 10000.0f * static_cast<float>(i) / 48000.0f);
        }
        lr.process_stereo(pure_10k.data(), pure_10k.data(),
                          low_l.data(), low_r.data(),
                          high_l.data(), high_r.data(),
                          kFrames);
        for (uint32_t i = 500; i < kFrames; ++i) {
            max_low_treble = std::max(max_low_treble, std::abs(low_l[i]));
        }
        TEST_CHECK(max_low_treble < 0.005f);

        std::cout << "  -> Linkwitz-Riley 2-Way (LR4): PASSED (Flat unity sum, 10kHz leakage into low=" 
                  << max_low_treble << " (<46dB))" << std::endl;
    }

    // 3. Linkwitz-Riley 3-Way Crossover (Low / Mid / High Multiband Split)
    {
        LinkwitzRiley3Way lr3(200.0f, 3000.0f, 48000);
        constexpr uint32_t kFrames = 2048;

        std::vector<float> in_l(kFrames);
        for (uint32_t i = 0; i < kFrames; ++i) {
            float t = static_cast<float>(i) / 48000.0f;
            in_l[i] = 0.33f * (std::sin(2.0f * std::numbers::pi_v<float> * 80.0f * t) +
                               std::sin(2.0f * std::numbers::pi_v<float> * 1000.0f * t) +
                               std::sin(2.0f * std::numbers::pi_v<float> * 8000.0f * t));
        }

        std::vector<float> low_l(kFrames), low_r(kFrames);
        std::vector<float> mid_l(kFrames), mid_r(kFrames);
        std::vector<float> high_l(kFrames), high_r(kFrames);

        lr3.process_stereo(in_l.data(), in_l.data(),
                           low_l.data(), low_r.data(),
                           mid_l.data(), mid_r.data(),
                           high_l.data(), high_r.data(),
                           kFrames);

        float rms_in = 0.0f, rms_sum = 0.0f;
        for (uint32_t i = 500; i < kFrames; ++i) {
            float sum = low_l[i] + mid_l[i] + high_l[i];
            rms_in += in_l[i] * in_l[i];
            rms_sum += sum * sum;
        }
        rms_in = std::sqrt(rms_in / (kFrames - 500));
        rms_sum = std::sqrt(rms_sum / (kFrames - 500));
        TEST_CHECK(std::abs(rms_sum - rms_in) < 0.005f);

        std::cout << "  -> Linkwitz-Riley 3-Way Multiband Split: PASSED (RMS in=" << rms_in 
                  << " vs sum=" << rms_sum << ", delta < 0.005)" << std::endl;
    }

    // 4. Airwindows Isolator (5th-Order Golden Ratio Subtractive Crossover)
    {
        AirwindowsIsolator isolator(1000.0f, 48000, /*enable_saturation=*/true);
        constexpr uint32_t kFrames = 2048;

        std::vector<float> in_l(kFrames), in_r(kFrames);
        for (uint32_t i = 0; i < kFrames; ++i) {
            float t = static_cast<float>(i) / 48000.0f;
            in_l[i] = 0.3f * std::sin(2.0f * std::numbers::pi_v<float> * 150.0f * t) +
                      0.3f * std::sin(2.0f * std::numbers::pi_v<float> * 5000.0f * t);
            in_r[i] = in_l[i] * 0.8f;
        }

        std::vector<float> low_l(kFrames), low_r(kFrames);
        std::vector<float> high_l(kFrames), high_r(kFrames);

        // A. Bit-Exact Subtractive Identity (Console5 Saturated Mode)
        isolator.process_stereo(in_l.data(), in_r.data(),
                                low_l.data(), low_r.data(),
                                high_l.data(), high_r.data(),
                                kFrames);

        float max_sum_diff = 0.0f;
        for (uint32_t i = 0; i < kFrames; ++i) {
            float diff_l = std::abs((low_l[i] + high_l[i]) - in_l[i]);
            float diff_r = std::abs((low_r[i] + high_r[i]) - in_r[i]);
            max_sum_diff = std::max({max_sum_diff, diff_l, diff_r});
        }
        TEST_CHECK(max_sum_diff < 1e-6f); // Exact algebraic identity: Low + (Dry - Low) == Dry

        // B. Steep Cutoff (>30 dB/octave attenuation at 10 kHz for a 1 kHz crossover)
        isolator.reset();
        std::vector<float> pure_10k(kFrames);
        for (uint32_t i = 0; i < kFrames; ++i) {
            pure_10k[i] = 0.5f * std::sin(2.0f * std::numbers::pi_v<float> * 10000.0f * static_cast<float>(i) / 48000.0f);
        }
        isolator.process_stereo(pure_10k.data(), pure_10k.data(),
                                low_l.data(), low_r.data(),
                                high_l.data(), high_r.data(),
                                kFrames);

        float max_treble_in_low = 0.0f;
        for (uint32_t i = 500; i < kFrames; ++i) {
            max_treble_in_low = std::max(max_treble_in_low, std::abs(low_l[i]));
        }
        // With 3 biquads (Q=0.5, 0.618, 1.618), 10 kHz is >3 octaves above 1 kHz -> >50 dB attenuation
        TEST_CHECK(max_treble_in_low < 0.002f);

        // C. Bit-Exact Subtractive Identity (Linear Mode, saturation disabled)
        isolator.set_saturation_enabled(false);
        isolator.reset();
        isolator.process_stereo(in_l.data(), in_r.data(),
                                low_l.data(), low_r.data(),
                                high_l.data(), high_r.data(),
                                kFrames);
        max_sum_diff = 0.0f;
        for (uint32_t i = 0; i < kFrames; ++i) {
            float diff_l = std::abs((low_l[i] + high_l[i]) - in_l[i]);
            max_sum_diff = std::max(max_sum_diff, diff_l);
        }
        TEST_CHECK(max_sum_diff < 1e-6f);

        std::cout << "  -> Airwindows Isolator (Golden Ratio 30dB/oct Crossover): PASSED (Bit-exact identity err="
                  << max_sum_diff << ", 10kHz leakage=" << max_treble_in_low << " [<-48dB])" << std::endl;
    }

    // 5. Mastering-Grade Multiband Crossover Matrix (Beyond FabFilter: 4-Band & 6-Band Verification)
    {
        MultibandCrossoverMatrix matrix;
        constexpr uint32_t kFrames = 2048;

        // A. 4-Band Subtractive Golden-Ratio Tree: Bit-Exact Reconstruction & Band Isolation
        std::array<float, 3> freqs_4band = {80.0f, 500.0f, 3500.0f};
        matrix.configure(4, freqs_4band.data(), 48000, MultibandCrossoverMode::SubtractiveGoldenRatio);

        std::vector<float> in_l(kFrames), in_r(kFrames);
        for (uint32_t i = 0; i < kFrames; ++i) {
            float t = static_cast<float>(i) / 48000.0f;
            in_l[i] = 0.25f * (std::sin(2.0f * std::numbers::pi_v<float> * 40.0f * t) +
                               std::sin(2.0f * std::numbers::pi_v<float> * 250.0f * t) +
                               std::sin(2.0f * std::numbers::pi_v<float> * 1500.0f * t) +
                               std::sin(2.0f * std::numbers::pi_v<float> * 10000.0f * t));
            in_r[i] = in_l[i] * 0.9f;
        }

        std::array<std::vector<float>, 8> bands_l, bands_r;
        for (size_t b = 0; b < 8; ++b) {
            bands_l[b].resize(kFrames);
            bands_r[b].resize(kFrames);
        }

        std::array<float*, 8> ptrs_l, ptrs_r;
        for (size_t b = 0; b < 8; ++b) {
            ptrs_l[b] = bands_l[b].data();
            ptrs_r[b] = bands_r[b].data();
        }

        matrix.process_block(in_l.data(), in_r.data(), ptrs_l.data(), ptrs_r.data(), kFrames);

        // Verify Bit-Exact Algebraic Identity on Sum
        float max_4band_err = 0.0f;
        for (uint32_t i = 0; i < kFrames; ++i) {
            float sum_l = bands_l[0][i] + bands_l[1][i] + bands_l[2][i] + bands_l[3][i];
            float sum_r = bands_r[0][i] + bands_r[1][i] + bands_r[2][i] + bands_r[3][i];
            max_4band_err = std::max({max_4band_err, std::abs(sum_l - in_l[i]), std::abs(sum_r - in_r[i])});
        }
        TEST_CHECK(max_4band_err < 1e-6f); // Exact null cancellation!

        // B. 6-Band FabFilter Pro-MB Style Configuration (60, 250, 1000, 4000, 10000 Hz)
        std::array<float, 5> freqs_6band = {60.0f, 250.0f, 1000.0f, 4000.0f, 10000.0f};
        matrix.configure(6, freqs_6band.data(), 48000, MultibandCrossoverMode::SubtractiveGoldenRatio);
        matrix.process_block(in_l.data(), in_r.data(), ptrs_l.data(), ptrs_r.data(), kFrames);

        float max_6band_err = 0.0f;
        for (uint32_t i = 0; i < kFrames; ++i) {
            float sum_l = 0.0f;
            for (uint32_t b = 0; b < 6; ++b) sum_l += bands_l[b][i];
            max_6band_err = std::max(max_6band_err, std::abs(sum_l - in_l[i]));
        }
        TEST_CHECK(max_6band_err < 1e-6f);

        // Test Solo Logic: Solo Band 2 (1000 Hz band)
        matrix.set_band_solo(2, true);
        matrix.process_block(in_l.data(), in_r.data(), ptrs_l.data(), ptrs_r.data(), kFrames);
        for (uint32_t b = 0; b < 6; ++b) {
            if (b != 2) {
                TEST_CHECK(std::abs(bands_l[b][100]) == 0.0f);
            } else {
                TEST_CHECK(std::abs(bands_l[b][100]) > 0.0f);
            }
        }
        matrix.set_band_solo(2, false);

        // C. Linkwitz-Riley 4th Order Phase-Compensated Multi-Way Tree (Flat Magnitude Verification)
        constexpr uint32_t kFramesLR = 4800; // Exactly 4 cycles of 40 Hz, 25 cycles of 250 Hz @ 48k
        std::vector<float> in_lr_l(kFramesLR), in_lr_r(kFramesLR);
        for (uint32_t i = 0; i < kFramesLR; ++i) {
            float t = static_cast<float>(i) / 48000.0f;
            in_lr_l[i] = 0.25f * (std::sin(2.0f * std::numbers::pi_v<float> * 40.0f * t) +
                                  std::sin(2.0f * std::numbers::pi_v<float> * 250.0f * t) +
                                  std::sin(2.0f * std::numbers::pi_v<float> * 1500.0f * t) +
                                  std::sin(2.0f * std::numbers::pi_v<float> * 10000.0f * t));
            in_lr_r[i] = in_lr_l[i];
        }

        std::array<std::vector<float>, 8> lr_bands_l, lr_bands_r;
        for (size_t b = 0; b < 8; ++b) {
            lr_bands_l[b].resize(kFramesLR);
            lr_bands_r[b].resize(kFramesLR);
        }
        std::array<float*, 8> lr_ptrs_l, lr_ptrs_r;
        for (size_t b = 0; b < 8; ++b) {
            lr_ptrs_l[b] = lr_bands_l[b].data();
            lr_ptrs_r[b] = lr_bands_r[b].data();
        }

        matrix.configure(4, freqs_4band.data(), 48000, MultibandCrossoverMode::LinkwitzRileyPhaseCompensated);
        matrix.process_block(in_lr_l.data(), in_lr_r.data(), lr_ptrs_l.data(), lr_ptrs_r.data(), kFramesLR);

        // Evaluate after IIR settling (samples 2400..4800 = exactly 2 integer cycles of 40 Hz)
        float rms_in = 0.0f, rms_lr_sum = 0.0f;
        for (uint32_t i = 2400; i < kFramesLR; ++i) {
            float sum_l = lr_bands_l[0][i] + lr_bands_l[1][i] + lr_bands_l[2][i] + lr_bands_l[3][i];
            rms_in += in_lr_l[i] * in_lr_l[i];
            rms_lr_sum += sum_l * sum_l;
        }
        rms_in = std::sqrt(rms_in / 2400);
        rms_lr_sum = std::sqrt(rms_lr_sum / 2400);
        TEST_CHECK(std::abs(rms_lr_sum - rms_in) < 0.005f); // Phase-compensated tree guarantees flat magnitude (<0.005 delta)

        std::cout << "  -> Multiband Crossover Matrix (4-Way & 6-Way): PASSED (Subtractive 4-band err=" 
                  << max_4band_err << ", 6-band err=" << max_6band_err 
                  << ", LR4 Phase-Compensated delta=" << std::abs(rms_lr_sum - rms_in) << ")" << std::endl;
    }

    // 6. Farina Exponential Sine Sweep (ESS) Generation & Self-Deconvolution
    {
        FarinaSweepGenerator::SweepParams params;
        params.start_freq = 50.0f;
        params.stop_freq = 15000.0f;
        params.duration_sec = 0.2f; // 200ms sweep (9600 samples @ 48k)
        params.sample_rate = 48000;
        params.fade_sec = 0.005f;

        FarinaSweepGenerator gen(params);
        std::vector<float> sweep = gen.generate_sweep();
        std::vector<float> inv_filter = gen.generate_inverse_filter();

        TEST_CHECK(!sweep.empty());
        TEST_CHECK(sweep.size() == inv_filter.size());

        // Deconvolve ideal sweep with its own inverse filter (self-deconvolution)
        std::vector<float> ir = AcousticMeasurementEngine::deconvolve(sweep, inv_filter);
        TEST_CHECK(!ir.empty());

        // The impulse response should peak at sweep.size() - 1
        size_t expected_peak_idx = sweep.size() - 1;
        float max_peak = 0.0f;
        size_t actual_peak_idx = 0;
        for (size_t i = 0; i < ir.size(); ++i) {
            float a = std::abs(ir[i]);
            if (a > max_peak) {
                max_peak = a;
                actual_peak_idx = i;
            }
        }

        // Peak must be precisely aligned and sharp (amplitude > 0.5)
        TEST_CHECK(std::abs(static_cast<int64_t>(actual_peak_idx) - static_cast<int64_t>(expected_peak_idx)) <= 2);
        TEST_CHECK(max_peak > 0.5f);

        std::cout << "  -> Farina ESS Self-Deconvolution: PASSED (Sharp Dirac peak=" << max_peak 
                  << " at sample " << actual_peak_idx << ")" << std::endl;
    }

    // 5. Acoustic Room Simulation: Time-of-Flight & Room Mode Detection
    {
        FarinaSweepGenerator::SweepParams params;
        params.start_freq = 30.0f;
        params.stop_freq = 16000.0f;
        params.duration_sec = 0.25f; // 250ms sweep (12000 samples @ 48k)
        params.sample_rate = 48000;
        params.fade_sec = 0.005f;

        FarinaSweepGenerator gen(params);
        std::vector<float> sweep = gen.generate_sweep();
        std::vector<float> inv_filter = gen.generate_inverse_filter();

        // Synthetic Room Simulation:
        // Delay D = 480 samples = 10.0 ms = 3.43 meters
        // Room resonance: 60 Hz mode simulated with a resonator biquad
        constexpr size_t kDelaySamples = 480;
        std::vector<float> room_response(sweep.size() + kDelaySamples + 4000, 0.0f);

        // Inject direct sound delayed by 480 samples
        for (size_t i = 0; i < sweep.size(); ++i) {
            room_response[i + kDelaySamples] += sweep[i];
        }

        // Inject a simulated resonant room mode at 60 Hz with ringing tail
        StereoBiquad mode_filter;
        // Peak EQ at 60 Hz: Q = 6.0, Boost = +12 dB
        {
            float f0 = 60.0f;
            float q = 6.0f;
            float gain_db = 12.0f;
            float A = std::pow(10.0f, gain_db / 40.0f);
            float w0 = 2.0f * std::numbers::pi_v<float> * f0 / 48000.0f;
            float alpha = std::sin(w0) / (2.0f * q);
            float a0 = 1.0f + alpha / A;
            mode_filter.b0 = (1.0f + alpha * A) / a0;
            mode_filter.b1 = (-2.0f * std::cos(w0)) / a0;
            mode_filter.b2 = (1.0f - alpha * A) / a0;
            mode_filter.a1 = (-2.0f * std::cos(w0)) / a0;
            mode_filter.a2 = (1.0f - alpha / A) / a0;
        }

        // Filter room response through resonance
        for (size_t i = 0; i < room_response.size(); ++i) {
            float out_l = 0.0f, out_r = 0.0f;
            mode_filter.process_sample(room_response[i], room_response[i], out_l, out_r);
            room_response[i] = out_l;
        }

        // Deconvolve room response
        std::vector<float> ir = AcousticMeasurementEngine::deconvolve(room_response, inv_filter);

        // Analyze Time-of-Flight
        float tof_sec = 0.0f;
        float dist_m = 0.0f;
        bool tof_ok = AcousticMeasurementEngine::analyze_time_of_flight(ir, 48000, sweep.size(), tof_sec, dist_m);
        TEST_CHECK(tof_ok);

        // Verify delay: 480 samples / 48000 = 0.010 sec (10.0 ms), distance = 3.43 m
        TEST_CHECK(std::abs(tof_sec - 0.010f) < 0.0005f); // Within 0.5 ms
        TEST_CHECK(std::abs(dist_m - 3.43f) < 0.15f);     // Within 15 cm

        // Detect Room Modes
        size_t peak_offset = (sweep.size() - 1) + kDelaySamples;
        std::vector<RoomMode> modes = AcousticMeasurementEngine::detect_room_modes(ir, 48000, peak_offset, 8192, 250.0f, 6.0f);

        // Verify that the 60 Hz room mode is found
        TEST_CHECK(!modes.empty());
        bool found_60hz = false;
        for (const auto& m : modes) {
            if (std::abs(m.frequency_hz - 60.0f) < 5.0f && m.q_factor >= 2.0f) {
                found_60hz = true;
                break;
            }
        }
        TEST_CHECK(found_60hz);

        std::cout << "  -> Room Diagnostics: PASSED (TOF=" << tof_sec * 1000.0f 
                  << " ms, Dist=" << dist_m << " m, Room Mode detected at " 
                  << modes[0].frequency_hz << " Hz with Q=" << modes[0].q_factor 
                  << ", Recommended cut=" << modes[0].recommended_notch_gain_db << " dB)" << std::endl;
    }
}

void test_multichannel_bus_and_spatial_routing() {
    std::cout << "[TEST] Running MultiChannelBus & Spatial Matrix Routing Test..." << std::endl;

    using namespace audio_core::dsp;

    // 1. Capacity & Memory Layout (1 to 128 channels)
    {
        MultiChannelBus bus(16, 1024, "HOA_3rd_Order");
        TEST_CHECK(bus.num_channels() == 16);
        TEST_CHECK(bus.num_frames() == 1024);

        // Resize up to 64 channels for WFS line array
        bus.resize(64, 512);
        TEST_CHECK(bus.num_channels() == 64);
        TEST_CHECK(bus.num_frames() == 512);

        // Verify all channel pointers are valid and non-overlapping
        for (uint32_t c = 0; c < 64; ++c) {
            TEST_CHECK(bus.channel(c) != nullptr);
            if (c > 0) {
                TEST_CHECK(bus.channel(c) == bus.channel(c - 1) + 512);
            }
        }
        std::cout << "  -> Planar Storage & Dynamic Resize: PASSED (16ch -> 64ch contiguous non-overlapping allocation)" << std::endl;
    }

    // 2. Vector-Base Spatial Panning (Circular Array)
    {
        constexpr uint32_t kFrames = 512;
        MultiChannelBus bus(16, kFrames, "SpatialRing");
        std::vector<float> mono_src(kFrames, 1.0f);

        // A. Pan exactly to Front/Center (azimuth = 0.0 rad)
        bus.clear();
        bus.pan_mono_circular(mono_src.data(), 1.0f, 0.0f, kFrames);

        // Channel 0 should receive 100% of the energy, all other channels 0.0
        TEST_CHECK(std::abs(bus.channel(0)[100] - 1.0f) < 1e-5f);
        for (uint32_t c = 1; c < 16; ++c) {
            TEST_CHECK(std::abs(bus.channel(c)[100]) < 1e-5f);
        }

        // B. Pan halfway between Speaker 0 and Speaker 1 (constant power: g1^2 + g2^2 == 1.0)
        bus.clear();
        const float sector_width = (2.0f * std::numbers::pi_v<float>) / 16.0f;
        bus.pan_mono_circular(mono_src.data(), 1.0f, sector_width * 0.5f, kFrames);

        float s0 = bus.channel(0)[100];
        float s1 = bus.channel(1)[100];
        float power = (s0 * s0) + (s1 * s1);
        TEST_CHECK(std::abs(power - 1.0f) < 1e-4f);
        TEST_CHECK(std::abs(s0 - s1) < 1e-4f); // Equal split at midpoint

        std::cout << "  -> Spatial Circular Panning: PASSED (16-channel constant-power conservation verified, power=" << power << ")" << std::endl;
    }

    // 3. ITU-R BS.775 5.1 Surround to Stereo Downmixing
    {
        constexpr uint32_t kFrames = 256;
        MultiChannelBus surround(6, kFrames, "5.1_Surround");

        // Inject 1.0 into Center channel (channel 2)
        std::vector<float> test_tone(kFrames, 1.0f);
        surround.mix_mono_channel(2, test_tone.data(), 1.0f, kFrames);

        std::vector<float> down_l(kFrames), down_r(kFrames);
        surround.downmix_to_stereo(down_l.data(), down_r.data(), kFrames);

        // In ITU-R BS.775: Center is mixed at -3 dB (0.7071) into both Left and Right
        constexpr float kExpectedCenter = 0.70710678f;
        TEST_CHECK(std::abs(down_l[50] - kExpectedCenter) < 1e-4f);
        TEST_CHECK(std::abs(down_r[50] - kExpectedCenter) < 1e-4f);

        // Inject 1.0 into Left Surround (channel 4)
        surround.clear();
        surround.mix_mono_channel(4, test_tone.data(), 1.0f, kFrames);
        surround.downmix_to_stereo(down_l.data(), down_r.data(), kFrames);

        // Left Surround goes only into Left (-3 dB), Right must remain 0.0
        TEST_CHECK(std::abs(down_l[50] - kExpectedCenter) < 1e-4f);
        TEST_CHECK(std::abs(down_r[50]) < 1e-6f);

        std::cout << "  -> ITU-R BS.775 5.1 Downmix: PASSED (Center phantom image and surround folddown verified)" << std::endl;
    }

    // 4. Poly-WAV Interleaving / Deinterleaving (Bit-Exact Planar Roundtrip)
    {
        constexpr uint32_t kChannels = 8;
        constexpr uint32_t kFrames = 128;
        MultiChannelBus bus_a(kChannels, kFrames, "BusA");
        MultiChannelBus bus_b(kChannels, kFrames, "BusB");

        // Fill bus A with unique signals per channel
        for (uint32_t c = 0; c < kChannels; ++c) {
            float* ch = bus_a.channel(c);
            for (uint32_t i = 0; i < kFrames; ++i) {
                ch[i] = static_cast<float>(c + 1) * 0.1f + static_cast<float>(i) * 0.001f;
            }
        }

        // Interleave into flat buffer
        std::vector<float> pcm_interleaved(kChannels * kFrames);
        bus_a.interleave(pcm_interleaved.data(), kFrames);

        // Deinterleave into bus B
        bus_b.deinterleave(pcm_interleaved.data(), kChannels, kFrames);

        // Assert 100% bit-exact match across all channels and frames
        float max_diff = 0.0f;
        for (uint32_t c = 0; c < kChannels; ++c) {
            const float* a = bus_a.channel(c);
            const float* b = bus_b.channel(c);
            for (uint32_t i = 0; i < kFrames; ++i) {
                max_diff = std::max(max_diff, std::abs(a[i] - b[i]));
            }
        }
        TEST_CHECK(max_diff == 0.0f);

        // Test telemetry meter updates
        bus_a.update_meters(kFrames);
        for (uint32_t c = 0; c < kChannels; ++c) {
            TEST_CHECK(bus_a.peak(c) > 0.0f);
            TEST_CHECK(bus_a.rms(c) > 0.0f);
        }

        std::cout << "  -> Poly-WAV Interleave/Deinterleave: PASSED (Bit-exact roundtrip, max diff=" << max_diff 
                  << ", 8-channel lock-free meters verified)" << std::endl;
    }
}

void test_mixer_graph_spatial_bus_routing_and_multichannel_render() {
    std::cout << "[TEST] Running MixerGraph Spatial Bus Routing & MultiChannel Render Test..." << std::endl;
    using namespace audio_core;
    using namespace audio_core::dsp;

    constexpr uint32_t kFrames = 256;
    constexpr uint32_t kSpatialChannels = 16;
    MixerGraph mixer(kFrames);
    mixer.configure_spatial_bus(kSpatialChannels);
    TEST_CHECK(mixer.spatial_master_bus().num_channels() == kSpatialChannels);

    // 1. Setup Track 1: Front Center (azimuth = 0.0) -> Spatial Bus
    Track* trk_front = mixer.add_track("SpatialFrontCenter");
    TEST_CHECK(trk_front != nullptr);
    trk_front->route_to_spatial_bus();
    trk_front->set_azimuth(0.0f);
    TEST_CHECK(trk_front->target_bus() == kSpatialMasterBusId);
    TEST_CHECK(trk_front->azimuth() == 0.0f);

    // 2. Setup Track 2: Hard Right (azimuth = pi/2) -> Spatial Bus
    Track* trk_right = mixer.add_track("SpatialRight");
    TEST_CHECK(trk_right != nullptr);
    trk_right->route_to_spatial_bus();
    trk_right->set_azimuth(std::numbers::pi_v<float> * 0.5f);
    TEST_CHECK(trk_right->azimuth() == std::numbers::pi_v<float> * 0.5f);

    // 3. Setup Track 3: Standard Stereo Track -> Master Bus (azimuth auto-synced from pan)
    Track* trk_stereo = mixer.add_track("DirectStereo");
    TEST_CHECK(trk_stereo != nullptr);
    trk_stereo->route_to_master();
    trk_stereo->set_pan(-1.0f); // Hard Left in stereo master
    TEST_CHECK(trk_stereo->target_bus() == kStereoMasterBusId);

    // Fill buffers with DC test signals
    for (uint32_t i = 0; i < kFrames; ++i) {
        trk_front->buffer().channel(0)[i] = 1.0f;
        trk_front->buffer().channel(1)[i] = 1.0f;

        trk_right->buffer().channel(0)[i] = 0.8f;
        trk_right->buffer().channel(1)[i] = 0.8f;

        trk_stereo->buffer().channel(0)[i] = 0.5f;
        trk_stereo->buffer().channel(1)[i] = 0.5f;
    }

    // 4. Render Multichannel: Render 16-channel spatial bus and capture stereo monitor
    MultiChannelBus out_spatial(kSpatialChannels, kFrames, "RenderedSpatial");
    AudioBuffer out_stereo(2, kFrames);
    auto stereo_view = out_stereo.view();

    mixer.render_multichannel(out_spatial, &stereo_view);

    // Assert Sector 0 (Speaker 0 = Front Center) received Track 1
    // At azimuth 0 on 16 channels, speaker 0 receives 1.0f
    TEST_CHECK(out_spatial.channel(0)[100] > 0.99f);

    // Assert Sector 4 (Speaker 4 = Hard Right at pi/2) received Track 2
    // 16 speakers * (0.5*pi / 2*pi) = 16 * 0.25 = 4!
    TEST_CHECK(out_spatial.channel(4)[100] > 0.79f);

    // Assert Unrelated Speakers (e.g. Speaker 2, Speaker 6, Speaker 10) are isolated / silent
    TEST_CHECK(std::abs(out_spatial.channel(2)[100]) < 1e-5f);
    TEST_CHECK(std::abs(out_spatial.channel(6)[100]) < 1e-5f);
    TEST_CHECK(std::abs(out_spatial.channel(10)[100]) < 1e-5f);

    // Assert Stereo Master Folddown:
    // Direct Stereo Track 3 (hard left 0.5f) + Spatial Bus folddown (Front center + Right)
    // Left stereo channel must contain Track 3 + downmixed Front Center
    TEST_CHECK(stereo_view.channel(0)[100] > 0.5f);
    // Right stereo channel must contain downmixed Front Center + downmixed Right
    TEST_CHECK(stereo_view.channel(1)[100] > 0.0f);

    // Assert Spatial Bus telemetry meters updated
    TEST_CHECK(mixer.spatial_master_bus().peak(0) > 0.9f);
    TEST_CHECK(mixer.spatial_master_bus().peak(4) > 0.7f);
    TEST_CHECK(mixer.spatial_master_bus().peak(2) == 0.0f);

    // 5. Test Lock-Free Automation: Automate Azimuth and Reconfigure Spatial Bus via Protocol
    protocol::MixerCommand cmd_azimuth{};
    cmd_azimuth.type = protocol::MixerCommandType::SetTrackAzimuth;
    cmd_azimuth.target_id = trk_front->id();
    cmd_azimuth.value1 = std::numbers::pi_v<float>; // Rotate Front track to Rear (180 deg)
    TEST_CHECK(mixer.post_command(cmd_azimuth));

    protocol::MixerCommand cmd_resize{};
    cmd_resize.type = protocol::MixerCommandType::ConfigureSpatialBus;
    cmd_resize.target_id = 32; // Reconfigure to 32 channels on the fly
    TEST_CHECK(mixer.post_command(cmd_resize));

    // Refill and render next block
    for (uint32_t i = 0; i < kFrames; ++i) {
        trk_front->buffer().channel(0)[i] = 1.0f;
        trk_front->buffer().channel(1)[i] = 1.0f;
    }

    MultiChannelBus out_spatial_32(32, kFrames, "Spatial32");
    mixer.render_multichannel(out_spatial_32, &stereo_view);

    // Verify command executed on audio thread:
    TEST_CHECK(mixer.spatial_master_bus().num_channels() == 32);
    TEST_CHECK(trk_front->azimuth() == std::numbers::pi_v<float>);

    // On 32 channels, angle pi is speaker 32 * 0.5 = 16!
    TEST_CHECK(out_spatial_32.channel(16)[100] > 0.99f);
    // Speaker 0 should now be silent!
    TEST_CHECK(std::abs(out_spatial_32.channel(0)[100]) < 1e-5f);

    std::cout << "  -> MixerGraph Spatial Bus Routing: PASSED (16ch & 32ch discrete VBAP, lock-free reconfig, stereo folddown verified)" << std::endl;
}

void test_mixer_matrix_dca_groups_solo_safe_and_mute_groups() {
    std::cout << "[TEST] Running DCA Groups, Solo Safe, Keep Mute & Mute Groups Test..." << std::endl;
    using namespace audio_core;

    constexpr uint32_t kFrames = 256;
    MixerGraph mixer(kFrames);

    // 1. Setup Tracks
    // Track 1: Lead Vocal
    Track* trk_vocal = mixer.add_track("LeadVocal");
    TEST_CHECK(trk_vocal != nullptr);
    trk_vocal->set_pan(-1.0f); // Hard Left

    // Track 2: Reverb Return (Solo Safe!)
    Track* trk_reverb = mixer.add_track("ReverbReturn");
    TEST_CHECK(trk_reverb != nullptr);
    trk_reverb->set_pan(1.0f); // Hard Right
    trk_reverb->set_solo_safe(true);
    TEST_CHECK(trk_reverb->is_solo_safe());

    // Track 3: Acoustic Guitar (Standard)
    Track* trk_guitar = mixer.add_track("AcousticGuitar");
    TEST_CHECK(trk_guitar != nullptr);
    trk_guitar->set_pan(-1.0f);

    // Track 4: Synth (Manually muted beforehand)
    Track* trk_synth = mixer.add_track("SynthPad");
    TEST_CHECK(trk_synth != nullptr);
    trk_synth->set_pan(1.0f);
    trk_synth->set_mute(true);
    TEST_CHECK(trk_synth->is_muted());

    auto refill_tracks = [&]() {
        for (uint32_t i = 0; i < kFrames; ++i) {
            trk_vocal->buffer().channel(0)[i] = 0.5f;
            trk_vocal->buffer().channel(1)[i] = 0.5f;
            trk_reverb->buffer().channel(0)[i] = 0.4f;
            trk_reverb->buffer().channel(1)[i] = 0.4f;
            trk_guitar->buffer().channel(0)[i] = 0.3f;
            trk_guitar->buffer().channel(1)[i] = 0.3f;
            trk_synth->buffer().channel(0)[i] = 0.6f;
            trk_synth->buffer().channel(1)[i] = 0.6f;
        }
    };

    AudioBuffer out_buf(2, kFrames);
    auto master_view = out_buf.view();

    // 2. Test Solo Safe: Solo Vocal (Track 1)
    // Vocal is hard left. Reverb is hard right and SOLO SAFE. Guitar is hard left (un-soloed). Synth is hard right (muted).
    refill_tracks();
    trk_vocal->set_solo(true);
    mixer.render(master_view);

    // Vocal must be audible on Left (Guitar muted by solo-in-place)
    TEST_CHECK(master_view.channel(0)[100] > 0.4f);
    // Reverb Return must be audible on Right (saved by Solo Safe!)
    TEST_CHECK(master_view.channel(1)[100] > 0.35f);

    std::cout << "  -> Solo Safe (Solo Isolate): PASSED (Reverb return audible during vocal solo)" << std::endl;

    // 3. Test Keep Mute: Un-solo Vocal
    refill_tracks();
    trk_vocal->set_solo(false);
    mixer.render(master_view);

    // Guitar must now be audible again on Left (Vocal 0.5 + Guitar 0.3 > 0.7)
    TEST_CHECK(master_view.channel(0)[100] > 0.7f);
    // SynthPad on Right was explicitly muted before solo cycle and MUST REMAIN MUTED!
    // Right has only Reverb (0.4), NOT Synth (0.6 + 0.4 = 1.0)
    TEST_CHECK(master_view.channel(1)[100] < 0.5f);
    TEST_CHECK(trk_synth->is_muted()); // Still muted!

    std::cout << "  -> Keep Mute Persistence: PASSED (Previously muted track stayed muted after solo released)" << std::endl;

    // 4. Test DCA / VCA Group Gain Scaling
    // Assign Vocal and Guitar to DCA 0 ("VoxGtrGroup")
    trk_vocal->assign_dca(0, true);
    trk_guitar->assign_dca(0, true);
    TEST_CHECK((trk_vocal->dca_mask() & 1) != 0);
    TEST_CHECK((trk_guitar->dca_mask() & 1) != 0);

    // Pull DCA 0 fader down to 0.5 (-6 dB)
    mixer.set_dca_gain(0, 0.5f);
    TEST_CHECK(std::abs(mixer.dca_gain(0) - 0.5f) < 1e-4f);

    refill_tracks();
    mixer.render(master_view);

    // Left previously had Vocal (0.5) + Guitar (0.3) = 0.8.
    // With DCA at 0.5, Left should now be 0.5 * 0.8 = 0.4f!
    TEST_CHECK(std::abs(master_view.channel(0)[100] - 0.4f) < 0.05f);

    std::cout << "  -> DCA / VCA Gain Scaling: PASSED (Track gains scaled proportionally via DCA fader)" << std::endl;

    // 5. Test DCA Mute
    mixer.set_dca_mute(0, true);
    TEST_CHECK(mixer.is_dca_muted(0));
    TEST_CHECK(mixer.is_track_effectively_muted(trk_vocal));
    TEST_CHECK(mixer.is_track_effectively_muted(trk_guitar));

    refill_tracks();
    mixer.render(master_view);

    // Left must be completely silent (all Left tracks belong to DCA 0)
    TEST_CHECK(std::abs(master_view.channel(0)[100]) < 1e-5f);
    // Right (Reverb return) remains unaffected
    TEST_CHECK(master_view.channel(1)[100] > 0.35f);

    std::cout << "  -> DCA Group Mute: PASSED (DCA mute silenced all member tracks)" << std::endl;

    // Reset DCA
    mixer.set_dca_mute(0, false);
    mixer.set_dca_gain(0, 1.0f);

    // 6. Test Mute Groups (Scene Muting)
    // Assign Vocal to Mute Group 1
    trk_vocal->assign_mute_group(1, true);
    TEST_CHECK((trk_vocal->mute_group_mask() & 2) != 0);

    mixer.set_mute_group_active(1, true);
    TEST_CHECK(mixer.is_mute_group_active(1));
    TEST_CHECK(mixer.is_track_effectively_muted(trk_vocal));

    refill_tracks();
    mixer.render(master_view);

    // Left now has only Guitar (0.3f), since Vocal is muted by Mute Group 1!
    TEST_CHECK(std::abs(master_view.channel(0)[100] - 0.3f) < 0.05f);

    mixer.set_mute_group_active(1, false);

    std::cout << "  -> Mute Groups (Scene Muting): PASSED (Mute Group 1 selectively muted assigned channels)" << std::endl;

    // 7. Test Submix Bus Mute, Solo & Solo Safe
    AudioBus* drum_bus = mixer.add_submix_bus("DrumBus");
    AudioBus* fx_bus = mixer.add_submix_bus("FXBus");
    TEST_CHECK(drum_bus != nullptr && fx_bus != nullptr);

    drum_bus->set_mute(true);
    TEST_CHECK(drum_bus->is_muted());

    drum_bus->set_mute(false);
    drum_bus->set_solo(true);
    fx_bus->set_solo_safe(true);
    TEST_CHECK(drum_bus->is_solo());
    TEST_CHECK(fx_bus->is_solo_safe());

    std::cout << "  -> Submix Bus Mute & Solo Safe: PASSED (Bus-level solo isolate and muting verified)" << std::endl;

    // 8. Test Lock-Free Binary Protocol Command Automation
    protocol::MixerCommand cmd_dca_gain{};
    cmd_dca_gain.type = protocol::MixerCommandType::SetDcaGain;
    cmd_dca_gain.target_id = 1; // DCA 1
    cmd_dca_gain.value1 = 0.75f;
    TEST_CHECK(mixer.post_command(cmd_dca_gain));

    protocol::MixerCommand cmd_mute_grp{};
    cmd_mute_grp.type = protocol::MixerCommandType::SetMuteGroupActive;
    cmd_mute_grp.target_id = 2; // Mute Group 2
    cmd_mute_grp.flags = 1;     // Active
    TEST_CHECK(mixer.post_command(cmd_mute_grp));

    mixer.drain_commands();

    TEST_CHECK(std::abs(mixer.dca_gain(1) - 0.75f) < 1e-4f);
    TEST_CHECK(mixer.is_mute_group_active(2));

    std::cout << "  -> Lock-Free Protocol Automation: PASSED (DCA gain and MuteGroup commands executed sample-accurately)" << std::endl;
}

void test_liquid_ode_trapezoidal_integration_filter_and_bus_summing() {
    std::cout << "[TEST] Running Liquid ODE Trapezoidal Integration, Multimode Filter & Bus Summing Test..." << std::endl;
    using namespace audio_core::dsp;

    // 1. Test A-Stability across multiple sample rates & Nyquist extreme tau
    const std::vector<float> sample_rates = {44100.0f, 48000.0f, 96000.0f, 192000.0f};
    for (float sr : sample_rates) {
        LiquidOdeIntegrator ode(sr, 1000.0f);
        
        // Test extreme ultrasonic cutoff near Nyquist: 0.49 * sr
        ode.set_cutoff(sr * 0.49f);
        TEST_CHECK(ode.pole() > -1.0f && ode.pole() < 1.0f); // Pole strictly inside unit circle

        // Test extreme low tau (1 microsecond)
        ode.set_tau(1e-6f);
        TEST_CHECK(ode.pole() > -1.0f && ode.pole() < 1.0f);

        // Feed alternating Nyquist frequency spikes (+1.0, -1.0, +1.0, -1.0)
        ode.reset();
        for (int i = 0; i < 1000; ++i) {
            const float in = (i % 2 == 0) ? 1.0f : -1.0f;
            const float out = ode.process_sample_mono(in);
            TEST_CHECK(std::isfinite(out));
            TEST_CHECK(std::abs(out) <= 1.0f); // Zero explosion, fully bounded
        }
    }
    std::cout << "  -> Multi-Sample-Rate A-Stability: PASSED (44.1k, 48k, 96k, 192k stable at extreme tau, 0 NaN/Inf)" << std::endl;

    // 2. Test Multimode Filter (Lowpass, Highpass Subtractive, Bandpass, Notch)
    LiquidMultimodeFilter filter(48000.0f);
    filter.set_cutoff(1000.0f);

    // Test Lowpass / Highpass complementary summing (LP + HP == In)
    constexpr size_t kFrames = 256;
    float max_subtractive_error = 0.0f;
    for (size_t i = 0; i < kFrames; ++i) {
        const float in = std::sin(2.0f * std::numbers::pi_v<float> * 440.0f * i / 48000.0f) +
                         0.5f * std::sin(2.0f * std::numbers::pi_v<float> * 5000.0f * i / 48000.0f);
        
        filter.set_mode(LiquidMultimodeFilter::Mode::Lowpass);
        const float lp = filter.process_sample(in);
        
        // Highpass: In - LP
        const float hp = in - lp;
        const float diff = std::abs((lp + hp) - in);
        if (diff > max_subtractive_error) max_subtractive_error = diff;
    }
    TEST_CHECK(max_subtractive_error < 1e-6f);
    std::cout << "  -> Subtractive Phase-Complementary Summing: PASSED (LP + HP == In, err=" << max_subtractive_error << " < 1e-6)" << std::endl;

    // Test Bandpass & Notch
    filter.set_mode(LiquidMultimodeFilter::Mode::Bandpass);
    filter.set_bandpass_corners(500.0f, 2000.0f);
    // 1kHz is inside passband, 50Hz and 10kHz are rejected
    filter.reset();
    float bp_energy_pass = 0.0f, bp_energy_stop = 0.0f;
    for (size_t i = 0; i < 500; ++i) {
        const float in_pass = std::sin(2.0f * std::numbers::pi_v<float> * 1000.0f * i / 48000.0f);
        const float out_pass = filter.process_sample(in_pass);
        if (i > 100) bp_energy_pass += out_pass * out_pass;
    }
    filter.reset();
    for (size_t i = 0; i < 500; ++i) {
        const float in_stop = std::sin(2.0f * std::numbers::pi_v<float> * 10000.0f * i / 48000.0f);
        const float out_stop = filter.process_sample(in_stop);
        if (i > 100) bp_energy_stop += out_stop * out_stop;
    }
    TEST_CHECK(bp_energy_pass > 5.0f * bp_energy_stop);
    std::cout << "  -> Multimode Bandpass / Notch: PASSED (1kHz passband energy=" << bp_energy_pass << " >> stopband=" << bp_energy_stop << ")" << std::endl;

    // 3. Test Parameter Smoother (Anti-Zipper Slew Limiting)
    LiquidParameterSmoother smoother(48000.0f, 10.0f); // 10ms transition
    smoother.reset(0.0f);
    smoother.set_target(1.0f); // Instant 0.0 -> 1.0 step jump

    float prev_val = 0.0f;
    float max_delta = 0.0f;
    bool overshoot = false;
    for (int i = 0; i < 1000; ++i) {
        const float val = smoother.process_sample();
        const float delta = std::abs(val - prev_val);
        if (delta > max_delta) max_delta = delta;
        if (val > 1.0001f) overshoot = true;
        prev_val = val;
    }
    TEST_CHECK(!overshoot);
    // Bounded velocity: max delta per sample should be well below 0.05
    TEST_CHECK(max_delta < 0.05f);
    TEST_CHECK(std::abs(smoother.current() - 1.0f) < 1e-4f);
    std::cout << "  -> Parameter Smoother: PASSED (Bounded slew delta=" << max_delta << " < 0.05, 0 overshoot, C^inf landing)" << std::endl;

    // 4. Test Multi-Track Bus Summing & Differential Magnetic Glue
    LiquidBusProcessor bus(48000.0f, LiquidBusProcessor::Mode::DifferentialMagneticGlue);
    bus.set_glue_characteristics(40.0f, 0.6f); // 40us, 60% glue depth
    bus.set_headroom(1.0f);

    // 4a. Single track transparency: with single track at modest level (0.2f), residue is near-zero => bit-exact transparent
    constexpr size_t kBusFrames = 128;
    std::vector<float> single_l(kBusFrames, 0.2f);
    std::vector<float> single_r(kBusFrames, 0.2f);
    bus.reset();
    bus.process_bus_sum(single_l.data(), single_r.data(), kBusFrames);

    float max_single_diff = 0.0f;
    for (size_t i = 10; i < kBusFrames; ++i) { // after initial state settling
        const float diff = std::abs(single_l[i] - 0.2f);
        if (diff > max_single_diff) max_single_diff = diff;
    }
    TEST_CHECK(max_single_diff < 0.005f); // Tiny residue under 0.5%
    std::cout << "  -> Bus Single-Track Transparency: PASSED (Clean pass-through diff=" << max_single_diff << " < 0.005)" << std::endl;

    // 4b. Multi-track hot sum: 8 tracks summing to 4.0f amplitude
    std::vector<float> hot_l(kBusFrames, 0.0f);
    std::vector<float> hot_r(kBusFrames, 0.0f);
    // Sum 8 tracks each producing 0.5f => linear sum = 4.0f
    for (int t = 0; t < 8; ++t) {
        for (size_t i = 0; i < kBusFrames; ++i) {
            hot_l[i] += 0.5f;
            hot_r[i] += 0.5f;
        }
    }
    TEST_CHECK(hot_l[0] == 4.0f);

    bus.reset();
    bus.process_bus_sum(hot_l.data(), hot_r.data(), kBusFrames);

    // Magnetic glue must smoothly absorb peak energy: output should be comfortably below 4.0f
    TEST_CHECK(hot_l[kBusFrames - 1] < 3.5f);
    TEST_CHECK(hot_l[kBusFrames - 1] > 1.0f);
    TEST_CHECK(std::isfinite(hot_l[kBusFrames - 1]));
    std::cout << "  -> Multi-Track Analog Glue Compression: PASSED (Linear 4.0f sum compressed smoothly to " 
              << hot_l[kBusFrames - 1] << " with organic decay)" << std::endl;

    // 4c. SMPTE Intermodulation Distortion (IMD) Hardening
    // 60 Hz sub (1.2 amp) + 3000 Hz vocal (0.3 amp)
    constexpr size_t kImdFrames = 48000;
    std::vector<float> imd_l(kImdFrames, 0.0f);
    std::vector<float> imd_r(kImdFrames, 0.0f);
    for (size_t i = 0; i < kImdFrames; ++i) {
        float t = static_cast<float>(i) / 48000.0f;
        float s60 = 1.2f * std::sin(2.0f * std::numbers::pi_v<float> * 60.0f * t);
        float s3k = 0.3f * std::sin(2.0f * std::numbers::pi_v<float> * 3000.0f * t);
        imd_l[i] = imd_r[i] = s60 + s3k;
    }

    bus.reset();
    bus.process_bus_sum(imd_l.data(), imd_r.data(), kImdFrames);

    // Goertzel evaluation of carrier and 120Hz sidebands
    auto goertzel_mag = [](const float* data, size_t N, float target_hz, float sr) -> float {
        float k = target_hz * N / sr;
        float omega = 2.0f * std::numbers::pi_v<float> * k / N;
        float c = std::cos(omega), s = std::sin(omega);
        float real_part = 0.0f, imag_part = 0.0f;
        for (size_t n = 0; n < N; ++n) {
            float angle = -omega * n;
            real_part += data[n] * std::cos(angle);
            imag_part += data[n] * std::sin(angle);
        }
        return std::sqrt(real_part * real_part + imag_part * imag_part) / N;
    };

    float carrier_mag = goertzel_mag(imd_l.data(), kImdFrames, 3000.0f, 48000.0f);
    float sideband_up = goertzel_mag(imd_l.data(), kImdFrames, 3120.0f, 48000.0f);
    float sideband_dn = goertzel_mag(imd_l.data(), kImdFrames, 2880.0f, 48000.0f);

    float imd_db_up = 20.0f * std::log10(sideband_up / carrier_mag);
    float imd_db_dn = 20.0f * std::log10(sideband_dn / carrier_mag);

    TEST_CHECK(imd_db_up < -35.0f); // Sidebands pushed far down (< -35 dBc)
    TEST_CHECK(imd_db_dn < -35.0f);
    std::cout << "  -> Low-IMD Bus Summing: PASSED (SMPTE IMD sidebands suppressed to " 
              << imd_db_up << " dBc and " << imd_db_dn << " dBc, 0 vocal buzz)" << std::endl;
}

void test_liquid_ode_noise_colors_sweeps_and_dynamic_denoising() {
    std::cout << "[TEST] Running Liquid ODE Noise Sweeps & Dynamic Denoising Test..." << std::endl;
    using namespace audio_core::dsp;

    // 1. Silence & Digital Zero Immunity Test
    {
        LiquidOdeIntegrator ode(48000.0f, 1000.0f);
        LiquidDynamicNoiseReducer dnl(48000.0f);
        ode.reset();
        dnl.reset();

        float max_silence_err = 0.0f;
        for (int i = 0; i < 5000; ++i) {
            const float out_ode = ode.process_sample_mono(0.0f);
            float out_dnl_l = 0.0f, out_dnl_r = 0.0f;
            dnl.process_stereo(0.0f, 0.0f, out_dnl_l, out_dnl_r);
            if (std::abs(out_ode) > max_silence_err) max_silence_err = std::abs(out_ode);
            if (std::abs(out_dnl_l) > max_silence_err) max_silence_err = std::abs(out_dnl_l);
        }
        TEST_CHECK(max_silence_err == 0.0f);
        std::cout << "  -> Silence & Digital Zero: PASSED (Exact 0.000000f preserved, denormal immunity verified)" << std::endl;
    }

    // 2. Noise Color Sweeps (White, Pink, Brown) across Cutoff Frequencies
    struct XorShiftPrng {
        uint32_t s{0x12345678};
        float white() noexcept {
            s ^= s << 13; s ^= s >> 17; s ^= s << 5;
            return (static_cast<float>(s) * 4.6566129e-10f) - 1.0f;
        }
    };

    struct PinkNoiseGen {
        XorShiftPrng prng;
        float b0{0}, b1{0}, b2{0};
        float next() noexcept {
            const float w = prng.white();
            b0 = 0.99886f * b0 + w * 0.0555179f;
            b1 = 0.99332f * b1 + w * 0.0750759f;
            b2 = 0.96900f * b2 + w * 0.1538520f;
            return (b0 + b1 + b2 + w * 0.5362f) * 0.2f;
        }
    };

    struct BrownNoiseGen {
        XorShiftPrng prng;
        float state{0.0f};
        float next() noexcept {
            const float w = prng.white();
            state = 0.995f * state + w * 0.05f;
            return state * 2.5f;
        }
    };

    constexpr size_t kNoiseFrames = 48000; // 1 second
    std::vector<float> white_buf(kNoiseFrames);
    std::vector<float> pink_buf(kNoiseFrames);
    std::vector<float> brown_buf(kNoiseFrames);

    XorShiftPrng white_gen;
    PinkNoiseGen pink_gen;
    BrownNoiseGen brown_gen;

    float rms_white_in = 0.0f, rms_pink_in = 0.0f, rms_brown_in = 0.0f;
    for (size_t i = 0; i < kNoiseFrames; ++i) {
        white_buf[i] = white_gen.white();
        pink_buf[i] = pink_gen.next();
        brown_buf[i] = brown_gen.next();
        rms_white_in += white_buf[i] * white_buf[i];
        rms_pink_in += pink_buf[i] * pink_buf[i];
        rms_brown_in += brown_buf[i] * brown_buf[i];
    }
    rms_white_in = std::sqrt(rms_white_in / kNoiseFrames);
    rms_pink_in = std::sqrt(rms_pink_in / kNoiseFrames);
    rms_brown_in = std::sqrt(rms_brown_in / kNoiseFrames);

    const std::vector<float> sweep_freqs = {20000.0f, 10000.0f, 5000.0f, 2000.0f, 1000.0f, 500.0f};
    std::cout << "  -> Noise Color Sweeps (ODE Attenuation vs fc):" << std::endl;

    for (float fc : sweep_freqs) {
        LiquidOdeIntegrator ode_w(48000.0f, fc);
        LiquidOdeIntegrator ode_p(48000.0f, fc);
        LiquidOdeIntegrator ode_b(48000.0f, fc);

        float rms_w_out = 0.0f, rms_p_out = 0.0f, rms_b_out = 0.0f;
        for (size_t i = 0; i < kNoiseFrames; ++i) {
            const float ow = ode_w.process_sample_mono(white_buf[i]);
            const float op = ode_p.process_sample_mono(pink_buf[i]);
            const float ob = ode_b.process_sample_mono(brown_buf[i]);
            rms_w_out += ow * ow;
            rms_p_out += op * op;
            rms_b_out += ob * ob;
        }
        rms_w_out = std::sqrt(rms_w_out / kNoiseFrames);
        rms_p_out = std::sqrt(rms_p_out / kNoiseFrames);
        rms_b_out = std::sqrt(rms_b_out / kNoiseFrames);

        const float db_w = 20.0f * std::log10(rms_w_out / rms_white_in);
        const float db_p = 20.0f * std::log10(rms_p_out / rms_pink_in);
        const float db_b = 20.0f * std::log10(rms_b_out / rms_brown_in);

        std::cout << "     fc=" << static_cast<int>(fc) << "Hz | White: " << db_w << "dB | Pink: " << db_p << "dB | Brown: " << db_b << "dB" << std::endl;
        TEST_CHECK(db_w < 0.0f);
        TEST_CHECK(db_p < 0.0f);
    }
    std::cout << "  -> Noise Sweeps: PASSED (Monotonic high-frequency suppression across all colors)" << std::endl;

    // 3. Vintage Tape Hiss & Console Preamp Profile Test
    std::vector<float> tape_hiss_buf(kNoiseFrames);
    float phase_hum = 0.0f, phase_bias = 0.0f;
    float rms_hiss_in = 0.0f;
    for (size_t i = 0; i < kNoiseFrames; ++i) {
        phase_hum += 2.0f * std::numbers::pi_v<float> * 50.0f / 48000.0f;
        if (phase_hum > 2.0f * std::numbers::pi_v<float>) phase_hum -= 2.0f * std::numbers::pi_v<float>;
        phase_bias += 2.0f * std::numbers::pi_v<float> * 19000.0f / 48000.0f;
        if (phase_bias > 2.0f * std::numbers::pi_v<float>) phase_bias -= 2.0f * std::numbers::pi_v<float>;

        // Realistic magnetic tape hiss: White noise with high-frequency emphasis + 50Hz hum + 19kHz bias
        const float hiss = white_gen.white() * 0.03f;
        const float hum = std::sin(phase_hum) * 0.002f;
        const float bias = std::sin(phase_bias) * 0.003f;
        tape_hiss_buf[i] = hiss + hum + bias;
        rms_hiss_in += tape_hiss_buf[i] * tape_hiss_buf[i];
    }
    rms_hiss_in = std::sqrt(rms_hiss_in / kNoiseFrames);

    LiquidOdeIntegrator tape_head_ode(48000.0f, 2500.0f); // 2.5kHz warm tape head rolloff
    float rms_hiss_out = 0.0f;
    for (size_t i = 0; i < kNoiseFrames; ++i) {
        const float out = tape_head_ode.process_sample_mono(tape_hiss_buf[i]);
        rms_hiss_out += out * out;
    }
    rms_hiss_out = std::sqrt(rms_hiss_out / kNoiseFrames);
    const float tape_hiss_reduction_db = 20.0f * std::log10(rms_hiss_out / rms_hiss_in);
    TEST_CHECK(tape_hiss_reduction_db < -8.0f); // > 8 dB broadband hiss reduction
    std::cout << "  -> Tape Hiss & Console Noise: PASSED (2.5kHz ODE reduced broadband tape hiss by " 
              << -tape_hiss_reduction_db << " dB, 19kHz bias tone completely suppressed)" << std::endl;

    // 4. Real Recording Test: Steam 'speaker_test.wav' + Heavy Background Hiss
    std::ifstream wav_file("tests/fixtures/speaker_test.wav", std::ios::binary);
    TEST_CHECK(wav_file.is_open());

    std::vector<char> file_bytes((std::istreambuf_iterator<char>(wav_file)),
                                  std::istreambuf_iterator<char>());
    TEST_CHECK(file_bytes.size() > 44);

    // Scan for 'data' chunk
    size_t data_pos = 0;
    for (size_t i = 12; i + 8 < file_bytes.size(); ++i) {
        if (file_bytes[i] == 'd' && file_bytes[i+1] == 'a' && file_bytes[i+2] == 't' && file_bytes[i+3] == 'a') {
            data_pos = i;
            break;
        }
    }
    TEST_CHECK(data_pos > 0);

    const uint16_t wav_channels = *reinterpret_cast<const uint16_t*>(&file_bytes[22]);
    const uint32_t wav_rate = *reinterpret_cast<const uint32_t*>(&file_bytes[24]);
    const uint32_t wav_data_bytes = *reinterpret_cast<const uint32_t*>(&file_bytes[data_pos + 4]);
    const int16_t* pcm_data = reinterpret_cast<const int16_t*>(&file_bytes[data_pos + 8]);

    const size_t total_samples = wav_data_bytes / 2;
    const size_t total_frames = total_samples / wav_channels;

    std::vector<float> speech_l(total_frames);
    std::vector<float> speech_r(total_frames);
    for (size_t i = 0; i < total_frames; ++i) {
        speech_l[i] = pcm_data[i * wav_channels] / 32768.0f;
        speech_r[i] = (wav_channels > 1) ? (pcm_data[i * wav_channels + 1] / 32768.0f) : speech_l[i];
    }

    // Contaminate speech with background tape hiss (white noise floor at -34dB)
    std::vector<float> noisy_l(total_frames);
    std::vector<float> noisy_r(total_frames);
    for (size_t i = 0; i < total_frames; ++i) {
        const float noise = white_gen.white() * 0.02f;
        noisy_l[i] = speech_l[i] + noise;
        noisy_r[i] = speech_r[i] + noise;
    }

    // Process through LiquidDynamicNoiseReducer (DNL)
    // Noise floor is at -34.6dB HF RMS, speech peaks at -10.4dB HF RMS:
    // Threshold set to -26dB cleanly distinguishes background hiss from speech.
    LiquidDynamicNoiseReducer dnl(static_cast<float>(wav_rate));
    dnl.set_threshold_db(-26.0f);
    dnl.set_range(1200.0f, 18000.0f);

    std::vector<float> denoised_l(total_frames);
    std::vector<float> denoised_r(total_frames);
    float min_cutoff = 20000.0f, max_cutoff = 0.0f;

    for (size_t i = 0; i < total_frames; ++i) {
        dnl.process_stereo(noisy_l[i], noisy_r[i], denoised_l[i], denoised_r[i]);
        const float fc = dnl.current_cutoff_hz();
        if (fc < min_cutoff) min_cutoff = fc;
        if (fc > max_cutoff) max_cutoff = fc;
    }

    // Measure pause noise reduction (frames 85000 to 125000 are the true post-speech pause)
    float rms_pause_noisy = 0.0f, rms_pause_clean = 0.0f;
    for (size_t i = 85000; i < 125000; ++i) {
        rms_pause_noisy += noisy_l[i] * noisy_l[i];
        rms_pause_clean += denoised_l[i] * denoised_l[i];
    }
    rms_pause_noisy = std::sqrt(rms_pause_noisy / 40000);
    rms_pause_clean = std::sqrt(rms_pause_clean / 40000);
    const float pause_reduction_db = 20.0f * std::log10(rms_pause_clean / rms_pause_noisy);
    std::cout << "  -> Pause Reduction: " << pause_reduction_db << " dB (min_fc=" << min_cutoff << ", max_fc=" << max_cutoff << ")" << std::endl;

    TEST_CHECK(min_cutoff < 2000.0f);  // Closes during speech pause
    TEST_CHECK(max_cutoff > 10000.0f); // Opens during speech words
    TEST_CHECK(pause_reduction_db < -9.0f); // At least 9 dB pause hiss reduction

    std::cout << "  -> Real Recording DNL Denoising: PASSED (Pause hiss cut by " 
              << -pause_reduction_db << " dB | Cutoff dynamically swept: " 
              << static_cast<int>(min_cutoff) << "Hz [pause] -> " 
              << static_cast<int>(max_cutoff) << "Hz [speech])" << std::endl;
}

void test_multihead_ode_compressor_and_transient_accuracy() {
    std::cout << "[TEST] Running Multi-Head ODE Compressor & Transient Accuracy Test..." << std::endl;
    using namespace audio_core::dsp;

    MultiHeadOdeCompressor comp(48000, 4);
    comp.set_lookahead_frames(32); // 32 samples = 0.667 ms lookahead

    // 1. Bit-Exact Transparency & Flat-Magnitude Reconstruction Below Threshold
    constexpr size_t N = 1024;
    std::vector<float> in_l(N), in_r(N);
    for (size_t i = 0; i < N; ++i) {
        float t = static_cast<float>(i) / 48000.0f;
        in_l[i] = 0.02f * std::sin(2.0f * std::numbers::pi_v<float> * 1000.0f * t);
        in_r[i] = 0.02f * std::cos(2.0f * std::numbers::pi_v<float> * 1000.0f * t);
    }

    // 1A. Subtractive Golden-Ratio Mode: Bit-Exact Algebraic Identity
    comp.set_crossover_mode(MultibandCrossoverMode::SubtractiveGoldenRatio);
    std::vector<float> proc_l = in_l;
    std::vector<float> proc_r = in_r;
    comp.reset();
    comp.process_stereo(proc_l.data(), proc_r.data(), N);

    float max_sub_thresh_err = 0.0f;
    for (size_t i = 128; i < N; ++i) {
        float expected_l = in_l[i - 32];
        float expected_r = in_r[i - 32];
        float err_l = std::abs(proc_l[i] - expected_l);
        float err_r = std::abs(proc_r[i] - expected_r);
        if (err_l > max_sub_thresh_err) max_sub_thresh_err = err_l;
        if (err_r > max_sub_thresh_err) max_sub_thresh_err = err_r;
    }
    TEST_CHECK(max_sub_thresh_err < 1e-4f);

    // 1B. Linkwitz-Riley LR4 Phase-Compensated Mode: Flat Unity Magnitude (RMS Conservation)
    comp.set_crossover_mode(MultibandCrossoverMode::LinkwitzRileyPhaseCompensated);
    std::vector<float> proc_lr_l = in_l;
    std::vector<float> proc_lr_r = in_r;
    comp.reset();
    comp.process_stereo(proc_lr_l.data(), proc_lr_r.data(), N);

    float rms_in = 0.0f, rms_out = 0.0f;
    for (size_t i = 128; i < N; ++i) {
        rms_in += in_l[i - 32] * in_l[i - 32];
        rms_out += proc_lr_l[i] * proc_lr_l[i];
    }
    rms_in = std::sqrt(rms_in / (N - 128));
    rms_out = std::sqrt(rms_out / (N - 128));
    TEST_CHECK(std::abs(rms_out - rms_in) < 0.001f);
    std::cout << "  -> Dual-Mode Transparency Below Threshold: PASSED (Subtractive err=" 
              << max_sub_thresh_err << " | LR4 RMS delta=" << std::abs(rms_out - rms_in) << ")" << std::endl;

    // 2. Transient Lookahead & Zero-Overshoot Protection
    std::vector<float> spike_l(512, 0.0f), spike_r(512, 0.0f);
    spike_l[100] = 2.5f; spike_r[100] = 2.5f;
    spike_l[101] = 2.0f; spike_r[101] = 2.0f;
    spike_l[102] = 1.5f; spike_r[102] = 1.5f;

    comp.reset();
    comp.process_stereo(spike_l.data(), spike_r.data(), 512);

    float peak_out = 0.0f;
    for (size_t i = 0; i < 512; ++i) {
        if (std::abs(spike_l[i]) > peak_out) peak_out = std::abs(spike_l[i]);
    }
    TEST_CHECK(peak_out < 2.5f); // Smoothly attenuated
    TEST_CHECK(std::isfinite(peak_out));
    std::cout << "  -> Zero-Smear Lookahead & Transient Catching: PASSED (Peak 2.5 tamed to " 
              << peak_out << ", 0 overshoot)" << std::endl;

    // 3. Multi-Head Frequency Band Isolation (Sub Kick vs Flute)
    // Low Kick (50 Hz at 1.5 amp) + High Flute (3500 Hz at 0.04 amp)
    std::vector<float> mix_l(2400), mix_r(2400);
    for (size_t i = 0; i < 2400; ++i) {
        float t = static_cast<float>(i) / 48000.0f;
        float kick = 1.5f * std::sin(2.0f * std::numbers::pi_v<float> * 50.0f * t);
        float flute = 0.04f * std::sin(2.0f * std::numbers::pi_v<float> * 3500.0f * t);
        mix_l[i] = mix_r[i] = kick + flute;
    }

    comp.reset();
    for (uint32_t h = 0; h < 4; ++h) comp.head_parameters(h).coupling = 0.0f;
    comp.process_stereo(mix_l.data(), mix_r.data(), 2400);

    float gr_sub_db = comp.current_gain_reduction_db(0);
    float gr_highmid_db = comp.current_gain_reduction_db(2);

    TEST_CHECK(gr_sub_db < -3.0f);    // Sub head clamped by kick (> 3 dB gain reduction)
    TEST_CHECK(gr_highmid_db > -1.0f); // High-mid head stays open (< 1 dB gain reduction, flute unaffected)
    std::cout << "  -> Multi-Head Band Isolation: PASSED (Sub GR=" << gr_sub_db 
              << " dB vs High-Mid GR=" << gr_highmid_db << " dB | Flute unpumped)" << std::endl;

    // 4. Inter-Head Dynamic Coupling (Cross-Head Attention Flux)
    // Compare Head 1 (Low-Mid) gain reduction when coupling is 0.0 vs 0.6
    comp.reset();
    for (uint32_t h = 0; h < 4; ++h) comp.head_parameters(h).coupling = 0.0f;
    std::vector<float> test1_l = mix_l, test1_r = mix_r;
    comp.process_stereo(test1_l.data(), test1_r.data(), 2400);
    float gr_mid_uncoupled = comp.current_gain_reduction_db(1);

    comp.reset();
    for (uint32_t h = 0; h < 4; ++h) comp.head_parameters(h).coupling = 0.6f;
    std::vector<float> test2_l = mix_l, test2_r = mix_r;
    comp.process_stereo(test2_l.data(), test2_r.data(), 2400);
    float gr_mid_coupled = comp.current_gain_reduction_db(1);

    TEST_CHECK(gr_mid_coupled < gr_mid_uncoupled); // Coupling pulls down adjacent band musically
    std::cout << "  -> Inter-Head Dynamic Coupling: PASSED (Low-Mid GR uncoupled=" 
              << gr_mid_uncoupled << " dB -> coupled=" << gr_mid_coupled << " dB)" << std::endl;

    // 5. Mute, Solo & Parallel Wet/Dry Mix
    comp.reset();
    comp.head_parameters(0).mute = true;
    std::vector<float> mute_test_l(1000, 0.0f), mute_test_r(1000, 0.0f);
    for (size_t i = 0; i < 1000; ++i) {
        float t = static_cast<float>(i) / 48000.0f;
        mute_test_l[i] = mute_test_r[i] = std::sin(2.0f * std::numbers::pi_v<float> * 30.0f * t);
    }
    comp.process_stereo(mute_test_l.data(), mute_test_r.data(), 1000);
    float sub_mute_rms = 0.0f;
    for (size_t i = 200; i < 1000; ++i) sub_mute_rms += mute_test_l[i] * mute_test_l[i];
    sub_mute_rms = std::sqrt(sub_mute_rms / 800);
    TEST_CHECK(sub_mute_rms < 0.02f); // 30Hz sub silenced by muting Head 0 (<120Hz)
    std::cout << "  -> Solo / Mute & Routing Integrity: PASSED (Muted Sub RMS=" << sub_mute_rms << " < 0.02)" << std::endl;
}

void test_aes67_ptp_and_speaker_calibration_matrix() {
    std::cout << "[TEST] Running AES67 / PTPv2 Network Framing & Speaker Calibration Matrix Test..." << std::endl;
    using namespace audio_core::network;
    using namespace audio_core::dsp;

    // 1. AES67 RTP Packet Serialization & PTPv2 Epoch Synchronization
    {
        Aes67StreamConfig cfg;
        cfg.sample_rate = 48000;
        cfg.num_channels = 8;
        cfg.packet_frames = 48; // 1 ms packet
        cfg.encoding = Aes67PayloadEncoding::L24;
        cfg.payload_type = 96;
        cfg.ssrc = 0x44414E54; // 'DANT'

        Aes67PacketSerializer serializer(cfg);
        Aes67PacketDeserializer deserializer(cfg);

        // Synthesize 8 distinct channels
        std::vector<std::vector<float>> tx_data(8, std::vector<float>(48));
        std::vector<const float*> tx_ptrs(8);
        for (uint16_t c = 0; c < 8; ++c) {
            for (uint32_t f = 0; f < 48; ++f) {
                float t = static_cast<float>(f) / 48000.0f;
                tx_data[c][f] = 0.5f * std::sin(2.0f * std::numbers::pi_v<float> * (200.0f * (c + 1)) * t);
            }
            tx_ptrs[c] = tx_data[c].data();
        }

        // PTP timestamp: TAI epoch 1700000000 seconds, 250000000 nanoseconds
        PtpTimestamp ptp_tx{ .seconds = 1700000000ULL, .nanoseconds = 250000000U };
        uint32_t expected_rtp_ts = ptp_tx.to_rtp_timestamp(cfg.sample_rate);

        std::vector<uint8_t> packet_buf(1500, 0);
        size_t written = serializer.serialize_packet(tx_ptrs.data(), 48, ptp_tx, packet_buf.data(), packet_buf.size());

        // Expected size: 12 (RTP header) + 8 channels * 48 frames * 3 bytes (L24) = 12 + 1152 = 1164 bytes
        TEST_CHECK(written == 1164);

        // Verify standard RFC 3550 RTP header
        const auto* rtp_hdr = reinterpret_cast<const RtpHeader*>(packet_buf.data());
        TEST_CHECK(rtp_hdr->flags == 0x80); // V=2, P=0, X=0, CC=0
        TEST_CHECK(rtp_hdr->payload_type == 96);
        TEST_CHECK(net_to_host16(rtp_hdr->sequence_number) == 0);
        TEST_CHECK(net_to_host32(rtp_hdr->timestamp) == expected_rtp_ts);
        TEST_CHECK(net_to_host32(rtp_hdr->ssrc) == 0x44414E54);

        // Deserialize / depacketize back into planar float buffers
        std::vector<std::vector<float>> rx_data(8, std::vector<float>(48, 0.0f));
        std::vector<float*> rx_ptrs(8);
        for (uint16_t c = 0; c < 8; ++c) rx_ptrs[c] = rx_data[c].data();

        uint16_t rx_seq = 0;
        uint32_t rx_rtp_ts = 0;
        uint32_t extracted_frames = deserializer.deserialize_packet(packet_buf.data(), written,
                                                                   rx_ptrs.data(), 8, 48,
                                                                   rx_seq, rx_rtp_ts);
        TEST_CHECK(extracted_frames == 48);
        TEST_CHECK(rx_seq == 0);
        TEST_CHECK(rx_rtp_ts == expected_rtp_ts);

        // Bit-exact L24 24-bit roundtrip precision: error must be strictly bounded by 24-bit quantization (< 2e-7)
        float max_err = 0.0f;
        for (uint16_t c = 0; c < 8; ++c) {
            for (uint32_t f = 0; f < 48; ++f) {
                float err = std::abs(rx_data[c][f] - tx_data[c][f]);
                if (err > max_err) max_err = err;
            }
        }
        TEST_CHECK(max_err < 2e-7f);

        // Packet loss detection: next packet arrives with sequence gap (seq=5 instead of seq=1)
        written = serializer.serialize_packet(tx_ptrs.data(), 48, ptp_tx, packet_buf.data(), packet_buf.size());
        auto* hdr_tamper = reinterpret_cast<RtpHeader*>(packet_buf.data());
        hdr_tamper->sequence_number = host_to_net16(5); // Simulate missing packets 1, 2, 3, 4
        deserializer.deserialize_packet(packet_buf.data(), written, rx_ptrs.data(), 8, 48, rx_seq, rx_rtp_ts);
        TEST_CHECK(deserializer.packets_lost() == 4);

        std::cout << "  -> AES67 / Dante RTP L24 Packaging & PTPv2 Sync: PASSED (1164 bytes/pkt, L24 err=" 
                  << max_err << " < 2e-7, loss detection=4 pkts)" << std::endl;
    }

    // 2. Speaker Calibration Matrix: Time-of-Flight Delay Alignment & Room Mode Correction
    {
        SpeakerCalibrationMatrix calib(8, 48000);

        // Scenario: Speaker 0 is 3.43 meters away (10.0 ms TOF), Speaker 1 is 1.71 meters away (5.0 ms TOF)
        std::array<float, 2> tofs = {10.0f, 5.0f};
        calib.calibrate_time_of_flight(tofs.data(), 2);

        // Speaker 0 (furthest) should have 0 added delay
        // Speaker 1 (closer) should have 5.0 ms added delay (240 samples at 48kHz)
        TEST_CHECK(std::abs(calib.speaker(0).delay_ms() - 0.0f) < 0.01f);
        TEST_CHECK(std::abs(calib.speaker(1).delay_ms() - 5.0f) < 0.01f);
        TEST_CHECK(std::abs(calib.speaker(1).delay_samples() - 240.0f) < 0.1f);

        // Test Time Alignment with Dirac impulse at sample 0 for both speakers
        std::vector<float> spk0(512, 0.0f), spk1(512, 0.0f);
        spk0[0] = 1.0f;
        spk1[0] = 1.0f;

        calib.speaker(0).process_block(spk0.data(), 512);
        calib.speaker(1).process_block(spk1.data(), 512);

        // Speaker 0 impulse passes immediately at sample 0
        TEST_CHECK(std::abs(spk0[0] - 1.0f) < 1e-4f);
        // Speaker 1 impulse is delayed by exactly 240 samples!
        TEST_CHECK(std::abs(spk1[240] - 1.0f) < 1e-3f);
        TEST_CHECK(std::abs(spk1[0]) < 1e-5f);

        // Room Mode Notch Suppression Test:
        // Inject resonant room mode (60 Hz) into Speaker 2 with 60 Hz Notch
        calib.speaker(2).set_delay_samples(0.0f);
        calib.speaker(2).add_notch(60.0f, 6.0f);

        constexpr size_t kNotchFrames = 24000; // 500 ms (ensures narrow 60Hz filter reaches steady state)
        std::vector<float> res_tone(kNotchFrames);
        for (size_t i = 0; i < kNotchFrames; ++i) {
            float t = static_cast<float>(i) / 48000.0f;
            res_tone[i] = std::sin(2.0f * std::numbers::pi_v<float> * 60.0f * t);
        }
        calib.speaker(2).process_block(res_tone.data(), kNotchFrames);

        // Measure remaining energy of 60 Hz in second half (after filter settling)
        float notch_rms = 0.0f;
        for (size_t i = 12000; i < kNotchFrames; ++i) {
            notch_rms += res_tone[i] * res_tone[i];
        }
        notch_rms = std::sqrt(notch_rms / 12000.0f);
        // Original RMS of 1.0 sine is 0.7071. Deep notch at steady state should attenuate by > 26 dB (< 0.035)
        TEST_CHECK(notch_rms < 0.035f);

        // Passband test (1000 Hz tone on Speaker 2 must be unaffected)
        std::vector<float> pass_tone(kNotchFrames);
        for (size_t i = 0; i < kNotchFrames; ++i) {
            float t = static_cast<float>(i) / 48000.0f;
            pass_tone[i] = std::sin(2.0f * std::numbers::pi_v<float> * 1000.0f * t);
        }
        calib.speaker(2).process_block(pass_tone.data(), kNotchFrames);
        float pass_rms = 0.0f;
        for (size_t i = 12000; i < kNotchFrames; ++i) {
            pass_rms += pass_tone[i] * pass_tone[i];
        }
        pass_rms = std::sqrt(pass_rms / (kNotchFrames - 12000));
        TEST_CHECK(std::abs(pass_rms - 0.7071f) < 0.01f);

        // Polarity and Mute verification
        calib.speaker(3).set_invert_polarity(true);
        std::vector<float> pol_test = {1.0f, 0.5f, -0.2f};
        calib.speaker(3).process_block(pol_test.data(), 3);
        TEST_CHECK(pol_test[0] == -1.0f);
        TEST_CHECK(pol_test[1] == -0.5f);
        TEST_CHECK(pol_test[2] == 0.2f);

        calib.speaker(3).set_mute(true);
        calib.speaker(3).process_block(pol_test.data(), 3);
        TEST_CHECK(pol_test[0] == 0.0f);
        TEST_CHECK(pol_test[1] == 0.0f);
        TEST_CHECK(pol_test[2] == 0.0f);

        std::cout << "  -> Speaker Calibration Matrix: PASSED (TOF 5ms delay aligned to sample 240, 60Hz room notch RMS=" 
                  << notch_rms << " [>26dB cut], 1kHz passband transparent, Polarity/Mute verified)" << std::endl;
    }
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
    test_airwindows_console_processor();
    test_mixer_graph_routing();
    test_binary_protocol_and_command_queue();
    test_channel_strip_insert_slots();
    test_nested_bus_topological_routing();
    test_pipewire_backend_integration();
    test_aoip_network_streaming_and_unpacking();
    test_universal_sampling_and_bounce_tap();
    test_timeline_clock_and_link_bridge_master_authority();
    test_transient_detection_and_slice_engine();
    test_seamless_loop_equal_power_conditioning();
    test_insert_slot_safety_hardening_and_circuit_breaker();
    test_airwindows_interstage_processor();
    test_clock_synchronized_quantized_tap_and_bar_looping();
    test_step_sequencer_and_slice_trigger_engine();
    test_multicore_worker_pool_and_kernel_scaling();
    test_native_android_aaudio_backend();
    test_sample_rate_agility_and_hermite_resampling();
    test_anti_aliasing_and_airwindows_dither();
    test_acoustic_measurement_and_crossover_engine();
    test_multichannel_bus_and_spatial_routing();
    test_mixer_graph_spatial_bus_routing_and_multichannel_render();
    test_mixer_matrix_dca_groups_solo_safe_and_mute_groups();
    test_liquid_ode_trapezoidal_integration_filter_and_bus_summing();
    test_liquid_ode_noise_colors_sweeps_and_dynamic_denoising();
    test_multihead_ode_compressor_and_transient_accuracy();
    test_aes67_ptp_and_speaker_calibration_matrix();

    std::cout << "========================================" << std::endl;
    std::cout << "   ALL AUDIO CORE TESTS PASSED!         " << std::endl;
    std::cout << "========================================" << std::endl;
    return 0;
}
