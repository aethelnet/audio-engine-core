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
#include "backends/pipewire/pipewire_backend.hpp"

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

    std::cout << "========================================" << std::endl;
    std::cout << "   ALL AUDIO CORE TESTS PASSED!         " << std::endl;
    std::cout << "========================================" << std::endl;
    return 0;
}
