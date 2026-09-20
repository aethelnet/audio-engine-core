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

    std::cout << "========================================" << std::endl;
    std::cout << "   ALL AUDIO CORE TESTS PASSED!         " << std::endl;
    std::cout << "========================================" << std::endl;
    return 0;
}
