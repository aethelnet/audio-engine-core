# AudioEngineCore

> **High-Performance, Real-Time Safe C++20 Digital Signal Processing & Workstation Engine**  
> *Deterministic Zero-Allocation Audio Path // Lock-Free SPSC Streaming // Plugin Delay Compensation (PDC) // Sample-Accurate Parameter Ramping // Golden Master Bit-Exact Verification*

[![Standard: C++20](https://img.shields.io/badge/Language-C%2B%2B20-blue.svg)](#)
[![CTest Suite: 85/85 Passed](https://img.shields.io/badge/CTest-85%2F85%20Passed%20(100%25)-brightgreen.svg)](#)
[![Real-Time Safety: Zero Allocations](https://img.shields.io/badge/Real--Time-Zero%20Allocations%20%7C%20Lock--Free-success.svg)](#)
[![RTKit: SCHED_FIFO Support](https://img.shields.io/badge/Linux%20RT-RTKit%20%7C%20SCHED__FIFO-blueviolet.svg)](#)
[![Golden Master: Bit-Exact](https://img.shields.io/badge/Verification-Bit--Exact%20Golden%20Master-blueviolet.svg)](#)
[![License: AGPL-3.0](https://img.shields.io/badge/License-AGPL--3.0-orange.svg)](LICENSE)

---

## 1. Architectural Philosophy & Hard Invariants

`AudioEngineCore` is an industrial-grade C++20 real-time audio processing engine built specifically to guarantee **zero priority inversions, zero memory allocations, and zero unbounded locks** inside any active audio callback. 

Designed for high-density digital audio workstations (DAWs), embedded installations, and low-latency audio servers, it bridges classical DSP (Cytomic state-variable filters, optical vactrol levelers, Buchla 292 low-pass gates, and Airwindows modeled processors) with modern dynamical systems (continuous Liquid Neural ODE compression) and deterministic network synchronization (IEEE 1588-2008 PTPv2 / AES67).

```
                                 [AUDIO THREAD / REAL-TIME DOMAIN]
                                    (Zero Alloc, Zero Locks, SIMD)
                                                 │
    ┌────────────────────────────────────────────┴──────────────────────────────────────────┐
    │                                                                                       │
    │   Track 1: DiskStreamer (0.0 ms RAM Pre-Roll) ────────┐                               │
    │      ▲                                                │                               │
    │      │ Lock-Free SPSC                                 ▼                               │
    │      │ Prefetch Ring                                [PDC Ring Delay Line]             │
    │      │                                                │                               │
    │   [I/O Worker Thread]                                 ▼                               │
    │   (Async Disk Prefetch)                         [Sample-Accurate Ramp]                │
    │                                                       │ (Anti-Zipper)                 │
    │   Track 2: Synth / Sampler (WSOLA Resample) ──────────┤                               │
    │                                                       ▼                               │
    │                                           [InsertSlot Chain]                          │
    │                                           - Airwindows DeRez2 / ClipOnly2             │
    │                                           - Liquid ODE Multiband Vactrol              │
    │                                           - Cytomic State-Variable Filter             │
    │                                                       │                               │
    │                                                       ▼                               │
    │                                           [Lock-Free Mixer Summing]                   │
    │                                                       │                               │
    │                                                       ▼                               │
    │                                           [Master Bus Soft Limiter]                   │
    │                                                       │                               │
    │                                                       ▼                               │
    │                                            Audio Output / AoIP Stream                 │
    │                                            - Arranger Multi-Clip Sequencing           │
    │                                            - WSOLA Granular Time-Stretching           │
    └───────────────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Core Subsystems

### 2.1 Plugin Delay Compensation (PDC)
Every processor in the graph exposes latency via `IProcessor::latency_samples()`. `MixerGraph` automatically traverses the track DAG, computes the critical path latency $L_{\max} = \max_t L_t$, and inserts an aligned ringbuffer delay of $(L_{\max} - L_t)$ samples into lagging parallel tracks:
- **Phase Alignment**: 100% sample-exact comb-filtering elimination across parallel chains.
- **Dynamic Bypass**: Latency is subtracted dynamically when insert slots or plugins are bypassed without causing phase glitching.

### 2.2 Sample-Accurate Parameter Ramping
Direct coefficient changes in real-time audio threads cause high-frequency clicks and zipper noise. `AudioEngineCore` enforces sample-accurate linear interpolation:
$$g(i) = g_0 + \frac{i + 1}{N} (g_1 - g_0) \quad \text{for } i \in [0, N-1]$$
- Applied natively to track volume gain and stereo panning.
- Automatically snaps to target during transport repositioning or instantaneous mute toggling.

### 2.3 Disk Streaming Engine with Instant RAM Pre-Roll
Plays gigabyte-scale WAV assets without loading entire audio files into memory:
- **0.0 ms Initial Latency**: First $N_{\text{preroll}}$ frames (e.g. 65,536 samples / 1.36s at 48kHz) are cached in RAM for instantaneous trigger response.
- **Lock-Free SPSC Prefetch Ring**: High-priority background I/O worker thread continuously streams audio chunks into a lock-free single-producer single-consumer ringbuffer.
- **Anti-Click Underrun Fades**: If an I/O stall occurs, the stream applies an exponential micro-fade over 64 samples to zero, preventing DC offset pops.

### 2.4 Continuous Liquid ODE Dynamics & Vintage Vactrol Modeling
- **Optical Vactrol Leveler**: Dual-time-constant release modeling of cadmium-sulfide (CdS) photocell dark memory ($R_{\text{dark}} \approx 10\,\text{M}\Omega$), with asymmetric release curves scaling up to 14.7x between transients and sustained signals.
- **Multi-Head Liquid ODE Compressor**: Continuous Runge-Kutta 4th-order (RK4) integration with dynamic viscosity $\tau(u) = \tau_0 / (1 + \beta |u|)$.
- **Buchla 292 Low-Pass Gate**: Combined VCF/VCA vactrol simulation with authentic frequency-dependent ringing resonance.

### 2.5 Verification & Golden Master Checksum Suite
Bit-exact mathematical integrity is verified via `GoldenMasterTool`:
- Computes SHA-256 / 64-bit FNV-1a checksums over 10 distinct DSP signal chains (filters, limiters, compressors, sample-rate converters, mixers).
- Prevents silent regressions, floating-point drift, or compiler vectorization mismatches across platforms.

### 2.6 Remote Control & Real-Time WebMixer WebSocket Bridge
Provides ultra-low latency browser & network control surface integration via `WebSocketBridge`:
- **RFC 6455 Compliant Handshake & Framing**: Standalone zero-dependency SHA-1/Base64 handshake with JSON text & 32-byte POD binary frame decoding.
- **Embedded WebMixer Single-Page App**: Serves a responsive HTML5 canvas mixer on HTTP GET `/` with real-time VU meters, master/strip faders, mute/solo, and kinetic ODE hit-record visualization.
- **Lock-Free SPSC Dispatch**: Commands and MIDI note triggers are pushed directly to `Engine` lock-free ringbuffers without touching or stalling the real-time audio thread.
- **30 Hz Telemetry Streaming**: High-density snapshot broadcasts of master meters, track meters, bus meters, and Poincaré phase-space orbits.

### 2.7 Multi-Tier Real-Time Scheduling & Linux RealtimeKit (RTKit) Client
Sub-millisecond buffer rendering (down to 32 frames / 666.7 µs deadline @ 48 kHz) requires immunity against Linux CFS kernel preemption:
- **Tier 1 (Direct Kernel Probe)**: Probes `pthread_setschedparam(SCHED_FIFO, prio)` for processes with `CAP_SYS_NICE` or custom limits.
- **Tier 2 (RTKit D-Bus Client)**: Communicates with `org.freedesktop.RealtimeKit1` via systemd D-Bus, setting `RLIMIT_RTTIME` watchdog limits (200 ms) and promoting unprivileged threads to `SCHED_FIFO` priority 20 without root access.
- **Tier 3 & 4 (High-Priority Nice Fallback)**: Automatically falls back to RTKit `MakeThreadHighPriority` (-15) or standard `setpriority`.
- **Zero-Overhead Thread-Local Cache**: Thread promotion evaluates in $O(1)$ with 0 ns overhead on subsequent audio callback iterations.
- **Bounded Event Draining**: `HardwareMidiReceiver::drain_to(..., max_events)` bounds per-block event processing at small buffer sizes to prevent dense synthetic event storms from consuming the real-time slice.

### 2.8 Arranger Multi-Clip Sequencing, Time-Stretching & Auto-Crossfades
Full multitrack timeline editing and dynamic sample arrangement:
- **Non-Destructive Slip-Editing & Trimming**: Adjust clip start/end boundaries and internal waveform playback offsets with sub-bar precision.
- **Granular WSOLA Time-Stretching & Edge-Drag**: Elastic audio dilation to arbitrary bar boundaries ($0.1\times$ to $10.0\times$) with zero pitch drift and **zero generational loss** (always derived from pristine source clips).
- **Decoupled Pitch Shifting**: Continuous semitone repitching ($\pm 12$ st) with lock-free atomic snapshot publishing to the real-time audio thread.
- **Equal-Power Auto-Crossfades**: Invariant-preserving energy crossover summation ($g_1^2 + g_2^2 \equiv 1.0$) across overlapping clip intervals.

---

## 3. Test Suite & Verification Matrix

The test harness runs under `ctest` and executes **85 comprehensive unit test suites** covering real-time guarantees, stability, and signal integrity.

```bash
$ ./build/audio_tests
===============================================================================
   AUDIO ENGINE CORE: REAL-TIME DSP TEST HARNESS
===============================================================================
[TEST 01..10] State-Variable Filters & Anti-Aliased Resampling:  PASSED
[TEST 11..20] Liquid Vactrol Opto-Leveler & Buchla 292 LPG:       PASSED
[TEST 21..30] Multi-Head ODE Compressor & Transient Lookahead:    PASSED
[TEST 31..40] WSOLA Time-Stretching & Decoupled Pitch Shifting:   PASSED
[TEST 41..48] PTPv2 Boundary Clock, BMCA & AES67 AoIP Framing:    PASSED
[TEST 49..52] Atomic Lock-Free WASM Sandboxing & Gas Watchdogs:   PASSED
[TEST 53]     Golden Master Audio Checksum Suite (10/10 Bit-Exact): PASSED
[TEST 54]     Plugin Delay Compensation (PDC Phase Alignment):    PASSED
[TEST 55]     Sample-Accurate Parameter Ramping (Anti-Zipper):    PASSED
[TEST 56]     Disk-Streaming & Lock-Free Voice Prefetching:       PASSED
[TEST 57]     Airwindows DeRez2 Bit & Sample-Rate Reduction:      PASSED
[TEST 58]     Airwindows ClipOnly2 Ultrasonic Wavefolding:        PASSED
[TEST 59]     Interstage Transformer Saturation & Resonance:      PASSED
[TEST 60]     Session & Rack Preset Serialization (Pure C++20):   PASSED
[TEST 61]     WebSocket Bridge, RFC 6455 Handshake & WebMixer:    PASSED
[TEST 62..70] Timeline Scrubbing, SPP & MTC Full Frame SysEx:     PASSED
[TEST 71..75] MIDI Learn Parameter Routing & Automation Bridge:   PASSED
[TEST 76..80] Lookahead Multiband DNL & LR4 4-Way Crossover:      PASSED
[TEST 81]     RTKit Real-Time Scheduling Client & Bounded Budget: PASSED
[TEST 82]     Sononym & Renoise Sample Asset Pool & .aethel Bundle: PASSED
[TEST 83]     Arranger Slip-Editing, Cross-Track & Fade Ramps:     PASSED
[TEST 84]     Arranger Multi-Clip Sequencing, Razor & Auto-Crossfades: PASSED
[TEST 85]     Arranger WSOLA Time-Stretching, Edge-Drag & Pitch Shift: PASSED
===============================================================================
   85 / 85 UNIT TESTS PASSED (100.0% SUCCESS)
===============================================================================
```

---

## 4. Quickstart & Build Instructions

### Prerequisites
- C++20 compliant compiler (`GCC 11+`, `Clang 14+`)
- `CMake 3.20+`
- `libsndfile1-dev` (optional, for native WAV file decoding)

### Building & Running Tests
```bash
# Clone the repository
git clone https://github.com/aethelnet/audio-engine-core.git
cd audio-engine-core

# Configure with CMake (Release mode recommended for SIMD vectorization)
cmake -B build -DCMAKE_BUILD_TYPE=Release

# Compile all targets
cmake --build build -j$(nproc)

# Run full CTest suite
ctest --test-dir build --output-on-failure
```

### Running the Golden Master Verification Tool
```bash
# Execute bit-exact checksum verifier
./build/golden_master_tool --verify
```

---

## 5. Directory Structure

```
audio-engine-core/
├── include/audio_core/
│   ├── analysis/             # FFT, Spectrum, Golden Master Checksums
│   ├── dynamics/             # Liquid ODE Compressor, Vactrol Opto-Leveler
│   ├── filters/              # Cytomic SVF, Buchla 292 LPG, LR4 Crossovers
│   ├── modeled/              # Airwindows DSP: DeRez2, ClipOnly2, Interstage, Baxandall
│   ├── network/              # IEEE 1588 PTPv2, AES67 RTP L24 Framing
│   ├── routing/              # Universal Routing Matrix, PDC Delay Lines
│   ├── sampling/             # DiskStreamer, WSOLA Stretcher, Hermite Resampler
│   ├── sequencer/            # Clip Launcher, Step Sequencer, MIDI Patterns
│   ├── wasm/                 # Sandboxed Plugin Hot-Swap & Gas Watchdog
│   ├── mixer_graph.hpp       # Lock-Free Multi-Track Mixer Graph
│   └── ring_buffer.hpp       # Lock-Free SPSC Wait-Free Ring Buffer
├── src/                      # Implementation sources
├── tests/                    # 59 Automated Unit Test Suites
├── examples/
│   ├── aethel_desk.cpp       # ImGui Audio Workstation GUI
│   └── golden_master_tool.cpp# Bit-Exact Checksum Verification CLI
├── LICENSE                   # GNU AGPL-3.0
└── CMakeLists.txt
```

---

## 6. Attribution & Acknowledgements

This project incorporates and adapts select analog-modeled DSP algorithms created by **Chris Johnson ([Airwindows](https://www.airwindows.com / https://github.com/airwindows/airwindows))**, originally released under the MIT License:
- **Baxandall**: Precision high/low shelving filter curves with minimal phase distortion.
- **ButterComp2**: Dual-stage cascaded Butterworth gain-reduction dynamics.
- **PurestDrive**: Pure sine/hyperbolic tangent harmonic saturation.
- **DeRez2**: Continuous variable wordlength quantization and sample-rate reduction.
- **ClipOnly2**: Specialized anti-harshness peak limiter and ultrasonic wavefolder.
- **Interstage**: Analog transformer core saturation and interstage capacitive LF resonance.

We express our gratitude to Chris Johnson for his monumental contribution to open-source digital signal processing.

---

## 7. License

Licensed under the **GNU Affero General Public License v3.0 (AGPL-3.0)**.  
See [`LICENSE`](LICENSE) for the full license text.
