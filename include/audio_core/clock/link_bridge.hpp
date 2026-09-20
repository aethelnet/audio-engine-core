#pragma once

#define LINK_PLATFORM_LINUX 1
#include <ableton/Link.hpp>
#include "audio_core/clock/timeline_clock.hpp"
#include <memory>
#include <atomic>
#include <chrono>

namespace audio_core::clock {

// ============================================================================
// LinkBridge: Ableton Link Network Synchronizer with Master Authority
// Enforces local master sovereignty: Dictates tempo to peers, rejects remote overrides
// ============================================================================
class LinkBridge {
public:
    explicit LinkBridge(double initial_bpm = 120.0)
        : m_link(std::make_unique<ableton::Link>(initial_bpm)),
          m_quantum(4.0) {
        setup_callbacks();
    }

    ~LinkBridge() {
        if (m_link) {
            m_link->enable(false);
        }
    }

    void bind_clock(TimelineClock* clock) noexcept {
        m_clock = clock;
    }

    void enable(bool enabled) {
        if (m_link) {
            m_link->enable(enabled);
        }
    }

    [[nodiscard]] bool is_enabled() const {
        return m_link ? m_link->isEnabled() : false;
    }

    [[nodiscard]] uint64_t num_peers() const {
        return m_link ? m_link->numPeers() : 0;
    }

    void set_quantum(double quantum) noexcept {
        if (quantum > 0.0) {
            m_quantum = quantum;
        }
    }
    [[nodiscard]] double quantum() const noexcept {
        return m_quantum;
    }

    // Called inside RT Audio Render Loop: Synchronizes Link session state with TimelineClock
    void sync_audio_thread(uint32_t frames) noexcept {
        if (!m_link || !m_link->isEnabled() || !m_clock) {
            return;
        }

        const auto auth = m_clock->authority();
        if (auth == ClockAuthority::Isolated) {
            return;
        }

        auto state = m_link->captureAudioSessionState();
        const auto now = m_link->clock().micros();
        const double internal_bpm = m_clock->bpm();

        if (auth == ClockAuthority::Master) {
            // ================================================================
            // MASTER AUTHORITY ENFORCEMENT
            // Our internal clock is the absolute law.
            // If any network peer altered Link's tempo, we override it back!
            // ================================================================
            if (std::abs(state.tempo() - internal_bpm) > 0.001) {
                state.setTempo(internal_bpm, now);
            }

            // Sync beat phase to internal musical position
            if (m_clock->is_playing()) {
                const auto pos = m_clock->position_snapshot();
                state.requestBeatAtTime(pos.total_beats, now, m_quantum);
            }

            m_link->commitAudioSessionState(state);

        } else if (auth == ClockAuthority::Follower) {
            // Follower mode: Adopt external tempo if different
            const double link_bpm = state.tempo();
            if (std::abs(link_bpm - internal_bpm) > 0.001) {
                m_clock->set_bpm(link_bpm);
            }
        }
    }

    // Explicitly push current master tempo across network
    void broadcast_master_tempo(double bpm) {
        if (!m_link || !m_link->isEnabled()) return;
        auto state = m_link->captureAppSessionState();
        auto now = m_link->clock().micros();
        state.setTempo(bpm, now);
        m_link->commitAppSessionState(state);
    }

private:
    void setup_callbacks() {
        if (!m_link) return;

        // Anti-hijack filter: Intercept tempo changes from network peers
        m_link->setTempoCallback([this](double remote_tempo) {
            if (!m_clock) return;

            if (m_clock->authority() == ClockAuthority::Master) {
                // REJECT remote tempo change! Re-assert our internal master tempo
                const double master_bpm = m_clock->bpm();
                if (std::abs(remote_tempo - master_bpm) > 0.001) {
                    auto state = m_link->captureAppSessionState();
                    auto now = m_link->clock().micros();
                    state.setTempo(master_bpm, now);
                    m_link->commitAppSessionState(state);
                }
            } else if (m_clock->authority() == ClockAuthority::Follower) {
                m_clock->set_bpm(remote_tempo);
            }
        });
    }

    std::unique_ptr<ableton::Link> m_link;
    TimelineClock* m_clock{nullptr};
    double m_quantum{4.0};
};

} // namespace audio_core::clock
