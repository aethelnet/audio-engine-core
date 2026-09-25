#pragma once

#include "audio_core/types.hpp"
#include "audio_core/sequencer/arranger_track.hpp"
#include "audio_core/insert_slot.hpp"
#include "audio_core/mixer_graph.hpp"

#include <vector>
#include <memory>
#include <string>
#include <functional>
#include <deque>
#include <utility>
#include <algorithm>
#include <iostream>

namespace audio_core::undo {

// ============================================================================
// IUndoAction: Reversible Command Interface for Real-Time DAW Workstation
// Encapsulates Arranger Edits, Routing Changes, Track and Plugin Parameters.
// ============================================================================
class IUndoAction {
public:
    virtual ~IUndoAction() = default;

    [[nodiscard]] virtual std::string description() const = 0;
    virtual void undo() = 0;
    virtual void redo() = 0;

    // Optional action coalescing (e.g. continuous fader drags)
    [[nodiscard]] virtual bool can_merge(const IUndoAction& /*next*/) const { return false; }
    virtual void merge(std::unique_ptr<IUndoAction> /*next*/) {}
};

// ============================================================================
// ArrangerSingleTrackAction: Timeline Clip Mutations on a Single Track
// Handles Razor Splits, Trim, Move, Fade, Slip, Warp Pins, Delete, Duplicate.
// ============================================================================
class ArrangerSingleTrackAction : public IUndoAction {
public:
    ArrangerSingleTrackAction(Track* track,
                              std::vector<sequencer::ArrangerClipInstance> old_clips,
                              std::vector<sequencer::ArrangerClipInstance> new_clips,
                              std::string description)
        : m_track(track),
          m_old_clips(std::move(old_clips)),
          m_new_clips(std::move(new_clips)),
          m_description(std::move(description)) {}

    [[nodiscard]] std::string description() const override {
        return m_description;
    }

    void undo() override {
        if (!m_track) return;
        m_track->arranger().clips() = m_old_clips;
        m_track->arranger().update_auto_crossfades();
        m_track->arranger().publish_snapshot();
    }

    void redo() override {
        if (!m_track) return;
        m_track->arranger().clips() = m_new_clips;
        m_track->arranger().update_auto_crossfades();
        m_track->arranger().publish_snapshot();
    }

private:
    Track* m_track{nullptr};
    std::vector<sequencer::ArrangerClipInstance> m_old_clips;
    std::vector<sequencer::ArrangerClipInstance> m_new_clips;
    std::string m_description;
};

// ============================================================================
// ArrangerMultiTrackAction: Timeline Mutations Spanning Multiple Tracks
// (e.g. Cross-Track Drag & Drop, Multi-Track Split, Ripple Edits)
// ============================================================================
class ArrangerMultiTrackAction : public IUndoAction {
public:
    struct TrackState {
        Track* track{nullptr};
        std::vector<sequencer::ArrangerClipInstance> old_clips;
        std::vector<sequencer::ArrangerClipInstance> new_clips;
    };

    ArrangerMultiTrackAction(std::vector<TrackState> states, std::string description)
        : m_states(std::move(states)), m_description(std::move(description)) {}

    [[nodiscard]] std::string description() const override {
        return m_description;
    }

    void undo() override {
        for (auto& s : m_states) {
            if (s.track) {
                s.track->arranger().clips() = s.old_clips;
                s.track->arranger().update_auto_crossfades();
                s.track->arranger().publish_snapshot();
            }
        }
    }

    void redo() override {
        for (auto& s : m_states) {
            if (s.track) {
                s.track->arranger().clips() = s.new_clips;
                s.track->arranger().update_auto_crossfades();
                s.track->arranger().publish_snapshot();
            }
        }
    }

private:
    std::vector<TrackState> m_states;
    std::string m_description;
};

// ============================================================================
// TrackRoutingAction: Target Submix Bus or Master Routing Mutation
// ============================================================================
class TrackRoutingAction : public IUndoAction {
public:
    TrackRoutingAction(Track* track, int32_t old_bus_id, int32_t new_bus_id,
                       std::string track_name, std::string bus_name)
        : m_track(track),
          m_old_bus_id(old_bus_id),
          m_new_bus_id(new_bus_id),
          m_track_name(std::move(track_name)),
          m_bus_name(std::move(bus_name)) {}

    [[nodiscard]] std::string description() const override {
        return "Route Track '" + m_track_name + "' -> " + m_bus_name;
    }

    void undo() override {
        if (m_track) {
            m_track->set_target_bus(m_old_bus_id);
        }
    }

    void redo() override {
        if (m_track) {
            m_track->set_target_bus(m_new_bus_id);
        }
    }

private:
    Track* m_track{nullptr};
    int32_t m_old_bus_id{0};
    int32_t m_new_bus_id{0};
    std::string m_track_name;
    std::string m_bus_name;
};

// ============================================================================
// TrackGainPanAction: Volume Gain or Stereo Panning Adjustments
// ============================================================================
class TrackGainPanAction : public IUndoAction {
public:
    enum class TargetProperty { Gain, Pan };

    TrackGainPanAction(Track* track, TargetProperty prop, float old_val, float new_val, std::string track_name)
        : m_track(track),
          m_prop(prop),
          m_old_val(old_val),
          m_new_val(new_val),
          m_track_name(std::move(track_name)) {}

    [[nodiscard]] std::string description() const override {
        if (m_prop == TargetProperty::Gain) {
            return "Set Track '" + m_track_name + "' Gain";
        } else {
            return "Set Track '" + m_track_name + "' Pan";
        }
    }

    void undo() override {
        if (!m_track) return;
        if (m_prop == TargetProperty::Gain) {
            m_track->set_gain(m_old_val);
        } else {
            m_track->set_pan(m_old_val);
        }
    }

    void redo() override {
        if (!m_track) return;
        if (m_prop == TargetProperty::Gain) {
            m_track->set_gain(m_new_val);
        } else {
            m_track->set_pan(m_new_val);
        }
    }

private:
    Track* m_track{nullptr};
    TargetProperty m_prop;
    float m_old_val;
    float m_new_val;
    std::string m_track_name;
};

// ============================================================================
// TrackMuteSoloAction: Mute / Solo State Toggling
// ============================================================================
class TrackMuteSoloAction : public IUndoAction {
public:
    enum class TargetProperty { Mute, Solo };

    TrackMuteSoloAction(Track* track, TargetProperty prop, bool old_val, bool new_val, std::string track_name)
        : m_track(track),
          m_prop(prop),
          m_old_val(old_val),
          m_new_val(new_val),
          m_track_name(std::move(track_name)) {}

    [[nodiscard]] std::string description() const override {
        if (m_prop == TargetProperty::Mute) {
            return std::string(m_new_val ? "Mute" : "Unmute") + " Track '" + m_track_name + "'";
        } else {
            return std::string(m_new_val ? "Solo" : "Unsolo") + " Track '" + m_track_name + "'";
        }
    }

    void undo() override {
        if (!m_track) return;
        if (m_prop == TargetProperty::Mute) {
            m_track->set_mute(m_old_val);
        } else {
            m_track->set_solo(m_old_val);
        }
    }

    void redo() override {
        if (!m_track) return;
        if (m_prop == TargetProperty::Mute) {
            m_track->set_mute(m_new_val);
        } else {
            m_track->set_solo(m_new_val);
        }
    }

private:
    Track* m_track{nullptr};
    TargetProperty m_prop;
    bool m_old_val;
    bool m_new_val;
    std::string m_track_name;
};

// ============================================================================
// ProcessorParameterAction: Parameter Changes on Any IProcessor (Native & VST)
// ============================================================================
class ProcessorParameterAction : public IUndoAction {
public:
    ProcessorParameterAction(std::shared_ptr<IProcessor> proc,
                             uint32_t param_idx,
                             float old_val,
                             float new_val,
                             std::string proc_name,
                             std::string param_name)
        : m_proc(std::move(proc)),
          m_param_idx(param_idx),
          m_old_val(old_val),
          m_new_val(new_val),
          m_proc_name(std::move(proc_name)),
          m_param_name(std::move(param_name)) {}

    [[nodiscard]] std::string description() const override {
        return "Tweak " + m_proc_name + " [" + m_param_name + "]";
    }

    void undo() override {
        if (m_proc) {
            m_proc->set_parameter(m_param_idx, m_old_val);
        }
    }

    void redo() override {
        if (m_proc) {
            m_proc->set_parameter(m_param_idx, m_new_val);
        }
    }

    [[nodiscard]] bool can_merge(const IUndoAction& next) const override {
        const auto* other = dynamic_cast<const ProcessorParameterAction*>(&next);
        if (!other) return false;
        return m_proc == other->m_proc && m_param_idx == other->m_param_idx;
    }

    void merge(std::unique_ptr<IUndoAction> next) override {
        auto* other = dynamic_cast<ProcessorParameterAction*>(next.get());
        if (other) {
            m_new_val = other->m_new_val; // Keep original m_old_val, update target m_new_val
        }
    }

private:
    std::shared_ptr<IProcessor> m_proc;
    uint32_t m_param_idx;
    float m_old_val;
    float m_new_val;
    std::string m_proc_name;
    std::string m_param_name;
};

// ============================================================================
// InsertSlotAction: Hot-swapping, Loading, or Removing Insert Processors
// ============================================================================
class InsertSlotAction : public IUndoAction {
public:
    InsertSlotAction(InsertSlot* slot,
                     std::shared_ptr<IProcessor> old_proc,
                     std::shared_ptr<IProcessor> new_proc,
                     std::string desc)
        : m_slot(slot),
          m_old_proc(std::move(old_proc)),
          m_new_proc(std::move(new_proc)),
          m_description(std::move(desc)) {}

    [[nodiscard]] std::string description() const override {
        return m_description;
    }

    void undo() override {
        if (m_slot) {
            m_slot->set_processor(m_old_proc);
        }
    }

    void redo() override {
        if (m_slot) {
            m_slot->set_processor(m_new_proc);
        }
    }

private:
    InsertSlot* m_slot{nullptr};
    std::shared_ptr<IProcessor> m_old_proc;
    std::shared_ptr<IProcessor> m_new_proc;
    std::string m_description;
};

// ============================================================================
// CustomLambdaAction: Generic Functional Action with Custom Undo/Redo Lambdas
// ============================================================================
class CustomLambdaAction : public IUndoAction {
public:
    CustomLambdaAction(std::string description,
                       std::function<void()> undo_fn,
                       std::function<void()> redo_fn)
        : m_description(std::move(description)),
          m_undo_fn(std::move(undo_fn)),
          m_redo_fn(std::move(redo_fn)) {}

    [[nodiscard]] std::string description() const override {
        return m_description;
    }

    void undo() override {
        if (m_undo_fn) m_undo_fn();
    }

    void redo() override {
        if (m_redo_fn) m_redo_fn();
    }

private:
    std::string m_description;
    std::function<void()> m_undo_fn;
    std::function<void()> m_redo_fn;
};

// ============================================================================
// UndoManager: Central Command History Engine for Digital Audio Workstations
// Manages dual Undo / Redo stacks with bounded capacity and action merging.
// ============================================================================
class UndoManager {
public:
    explicit UndoManager(size_t max_history = 128) noexcept
        : m_max_history(std::max<size_t>(max_history, 4)) {}

    // Push a new action performed by the user
    void push_action(std::unique_ptr<IUndoAction> action) {
        if (!action) return;

        // Try merging with top of undo stack (e.g. continuous slider drags)
        if (!m_undo_stack.empty() && m_undo_stack.back()->can_merge(*action)) {
            m_undo_stack.back()->merge(std::move(action));
            m_redo_stack.clear(); // Any new action invalidates redo stack
            return;
        }

        m_undo_stack.push_back(std::move(action));
        if (m_undo_stack.size() > m_max_history) {
            m_undo_stack.pop_front();
        }

        // Standard DAW invariant: Performing any new action clears the Redo history
        m_redo_stack.clear();
    }

    // Execute Undo
    bool undo() {
        if (m_undo_stack.empty()) return false;

        auto action = std::move(m_undo_stack.back());
        m_undo_stack.pop_back();

        // Perform reverse operation
        action->undo();

        // Push to redo stack
        m_redo_stack.push_back(std::move(action));
        if (m_redo_stack.size() > m_max_history) {
            m_redo_stack.pop_front();
        }
        return true;
    }

    // Execute Redo
    bool redo() {
        if (m_redo_stack.empty()) return false;

        auto action = std::move(m_redo_stack.back());
        m_redo_stack.pop_back();

        // Perform forward operation
        action->redo();

        // Push back to undo stack
        m_undo_stack.push_back(std::move(action));
        if (m_undo_stack.size() > m_max_history) {
            m_undo_stack.pop_front();
        }
        return true;
    }

    [[nodiscard]] bool can_undo() const noexcept {
        return !m_undo_stack.empty();
    }

    [[nodiscard]] bool can_redo() const noexcept {
        return !m_redo_stack.empty();
    }

    [[nodiscard]] std::string undo_description() const {
        if (m_undo_stack.empty()) return "";
        return m_undo_stack.back()->description();
    }

    [[nodiscard]] std::string redo_description() const {
        if (m_redo_stack.empty()) return "";
        return m_redo_stack.back()->description();
    }

    [[nodiscard]] size_t undo_count() const noexcept {
        return m_undo_stack.size();
    }

    [[nodiscard]] size_t redo_count() const noexcept {
        return m_redo_stack.size();
    }

    void clear() noexcept {
        m_undo_stack.clear();
        m_redo_stack.clear();
    }

    void set_max_history(size_t max_history) noexcept {
        m_max_history = std::max<size_t>(max_history, 4);
        while (m_undo_stack.size() > m_max_history) m_undo_stack.pop_front();
        while (m_redo_stack.size() > m_max_history) m_redo_stack.pop_front();
    }

private:
    size_t m_max_history{128};
    std::deque<std::unique_ptr<IUndoAction>> m_undo_stack;
    std::deque<std::unique_ptr<IUndoAction>> m_redo_stack;
};

} // namespace audio_core::undo
