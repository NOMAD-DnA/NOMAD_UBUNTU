#pragma once

namespace nomad {

// Who may drive the motor from LPP commands, as one explicit state instead of four flags.
//
//   Idle            no LPP output; a fresh command may start autonomy.
//   Active          LPP commands drive the raw controller.
//   SoftStopped     headless pause (normal LPP STOP, expiry, missing input); output released,
//                   resumes automatically on a fresh command. No RESET needed.
//   ResumePending   SoftStopped and a fresh command has arrived; resume when ready.
//   Blocked         headless operator STOP or hardware/driver/timer fault; RESET required.
//
// ROS-free so the stop/resume rules can be unit-tested without a node.
class PlanningSupervisor {
public:
    enum class State { Idle, Active, SoftStopped, ResumePending, Blocked };

    State state() const { return state_; }
    bool active() const { return state_ == State::Active; }
    bool blocked() const { return state_ == State::Blocked; }
    bool softStopped() const { return state_ == State::SoftStopped || state_ == State::ResumePending; }
    bool resumePending() const { return state_ == State::ResumePending; }

    // Autonomy started (from Idle) or resumed from a soft stop with drive output.
    void activate() { state_ = State::Active; }

    // A fresh LPP command arrived; a soft-stopped session may now resume.
    void onFreshCommand() {
        if (state_ == State::SoftStopped) state_ = State::ResumePending;
    }

    // Raw controller accepted a planning pause. A soft stop is kept (or entered) when the raw
    // controller is stopped; any pending resume is dropped.
    void onPaused(bool raw_stopped) {
        if (state_ == State::Blocked) return; // only RESET clears a block
        if (raw_stopped || softStopped()) state_ = State::SoftStopped;
        else state_ = State::Idle;
    }

    // Pause requested while blocked or faulted: leave autonomy without clearing a block.
    void onPauseWhileFaulted() {
        if (state_ != State::Blocked) state_ = State::Idle;
    }

    // Legacy (non measured-speed) resume: raw controller reset, autonomy restarts from Idle.
    void onResetResume() { state_ = State::Idle; }

    // Any stop request. Headless stops latch until RESET; GUI-supervised stops just end autonomy.
    void onStop(bool headless) {
        if (headless) state_ = State::Blocked;
        else if (state_ == State::Active) state_ = State::Idle;
    }

    // The raw controller left RUNNING on its own (fault, timeout, operator action).
    void onRawNotRunning(bool headless) {
        if (state_ == State::Active) state_ = headless ? State::Blocked : State::Idle;
    }

    // Accepted RESET in headless mode clears blocks and soft stops.
    void onReset() {
        if (state_ != State::Active) state_ = State::Idle;
    }

private:
    State state_ = State::Idle;
};

inline const char* planningSupervisorStateName(PlanningSupervisor::State state) {
    switch (state) {
    case PlanningSupervisor::State::Idle: return "IDLE";
    case PlanningSupervisor::State::Active: return "ACTIVE";
    case PlanningSupervisor::State::SoftStopped: return "SOFT_STOPPED";
    case PlanningSupervisor::State::ResumePending: return "RESUME_PENDING";
    case PlanningSupervisor::State::Blocked: return "BLOCKED";
    }
    return "UNKNOWN";
}

} // namespace nomad
