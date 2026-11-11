#include "nomad_control_cpp/planning_supervisor.hpp"
#include <iostream>
#include <string>

using namespace std;

namespace {
int checks = 0, failures = 0;
void expect(bool value, const char* name) {
    ++checks;
    if (!value) { ++failures; cerr << "FAIL: " << name << '\n'; }
}
using State = nomad::PlanningSupervisor::State;
} // namespace

int main() {
    {   // Normal headless LPP STOP: pause, fresh command, resume without RESET.
        nomad::PlanningSupervisor s;
        expect(s.state() == State::Idle, "starts idle");
        s.activate();
        expect(s.active(), "start activates");
        s.onPaused(true);
        expect(s.state() == State::SoftStopped && !s.active() && !s.blocked(), "LPP pause soft-stops");
        s.onFreshCommand();
        expect(s.resumePending() && s.softStopped(), "fresh command marks resume pending");
        s.activate();
        expect(s.active() && !s.softStopped(), "resume returns to active");
    }
    {   // A pause that does not leave the raw controller stopped ends autonomy without soft stop.
        nomad::PlanningSupervisor s;
        s.activate();
        s.onPaused(false);
        expect(s.state() == State::Idle, "pause without stopped raw state goes idle");
        s.onFreshCommand();
        expect(s.state() == State::Idle, "fresh command does not resume from idle");
    }
    {   // Re-pausing a soft stop keeps it and drops a pending resume.
        nomad::PlanningSupervisor s;
        s.activate(); s.onPaused(true); s.onFreshCommand();
        s.onPaused(false);
        expect(s.state() == State::SoftStopped, "re-pause keeps soft stop, drops pending resume");
    }
    {   // Headless operator STOP / hardware fault latches until RESET.
        nomad::PlanningSupervisor s;
        s.activate();
        s.onStop(true);
        expect(s.blocked() && !s.active(), "headless stop blocks");
        s.onFreshCommand();
        s.onPaused(true);
        s.onPauseWhileFaulted();
        expect(s.blocked(), "fresh commands and pauses cannot clear a block");
        s.onReset();
        expect(s.state() == State::Idle, "reset clears block");
    }
    {   // A headless stop also overrides a soft stop.
        nomad::PlanningSupervisor s;
        s.activate(); s.onPaused(true); s.onFreshCommand();
        s.onStop(true);
        expect(s.blocked(), "stop during soft stop blocks");
    }
    {   // Raw controller leaving RUNNING by itself.
        nomad::PlanningSupervisor headless, gui;
        headless.activate(); headless.onRawNotRunning(true);
        expect(headless.blocked(), "headless raw exit blocks");
        gui.activate(); gui.onRawNotRunning(false);
        expect(gui.state() == State::Idle, "GUI-supervised raw exit only ends autonomy");
        nomad::PlanningSupervisor soft;
        soft.activate(); soft.onPaused(true); soft.onRawNotRunning(true);
        expect(soft.state() == State::SoftStopped, "raw exit while soft-stopped keeps soft stop");
    }
    {   // GUI-supervised stop does not latch.
        nomad::PlanningSupervisor s;
        s.activate(); s.onStop(false);
        expect(s.state() == State::Idle, "GUI stop ends autonomy without block");
    }
    {   // Pause while faulted but not yet blocked leaves autonomy.
        nomad::PlanningSupervisor s;
        s.activate(); s.onPauseWhileFaulted();
        expect(s.state() == State::Idle, "faulted pause leaves autonomy");
    }
    {   // Legacy resume path resets to idle so autonomy restarts through the start gate.
        nomad::PlanningSupervisor s;
        s.activate(); s.onPaused(true); s.onFreshCommand(); s.onResetResume();
        expect(s.state() == State::Idle, "legacy resume goes idle");
    }
    {   // Reset never interrupts an active session.
        nomad::PlanningSupervisor s;
        s.activate(); s.onReset();
        expect(s.active(), "reset keeps active session");
    }
    expect(string(nomad::planningSupervisorStateName(State::ResumePending)) == "RESUME_PENDING", "state names");
    cout << checks << " checks, " << failures << " failures\n";
    return failures ? 1 : 0;
}
