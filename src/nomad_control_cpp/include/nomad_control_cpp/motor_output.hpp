#pragma once
// The motor output stage (2026-11-20 refactor "5b", the same behaviour as controller_node.cpp releaseMotor() and
// publishOutput() up to 6ab4442): what one output call sends to the VESC driver topics. ROS-free, so the topic and
// value of every output can be unit tested (test/test_motor_output.cpp); before, only the ROS tests covered it.
// The node builds the inputs, publishes the returned command (servo first, then the motor topic) and writes the log
// lines the command asks for. Who does what (the whole path, step by step: controller_node.cpp above releaseMotor):
//   decides what is sent       -> here (MotorOutputStage) and stop_output.hpp (StopOutput::decide for stops)
//   builds the inputs          -> controller_node.cpp publishOutput / releaseMotor / stopOutputInput
//   publishes and writes logs  -> controller_node.cpp publishMotorCommand
// - Drive with a nonzero ERPM: the speed topic. The first one after not driving starts a drive: the command carries
//   the stop output state just before it (the node's [RESTART_TRACE] start line), then StopOutput::onDrive.
// - Drive with 0 ERPM, or Release: the stop output (stop_output.hpp StopOutput::decide), only when the node owns the
//   output and has a current publisher (can_release). A handbrake or brake the node has no publisher for falls back
//   to 0 A on the current topic. Driving ends either way.
// - observeTelemetry: the drive current and speed for the load-sized hold, only while driving.
// The next stop-output change (braking during the planning stop ramp, review 11-20) belongs here.
#include "nomad_control_cpp/stop_output.hpp"
#include "nomad_control_cpp/trial_controller.hpp"  // OutputKind

#include <optional>

namespace nomad {

enum class MotorTopic { None, Speed, Current, Brake, Handbrake };

// The stop output state just before a drive started ([RESTART_TRACE] start from= hold_s= stop_a=).
struct RestartFrom {
    std::optional<StopOutputKind> kind;  // nullopt: no stop output yet
    double hold_s = 0.0;
    double stop_a = 0.0;
};

struct MotorCommand {
    MotorTopic topic = MotorTopic::None;      // None: nothing on the motor topics
    double value = 0.0;                       // ERPM (Speed) or A (Current, Brake, Handbrake)
    std::optional<double> servo;              // the servo topic, published before the motor topic
    std::optional<RestartFrom> drive_started; // this command started a drive
    bool hold_started = false;                // a new handbrake hold began: the node logs [STOP_OUTPUT] once
    double hold_a = 0.0;                      // with hold_started: the hold current StopOutput chose (logged even if
                                              // the node has no handbrake publisher and sends 0 A)
    double moved_erev = 0.0;                  // with hold_started: the creep before the hold (StopOutput::movedErev)
    bool fault_release_started = false;       // first release with a latched VESC fault: the node warns once
};

class MotorOutputStage {
public:
    // stop: the node's StopOutput (configured, ramped and read for status by the node; this stage decides with it).
    explicit MotorOutputStage(StopOutput& stop) : stop_(stop) {}
    // Whether the node has a brake / handbrake publisher (set once, after the publishers are created).
    void setStopTopics(bool brake, bool handbrake) {
        brake_topic_ = brake;
        handbrake_topic_ = handbrake;
    }

    // One output of the trial controller's tick. steady: the time for RestartFrom::hold_s. can_release: the node owns
    // the output and has a current publisher. make_input(): the StopOutput inputs, called only for a release.
    template <class MakeInput>
    MotorCommand output(OutputKind kind, double erpm, std::optional<double> servo, double steady, bool can_release,
                        MakeInput&& make_input) {
        // Every OutputKind is handled (-Wswitch flags a new one, e.g. a braking ramp output); a value outside the
        // enum sends nothing, as None does.
        switch (kind) {
            case OutputKind::None: return {};
            case OutputKind::Release: return release(can_release, make_input);
            case OutputKind::Drive: return drive(erpm, servo, steady, can_release, make_input);
        }
        return {};
    }

    // Release the motor (also from the node's stop paths and shutdown): driving ends; the stop output when
    // can_release, else nothing.
    template <class MakeInput>
    MotorCommand release(bool can_release, MakeInput&& make_input) {
        driving_ = false;
        if (!can_release) return {};
        const StopOutputInput input = make_input();
        MotorCommand command;
        command.fault_release_started = input.vesc_fault && !fault_release_logged_;
        fault_release_logged_ = input.vesc_fault;
        const bool was_holding = stop_.holding();
        const double moved_erev = stop_.movedErev();  // reset when a hold begins
        const auto stop = stop_.decide(input);
        if (!was_holding && stop_.holding()) {
            command.hold_started = true;
            command.hold_a = stop.current_a;
            command.moved_erev = moved_erev;
        }
        if (stop.kind == StopOutputKind::Hold && handbrake_topic_) {
            command.topic = MotorTopic::Handbrake;
            command.value = stop.current_a;
        } else if (stop.kind == StopOutputKind::Brake && brake_topic_) {
            command.topic = MotorTopic::Brake;
            command.value = stop.current_a;
        } else {
            command.topic = MotorTopic::Current;
            command.value = 0.0;
        }
        return command;
    }

    // Each telemetry sample: the drive load for the load-sized hold, while driving.
    void observeTelemetry(double steady, double iq_a, double erpm) {
        if (driving_) stop_.observeDrive(steady, iq_a, erpm);
    }
    // The last motor output was a nonzero speed command.
    bool driving() const { return driving_; }

private:
    // Drive: 0 ERPM is a release; otherwise the speed topic, starting a drive when not driving (the stop output
    // state is taken before StopOutput::onDrive). The servo goes with either.
    template <class MakeInput>
    MotorCommand drive(double erpm, std::optional<double> servo, double steady, bool can_release, MakeInput& make_input) {
        MotorCommand command;
        if (erpm == 0.0) {
            command = release(can_release, make_input);
        } else {
            if (!driving_) {
                RestartFrom from;
                if (stop_.last()) {
                    from.kind = stop_.last()->kind;
                    from.stop_a = stop_.last()->current_a;
                }
                from.hold_s = stop_.holdSeconds(steady);
                command.drive_started = from;
            }
            driving_ = true;
            stop_.onDrive(erpm);
            command.topic = MotorTopic::Speed;
            command.value = erpm;
        }
        command.servo = servo;
        return command;
    }

    StopOutput& stop_;
    bool brake_topic_ = false, handbrake_topic_ = false;
    bool driving_ = false;
    bool fault_release_logged_ = false;
};

}  // namespace nomad
