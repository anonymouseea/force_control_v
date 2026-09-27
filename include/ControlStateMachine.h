#pragma once

#include "ControlCycle.h"

// 所有时间均由调用者传入，离线测试可以推进虚拟时间，不需要真实等待。
class ControlStateMachine {
public:
    explicit ControlStateMachine(ControlHardware& hardware);
    void update(const ControlInputs& inputs, ControlTime now, bool exit_requested = false);
    void reportFault(const char* reason, ControlTime now, int code = 0);
    ControlState state() const { return state_; }
    StartStep startStep() const { return start_step_; }
    StopStep stopStep() const { return stop_step_; }
    bool isRunning() const;
    bool exitFinished() const;
    bool exitSucceeded() const;
    bool cleanupFailed() const { return cleanup_failed_; }
    long periodNanoseconds() const;

private:
    void transition(ControlState next);
    void beginStart(ControlTime now);
    void advanceStart(const ControlInputs& inputs, ControlTime now);
    void beginStop(ControlTime now);
    void advanceStop(ControlTime now);
    void finishStop(ControlTime now);
    void failCleanup(const char* reason, int code = 0);
    void requestOff();
    void beginZero(ControlTime now);
    void advanceZero(ControlTime now);
    void executeCycle(ControlTime now);

    ControlHardware& hardware_;
    ControlCycle cycle_;
    ControlState state_ = ControlState::Disabled;
    StartStep start_step_ = StartStep::CheckStopped;
    StopStep stop_step_ = StopStep::StopTracking;
    ControlTime deadline_{};
    ControlTime next_sample_{};
    ControlTime stable_since_{};
    JointTarget stable_reference_ = {{0, 0, 0, 0, 0, 0, 0}};
    bool have_stable_reference_ = false;
    bool power_attempted_ = false;
    bool tracking_attempted_ = false;
    bool fault_latched_ = false;
    bool cleanup_failed_ = false;
    bool exit_requested_ = false;
    bool zero_after_stop_ = false;
    bool armed_ = false;
    bool last_force_on_ = false;
    bool last_zero_on_ = false;
    bool small_sensor_ = false;
};
