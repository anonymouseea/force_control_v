#pragma once

#include "Admittance.h"
#include "ControlHardware.h"

enum class CycleResult { Success, ReadFailed, InvalidForce, InverseFailed, LimitExceeded, SendFailed };

// 只负责运行时的力控计算，不操作伺服上电、下电和跟踪开关。
class ControlCycle {
public:
    ControlCycle();
    bool synchronize(ControlHardware& hardware, bool rotation_mode, bool small_sensor);
    CycleResult execute(ControlHardware& hardware);
    void clearVelocity();
    int lastErrorCode() const { return last_error_code_; }

private:
    Admittance4 controller_;
    RobotSnapshot origin_;
    bool rotation_mode_ = false;
    bool small_sensor_ = false;
    ForceSample filtered_ = {{0, 0, 0, 0, 0, 0}};
    int last_error_code_ = 0;
};
