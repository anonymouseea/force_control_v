#pragma once

#include <array>
#include <chrono>

using ControlClock = std::chrono::steady_clock;
using ControlTime = ControlClock::time_point;
using JointTarget = std::array<double, 7>;
using ForceSample = std::array<double, 6>;

enum class ControlState {
    Disabled, Starting, PositionControl, RotationControl,
    Stopping, Zeroing, Fault
};

enum class StartStep {
    CheckStopped, ClearErrors, ZeroSensor, SetServoReady,
    PowerOn, OpenTracking, WaitServoEnabled, Synchronize
};

enum class StopStep {
    StopTracking, WaitRobotStopped, PowerOff, WaitServoDisabled
};

enum class ZeroResult { Pending, Complete, Failed };

struct ControlInputs {
    bool force_on = false;
    bool zero_on = false;
    bool small_sensor = false;
    bool rotation_mode = false;
};

struct RobotSnapshot {
    // 平移为米，姿态角为弧度；与原项目的坐标约定一致。
    std::array<double, 4> pose = {{0, 0, 0, 0}};
    JointTarget joints = {{0, 0, 0, 0, 0, 0, 0}};
};
