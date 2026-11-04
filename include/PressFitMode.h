#pragma once

#include "Admittance.h"
#include "PressFit.h"
#include "RobotUtils.h"

// 负责下压及贴合目标计算。开关、启停、限位与发送由主循环统一处理。
class PressFitMode {
public:
    PressFitMode();
    bool active() const { return press_.phase() != PressPhase::Idle; }
    void reset();
    bool begin(const MyRobotState& actual, const SensorData& force, bool small, double now);
    bool calculate(const MyRobotState& actual, NRC_Position& joints,
                   const SensorData& force, bool small, double now,
                   std::array<double, 7>& target, AsyncLogger& logger);
    const char* error() const { return error_; }
private:
    PressFit press_;
    Admittance4 controller_;
    MyRobotState origin_{};
    SensorData filtered_{};
    bool small_ = false;
    const char* error_ = "";
};
