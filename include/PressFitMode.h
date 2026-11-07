#pragma once

#include "Admittance.h"
#include "RobotUtils.h"
#include "ForceControlParameters.h"
#include "PressFit.h"

// 定义类型
enum class PressAction { Normal, Entered, Released, Target, Fault };

class PressFitMode {
public:
    PressFitMode();
    PressAction step(bool requested, bool small, bool new_session, bool rotation, double now,
                     const MyRobotState& actual, NRC_Position& joints, const SensorData& force,
                     std::array<double, 7>& target, AsyncLogger& logger);

private:
    //定义各个阶段
    enum class Phase { Idle, Building, Holding };

    void syncPlane(const MyRobotState& actual, const NRC_Position& joints);

    Phase phase_ = Phase::Idle;

    Admittance4 controller_;

    MyRobotState origin_{};

    SensorData filtered_{};

    double start_ = 0, previous_ = 0, stable_since_ = -1;
    double target_z_ = 0, filtered_fz_ = 0, entry_force_ = 0;
    std::array<double, 4> locked_joints_{};
    bool rotation_ = false;
    bool plane_paused_ = false;
};
