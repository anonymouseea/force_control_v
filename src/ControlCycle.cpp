#include "ControlCycle.h"
#include "ControlConfig.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr double RAD_TO_DEG = 180.0 / 3.14159265358979323846;

double deadZone(double value, double width) {
    if (value > width) return value - width;
    if (value < -width) return value + width;
    return 0.0;
}

bool validSnapshot(const RobotSnapshot& snapshot) {
    for (double value : snapshot.pose) if (!std::isfinite(value)) return false;
    for (double value : snapshot.joints) if (!std::isfinite(value)) return false;
    return true;
}
}

ControlCycle::ControlCycle()
    : controller_(ControlConfig::MASS, ControlConfig::DAMPING,
                  ControlConfig::STIFFNESS, ControlConfig::MAX_VELOCITY,
                  ControlConfig::DT) {}

bool ControlCycle::synchronize(ControlHardware& hardware, bool rotation_mode, bool small_sensor) {
    RobotSnapshot actual;
    if (!hardware.readRobot(actual) || !validSnapshot(actual)) return false;

    // 每次进入模式都把真机当前位置作为新的局部原点，避免继承旧目标偏差。
    origin_ = actual;
    rotation_mode_ = rotation_mode;
    small_sensor_ = small_sensor;
    controller_.set_state({{0, 0, 0, 0}}, {{0, 0, 0, 0}});
    filtered_.fill(0.0);
    return true;
}

void ControlCycle::clearVelocity() {
    controller_.set_state(controller_.x(), {{0, 0, 0, 0}});
    filtered_.fill(0.0);
}

CycleResult ControlCycle::execute(ControlHardware& hardware) {
    RobotSnapshot actual;
    if (!hardware.readRobot(actual) || !validSnapshot(actual)) return CycleResult::ReadFailed;
    ForceSample force;
    if (!hardware.readForce(small_sensor_, force)) return CycleResult::InvalidForce;
    for (double value : force) if (!std::isfinite(value)) return CycleResult::InvalidForce;

    const double force_dead_zone = small_sensor_ ? ControlConfig::SMALL_FORCE_DEAD_ZONE
                                                  : ControlConfig::LARGE_FORCE_DEAD_ZONE;
    const double moment_dead_zone = small_sensor_ ? ControlConfig::SMALL_MOMENT_DEAD_ZONE
                                                   : ControlConfig::LARGE_MOMENT_DEAD_ZONE;
    Admittance4::Vec4 input = {{0, 0, 0, 0}};
    for (int i = 0; i < 4; ++i) {
        const int channel = (i == 3) ? 5 : i;
        filtered_[channel] += ControlConfig::FILTER_ALPHA * (force[channel] - filtered_[channel]);
        const double limit = (i == 3) ? ControlConfig::MAX_MOMENT : ControlConfig::MAX_FORCE;
        const double value = deadZone(filtered_[channel], (i == 3) ? moment_dead_zone : force_dead_zone);
        input[i] = std::max(-limit, std::min(value, limit));
    }
    if (rotation_mode_) {
        input[0] = input[1] = input[2] = 0.0;
    } else {
        input[3] = 0.0;
    }

    const Admittance4::Vec4 offset = controller_.update(input).first;
    for (double value : offset) if (!std::isfinite(value)) return CycleResult::InvalidForce;
    JointTarget target = {{0, 0, 0, 0, 0, 0, 0}};
    if (rotation_mode_) {
        // 前三个关节固定在进入旋转模式时的实际位置，第4关节做自身旋转。
        for (int i = 0; i < 4; ++i) target[i] = origin_.joints[i];
        target[3] += offset[3] * RAD_TO_DEG;
    } else {
        // 沿用原项目的工具系到基座系投影符号与姿态单位。
        const double angle = -actual.pose[3];
        const double dx = offset[0] * std::cos(angle) - offset[1] * std::sin(angle);
        const double dy = offset[0] * std::sin(angle) + offset[1] * std::cos(angle);
        const std::array<double, 4> pose = {{origin_.pose[0] + dx,
                                           origin_.pose[1] + dy,
                                           origin_.pose[2] + offset[2],
                                           origin_.pose[3] + offset[3]}};
        if (!hardware.inverseKinematics(actual, pose, target)) return CycleResult::InverseFailed;
    }
    for (int i = 0; i < 4; ++i) {
        if (!std::isfinite(target[i]) || target[i] < ControlConfig::JOINT_MIN[i] ||
            target[i] > ControlConfig::JOINT_MAX[i]) return CycleResult::LimitExceeded;
    }
    last_error_code_ = hardware.sendTarget(target);
    return last_error_code_ == 0 ? CycleResult::Success : CycleResult::SendFailed;
}
