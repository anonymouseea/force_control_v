#include "PressFitMode.h"
#include "ForceControlParameters.h"

#include <algorithm>
#include <cmath>

namespace {
using namespace ForceControlParameters;

double filteredInput(double raw, double& filtered, double dead_zone, double limit) {
    filtered += FORCE_FILTER_ALPHA * (raw - filtered);
    double value = 0;
    if (filtered > dead_zone) value = filtered - dead_zone;
    if (filtered < -dead_zone) value = filtered + dead_zone;
    return std::max(-limit, std::min(value, limit));
}
}

PressFitMode::PressFitMode()
    : controller_({130, 130, 120, 5}, {4000, 3000, 5000, 120}, {0, 0, 0, 0},
                  {MAX_X_VELOCITY, MAX_Y_VELOCITY, 0, MAX_ROTATION_VELOCITY}, 0.001) {}

void PressFitMode::reset() {
    press_.reset();
    controller_.set_state({0, 0, 0, 0}, {0, 0, 0, 0});
    filtered_ = SensorData{};
}

bool PressFitMode::begin(const MyRobotState& actual, const SensorData& force,
                         bool small, double now) {
    reset();
    if (!press_.begin(now, actual.z, force.fz, small)) {
        error_ = press_.error();
        return false;
    }
    origin_ = actual;
    small_ = small;
    return true;
}

bool PressFitMode::calculate(const MyRobotState& actual, NRC_Position& joints,
                            const SensorData& force, bool small, double now,
                            std::array<double, 7>& target, AsyncLogger& logger) {
    if (small != small_) {
        error_ = "下压与贴合过程中禁止切换传感器，请退出后切换";
        return false;
    }
    const PressPhase before = press_.phase();
    if (!press_.update(now, actual.z, force.fz)) {
        error_ = press_.error();
        return false;
    }

    // 连续5秒稳定后，以真机实际位姿重建水平原点，清除全部旧导纳速度。
    if (before != PressPhase::Seated && press_.phase() == PressPhase::Seated) {
        origin_ = actual;
        controller_.set_state({0, 0, 0, 0}, {0, 0, 0, 0});
        filtered_ = SensorData{};
        logger.log("[贴合] 连续稳定5秒，锁定实际高度，开放水平平移和绕竖直轴旋转\n");
    }

    double x = origin_.x;
    double y = origin_.y;
    double rz = origin_.rz;
    if (press_.phase() == PressPhase::Seated) {
        // 将当前工具系的力投影到进入贴合时的固定工具系。
        // 累计位移也使用该固定坐标系，避免旋转时旧平移量被重新旋转。
        const double delta = origin_.rz - actual.rz;
        const double fx = force.fx * std::cos(delta) - force.fy * std::sin(delta);
        const double fy = force.fx * std::sin(delta) + force.fy * std::cos(delta);
        const double dead_f = small ? SMALL_SENSOR_FORCE_DEAD_ZONE : LARGE_SENSOR_FORCE_DEAD_ZONE;
        const double dead_m = small ? SMALL_SENSOR_MOMENT_DEAD_ZONE : LARGE_SENSOR_MOMENT_DEAD_ZONE;

        // 竖直输入始终为零且竖直速度上限为零；高度最终直接使用锁定值。
        const auto offset = controller_.update({
            filteredInput(fx, filtered_.fx, dead_f, MAX_CONTROL_FORCE),
            filteredInput(fy, filtered_.fy, dead_f, MAX_CONTROL_FORCE),
            0.0,
            filteredInput(force.mz, filtered_.mz, dead_m, MAX_CONTROL_MOMENT)
        }).first;
        const double angle = -origin_.rz;
        x += offset[0] * std::cos(angle) - offset[1] * std::sin(angle);
        y += offset[0] * std::sin(angle) + offset[1] * std::cos(angle);
        rz += offset[3];
    }

    // 下压阶段固定水平位置和姿态；贴合阶段固定实际高度，允许水平和旋转。
    // 两阶段都走完整逆解，不能套用原旋转模式锁前三关节的分支。
    NRC_Position result;
    if (!perform_ik(joints, x, y, press_.targetZ(), rz, result)) {
        error_ = "下压或贴合逆解失败";
        return false;
    }
    target = {result.pos[0], result.pos[1], result.pos[2], result.pos[3], 0, 0, 0};
    return true;
}
