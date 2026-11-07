#include "PressFitMode.h"

#include <algorithm>
#include <cmath>

namespace {
using namespace ForceControlParameters;
constexpr double RAD_TO_DEG = 180.0 / 3.14159265358979323846;
}
//初始化下压导纳控制参数
PressFitMode::PressFitMode()
    : controller_({130, 130, 120, 5}, {4000, 3000, 5000, 120}, {0, 0, 0, 0},
                  {MAX_X_VELOCITY, MAX_Y_VELOCITY, 0, MAX_ROTATION_VELOCITY}, 0.001) {}

//以机器人当前实际位置为新的拖动起点，并清掉之前的运动状态
void PressFitMode::syncPlane(const MyRobotState& actual, const NRC_Position& joints) {
    origin_ = actual;
    for (int i = 0; i < 4; ++i) locked_joints_[i] = joints.pos[i];
    controller_.set_state({0, 0, 0, 0}, {0, 0, 0, 0});
    filtered_ = SensorData{};
}


PressAction PressFitMode::step(bool requested, bool small, bool new_session, bool rotation, double now,
                              const MyRobotState& actual, NRC_Position& joints, const SensorData& force,
                              std::array<double, 7>& target, AsyncLogger& logger) {
    using namespace PressFitConfig;
    const auto fault = [&](const char* reason) 
    {
        logger.log(std::string("[下压] ") + reason + "\n");
        return PressAction::Fault;
    };


    if (small && (requested || phase_ != Phase::Idle)) return fault("必须使用大量程传感器");

    if (!requested) 
    {
        if (phase_ == Phase::Idle) return PressAction::Normal;
        phase_ = Phase::Idle;
        return PressAction::Released;
    }

    if (new_session) phase_ = Phase::Idle;
    // 进入：第一拍只同步，不发送运动目标；加载时从已有压力开始缓增。
    if (phase_ == Phase::Idle) {
        syncPlane(actual, joints);
        target_z_ = actual.z;
        filtered_fz_ = force.fz;
        entry_force_ = std::max(0.0, std::min(TARGET_FORCE_N, force.fz));
        previous_ = start_ = now;
        stable_since_ = -1;
        plane_paused_ = false;
        phase_ = Phase::Building;
        logger.log("[下压] 开始缓慢建立接触压力\n");
        return PressAction::Entered;
    }

    const double dt = now - previous_;
    previous_ = now;
    if (!(dt > 0 && dt <= MAX_CYCLE_SECONDS)) return fault("控制周期间隔异常");


    filtered_fz_ += FILTER_ALPHA * (force.fz - filtered_fz_);

    const double age = now - start_;

    double vz = 0;

    if (phase_ == Phase::Building) {
        // 受压为正、向下为负：反馈小于设定时，Z 目标减小。
        const double ramp = std::min(1.0, age / FORCE_RAMP_SECONDS);
        //进度计算期望力
        const double desired = entry_force_ + (TARGET_FORCE_N - entry_force_) * ramp;
        // 先卡住上限，再卡住下限。
        vz = std::max(-MAX_SPEED_M_S, std::min(MAX_SPEED_M_S, (filtered_fz_ - desired) / DAMPING_NS_M));
        //这一拍是否达标
        const bool stable = ramp >= 1 && std::abs(filtered_fz_ - TARGET_FORCE_N) <= FORCE_TOLERANCE_N;

        if (!stable) stable_since_ = -1;
        //now是当前秒数
        else if (stable_since_ < 0) stable_since_ = now;
        //如果稳定时间超过设定值，则进入Holding阶段
        if (stable_since_ >= 0 && now - stable_since_ >= STABLE_SECONDS) {
            phase_ = Phase::Holding;
            rotation_ = rotation;
            syncPlane(actual, joints);
            if (rotation_) target_z_ = actual.z;
            vz = 0;
            logger.log("[贴合] 压力稳定，可按模式开关选择水平调整或末端自转\n");
        }
    } else {
        // 贴合后不追踪向上力。明显失压时停止，不把脱离工件误当成要快速找面。
        if (filtered_fz_ <= FORCE_TOLERANCE_N) return fault("贴合后失去接触");
        // 切换时从实际位姿和实际关节角重新起步，不带入上一模式的速度。
        if (rotation != rotation_) {
            rotation_ = rotation;
            syncPlane(actual, joints);
            target_z_ = actual.z;
            logger.log(rotation_ ? "[贴合] 末端自转，锁定大臂和伸缩轴，允许升降补压\n" :
                                   "[贴合] 水平调整，按工具方向拖动\n");
        }

        const double deficit = TARGET_FORCE_N - FORCE_TOLERANCE_N - filtered_fz_;
        vz = -std::min(HOLD_SPEED_M_S, std::max(0.0, deficit / DAMPING_NS_M));
        // 两档都允许低压补偿，压力过高时仍暂停水平调整或旋转。
        const bool pause = filtered_fz_ > TARGET_FORCE_N + FORCE_TOLERANCE_N;
        if (pause != plane_paused_) {
            if (rotation_) {
                // 保留大臂、伸缩和已生成的升降目标，只重置腕轴拖动状态。
                locked_joints_[3] = joints.pos[3];
                controller_.set_state({0, 0, 0, 0}, {0, 0, 0, 0});
                filtered_ = SensorData{};
            } else {
                syncPlane(actual, joints);
            }
        }
        plane_paused_ = pause;
    }

    const double previous_target_z = target_z_;
    target_z_ += vz * dt;
    if (std::abs(target_z_ - actual.z) > MAX_FOLLOW_ERROR_M) return fault("竖直目标与实测位置偏差过大");

    if (phase_ == Phase::Holding && rotation_) {
        double wrist = locked_joints_[3];
        if (!plane_paused_) {
            const double mz = filterForceInput(force.mz, filtered_.mz,
                                               LARGE_SENSOR_MOMENT_DEAD_ZONE, MAX_CONTROL_MOMENT);
            wrist += controller_.update({0, 0, 0, mz}).first[3] * RAD_TO_DEG;
        }
        if (target_z_ != previous_target_z) {
            // 只改变基座 Z，用厂家逆解换算升降轴的控制器单位和方向。
            // 不采用逆解的其他轴结果：大臂和伸缩锁定，腕轴单独计算。
            NRC_Position lift;
            if (!perform_ik(joints, actual.x, actual.y, target_z_, actual.rz, lift)) {
                return fault("旋转档升降补压逆解失败");
            }
            locked_joints_[1] = lift.pos[1];
        }
        // 不需要补压时保持上一升降目标，不调用逆解。
        target = {locked_joints_[0], locked_joints_[1], locked_joints_[2], wrist, 0, 0, 0};
        return PressAction::Target;
    }

    if (phase_ == Phase::Holding && !plane_paused_) {
        const double fx = filterForceInput(force.fx, filtered_.fx,
                                           LARGE_SENSOR_FORCE_DEAD_ZONE, MAX_CONTROL_FORCE);
        const double fy = filterForceInput(force.fy, filtered_.fy,
                                           LARGE_SENSOR_FORCE_DEAD_ZONE, MAX_CONTROL_FORCE);
        const auto previous = controller_.x();
        const auto offset = controller_.update({fx, fy, 0, 0}).first;

        // 只旋转本拍的位移增量，旧的基座 XY 目标不随工具朝向转动。
        const double dx = offset[0] - previous[0];
        const double dy = offset[1] - previous[1];
        const double angle = -actual.rz;
        origin_.x += dx * std::cos(angle) - dy * std::sin(angle);
        origin_.y += dx * std::sin(angle) + dy * std::cos(angle);
    }

    NRC_Position result;
    if (!perform_ik(joints, origin_.x, origin_.y, target_z_, origin_.rz, result)) return fault("下压或贴合逆解失败");
    target = {result.pos[0], result.pos[1], result.pos[2], result.pos[3], 0, 0, 0};
    return PressAction::Target;
}
