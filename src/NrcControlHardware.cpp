#include "NrcControlHardware.h"
#include "ControlConfig.h"
#include "RobotUtils.h"

#include <cmath>
#include <string>

namespace {
ForceSample rawForce(bool small) {
    const SensorData data = small ? read_force_sensor_xiao_raw() : read_force_sensor_da_raw();
    return {{data.fx, data.fy, data.fz, data.mx, data.my, data.mz}};
}
}

bool NrcControlHardware::readInputs(ControlInputs& inputs) {
    const int mode = NRC_ReadDigInByBoard(ControlConfig::MODE_BOARD, ControlConfig::MODE_CHANNEL);
    if (mode != 0 && mode != 1) return false;
    inputs.force_on = NRC_ReadBoolVar(ControlConfig::FORCE_SWITCH_VAR);
    inputs.zero_on = NRC_ReadBoolVar(ControlConfig::ZERO_SWITCH_VAR);
    inputs.small_sensor = NRC_ReadBoolVar(ControlConfig::SMALL_SENSOR_VAR);
    inputs.rotation_mode = mode == 1;
    // 布尔读取接口没有错误码，通信有效性还需依赖厂家底层状态。
    return true;
}

int NrcControlHardware::clearErrors() { return NRC_ClearAllError(); }
int NrcControlHardware::setServoReady() { return NRC_SetServoReadyStatus(1); }
int NrcControlHardware::powerOn() { return NRC_PowerOn(); }
int NrcControlHardware::openTracking() {
    return NRC_RKG_Open(ControlConfig::TRACKING_VELOCITY,
                        ControlConfig::TRACKING_ACCELERATION, ControlConfig::TRACKING_JERK);
}
int NrcControlHardware::stopTracking() { return NRC_RKG_Stop(); }
int NrcControlHardware::powerOff() { return NRC_PowerOff(); }
int NrcControlHardware::servoStatus() { return NRC_GetServoStatus(); }
int NrcControlHardware::runStatus() { return NRC_GetRobotRunStatus(); }
int NrcControlHardware::sendTarget(const JointTarget& target) { return NRC_Set_ServoJ_Pos(target); }
int NrcControlHardware::requestForceOff() { return NRC_SetBoolVar(ControlConfig::FORCE_SWITCH_VAR, false); }
int NrcControlHardware::setLimitAlarm() { return NRC_SetBoolVar(ControlConfig::LIMIT_ALARM_VAR, true); }

bool NrcControlHardware::readJoints(JointTarget& joints) {
    NRC_Position actual;
    if (NRC_GetCurrentPos(NRC_COORD::NRC_ACS, actual) != 0) return false;
    for (int i = 0; i < 7; ++i) {
        if (!std::isfinite(actual.pos[i])) return false;
        joints[i] = actual.pos[i];
    }
    return true;
}

bool NrcControlHardware::readRobot(RobotSnapshot& snapshot) {
    NRC_Position mcs;
    NRC_Position acs;
    if (NRC_GetCurrentPos(NRC_COORD::NRC_MCS, mcs) != 0 ||
        NRC_GetCurrentPos(NRC_COORD::NRC_ACS, acs) != 0) return false;
    RobotSnapshot actual;
    actual.pose = {{mcs.pos[0] / 1000.0, mcs.pos[1] / 1000.0, mcs.pos[2] / 1000.0, mcs.pos[5]}};
    for (double value : actual.pose) if (!std::isfinite(value)) return false;
    for (int i = 0; i < 7; ++i) {
        if (!std::isfinite(acs.pos[i])) return false;
        actual.joints[i] = acs.pos[i];
    }
    reference_ = acs;
    snapshot = actual;
    return true;
}

bool NrcControlHardware::readForce(bool small_sensor, ForceSample& force) {
    force = rawForce(small_sensor);
    const ForceSample& offset = calibration_.offset(small_sensor);
    for (int i = 0; i < 6; ++i) {
        force[i] -= offset[i];
        if (!std::isfinite(force[i])) return false;
    }
    // 保持原项目方向：小量程只反转横向力的第一项。
    if (small_sensor) force[0] = -force[0];
    return true;
}

bool NrcControlHardware::inverseKinematics(const RobotSnapshot& reference,
                                          const std::array<double, 4>& pose,
                                          JointTarget& target) {
    NRC_Position acs = reference_;
    for (int i = 0; i < 7; ++i) acs.pos[i] = reference.joints[i];
    NRC_Position result;
    if (!perform_ik(acs, pose[0], pose[1], pose[2], pose[3], result)) return false;
    target.fill(0.0);
    for (int i = 0; i < 4; ++i) target[i] = result.pos[i];
    return true;
}

void NrcControlHardware::beginZero(ControlTime now) { calibration_.begin(now); }
void NrcControlHardware::cancelZero() { calibration_.cancel(); }
ZeroResult NrcControlHardware::pollZero(ControlTime now) {
    if (!calibration_.active()) return ZeroResult::Failed;
    if (!calibration_.sampleDue(now)) return ZeroResult::Pending;
    return calibration_.sample(now, rawForce(false), rawForce(true));
}

void NrcControlHardware::publishFeedback(ControlTime now) {
    if (feedback_channel_ >= 12) {
        if (now < next_feedback_) return;
        const ForceSample small = rawForce(true);
        const ForceSample large = rawForce(false);
        for (int i = 0; i < 6; ++i) { feedback_[i] = small[i]; feedback_[i + 6] = large[i]; }
        next_feedback_ = now + std::chrono::milliseconds(ControlConfig::FEEDBACK_PERIOD_MS);
        feedback_channel_ = 0;
        feedback_error_reported_ = false;
    }
    // 每拍最多写一个显示变量，避免一次集中执行十二次厂家调用。
    const int result = NRC_SetDoubleVar(7 + feedback_channel_, feedback_[feedback_channel_]);
    if (result != 0 && !feedback_error_reported_) {
        log("[显示] 写入传感器显示变量失败", result);
        feedback_error_reported_ = true;
    }
    ++feedback_channel_;
}

void NrcControlHardware::log(const char* message, int code) {
    logger_.log(std::string(message) + "，代码=" + std::to_string(code) + "\n");
}
