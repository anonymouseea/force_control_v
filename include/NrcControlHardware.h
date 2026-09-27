#pragma once

#include "ControlHardware.h"
#include "SensorCalibration.h"
#include "AsyncLogger.h"
#include "nrcAPI.h"

// 厂家接口集中到此处，所有方法均由主控制线程调用。
class NrcControlHardware : public ControlHardware {
public:
    explicit NrcControlHardware(AsyncLogger& logger) : logger_(logger) {}
    bool readInputs(ControlInputs& inputs);
    void publishFeedback(ControlTime now);
    int clearErrors() override;
    int setServoReady() override;
    int powerOn() override;
    int openTracking() override;
    int stopTracking() override;
    int powerOff() override;
    int servoStatus() override;
    int runStatus() override;
    bool readRobot(RobotSnapshot& snapshot) override;
    bool readJoints(JointTarget& joints) override;
    bool readForce(bool small_sensor, ForceSample& force) override;
    bool inverseKinematics(const RobotSnapshot& reference,
                           const std::array<double, 4>& pose,
                           JointTarget& target) override;
    int sendTarget(const JointTarget& target) override;
    int requestForceOff() override;
    int setLimitAlarm() override;
    void beginZero(ControlTime now) override;
    ZeroResult pollZero(ControlTime now) override;
    void cancelZero() override;
    void log(const char* message, int code = 0) override;

private:
    AsyncLogger& logger_;
    SensorCalibration calibration_;
    NRC_Position reference_;
    ControlTime next_feedback_{};
    std::array<double, 12> feedback_{};
    int feedback_channel_ = 12;
    bool feedback_error_reported_ = false;
};
