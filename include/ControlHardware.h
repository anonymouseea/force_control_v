#pragma once

#include "ControlTypes.h"

// 状态机只依赖此接口。真机和离线测试分别实现，避免测试连接机器人。
// 一个控制线程负责所有硬件调用；接口内部不得自行循环等待。
class ControlHardware {
public:
    virtual ~ControlHardware() {}
    virtual int clearErrors() = 0;
    virtual int setServoReady() = 0;
    virtual int powerOn() = 0;
    virtual int openTracking() = 0;
    virtual int stopTracking() = 0;
    virtual int powerOff() = 0;
    virtual int servoStatus() = 0;
    virtual int runStatus() = 0;
    virtual bool readRobot(RobotSnapshot& snapshot) = 0;
    virtual bool readJoints(JointTarget& joints) = 0;
    virtual bool readForce(bool small_sensor, ForceSample& force) = 0;
    virtual bool inverseKinematics(const RobotSnapshot& reference,
                                   const std::array<double, 4>& pose,
                                   JointTarget& target) = 0;
    virtual int sendTarget(const JointTarget& target) = 0;
    virtual int requestForceOff() = 0;
    virtual int setLimitAlarm() = 0;
    virtual void beginZero(ControlTime now) = 0;
    virtual ZeroResult pollZero(ControlTime now) = 0;
    virtual void cancelZero() = 0;
    virtual void log(const char* message, int code = 0) = 0;
};
