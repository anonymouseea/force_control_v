#include "ControlStateMachine.h"
#include "ControlConfig.h"

#include <cmath>

namespace {
using Milliseconds = std::chrono::milliseconds;

const char* stateName(ControlState state) {
    switch (state) {
    case ControlState::Disabled: return "[状态] 力控关闭";
    case ControlState::Starting: return "[状态] 正在启动";
    case ControlState::PositionControl: return "[状态] 位置力控";
    case ControlState::RotationControl: return "[状态] 末端旋转力控";
    case ControlState::Stopping: return "[状态] 正在停机";
    case ControlState::Zeroing: return "[状态] 单独清零传感器";
    case ControlState::Fault: return "[状态] 故障锁定";
    }
    return "[状态] 未知状态";
}

bool servoDisabled(int status) {
    // 厂家头文件：0为禁止，1为就绪，2为报警，3为运行。
    // 报警和未知返回值都不能当成已断使能。
    return status == 0 || status == 1;
}
}

ControlStateMachine::ControlStateMachine(ControlHardware& hardware) : hardware_(hardware) {}

bool ControlStateMachine::isRunning() const {
    return state_ == ControlState::PositionControl || state_ == ControlState::RotationControl;
}

long ControlStateMachine::periodNanoseconds() const {
    return state_ == ControlState::Disabled || state_ == ControlState::Fault
        ? ControlConfig::IDLE_PERIOD_NS : ControlConfig::CONTROL_PERIOD_NS;
}

bool ControlStateMachine::exitFinished() const {
    return exit_requested_ && (state_ == ControlState::Disabled || state_ == ControlState::Fault);
}

bool ControlStateMachine::exitSucceeded() const {
    return exitFinished() && !cleanup_failed_ && !power_attempted_ && !tracking_attempted_ && !fault_latched_;
}

void ControlStateMachine::transition(ControlState next) {
    if (state_ != next) hardware_.log(stateName(next));
    state_ = next;
}

void ControlStateMachine::requestOff() {
    const int result = hardware_.requestForceOff();
    if (result != 0) hardware_.log("[错误] 力控开关复位失败，本地仍禁止输出目标", result);
}

void ControlStateMachine::reportFault(const char* reason, ControlTime now, int code) {
    if (fault_latched_) return;
    hardware_.log(reason, code);
    fault_latched_ = true;
    zero_after_stop_ = false;
    requestOff();
    beginStop(now);
}

void ControlStateMachine::beginStart(ControlTime now) {
    fault_latched_ = false;
    cleanup_failed_ = false;
    armed_ = false;
    start_step_ = StartStep::CheckStopped;
    deadline_ = now;
    transition(ControlState::Starting);
}

void ControlStateMachine::advanceStart(const ControlInputs& inputs, ControlTime now) {
    int result = 0;
    switch (start_step_) {
    case StartStep::CheckStopped:
        // 只有未使能且停止时才允许重新清零和接管机器人。
        if (!servoDisabled(hardware_.servoStatus()) || hardware_.runStatus() != 0) {
            reportFault("[错误] 启动前机器人未处于未使能且停止状态", now);
            return;
        }
        start_step_ = StartStep::ClearErrors;
        break;
    case StartStep::ClearErrors:
        hardware_.log("[启动] 清除错误并开始传感器清零，请勿施加外力");
        result = hardware_.clearErrors();
        if (result != 0) { reportFault("[错误] 清除错误失败", now, result); return; }
        hardware_.beginZero(now);
        deadline_ = now + Milliseconds(ControlConfig::ZERO_TIMEOUT_MS);
        start_step_ = StartStep::ZeroSensor;
        break;
    case StartStep::ZeroSensor: {
        const ZeroResult zero = hardware_.pollZero(now);
        if (zero == ZeroResult::Failed || now >= deadline_) {
            reportFault("[错误] 启动清零失败或超时", now);
        } else if (zero == ZeroResult::Complete) {
            hardware_.log("[启动] 传感器清零完成");
            start_step_ = StartStep::SetServoReady;
        }
        break;
    }
    case StartStep::SetServoReady:
        hardware_.log("[启动] 设置伺服准备状态");
        result = hardware_.setServoReady();
        if (result != 0) { reportFault("[错误] 设置伺服准备状态失败", now, result); return; }
        start_step_ = StartStep::PowerOn;
        break;
    case StartStep::PowerOn:
        hardware_.log("[启动] 请求伺服上使能");
        // 即使接口返回失败也不能假定硬件未动作，后续进入清理流程。
        power_attempted_ = true;
        result = hardware_.powerOn();
        if (result != 0) { reportFault("[错误] 伺服上使能接口失败", now, result); return; }
        start_step_ = StartStep::OpenTracking;
        break;
    case StartStep::OpenTracking:
        hardware_.log("[启动] 开启关节跟踪");
        // 保留实际仓库顺序：上使能后开启跟踪，再等待伺服运行状态。
        tracking_attempted_ = true;
        result = hardware_.openTracking();
        if (result != 0) { reportFault("[错误] 开启关节跟踪失败", now, result); return; }
        deadline_ = now + Milliseconds(ControlConfig::SERVO_TIMEOUT_MS);
        start_step_ = StartStep::WaitServoEnabled;
        break;
    case StartStep::WaitServoEnabled: {
        const int status = hardware_.servoStatus();
        if (status == 3) start_step_ = StartStep::Synchronize;
        else if (status != 0 && status != 1) reportFault("[错误] 等待使能时伺服报警或状态无效", now, status);
        else if (now >= deadline_) reportFault("[错误] 等待伺服使能超时", now, status);
        break;
    }
    case StartStep::Synchronize:
        if (hardware_.servoStatus() != 3) {
            reportFault("[错误] 同步位置前伺服已退出运行状态", now);
            return;
        }
        if (!cycle_.synchronize(hardware_, inputs.rotation_mode, inputs.small_sensor)) {
            reportFault("[错误] 启动时读取真机位置失败", now);
            return;
        }
        small_sensor_ = inputs.small_sensor;
        transition(inputs.rotation_mode ? ControlState::RotationControl : ControlState::PositionControl);
        break;
    }
}

void ControlStateMachine::beginStop(ControlTime now) {
    hardware_.cancelZero();
    cycle_.clearVelocity();
    armed_ = false;
    if (state_ == ControlState::Stopping || cleanup_failed_) return;
    if (!power_attempted_ && !tracking_attempted_) {
        finishStop(now);
        return;
    }
    transition(ControlState::Stopping);
    stop_step_ = tracking_attempted_ ? StopStep::StopTracking : StopStep::WaitRobotStopped;
    have_stable_reference_ = false;
    next_sample_ = now;
    deadline_ = now + Milliseconds(ControlConfig::STOP_TIMEOUT_MS);
}

void ControlStateMachine::failCleanup(const char* reason, int code) {
    hardware_.log(reason, code);
    hardware_.log("[故障] 停机未确认完成，禁止重新启动；需要人工检查控制器状态");
    fault_latched_ = true;
    cleanup_failed_ = true;
    armed_ = false;
    transition(ControlState::Fault);
}

void ControlStateMachine::advanceStop(ControlTime now) {
    switch (stop_step_) {
    case StopStep::StopTracking: {
        hardware_.log("[停机] 停止关节轨迹生成");
        const int result = hardware_.stopTracking();
        if (result != 0) {
            hardware_.log("[诊断] 当前伺服状态", hardware_.servoStatus());
            failCleanup("[错误] 关闭关节跟踪失败，不能宣称停机完成", result);
            return;
        }
        tracking_attempted_ = false;
        have_stable_reference_ = false;
        next_sample_ = now;
        deadline_ = now + Milliseconds(ControlConfig::STOP_TIMEOUT_MS);
        stop_step_ = StopStep::WaitRobotStopped;
        hardware_.log("[停机] 等待机器人连续停稳");
        break;
    }
    case StopStep::WaitRobotStopped: {
        if (now >= deadline_) { failCleanup("[错误] 等待停稳超时，未执行下使能"); return; }
        if (now < next_sample_) return;
        next_sample_ = now + Milliseconds(ControlConfig::STOP_SAMPLE_MS);
        JointTarget actual;
        bool valid = hardware_.runStatus() == 0 && hardware_.readJoints(actual);
        if (valid) for (double value : actual) if (!std::isfinite(value)) valid = false;
        if (!valid) { have_stable_reference_ = false; return; }
        bool stable = have_stable_reference_;
        if (stable) {
            for (int i = 0; i < 4; ++i) {
                if (std::abs(actual[i] - stable_reference_[i]) > ControlConfig::STOP_POSITION_TOLERANCE[i]) stable = false;
            }
        }
        if (!stable) {
            stable_reference_ = actual;
            stable_since_ = now;
            have_stable_reference_ = true;
        } else if (now - stable_since_ >= Milliseconds(ControlConfig::STOP_STABLE_MS)) {
            stop_step_ = StopStep::PowerOff;
        }
        break;
    }
    case StopStep::PowerOff: {
        hardware_.log("[停机] 已确认停稳，请求伺服下使能");
        const int result = hardware_.powerOff();
        if (result != 0) { failCleanup("[错误] 伺服下使能接口失败", result); return; }
        deadline_ = now + Milliseconds(ControlConfig::SERVO_TIMEOUT_MS);
        stop_step_ = StopStep::WaitServoDisabled;
        break;
    }
    case StopStep::WaitServoDisabled: {
        const int status = hardware_.servoStatus();
        if (servoDisabled(status)) {
            power_attempted_ = false;
            hardware_.log("[停机] 伺服已退出使能状态");
            finishStop(now);
        } else if (now >= deadline_) {
            failCleanup("[错误] 下使能确认超时，当前伺服状态", status);
        }
        break;
    }
    }
}

void ControlStateMachine::finishStop(ControlTime now) {
    armed_ = false;
    if (fault_latched_) {
        transition(ControlState::Fault);
    } else if (zero_after_stop_ && !exit_requested_) {
        zero_after_stop_ = false;
        beginZero(now);
    } else {
        transition(ControlState::Disabled);
    }
}

void ControlStateMachine::beginZero(ControlTime now) {
    if (!servoDisabled(hardware_.servoStatus()) || hardware_.runStatus() != 0) {
        reportFault("[错误] 传感器清零要求机器人未使能且停止", now);
        return;
    }
    armed_ = false;
    requestOff();
    hardware_.log("[清零] 开始采样，请勿施加外力");
    hardware_.beginZero(now);
    deadline_ = now + Milliseconds(ControlConfig::ZERO_TIMEOUT_MS);
    transition(ControlState::Zeroing);
}

void ControlStateMachine::advanceZero(ControlTime now) {
    const ZeroResult result = hardware_.pollZero(now);
    if (result == ZeroResult::Failed || now >= deadline_) {
        reportFault("[错误] 传感器清零失败或超时", now);
    } else if (result == ZeroResult::Complete) {
        cycle_.clearVelocity();
        requestOff();
        armed_ = false;
        hardware_.log("[清零] 完成，重新关闭再打开力控开关后才能启动");
        transition(ControlState::Disabled);
    }
}

void ControlStateMachine::executeCycle(ControlTime now) {
    const CycleResult result = cycle_.execute(hardware_);
    switch (result) {
    case CycleResult::Success: return;
    case CycleResult::ReadFailed: reportFault("[错误] 控制周期读取真机状态失败", now); break;
    case CycleResult::InvalidForce: reportFault("[错误] 传感器或导纳数据无效", now); break;
    case CycleResult::InverseFailed: reportFault("[错误] 逆解失败", now); break;
    case CycleResult::LimitExceeded:
        if (hardware_.setLimitAlarm() != 0) hardware_.log("[错误] 写入限位报警失败");
        reportFault("[错误] 关节目标超限或无效", now);
        break;
    case CycleResult::SendFailed:
        reportFault("[错误] 发送关节目标失败", now, cycle_.lastErrorCode());
        break;
    }
}

void ControlStateMachine::update(const ControlInputs& inputs, ControlTime now, bool exit_requested) {
    const bool force_rising = inputs.force_on && !last_force_on_;
    const bool zero_rising = inputs.zero_on && !last_zero_on_;
    last_force_on_ = inputs.force_on;
    last_zero_on_ = inputs.zero_on;

    if (exit_requested && !exit_requested_) {
        exit_requested_ = true;
        zero_after_stop_ = false;
        requestOff();
        beginStop(now);
    }
    if (state_ == ControlState::Stopping) { advanceStop(now); return; }
    if (exit_requested_) return;

    if (state_ == ControlState::Disabled || state_ == ControlState::Fault) {
        // 停机清理失败时不能通过拨动开关绕过故障。
        if (cleanup_failed_) return;
        if (!inputs.force_on) armed_ = true;
        if (zero_rising && state_ == ControlState::Disabled) { beginZero(now); return; }
        if (armed_ && force_rising) beginStart(now);
        return;
    }
    if (state_ == ControlState::Zeroing) { advanceZero(now); return; }
    if (!inputs.force_on) { beginStop(now); return; }
    if (state_ == ControlState::Starting) { advanceStart(inputs, now); return; }

    if (zero_rising) {
        hardware_.log("[清零] 运行中收到清零请求，先停机，清零后等待重新开启");
        zero_after_stop_ = true;
        requestOff();
        beginStop(now);
        return;
    }
    if (hardware_.servoStatus() != 3) {
        reportFault("[错误] 力控运行中伺服退出运行状态", now);
        return;
    }
    const bool rotation = state_ == ControlState::RotationControl;
    if (inputs.rotation_mode != rotation || inputs.small_sensor != small_sensor_) {
        if (!cycle_.synchronize(hardware_, inputs.rotation_mode, inputs.small_sensor)) {
            reportFault("[错误] 模式或传感器切换时读取真机状态失败", now);
            return;
        }
        small_sensor_ = inputs.small_sensor;
        hardware_.log("[同步] 已采用真机实际位置，清除导纳速度及滤波历史");
        transition(inputs.rotation_mode ? ControlState::RotationControl : ControlState::PositionControl);
        // 同步这一拍不发送新目标，下一拍从零偏移计算。
        return;
    }
    executeCycle(now);
}
