#include "ControlLoop.h"
#include "Admittance.h"
#include "ForceControlParameters.h"
#include "PressFitMode.h"
#include "RobotUtils.h"
#include "nrcAPI.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <thread>
#include <time.h>

namespace {
using namespace ForceControlParameters;
constexpr double RAD_TO_DEG = 180.0 / 3.14159265358979323846;

// ==================== 一、公共时间与数据检查 ====================

void resetCycle(timespec& next) {
    clock_gettime(CLOCK_MONOTONIC, &next);
}

void waitCycle(timespec& next, bool force_on) {
    // 力控开启为1毫秒，关闭为200毫秒；独立显示线程保持500毫秒。
    next.tv_nsec += force_on ? 1000000L : 200000000L;
    while (next.tv_nsec >= 1000000000L) {
        next.tv_nsec -= 1000000000L;
        ++next.tv_sec;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
}

double monotonicSeconds() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool validFeedback(const MyRobotState& state, const NRC_Position& joints,
                   const SensorData& force) {
    if (!std::isfinite(state.x) || !std::isfinite(state.y) ||
        !std::isfinite(state.z) || !std::isfinite(state.rz) ||
        !std::isfinite(force.fx) || !std::isfinite(force.fy) ||
        !std::isfinite(force.fz) || !std::isfinite(force.mz)) {
        return false;
    }
    for (int axis = 0; axis < 4; ++axis) {
        if (!std::isfinite(joints.pos[axis])) return false;
    }
    return true;
}

bool withinLimits(const std::array<double, 7>& target) {
    const double lower[4] = {-44, -820, 5, -60};
    const double upper[4] = {44, 1148, 848, 60};
    for (int axis = 0; axis < 4; ++axis) {
        if (!std::isfinite(target[axis]) ||
            target[axis] < lower[axis] || target[axis] > upper[axis]) {
            return false;
        }
    }
    return true;
}

// ==================== 二、伺服启动与停止 ====================

bool waitServoEnabled() {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (std::chrono::steady_clock::now() < deadline) {
        if (NRC_GetServoStatus() == 3) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

bool waitRobotStopped() {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    NRC_Position reference;
    bool have_reference = false;
    auto stable_since = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() < deadline) {
        NRC_Position actual;
        bool valid = NRC_GetRobotRunStatus() == 0 &&
                     NRC_GetCurrentPos(NRC_COORD::NRC_ACS, actual) == 0;
        if (valid) {
            for (int axis = 0; axis < 4; ++axis) {
                if (!std::isfinite(actual.pos[axis])) valid = false;
            }
        }
        if (!valid) {
            have_reference = false;
        } else {
            bool stable = have_reference;
            for (int axis = 0; axis < 4 && have_reference; ++axis) {
                if (std::abs(actual.pos[axis] - reference.pos[axis]) > 0.01) stable = false;
            }
            if (!stable) {
                reference = actual;
                stable_since = std::chrono::steady_clock::now();
                have_reference = true;
            } else if (std::chrono::steady_clock::now() - stable_since >=
                       std::chrono::milliseconds(100)) {
                return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

bool stopAndPowerOff(AsyncLogger& logger) {
    const int stop_result = NRC_RKG_Stop();
    if (stop_result != 0) {
        logger.log("[错误] 关闭关节跟踪失败，返回码=" + std::to_string(stop_result) + "\n");
        return false;
    }
    if (!waitRobotStopped()) {
        logger.log("[错误] 等待机器人停稳超过3秒，未执行下使能\n");
        return false;
    }
    const int off_result = NRC_PowerOff();
    if (off_result != 0) {
        logger.log("[错误] 下使能接口失败，返回码=" + std::to_string(off_result) + "\n");
        return false;
    }
    logger.log("[控制] 已确认停稳，下使能接口执行成功\n");
    return true;
}

bool startControl(AsyncLogger& logger, MyRobotState& initial, NRC_Position& joints) {
    logger.log("[控制] 开始力控，请在无接触载荷时进行传感器清零\n");
    NRC_ClearAllError();
    zero_force_sensor(logger);
    if (NRC_SetServoReadyStatus(1) != 0) {
        logger.log("[错误] 设置伺服准备状态失败\n");
        return false;
    }
    if (NRC_PowerOn() != 0) {
        logger.log("[错误] 上使能接口失败\n");
        NRC_PowerOff();
        return false;
    }
    // 保留原实际顺序：上使能后开启跟踪，再等待伺服状态3。
    const int result = NRC_RKG_Open(
        {60, 60, 60, 60, 20, 20, 20},
        {1500, 1500, 1500, 1500, 2000, 2000, 2000},
        {2000, 2000, 2000, 2000, 2000, 2000, 2000});
    if (result != 0 || !waitServoEnabled() || !read_robot_full_state(initial) ||
        NRC_GetCurrentPos(NRC_COORD::NRC_ACS, joints) != 0) {
        logger.log("[错误] 开启跟踪、等待使能或初始位置读取失败，跟踪返回码=" +
                   std::to_string(result) + "\n");
        stopAndPowerOff(logger);
        return false;
    }
    return true;
}

// ==================== 三、原位置与末端旋转模式 ====================

class NormalControl {
public:
    NormalControl()
        : controller_({130, 130, 120, 5}, {4000, 3000, 5000, 120}, {0, 0, 0, 0},
                      {MAX_X_VELOCITY, MAX_Y_VELOCITY, MAX_Z_VELOCITY,
                       MAX_ROTATION_VELOCITY}, 0.001) {}

    void reset(const MyRobotState& actual, const NRC_Position& joints,
               bool rotation, bool small) {
        origin_ = actual;
        rotation_ = rotation;
        small_ = small;
        last_ = {0, 0, 0, 0};
        lock_ = last_;
        lockJoints(joints);
        controller_.set_state(last_, {0, 0, 0, 0});
        clearFilter();
    }

    void clearFilter() { filtered_ = SensorData{}; }

    void clearVelocity() {
        controller_.set_state(last_, {0, 0, 0, 0});
        clearFilter();
    }

    bool calculate(const MyRobotState& actual, NRC_Position& joints, SensorData force,
                   bool rotation, bool small, std::array<double, 7>& target,
                   AsyncLogger& logger) {
        if (small != small_) {
            small_ = small;
            clearVelocity();
            logger.log("[传感器] 已切换量程，清除速度和滤波历史\n");
        }
        const double dead_f = small ? SMALL_SENSOR_FORCE_DEAD_ZONE : LARGE_SENSOR_FORCE_DEAD_ZONE;
        const double dead_m = small ? SMALL_SENSOR_MOMENT_DEAD_ZONE : LARGE_SENSOR_MOMENT_DEAD_ZONE;
        Admittance4::Vec4 input = {
            filter(force.fx, filtered_.fx, dead_f, MAX_CONTROL_FORCE),
            filter(force.fy, filtered_.fy, dead_f, MAX_CONTROL_FORCE),
            filter(force.fz, filtered_.fz, dead_f, MAX_CONTROL_FORCE),
            filter(force.mz, filtered_.mz, dead_m, MAX_CONTROL_MOMENT)
        };

        // 双向模式切换沿用原坐标约定，将导纳状态同步到实际位姿。
        if (rotation != rotation_) {
            const double dx = actual.x - origin_.x;
            const double dy = actual.y - origin_.y;
            last_ = {
                dx * std::cos(actual.rz) - dy * std::sin(actual.rz),
                dx * std::sin(actual.rz) + dy * std::cos(actual.rz),
                actual.z - origin_.z, actual.rz - origin_.rz
            };
            rotation_ = rotation;
            lock_ = last_;
            lockJoints(joints);
            clearVelocity();
            input = {0, 0, 0, 0};
            logger.log(rotation ? "[模式] 末端自转，已同步实际位置\n" :
                                  "[模式] 位置拖动，已同步实际位置\n");
        }
        if (rotation_) {
            input[0] = input[1] = input[2] = 0;
        } else {
            input[3] = 0;
        }
        last_ = controller_.update(input).first;
        if (rotation_) {
            last_[0] = lock_[0];
            last_[1] = lock_[1];
            last_[2] = lock_[2];
            target = {joints_lock_[0], joints_lock_[1], joints_lock_[2],
                      joints_lock_[3] + (last_[3] - lock_[3]) * RAD_TO_DEG, 0, 0, 0};
            return true;
        }
        const double angle = -actual.rz;
        const double dx = last_[0] * std::cos(angle) - last_[1] * std::sin(angle);
        const double dy = last_[0] * std::sin(angle) + last_[1] * std::cos(angle);
        NRC_Position result;
        if (!perform_ik(joints, origin_.x + dx, origin_.y + dy,
                        origin_.z + last_[2], origin_.rz + last_[3], result)) {
            return false;
        }
        target = {result.pos[0], result.pos[1], result.pos[2], result.pos[3], 0, 0, 0};
        return true;
    }

private:
    static double filter(double raw, double& value, double dead_zone, double limit) {
        value += FORCE_FILTER_ALPHA * (raw - value);
        const double output = value > dead_zone ? value - dead_zone :
                              (value < -dead_zone ? value + dead_zone : 0);
        return std::max(-limit, std::min(output, limit));
    }

    void lockJoints(const NRC_Position& actual) {
        for (int i = 0; i < 4; ++i) joints_lock_[i] = actual.pos[i];
    }

    Admittance4 controller_;
    MyRobotState origin_{};
    SensorData filtered_{};
    Admittance4::Vec4 last_ = {{0, 0, 0, 0}};
    Admittance4::Vec4 lock_ = {{0, 0, 0, 0}};
    std::array<double, 4> joints_lock_{};
    bool rotation_ = false;
    bool small_ = false;
};

// ==================== 四、独立传感器显示线程 ====================

void publishSensor(int first, const SensorData& force) {
    NRC_SetDoubleVar(first + 0, force.fx);
    NRC_SetDoubleVar(first + 1, force.fy);
    NRC_SetDoubleVar(first + 2, force.fz);
    NRC_SetDoubleVar(first + 3, force.mx);
    NRC_SetDoubleVar(first + 4, force.my);
    NRC_SetDoubleVar(first + 5, force.mz);
}

void feedbackLoop(std::atomic<bool>& running) {
    sched_param param{};
    param.sched_priority = 0;
    pthread_setschedparam(pthread_self(), SCHED_OTHER, &param);
    while (running) {
        publishSensor(7, read_force_sensor_xiao_raw());
        publishSensor(13, read_force_sensor_da_raw());
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}
} // 辅助实现结束

// ==================== 五、主循环：按实际执行顺序阅读 ====================

void RunControlLoop(AsyncLogger& logger, std::atomic<bool>& running) {
    NormalControl normal;
    PressFitMode press;
    bool force_active = false;
    bool last_zero = false;
    bool press_armed = false;
    bool stop_failed = false;
    timespec next;
    resetCycle(next);
    std::thread feedback(feedbackLoop, std::ref(running));

    // 所有运行期错误共用同一停机出口，避免超限后重复调用停止接口。
    auto stop = [&](const char* reason) {
        logger.log(std::string("[停机] ") + reason + "\n");
        NRC_SetBoolVar(1, 0);
        normal.clearVelocity();
        if (force_active) stop_failed = !stopAndPowerOff(logger);
        force_active = false;
        press.reset();
        press_armed = false;
        if (stop_failed) logger.log("[错误] 停机未完成，禁止重新启动，请检查控制器\n");
        resetCycle(next);
    };

    while (running) {
        // 1. 周期等待，随后重新读取本拍开关，避免使用等待前的旧值。
        waitCycle(next, NRC_ReadBoolVar(1));
        const bool force_on = NRC_ReadBoolVar(1);
        const int press_input = NRC_ReadDigInByBoard(1, 2);
        const int rotation_input = NRC_ReadDigInByBoard(1, 1);
        const bool zero_on = NRC_ReadBoolVar(2);
        const bool zero_rising = zero_on && !last_zero;
        last_zero = zero_on;
        const bool small = NRC_ReadBoolVar(5);

        // 2. 检查输入与故障锁定。下压必须先观察到低电平才能启动。
        if (stop_failed) {
            NRC_SetBoolVar(1, 0);
            continue;
        }
        if ((press_input != 0 && press_input != 1) ||
            (rotation_input != 0 && rotation_input != 1)) {
            stop("模式输入读取失败");
            continue;
        }
        const bool press_requested = press_input == 1;
        const bool rotation = rotation_input == 1;
        if (!press_requested) press_armed = true;

        // 3. 关闭请求优先处理，不会在请求关闭后再生成一拍目标。
        if (!force_on && force_active) {
            stop("力控开关关闭");
            continue;
        }

        // 4. 单独清零请求。下压输入有效或贴合过程中不允许改变传感器零点。
        if (zero_rising) {
            if (press_requested || press.active()) {
                logger.log("[下压] 下压和贴合期间禁止传感器清零\n");
            } else {
                zero_force_sensor(logger);
                normal.clearFilter();
                resetCycle(next);
            }
        }

        // 5. 力控开启流程。必须先关闭下压输入，在无接触载荷时完成初始清零。
        if (force_on && !force_active) {
            if (press_requested) {
                logger.log("[下压] 请先关闭下压输入，再开启力控\n");
                NRC_SetBoolVar(1, 0);
                press_armed = false;
                continue;
            }
            MyRobotState initial{};
            NRC_Position initial_joints;
            if (!startControl(logger, initial, initial_joints)) {
                NRC_SetBoolVar(1, 0);
                // 启动失败后可能遗留硬件状态，禁止未经检查反复自动重试。
                stop_failed = true;
                resetCycle(next);
                continue;
            }
            normal.reset(initial, initial_joints, rotation, small);
            press.reset();
            force_active = true;
            resetCycle(next);
        }
        if (!force_active) continue;

        // 6. 读取本拍真实反馈；后续两个模式分支使用同一份位置和传感器数据。
        MyRobotState actual{};
        NRC_Position joints;
        if (!read_robot_full_state(actual) ||
            NRC_GetCurrentPos(NRC_COORD::NRC_ACS, joints) != 0) {
            stop("读取实际位姿失败");
            continue;
        }
        const SensorData force = small ? read_force_sensor_xiao() : read_force_sensor_da();
        if (!validFeedback(actual, joints, force)) {
            stop("位置或力传感器数值无效");
            continue;
        }

        // 7. 下压输入释放：从实际位置重建原模式状态，不自动抬升。
        if (!press_requested && press.active()) {
            press.reset();
            normal.reset(actual, joints, rotation, small);
            logger.log("[下压] 已退出并同步实际位置，恢复原模式，不自动抬升\n");
            resetCycle(next);
            continue;
        }

        // 8. 计算目标：下压优先；否则执行原位置或末端自转模式。
        std::array<double, 7> target{};
        if (press_requested) {
            if (NRC_GetServoStatus() != 3) {
                stop("下压时伺服未处于运行状态");
                continue;
            }
            if (!press.active()) {
                if (!press_armed || !press.begin(actual, force, small, monotonicSeconds())) {
                    stop(press_armed ? press.error() : "下压输入需要先关闭再打开");
                    continue;
                }
                press_armed = false;
                normal.clearVelocity();
                logger.log("[下压] 建立目标力，锁定水平位置与姿态\n");
                resetCycle(next);
                continue;
            }
            if (!press.calculate(actual, joints, force, small, monotonicSeconds(), target, logger)) {
                stop(press.error());
                continue;
            }
        } else if (!normal.calculate(actual, joints, force, rotation, small, target, logger)) {
            stop("位置模式逆解失败");
            continue;
        }

        // 9. 所有模式统一检查关节限位，再发送目标；失败时走统一停机出口。
        if (!withinLimits(target)) {
            NRC_SetBoolVar(4, 1);
            stop("关节目标超限或无效");
            continue;
        }
        if (NRC_Set_ServoJ_Pos(target) != 0) {
            stop("发送关节目标失败");
            continue;
        }
    }

    // 10. 程序退出：停止机器人并等待显示线程结束。
    stop("程序退出");
    if (feedback.joinable()) feedback.join();
}
