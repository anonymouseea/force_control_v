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
    //CLOCK_MONOTONIC为单调时钟id，TIMER_ABSTIME表示绝对时间，next为下一拍的绝对时间
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
}

//返回当前单调时钟的秒数，精度为小数点后若干位
double monotonicSeconds() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

//输入正确性判断
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
//关节限位
bool withinLimits(const std::array<double, 7>& target) {
    const double lower[4] = {-44, -820, 5, -60};
    const double upper[4] = {44, 1148, 848, 60};
    // 如果输入的目标关节位置不在规定的范围内，则返回false，否则返回true
    for (int axis = 0; axis < 4; ++axis) {
        if (!std::isfinite(target[axis]) ||
            target[axis] < lower[axis] || target[axis] > upper[axis]) {
            return false;
        }
    }
    return true;
}

// ==================== 二、伺服启动与停止 ====================
// 等待伺服状态为3，超时返回false,在1s内每10ms检查一次
bool waitServoEnabled() {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);

    while (std::chrono::steady_clock::now() < deadline) {
        if (NRC_GetServoStatus() == 3) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

/*等待机器人停稳，超时返回false,在3s内每10ms检查一次，首先判断机器人运行状态是否为0，
然后判断当前关节位置是否稳定在一个参考位置附近，如果稳定则返回true，否则继续等待，直到超时
同时waitRobotStopped这个也会作为后面其他函数的判断依赖*/
bool waitRobotStopped() {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    bool have_reference = false;
    NRC_Position reference;
    auto stable_since = std::chrono::steady_clock::now();

    while (std::chrono::steady_clock::now() < deadline) {
        NRC_Position actual;
        bool valid = NRC_GetRobotRunStatus() == 0 &&
                     NRC_GetCurrentPos(NRC_COORD::NRC_ACS, actual) == 0;

        if (valid) 
        {
            for (int axis = 0; axis < 4; ++axis) {
                if (!std::isfinite(actual.pos[axis])) valid = false;
            }
        }
        if (!valid) 
        {
            have_reference = false;
        } 
        else 
        {
            bool stable = have_reference;
            for (int axis = 0; axis < 4 && have_reference; ++axis) {
                if (std::abs(actual.pos[axis] - reference.pos[axis]) > 0.01) stable = false;
            }
            if (!stable) {
                reference = actual;
                stable_since = std::chrono::steady_clock::now();
                have_reference = true;
            } 
            else if (std::chrono::steady_clock::now() - stable_since >=
                       std::chrono::milliseconds(100)) 
            {
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
    // 保留统一清理流程，但明确记录失败发生在哪一步。
    const auto fail_start = [&](const std::string& reason) {
        logger.log("[错误] " + reason + "\n");
        stopAndPowerOff(logger);
        return false;
    };
    if (result != 0) {
        return fail_start("开启关节跟踪失败，返回码=" + std::to_string(result));
    }
    if (!waitServoEnabled()) {
        return fail_start("等待伺服使能超时");
    }
    if (!read_robot_full_state(initial)) {
        return fail_start("读取初始末端位姿失败");
    }
    const int position_result = NRC_GetCurrentPos(NRC_COORD::NRC_ACS, joints);
    if (position_result != 0) {
        return fail_start("读取初始关节位置失败，返回码=" + std::to_string(position_result));
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
               bool rotation, bool small) 
    {
        origin_ = actual;
        rotation_ = rotation;
        small_ = small;
        last_ = {0, 0, 0, 0};
        lock_ = last_;
        lockJoints(joints);
        controller_.set_state(last_, {0, 0, 0, 0});
        // 同步导纳位置，清除速度和滤波历史。
        clearVelocity();
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
            filterForceInput(force.fx, filtered_.fx, dead_f, MAX_CONTROL_FORCE),
            filterForceInput(force.fy, filtered_.fy, dead_f, MAX_CONTROL_FORCE),
            filterForceInput(force.fz, filtered_.fz, dead_f, MAX_CONTROL_FORCE),
            filterForceInput(force.mz, filtered_.mz, dead_m, MAX_CONTROL_MOMENT)
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
    bool first_force_cycle = false;
    bool previous_press = false;
    bool stop_failed = false;
    timespec next;
    resetCycle(next);
    std::thread feedback(feedbackLoop, std::ref(running));
    
    // 所有运行期错误共用同一停机出口，避免超限后重复调用停止接口。
    auto stop = [&](const char* reason) {
        logger.log(std::string("[停机] ") + reason + "\n");
        //示教器界面的按钮状态置为关闭
        NRC_SetBoolVar(1, 0);
        normal.clearVelocity();
        if (force_active) stop_failed = !stopAndPowerOff(logger);
        force_active = false;
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
        

        // 小传感器触发
        // 1 → 小量程传感器
        // 0 → 大量程传感器
        const bool small = NRC_ReadBoolVar(5);

        // 查输入与故障锁定。每拍强制关闭力控开关、跳过本轮所有业务逻辑，
        // 让程序进入"活着但什么都不做"的锁定状态，直到人工处理。
        // 下压必须先观察到低电平才能启动。
        if (stop_failed) {
            NRC_SetBoolVar(1, 0);
            continue;
        }
        // 与上force_active就不会关闭力控后每次重复停机
        if (!force_on && force_active) {
            stop("力控开关关闭");
            continue;
        }


        //比较press_input和1是否相等，如果相等，press_requested就返回1
        const bool press_requested = press_input == 1;
        const bool rotation = rotation_input == 1;

        const bool releasing_press = previous_press && !press_requested;
        previous_press = press_requested;


        // 4. 下压输入有效或贴合过程中不允许改变传感器零点。
        const bool zero_rising = zero_on && !last_zero;
        last_zero = zero_on;
        if (zero_rising) 
        {
            if (press_requested || releasing_press) {
                logger.log("[下压] 下压和贴合期间禁止传感器清零\n");
            } else {
                zero_force_sensor(logger);
                normal.clearVelocity();
                resetCycle(next);
            }
        }

        // 5. 力控开启流程。必须先关闭下压输入，在无接触载荷时完成初始清零。
        if (force_on && !force_active) {
            if (press_requested) {
                logger.log("[下压] 请先关闭下压输入，再开启力控\n");
                NRC_SetBoolVar(1, 0);
                continue;
            }
            MyRobotState initial{};
            NRC_Position initial_joints;
            /*  
                调用startControl函数，传入logger、initial和initial_joints参数，
                如果返回false，则执行停机操作，并将stop_failed设置为true，重置循环并继续下一次循环
            */
            if (!startControl(logger, initial, initial_joints)) {
                NRC_SetBoolVar(1, 0);
                // 启动失败后可能遗留硬件状态，禁止未经检查反复自动重试。
                stop_failed = true;
                resetCycle(next);
                continue;
            }
            normal.reset(initial, initial_joints, rotation, small);
            first_force_cycle = true;
            force_active = true;
            resetCycle(next);
            continue;  // 启动会阻塞，下一拍重新读取开关和实际反馈。
        }
        // 如果力控没有启动，就跳过本轮后面的运动控制代码，进入下一轮循环
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

        // 7. 一次调用处理下压的进入、建压、贴合和退出。普通模式仍走原计算。
        std::array<double, 7> target{};
        const PressAction action = press.step(press_requested, small, first_force_cycle, rotation, monotonicSeconds(),
                                              actual, joints, force, target, logger);
        first_force_cycle = false;

        switch (action) {
        case PressAction::Fault:
            stop("下压模式故障，详见上条日志");
            continue;
        case PressAction::Entered:
            normal.clearVelocity();
            continue;
        case PressAction::Released:
            normal.reset(actual, joints, rotation, small);
            logger.log("[下压] 已退出，从实际位置恢复普通模式，不自动抬升\n");
            continue;
        case PressAction::Normal:
            if (!normal.calculate(actual, joints, force, rotation, small, target, logger)) {
                stop("位置模式逆解失败");
                continue;
            }
            break;
        case PressAction::Target:
            break;
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
