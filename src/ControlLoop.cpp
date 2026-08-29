#include "ControlLoop.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <thread>
#include <time.h>

#include "Admittance.h"
#include "RobotUtils.h"
#include "nrcAPI.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

// ==================== 力控可调参数 ====================

// 小量程传感器死区。
constexpr double SMALL_SENSOR_FORCE_DEAD_ZONE = 1.0;
constexpr double SMALL_SENSOR_MOMENT_DEAD_ZONE = 0.2;

// 大量程传感器死区。
constexpr double LARGE_SENSOR_FORCE_DEAD_ZONE = 2.0;
constexpr double LARGE_SENSOR_MOMENT_DEAD_ZONE = 0.5;

// 10HZ，若抖动继续jiang
constexpr double FORCE_FILTER_ALPHA = 0.06;

// 允许进入导纳控制的最大平移力和旋转力矩。
constexpr double MAX_CONTROL_FORCE = 80.0;
constexpr double MAX_CONTROL_MOMENT = 2.0;

// 导纳速度上限m/s 弧度/s
constexpr double MAX_X_VELOCITY = 0.05;
constexpr double MAX_Y_VELOCITY = 0.05;
constexpr double MAX_Z_VELOCITY = 0.03;
constexpr double MAX_ROTATION_VELOCITY = 0.174533;

// ======================================================

// 连续死区处理，避免越过死区边界时输出突然跳变。
double apply_continuous_dead_zone(double value,double dead_zone) {
    if (value > dead_zone) {return value - dead_zone;}
    if (value < -dead_zone) {return value + dead_zone;}
    return 0.0;}

// 停止关节跟踪接口只负责停止轨迹生成。这里继续确认控制器已经报告停止，并且四个有效关节的位置在一段时间内保持稳定，之后才允许下使能。
bool wait_robot_stopped(std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    const auto required_stable_time = std::chrono::milliseconds(100);
    const double max_position_change = 0.01;

    NRC_Position stable_reference;
    bool have_stable_reference = false;
    auto stable_since = std::chrono::steady_clock::now();

    while (std::chrono::steady_clock::now() < deadline) {
        const int run_status = NRC_GetRobotRunStatus();

        NRC_Position current_position;
        const bool position_valid =
            (NRC_GetCurrentPos(NRC_COORD::NRC_ACS, current_position) == 0);

        if (run_status == 0 && position_valid) {
            if (!have_stable_reference) {
                stable_reference = current_position;
                stable_since = std::chrono::steady_clock::now();
                have_stable_reference = true;
            } else {
                bool position_stable = true;
                for (int axis = 0; axis < 4; ++axis) {
                    if (std::abs(current_position.pos[axis] -
                                 stable_reference.pos[axis]) >
                        max_position_change) {
                        position_stable = false;
                        break;
                    }
                }

                if (!position_stable) {
                    stable_reference = current_position;
                    stable_since = std::chrono::steady_clock::now();
                } else if (std::chrono::steady_clock::now() - stable_since >=
                           required_stable_time) {
                    return true;
                }
            }
        } else {
            // 运动状态重新变为运行，或者位置读取失败，重新开始计时。
            have_stable_reference = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return false;
}

bool wait_servo_enabled(std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (NRC_GetServoStatus() == 3) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

bool stop_and_power_off(AsyncLogger& logger) {
    const int stop_result = NRC_RKG_Stop();
    if (stop_result != 0) {
        logger.log("[错误] 停止关节跟踪失败，伺服保持使能\n");
        return false;
    }
    // 最多等待3秒。
    if (!wait_robot_stopped(std::chrono::milliseconds(3000))) {
        logger.log("[错误] 等待机器人停稳超时，取消下使能，伺服保持使能\n");
        return false;
    }
    const int power_off_result = NRC_PowerOff();
    if (power_off_result != 0) {
        logger.log("[错误] 伺服下使能失败\n");
        return false;
    }
    logger.log("[控制] 机器人已经停稳，伺服已下使能\n");
    return true;
}



} // 匿名命名空间

void RunControlLoop(AsyncLogger& logger, std::atomic<bool>& running) {
    Admittance4 controller({130.0, 130.0, 120.0, 5},
                           {4000.0, 3000.0, 5000.0, 120},
                           {0, 0, 0, 0},
                           {MAX_X_VELOCITY,MAX_Y_VELOCITY,MAX_Z_VELOCITY,MAX_ROTATION_VELOCITY},
                           0.001);
    //状态初始化
    MyRobotState init_s{};
    double base_x = 0.0, base_y = 0.0, base_z = 0.0;
    double initial_total_rz = 0.0;
    enum ControlMode { MODE_POS, MODE_ROT };
    //默认初始模式为位置模式
    ControlMode current_mode = MODE_POS;

    Admittance4::Vec4 last_target_tool = {0, 0, 0, 0};
    Admittance4::Vec4 active_pos_lock = {0, 0, 0, 0};
    std::array<double, 7> target_joints{};

    // 旋转模式下保存进入模式时四个关节的实际位置。
    std::array<double, 4> rotation_locked_joints{0, 0, 0, 0};
    // 记录进入旋转模式时的导纳旋转偏移，用于计算第4关节增量。
    double rotation_start_tool_rz = 0.0;
    bool rotation_joint_lock_valid = false;

    std::thread force_feedback_thread([&running]() {
        struct sched_param param;
        param.sched_priority = 0;
        pthread_setschedparam(pthread_self(), SCHED_OTHER, &param);
        while (running) {
            SensorData data_xiao = read_force_sensor_xiao_raw();
            NRC_SetDoubleVar(7, data_xiao.fx); NRC_SetDoubleVar(8, data_xiao.fy); NRC_SetDoubleVar(9, data_xiao.fz);
            NRC_SetDoubleVar(10, data_xiao.mx); NRC_SetDoubleVar(11, data_xiao.my); NRC_SetDoubleVar(12, data_xiao.mz);

            SensorData data_da = read_force_sensor_da_raw();
            NRC_SetDoubleVar(13, data_da.fx); NRC_SetDoubleVar(14, data_da.fy); NRC_SetDoubleVar(15, data_da.fz);
            NRC_SetDoubleVar(16, data_da.mx); NRC_SetDoubleVar(17, data_da.my); NRC_SetDoubleVar(18, data_da.mz);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    });

    bool last_force_switch = false, last_zero_switch = false;

    // --- 高精度硬实时循环起始 ---
    struct timespec next_p;
    clock_gettime(CLOCK_MONOTONIC, &next_p);
    SensorData ft{};
    double dead_zone_f = 0.0, dead_zone_m = 0.0;
    // 力信号低通滤波状态。
    SensorData filtered_ft{};

    /*主循环*/
    while (running) {
        // 判断当前是否力控开启，动态调整周期
        bool force_on = (NRC_ReadBoolVar(1) == 1);
        int cycle_ns = force_on ? 1000000 : 200000000; // 力控时1ms, 否则100ms

        // 设置下一个唤醒时间点
        next_p.tv_nsec += cycle_ns;
        while (next_p.tv_nsec >= 1000000000L) { next_p.tv_nsec -= 1000000000L; next_p.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_p, NULL);

        // 传感器清零逻辑
        bool zero_on = (NRC_ReadBoolVar(2) == 1);
        if (zero_on && !last_zero_switch) {
            zero_force_sensor(logger);
            // 传感器零点变化后，清空旧的滤波数据。
            filtered_ft = SensorData{};
            // 清零过程会阻塞控制循环，重新建立周期基准。
            clock_gettime(CLOCK_MONOTONIC, &next_p);
        }
        last_zero_switch = zero_on;

        /*力控开关*/ 
        force_on = (NRC_ReadBoolVar(1) == 1); // 再次获取，保证后续逻辑一致
        if (force_on && !last_force_switch) {
            logger.log("[控制] 力控开启\n");
            NRC_ClearAllError();
            zero_force_sensor(logger);
            // 设置伺服准备状态。
            if (NRC_SetServoReadyStatus(1) != 0) {logger.log("[错误] 设置伺服准备状态失败\n");NRC_SetBoolVar(1, 0);clock_gettime(CLOCK_MONOTONIC, &next_p);continue;}
            // 伺服上使能。
            if (NRC_PowerOn() != 0) {logger.log("[错误] 伺服上使能失败\n");NRC_SetBoolVar(1, 0);clock_gettime(CLOCK_MONOTONIC, &next_p);continue;}

            // 等待伺服真正进入运行状态。
            if (!wait_servo_enabled(std::chrono::milliseconds(1000))) {
                logger.log("[错误] 等待伺服上使能超时\n");
                // 此时还没有开启关节跟踪，也没有发送运动目标，可以执行下使能清理。
                NRC_PowerOff();
                NRC_SetBoolVar(1, 0);
                clock_gettime(CLOCK_MONOTONIC, &next_p);
                continue;
            }
            /*读取当前机器人位姿*/
            if (!read_robot_full_state(init_s)) {logger.log("[错误] 读取机器人当前位置失败，取消开启力控\n");
                NRC_PowerOff();
                NRC_SetBoolVar(1, 0);
                clock_gettime(CLOCK_MONOTONIC, &next_p);
                continue;}
            /*初始化进入力控时的位姿*/
            base_x = init_s.x;
            base_y = init_s.y;
            base_z = init_s.z;
            initial_total_rz = init_s.rz;
            // 每次开启力控时，根据当前输入重新初始化控制模式。
            current_mode =(NRC_ReadDigInByBoard(1, 1) == 1)? MODE_ROT: MODE_POS;
            // 新一轮力控不能使用上一轮保存的关节锁定位置。
            rotation_joint_lock_valid = false;
            // 新一轮力控的平移锁定位置从零偏移开始。
            active_pos_lock = {0, 0, 0, 0};

            controller.set_state({0,0,0,0}, {0,0,0,0});
            last_target_tool = {0,0,0,0};
            // 每次重新开启力控时，从零开始建立滤波状态。
            filtered_ft = SensorData{};
            // 最后才开启关节跟踪。
            const int open_result = NRC_RKG_Open(
                {60, 60, 60, 60, 20, 20, 20},
                {1500, 1500, 1500, 1500, 2000, 2000, 2000},
                {2000, 2000, 2000, 2000, 2000, 2000, 2000});
            if (open_result != 0) {
                logger.log("[错误] 开启关节跟踪失败\n");
                NRC_PowerOff();
                NRC_SetBoolVar(1, 0);
                clock_gettime(CLOCK_MONOTONIC, &next_p);
                continue;
            }
            // 初始化过程耗时较长，禁止追赶之前积压的控制周期。
            clock_gettime(CLOCK_MONOTONIC, &next_p);
        }

        /*关闭力控：停止轨迹，确认机器人停稳后再关闭使能。*/ 
        if (!force_on && last_force_switch) {
            logger.log("[控制] 力控关闭，正在停止机器人\n");
            // 下次开启时不得继承本轮导纳速度。
            controller.set_state(last_target_tool, {0, 0, 0, 0});
            stop_and_power_off(logger);
            // 停机等待会阻塞周期循环，重新建立绝对定时基准，禁止追赶旧周期。
            clock_gettime(CLOCK_MONOTONIC, &next_p);
        }
        last_force_switch = force_on;
        if (!force_on) continue;

        MyRobotState curr_s{};
        if (!read_robot_full_state(curr_s)) {
            logger.log("[错误] 控制过程中读取机器人状态失败，关闭力控\n");
            NRC_SetBoolVar(1, 0);
            continue;
        }

        NRC_Position ref_acs;
        if (NRC_GetCurrentPos(NRC_COORD::NRC_ACS, ref_acs) != 0) {
            logger.log("[错误] 读取逆解参考关节位置失败（当前位置），关闭力控\n");
            NRC_SetBoolVar(1, 0);
            continue;
        }

        // 如果开启力控时已经处于旋转模式，先同步真机实际关节位置。
        if (current_mode == MODE_ROT && !rotation_joint_lock_valid) {
            rotation_locked_joints[0] = ref_acs.pos[0];
            rotation_locked_joints[1] = ref_acs.pos[1];
            rotation_locked_joints[2] = ref_acs.pos[2];
            rotation_locked_joints[3] = ref_acs.pos[3];
            rotation_start_tool_rz = last_target_tool[3];
            rotation_joint_lock_valid = true;

            logger.log("[控制] 已同步真机状态，旋转模式只控制第4关节\n");
        }

        SensorData ft_da = read_force_sensor_da();
        SensorData ft_xiao = read_force_sensor_xiao();

        // 传感器切换逻辑
        static bool last_use_small_sensor = false;
        bool use_small_sensor = (NRC_ReadBoolVar(5) == 1);
        if (use_small_sensor != last_use_small_sensor) {
            // 切换传感器时，保持当前位置偏移，将速度置0，防止跳动
            controller.set_state(last_target_tool, {0,0,0,0});
            // 两个传感器的零点和噪声不同，切换时清空滤波状态。
            filtered_ft = SensorData{};
            logger.log(std::string("[传感器切换] 使用") + (use_small_sensor ? "小量程" : "大量程") + "传感器\n");
            last_use_small_sensor = use_small_sensor;
        }

        if (use_small_sensor) {
            ft = ft_xiao;
            // 小量程传感器的死区
            dead_zone_f = SMALL_SENSOR_FORCE_DEAD_ZONE;
            dead_zone_m = SMALL_SENSOR_MOMENT_DEAD_ZONE;
        } else {
            ft = ft_da;
            // 大量程传感器的死区
            dead_zone_f = LARGE_SENSOR_FORCE_DEAD_ZONE;
            dead_zone_m = LARGE_SENSOR_MOMENT_DEAD_ZONE;
        }

        // 一阶低通滤波。
        filtered_ft.fx += FORCE_FILTER_ALPHA * (ft.fx - filtered_ft.fx);
        filtered_ft.fy += FORCE_FILTER_ALPHA * (ft.fy - filtered_ft.fy);
        filtered_ft.fz += FORCE_FILTER_ALPHA * (ft.fz - filtered_ft.fz);
        filtered_ft.mz += FORCE_FILTER_ALPHA * (ft.mz - filtered_ft.mz);

        // 连续死区处理。
        ft.fx = apply_continuous_dead_zone(
            filtered_ft.fx,
            dead_zone_f);

        ft.fy = apply_continuous_dead_zone(
            filtered_ft.fy,
            dead_zone_f);

        ft.fz = apply_continuous_dead_zone(
            filtered_ft.fz,
            dead_zone_f);

        ft.mz = apply_continuous_dead_zone(
            filtered_ft.mz,
            dead_zone_m);

        /*限制进入导纳控制的外力，防止传感器异常尖峰造成目标突变。*/ 
        ft.fx = std::max(
            -MAX_CONTROL_FORCE,
            std::min(ft.fx, MAX_CONTROL_FORCE));
        ft.fy = std::max(
            -MAX_CONTROL_FORCE,
            std::min(ft.fy, MAX_CONTROL_FORCE));
        ft.fz = std::max(
            -MAX_CONTROL_FORCE,
            std::min(ft.fz, MAX_CONTROL_FORCE));
        ft.mz = std::max(
            -MAX_CONTROL_MOMENT,
            std::min(ft.mz, MAX_CONTROL_MOMENT));
 
        /*模式切换*/ 
        ControlMode target_mode =
            (NRC_ReadDigInByBoard(1, 1) == 1) ? MODE_ROT : MODE_POS;
        if (target_mode != current_mode) 
        {
            current_mode = target_mode;
            logger.log(std::string("[模式] 已切换到") +
                    (current_mode == MODE_ROT ? "旋转模式\n" : "位置模式\n"));

            // 计算真机当前位置相对于本轮力控起始位置的基座坐标偏移。
            const double actual_dx = curr_s.x - base_x;
            const double actual_dy = curr_s.y - base_y;
            const double actual_dz = curr_s.z - base_z;

            // 将真机实际基座坐标偏移转换成导纳控制器使用的工具坐标偏移。
            const double c = std::cos(curr_s.rz);
            const double s = std::sin(curr_s.rz);

            /* 工具坐标系下的位移 */
            Admittance4::Vec4 actual_tool = {
                actual_dx * c - actual_dy * s,
                actual_dx * s + actual_dy * c,
                actual_dz,
                curr_s.rz - initial_total_rz};

            // 将导纳虚拟位置同步到真机实际位置，并清零虚拟速度。
            controller.set_state(actual_tool, {0, 0, 0, 0});
            last_target_tool = actual_tool;

            if (current_mode == MODE_ROT) {
                // 进入旋转模式时同步真机实际平移位置和四个关节位置。
                active_pos_lock = actual_tool;
                rotation_locked_joints[0] = ref_acs.pos[0];
                rotation_locked_joints[1] = ref_acs.pos[1];
                rotation_locked_joints[2] = ref_acs.pos[2];
                rotation_locked_joints[3] = ref_acs.pos[3];
                rotation_start_tool_rz = actual_tool[3];
                rotation_joint_lock_valid = true;
            } else {
                // 切回位置模式前已同步真机实际位姿，此处解除旋转关节锁定。
                rotation_joint_lock_valid = false;
            }
            // 清除切换前的力信号残留，避免切换当周期产生跳动。
            filtered_ft = SensorData{};
            ft = SensorData{};
        }

        /*锁定力信号*/
        if (current_mode == MODE_POS) 
        {
            ft.mz = 0.0;
        }
        else{
            ft.fx=0.0;
            ft.fy=0.0;
            ft.fz=0.0;
        }
        /*导纳控制器计算*/
        auto result = controller.update({ft.fx, ft.fy, ft.fz, ft.mz});
        Admittance4::Vec4 target_tool = result.first;

        if (current_mode == MODE_ROT) { 
            target_tool[0] = active_pos_lock[0];
            target_tool[1] = active_pos_lock[1]; 
            target_tool[2] = active_pos_lock[2]; 
        }
        last_target_tool = target_tool;

        if (current_mode == MODE_ROT && rotation_joint_lock_valid) {
            // 旋转模式不进行笛卡尔逆解，只将导纳旋转增量转换成第4关节角度。
            const double rotation_delta_deg =
                (target_tool[3] - rotation_start_tool_rz) * 180.0 / M_PI;

            // 前三个关节保持进入旋转模式时的位置，第4关节从实际位置继续旋转。
            target_joints = {
                rotation_locked_joints[0],
                rotation_locked_joints[1],
                rotation_locked_joints[2],
                rotation_locked_joints[3] + rotation_delta_deg,
                0,
                0,
                0
            };
        } else {
            // 位置模式将工具坐标偏移转换到基座坐标，再进行完整逆解。
            const double angle = -curr_s.rz;
            const double dx =
                target_tool[0] * cos(angle) - target_tool[1] * sin(angle);
            const double dy =
                target_tool[0] * sin(angle) + target_tool[1] * cos(angle);
            const double target_total_rz =
                initial_total_rz + target_tool[3];

            NRC_Position ik_res;
            if (!perform_ik(
                    ref_acs,
                    base_x + dx,
                    base_y + dy,
                    base_z + target_tool[2],
                    target_total_rz,
                    ik_res)) {
                logger.log("[错误] 机器人逆解失败，关闭力控\n");
                NRC_SetBoolVar(1, 0);
                continue;
            }

            target_joints = {
                ik_res.pos[0],
                ik_res.pos[1],
                ik_res.pos[2],
                ik_res.pos[3],
                0,
                0,
                0
            };
        }

        /*关节限位保护。*/ 
        if (target_joints[0] < -44 || target_joints[0] > 44 ||
            target_joints[1] < -840 || target_joints[1] > 1148 ||
            target_joints[2] < 5 || target_joints[2] > 848 ||
            target_joints[3] < -60 || target_joints[3] > 60) {
            NRC_SetBoolVar(4, 1);
            NRC_SetBoolVar(1, 0);
            logger.log("[错误] 关节超限，关闭力控\n");
            continue;
        }

        /*透传接口*/
        const int send_result = NRC_Set_ServoJ_Pos(target_joints);
        if (send_result != 0) {
            logger.log("[错误] 发送关节目标失败，关闭力控\n");
            NRC_SetBoolVar(1, 0);
            continue;
        }
    }

    // 程序退出前先关闭力控开关，防止外部状态仍显示力控开启。
    NRC_SetBoolVar(1, 0);

    // 如果退出时力控仍然开启，执行完整的安全停机和下使能流程。
    if (last_force_switch) {
        logger.log("[控制] 程序正在退出，开始安全停止机器人\n");
        stop_and_power_off(logger);
    }

    // 等待传感器反馈线程结束，避免程序退出时仍有后台访问控制器。
    if (force_feedback_thread.joinable()) {
        force_feedback_thread.join();
    }

}
