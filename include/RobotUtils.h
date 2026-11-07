#ifndef ROBOT_UTILS_H
#define ROBOT_UTILS_H

#include <string>

#include "AsyncLogger.h"
#include "nrcAPI.h"

struct SensorData {
    double fx, fy, fz;
    double mx, my, mz;
};

struct MyRobotState {
    double x, y, z, rz;
    double theta2, theta4;
};

// 公共力输入处理：低通滤波、连续死区、输出限幅。
// filtered 保存本通道的滤波历史；各轴、各控制对象仍分别维护，不共享状态。
// 返回处理后的输入值；普通拖动系数统一使用 FORCE_FILTER_ALPHA。
double filterForceInput(double raw, double& filtered, double dead_zone, double limit);

bool setup_realtime();
void SystemStartup();
bool init_force_sensor_mapping();
void zero_force_sensor(AsyncLogger& logger);

SensorData read_force_sensor_da_raw();
SensorData read_force_sensor_xiao_raw();
SensorData read_force_sensor_da();
SensorData read_force_sensor_xiao();

bool read_robot_full_state(MyRobotState& state);
bool perform_ik(NRC_Position& ref_acs, double x_m, double y_m, double z_m, double rz_rad, NRC_Position& res);

std::string MakeLogFileName();

#endif
