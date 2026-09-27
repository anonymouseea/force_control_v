#pragma once

#include <array>

// 力控参数统一放在这里；保持原项目的导纳、滤波和关节限位数值。
namespace ControlConfig {
constexpr double DT = 0.001;
constexpr long CONTROL_PERIOD_NS = 1000000L;
constexpr long IDLE_PERIOD_NS = 10000000L;
constexpr int MAX_CONTROL_GAP_MS = 10;

constexpr double FILTER_ALPHA = 0.03;
constexpr double SMALL_FORCE_DEAD_ZONE = 1.0;
constexpr double SMALL_MOMENT_DEAD_ZONE = 0.2;
constexpr double LARGE_FORCE_DEAD_ZONE = 2.0;
constexpr double LARGE_MOMENT_DEAD_ZONE = 0.5;
constexpr double MAX_FORCE = 50.0;
constexpr double MAX_MOMENT = 2.0;

// 前三项为米每秒，第四项为弧度每秒。
const std::array<double, 4> MAX_VELOCITY = {{0.05, 0.05, 0.03, 0.174533}};
const std::array<double, 4> MASS = {{130.0, 130.0, 120.0, 5.0}};
const std::array<double, 4> DAMPING = {{4000.0, 3000.0, 5000.0, 120.0}};
const std::array<double, 4> STIFFNESS = {{0.0, 0.0, 0.0, 0.0}};

// 保留原关节单位：旋转轴为度，直线轴按控制器配置的关节单位。
const std::array<double, 4> JOINT_MIN = {{-44.0, -820.0, 5.0, -60.0}};
const std::array<double, 4> JOINT_MAX = {{44.0, 1148.0, 848.0, 60.0}};
const std::array<double, 7> TRACKING_VELOCITY = {{60, 60, 60, 60, 20, 20, 20}};
const std::array<double, 7> TRACKING_ACCELERATION = {{1500, 1500, 1500, 1500, 2000, 2000, 2000}};
const std::array<double, 7> TRACKING_JERK = {{2000, 2000, 2000, 2000, 2000, 2000, 2000}};

constexpr int SERVO_TIMEOUT_MS = 1000;
constexpr int STOP_TIMEOUT_MS = 3000;
constexpr int STOP_SAMPLE_MS = 10;
constexpr int STOP_STABLE_MS = 100;
// 四个轴分别配置停稳阈值，避免混淆角度和直线单位。
const std::array<double, 4> STOP_POSITION_TOLERANCE = {{0.01, 0.01, 0.01, 0.01}};
constexpr int ZERO_WARMUP_MS = 10;
constexpr int ZERO_SAMPLE_MS = 2;
constexpr int ZERO_SAMPLE_COUNT = 50;
constexpr int ZERO_TIMEOUT_MS = 1000;
constexpr int FEEDBACK_PERIOD_MS = 500;

constexpr int FORCE_SWITCH_VAR = 1;
constexpr int ZERO_SWITCH_VAR = 2;
constexpr int LIMIT_ALARM_VAR = 4;
constexpr int SMALL_SENSOR_VAR = 5;
constexpr int MODE_BOARD = 1;
constexpr int MODE_CHANNEL = 1;
}
