#pragma once

// 原双模式分支参数，数值保持不变；下压参数另见PressFit.h。
namespace ForceControlParameters {
constexpr double SMALL_SENSOR_FORCE_DEAD_ZONE = 1.0;
constexpr double SMALL_SENSOR_MOMENT_DEAD_ZONE = 0.2;
constexpr double LARGE_SENSOR_FORCE_DEAD_ZONE = 2.0;
constexpr double LARGE_SENSOR_MOMENT_DEAD_ZONE = 0.5;
constexpr double FORCE_FILTER_ALPHA = 0.03;
constexpr double MAX_CONTROL_FORCE = 50.0;
constexpr double MAX_CONTROL_MOMENT = 2.0;
constexpr double MAX_X_VELOCITY = 0.05;
constexpr double MAX_Y_VELOCITY = 0.05;
constexpr double MAX_Z_VELOCITY = 0.03;
constexpr double MAX_ROTATION_VELOCITY = 0.174533;
}
