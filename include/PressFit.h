#pragma once

// 下压参数集中在此；基座 Z 减小为向下，大量程传感器受压时 fz 为正。
namespace PressFitConfig {
constexpr double TARGET_FORCE_N = 50.0;          // 期望接触力，牛顿。
constexpr double FILTER_ALPHA = 0.7;             // 下压专用滤波系数，越大响应越快。
constexpr double FORCE_RAMP_SECONDS = 2.0;       // 建压时间，秒。
constexpr double MAX_SPEED_M_S = 0.010;          // 建压速度上限，10 毫米/秒，不是恒定速度。
constexpr double DAMPING_NS_M = 10000.0;         // 竖直阻尼，牛·秒/米；沿用原力速度增益。
constexpr double FORCE_TOLERANCE_N = 3.0;        // 稳态容差，默认 47～53 牛。
constexpr double STABLE_SECONDS = 5.0;           // 缓增结束后，连续在容差内的时间。
constexpr double HOLD_SPEED_M_S = 0.0002;        // 贴合后仅慢速向下补压，最多 0.2 毫米/秒。
constexpr double MAX_CYCLE_SECONDS = 0.01;       // 周期异常时不按大时间步积分。
constexpr double MAX_FOLLOW_ERROR_M = 0.001;     // 目标与实测高度偏差，不是总下探行程。
static_assert(TARGET_FORCE_N > FORCE_TOLERANCE_N && FORCE_TOLERANCE_N > 0 &&
              FILTER_ALPHA > 0 && FILTER_ALPHA <= 1 && DAMPING_NS_M > 0 &&
              FORCE_RAMP_SECONDS > 0 && STABLE_SECONDS > 0 && MAX_SPEED_M_S > 0 &&
              HOLD_SPEED_M_S > 0 && HOLD_SPEED_M_S <= MAX_SPEED_M_S, "下压参数范围错误");
}
