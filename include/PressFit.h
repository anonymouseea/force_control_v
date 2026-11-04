#pragma once
#include <algorithm>
#include <cmath>

// 下压沿基座竖直轴，前提是传感器轴向力已标定为牛顿且与基座竖直方向平行。
namespace PressFitConfig {
constexpr double TARGET_FORCE_N = 50.0;
// 未确认时保持0，确认后填写+1或-1。0会拒绝进入下压模式。
constexpr int DOWN_Z_SIGN = 0;
constexpr int LARGE_CONTACT_FORCE_SIGN = 0;
constexpr double FORCE_TOLERANCE_N = 3.0;
constexpr double MAX_FORCE_N = 80.0;
constexpr double FILTER_ALPHA = 0.03;
// 力处于容差内且竖直位置、速度稳定，连续5秒才锁定高度。
constexpr double STABLE_SECONDS = 5.0;
constexpr double FORCE_RAMP_SECONDS = 1.0;
constexpr double TIMEOUT_SECONDS = 15.0;
constexpr double MAX_TRAVEL_M = 0.02;
constexpr double MAX_SPEED_M_S = 0.002;
constexpr double MAX_ACCEL_M_S2 = 0.01;
constexpr double FORCE_TO_SPEED = 0.0001;
constexpr double MAX_FOLLOW_ERROR_M = 0.001;
constexpr double STABLE_Z_WINDOW_M = 0.0001;
constexpr double STABLE_SPEED_M_S = 0.0001;
constexpr double MAX_CYCLE_SECONDS = 0.01;
}

enum class PressPhase { Idle, Pressing, Seated, Fault };

// 下压使用力误差生成限速、限加速度的竖直速度；贴合后锁实际高度。
class PressFit {
public:
    bool begin(double now, double actual_z, double raw_fz, bool small) {
        // 入口再次限制量程，避免其他调用者使用小量程建立下压力。
        if (small) return fail("下压及贴合仅允许大量程传感器");
        return beginWithSigns(now, actual_z, raw_fz, PressFitConfig::DOWN_Z_SIGN,
                              PressFitConfig::LARGE_CONTACT_FORCE_SIGN);
    }
    // 独立的方向入口方便离线测试，真机入口使用上面的集中配置。
    bool beginWithSigns(double now, double z, double force, int down, int contact) {
        reset();
        if ((down != 1 && down != -1) || (contact != 1 && contact != -1)) return fail("下压方向或接触力符号尚未配置");
        if (!std::isfinite(now) || !std::isfinite(z) || !std::isfinite(force)) return fail("进入下压时数据无效");
        if (std::abs(force) >= PressFitConfig::MAX_FORCE_N) return fail("进入下压时轴向力已经超限");
        down_ = down; sign_ = contact;
        start_ = previous_ = now;
        initial_z_ = target_z_ = z;
        filtered_ = sign_ * force;
        phase_ = PressPhase::Pressing;
        return true;
    }
    bool update(double now, double actual_z, double raw_fz) {
        if (phase_ == PressPhase::Fault || phase_ == PressPhase::Idle) return false;
        if (!std::isfinite(now) || !std::isfinite(actual_z) || !std::isfinite(raw_fz)) return fail("下压反馈数据无效");
        const double dt = now - previous_;
        previous_ = now;
        if (dt <= 0 || dt > PressFitConfig::MAX_CYCLE_SECONDS) return fail("下压控制周期异常");
        // 过力检查使用未经死区、限幅、滤波的反馈，避免把危险峰值掩盖掉。
        if (std::abs(raw_fz) >= PressFitConfig::MAX_FORCE_N) return fail("下压或贴合时轴向力超限");
        if (std::abs(actual_z - initial_z_) > PressFitConfig::MAX_TRAVEL_M) return fail("实际竖直行程超限");
        if (std::abs(target_z_ - actual_z) > PressFitConfig::MAX_FOLLOW_ERROR_M) return fail("竖直目标与实际位置偏差超限");
        if (phase_ == PressPhase::Seated) return true;
        if (now - start_ > PressFitConfig::TIMEOUT_SECONDS) return fail("下压贴合超时");
        filtered_ += PressFitConfig::FILTER_ALPHA * (sign_ * raw_fz - filtered_);
        // 明显反向受力提示标定或接触方向异常，禁止继续追求目标力。
        if (sign_ * raw_fz < -PressFitConfig::FORCE_TOLERANCE_N) return fail("接触力方向与配置不一致");
        const double ramp = std::min(1.0, (now - start_) / PressFitConfig::FORCE_RAMP_SECONDS);
        const double error = ramp * PressFitConfig::TARGET_FORCE_N - filtered_;
        double requested = down_ * error * PressFitConfig::FORCE_TO_SPEED;
        requested = std::max(-PressFitConfig::MAX_SPEED_M_S, std::min(requested, PressFitConfig::MAX_SPEED_M_S));
        const double dv = PressFitConfig::MAX_ACCEL_M_S2 * dt;
        velocity_ += std::max(-dv, std::min(requested - velocity_, dv));

        const bool in_band = ramp == 1.0 &&
            std::abs(filtered_ - PressFitConfig::TARGET_FORCE_N) <= PressFitConfig::FORCE_TOLERANCE_N &&
            std::abs(sign_ * raw_fz - PressFitConfig::TARGET_FORCE_N) <= PressFitConfig::FORCE_TOLERANCE_N &&
            std::abs(velocity_) <= PressFitConfig::STABLE_SPEED_M_S;
        if (!in_band) stable_ = false;
        else if (!stable_ || std::abs(actual_z - stable_z_) > PressFitConfig::STABLE_Z_WINDOW_M) {
            stable_ = true; stable_since_ = now; stable_z_ = actual_z;
        } else if (now - stable_since_ >= PressFitConfig::STABLE_SECONDS) {
            target_z_ = actual_z;
            velocity_ = 0;
            phase_ = PressPhase::Seated;
            return true;
        }
        const double next = target_z_ + velocity_ * dt;
        if (std::abs(next - initial_z_) > PressFitConfig::MAX_TRAVEL_M) return fail("下压目标行程超限");
        if (std::abs(next - actual_z) > PressFitConfig::MAX_FOLLOW_ERROR_M) return fail("下压目标追赶偏差超限");
        target_z_ = next;
        return true;
    }
    void reset() { phase_ = PressPhase::Idle; velocity_ = 0; stable_ = false; error_ = ""; }
    PressPhase phase() const { return phase_; }
    double targetZ() const { return target_z_; }
    const char* error() const { return error_; }
private:
    bool fail(const char* text) { error_ = text; phase_ = PressPhase::Fault; velocity_ = 0; return false; }
    PressPhase phase_ = PressPhase::Idle;
    int down_ = 0, sign_ = 0;
    bool stable_ = false;
    double initial_z_ = 0, target_z_ = 0, velocity_ = 0, filtered_ = 0;
    double start_ = 0, previous_ = 0, stable_since_ = 0, stable_z_ = 0;
    const char* error_ = "";
};
