#include "Admittance.h"
#include <algorithm>
#include <cmath>

namespace {
    // 辅助函数：生成全0向量
    static inline Admittance4::Vec4 zeros4() {
        return {0.0, 0.0, 0.0, 0.0};
    }
}

Admittance4::Admittance4(
    const Vec4& M,
    const Vec4& D,
    const Vec4& K,
    const Vec4& max_velocity,
    double dt)
    : M_(M),
      D_(D),
      K_(K),
      max_velocity_(max_velocity),
      dt_(dt) {
    // 初始化所有状态为 0
    x_ = zeros4();
    xdot_ = zeros4();
    xdesir_ = zeros4();
    x_e_ = zeros4();
}

void Admittance4::set_state(const Vec4& x, const Vec4& xdesir) {
    x_ = x;
    xdesir_ = xdesir;
    // 清除上一轮导纳运动速度 修改
    xdot_ = zeros4();
    // 保证误差状态与新位置一致 修改
    caculate_error(x_);
}

void Admittance4::caculate_error(const Vec4& x) {

    for (int i = 0; i < 4; ++i) {
        x_e_[i] = xdesir_[i] - x[i];
    }
}

std::pair<Admittance4::Vec4, Admittance4::Vec4> Admittance4::update(const Vec4& F_ext) {
    // 1. 更新位置和角度误差
    caculate_error(x_);

    Vec4 xddot = zeros4();

    // 2. 计算加速度 (M*a = F_ext - D*v - K*e)
    for (int i = 0; i < 4; ++i) {
        // 防止除以0
        if (M_[i] == 0.0) continue; 
        
        // 核心导纳公式
        double force_total = F_ext[i] - D_[i] * xdot_[i] + K_[i] * x_e_[i]; // 注意: 这里通常是 +K*xe (因为 xe = x_des - x) 或者是 -K*(x - x_des)
        xddot[i] = force_total / M_[i];
    }

    // 3. 积分更新速度和位置
    for (int i = 0; i < 4; ++i) {
        // 更新速度 v = v0 + a * dt
        xdot_[i] += xddot[i] * dt_;

        // 对导纳速度进行限幅，防止异常外力造成速度过大。
        const double velocity_limit = std::abs(max_velocity_[i]);
        
        xdot_[i] = std::max(-velocity_limit,std::min(xdot_[i], velocity_limit));
        // 更新位置 x = x0 + v * dt
        // 对于第4维 (theta)，这里直接线性累加即可，无需旋转矩阵
        x_[i] += xdot_[i] * dt_;
    }

    return {x_, xdot_};
}
