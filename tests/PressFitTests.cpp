#include "PressFit.h"
#include <iostream>
#include <stdexcept>
#include <limits>
#include <string>

void require(bool value, const char* reason) {
    if (!value) throw std::runtime_error(reason);
}

int main() {
    try {
        PressFit small_sensor;
        require(!small_sensor.begin(0, 0, 0, true), "小量程被允许进入下压");
        require(std::string(small_sensor.error()) == "下压及贴合仅允许大量程传感器",
                "小量程没有被量程限制拒绝");
        require(!small_sensor.update(0.001, 0, 0), "拒绝小量程后仍继续下压计算");
        PressFit unconfigured;
        require(!unconfigured.begin(0, 0, 0, false), "未确认方向却允许自动下压");
        PressFit fit;
        require(fit.beginWithSigns(0, 0, 0, -1, 1), "测试初始化失败");
        double actual = 0;
        double now = 0;
        for (int i = 1; i <= 14000 && fit.phase() == PressPhase::Pressing; ++i) {
            now = i * 0.001;
            // 用弹性装配面验证力反馈方向：压入越多，反作用力越大。
            const double force = std::max(0.0, -actual * 20000.0);
            require(fit.update(now, actual, force), fit.error());
            actual = fit.targetZ();
        }
        require(fit.phase() == PressPhase::Seated, "未建立接触并完成连续5秒稳定计时");
        require(now >= 6.0, "未等待力爬升和连续5秒稳定就锁定");
        const double locked = fit.targetZ();
        // 完成贴合后，低于报警阈值的正负竖直力都不能改变高度目标。
        for (int i = 1; i <= 100; ++i) {
            require(fit.update(now + i * 0.001, locked, i % 2 ? -30.0 : 70.0), fit.error());
            require(fit.targetZ() == locked, "贴合后竖直力改变了高度目标");
        }
        require(!fit.update(now + 0.101, locked, 81.0), "贴合后失去过力保护");

        PressFit interrupted;
        require(interrupted.beginWithSigns(0, 0, 50, -1, 1), "初始化失败");
        actual = 0;
        for (int i = 1; i <= 9000; ++i) {
            // 第4秒施加短时偏差，稳定计时必须重新开始。
            const double force = i >= 4000 && i < 4100 ? 40.0 : 50.0;
            require(interrupted.update(i * 0.001, actual, force), interrupted.error());
            actual = interrupted.targetZ();
        }
        require(interrupted.phase() == PressPhase::Pressing, "力越界没有重置连续5秒计时");
        for (int i = 9001; i <= 11000; ++i) {
            require(interrupted.update(i * 0.001, actual, 50), interrupted.error());
            actual = interrupted.targetZ();
        }
        require(interrupted.phase() == PressPhase::Seated, "恢复稳定后未锁定高度");

        PressFit wrong;
        wrong.beginWithSigns(0, 0, 0, -1, 1);
        require(!wrong.update(0.001, 0, -10), "反向受力未被识别");
        PressFit stalled;
        stalled.beginWithSigns(0, 0, 0, -1, 1);
        require(!stalled.update(0.02, 0, 0), "周期超时未被识别");
        PressFit bad;
        bad.beginWithSigns(0, 0, 0, -1, 1);
        require(!bad.update(0.001, 0, std::numeric_limits<double>::quiet_NaN()), "无效力未被识别");
        PressFit lag;
        lag.beginWithSigns(0, 0, 0, -1, 1);
        require(!lag.update(0.001, 0.002, 0), "目标跟踪偏差未被识别");
        PressFit timeout;
        timeout.beginWithSigns(0, 0, 0, -1, 1);
        bool ok = true;
        for (int i = 1; i <= 16000 && ok; ++i) ok = timeout.update(i * 0.001, timeout.targetZ(), 0);
        require(!ok && timeout.phase() == PressPhase::Fault, "没有装配面时未被行程或超时保护停止");
        std::cout << "[通过] 建立接触、连续5秒、越界重计时、锁高度、过力、方向、周期、无效值、跟踪偏差和无接触保护\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[失败] " << error.what() << '\n';
        return 1;
    }
}
