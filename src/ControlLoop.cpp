#include "ControlLoop.h"
#include "ControlConfig.h"
#include "ControlStateMachine.h"
#include "NrcControlHardware.h"

#include <cerrno>
#include <time.h>

namespace {
void addPeriod(timespec& time, long period_ns) {
    time.tv_nsec += period_ns;
    if (time.tv_nsec >= 1000000000L) {
        time.tv_nsec -= 1000000000L;
        ++time.tv_sec;
    }
}

bool reached(const timespec& now, const timespec& deadline) {
    return now.tv_sec > deadline.tv_sec ||
        (now.tv_sec == deadline.tv_sec && now.tv_nsec >= deadline.tv_nsec);
}

int waitNextCycle(timespec& next, long period_ns, const volatile std::sig_atomic_t& exit_requested) {
    addPeriod(next, period_ns);
    timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) return errno;
    if (reached(now, next)) {
        // 过期周期不连发补算，从当前时刻建立下一拍。
        next = now;
        addPeriod(next, period_ns);
    }
    int result;
    do {
        result = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    } while (result == EINTR && !exit_requested);
    return result == EINTR ? 0 : result;
}
}

bool RunControlLoop(AsyncLogger& logger, const volatile std::sig_atomic_t& exit_requested) {
    NrcControlHardware hardware(logger);
    ControlStateMachine machine(hardware);
    timespec next;
    if (clock_gettime(CLOCK_MONOTONIC, &next) != 0) {
        hardware.log("[错误] 无法建立控制周期时间基准", errno);
        return false;
    }
    bool timer_failed = false;
    ControlTime previous = ControlClock::now();
    hardware.log("[控制] 状态机已启动，请先确认力控开关为关闭状态");

    while (!machine.exitFinished()) {
        const int timing_error = waitNextCycle(next, machine.periodNanoseconds(), exit_requested);
        const ControlTime now = ControlClock::now();
        if (timing_error != 0) {
            timer_failed = true;
            machine.reportFault("[错误] 控制周期定时失败", now, timing_error);
            // 定时器失效后降低轮询频率，继续尝试完成停机步骤。
            const timespec fallback = {0, 10000000L};
            nanosleep(&fallback, NULL);
        }
        //新增的控制周期间隔超限检测，避免因系统调度延迟导致的控制异常，如果有问题就注释掉
        if (machine.isRunning() && now - previous > std::chrono::milliseconds(ControlConfig::MAX_CONTROL_GAP_MS)) {
            machine.reportFault("[错误] 控制周期间隔超限，进入停机流程", now);
        }
        previous = now;

        ControlInputs inputs;
        if (!hardware.readInputs(inputs)) {
            machine.reportFault("[错误] 模式开关读取失败", now);
        }
        machine.update(inputs, now, exit_requested != 0 || timer_failed);

        if (machine.isRunning() || machine.state() == ControlState::Disabled) {
            hardware.publishFeedback(ControlClock::now());
        }
    }
    if (!machine.exitSucceeded()) {
        hardware.log("[退出] 存在故障，请检查状态日志；不能把进程退出视为停机成功");
    }
    return machine.exitSucceeded();
}
