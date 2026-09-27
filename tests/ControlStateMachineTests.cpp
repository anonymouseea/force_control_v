#include "ControlStateMachine.h"
#include "SensorCalibration.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

class FakeHardware : public ControlHardware {
public:
    RobotSnapshot actual;
    ForceSample force = {{0, 0, 0, 0, 0, 0}};
    JointTarget sent{};
    std::array<double, 4> inverse_pose{};
    std::vector<std::string> calls;
    std::vector<std::string> logs;
    SensorCalibration calibration;
    std::string fail_command;
    int servo = 1;
    int run = 0;
    int servo_after_on = 3;
    int servo_after_off = 1;
    int sends = 0;
    int inverses = 0;
    bool read_ok = true;
    bool inverse_ok = true;
    bool force_ok = true;
    bool zero_pending = false;
    bool zero_failed = false;
    bool out_of_limit = false;
    bool nan_target = false;
    bool print_logs = false;

    FakeHardware() {
        actual.pose = {{0.4, 0.2, 0.3, 0.1}};
        actual.joints = {{0, 0, 100, 10, 0, 0, 0}};
    }
    int command(const char* name) {
        calls.push_back(name);
        return fail_command == name ? -103 : 0;
    }
    int count(const char* name) const { return static_cast<int>(std::count(calls.begin(), calls.end(), name)); }
    int clearErrors() override { return command("清除错误"); }
    int setServoReady() override { return command("伺服准备"); }
    int powerOn() override { servo = servo_after_on; return command("上使能"); }
    int openTracking() override { return command("开启跟踪"); }
    int stopTracking() override { return command("关闭跟踪"); }
    int powerOff() override { servo = servo_after_off; return command("下使能"); }
    int servoStatus() override { return servo; }
    int runStatus() override { return run; }
    bool readRobot(RobotSnapshot& snapshot) override { snapshot = actual; return read_ok; }
    bool readJoints(JointTarget& joints) override { joints = actual.joints; return read_ok; }
    bool readForce(bool, ForceSample& sample) override { sample = force; return force_ok; }
    bool inverseKinematics(const RobotSnapshot&, const std::array<double, 4>& pose, JointTarget& target) override {
        ++inverses;
        inverse_pose = pose;
        target = actual.joints;
        if (out_of_limit) target[0] = 100;
        if (nan_target) target[3] = std::numeric_limits<double>::quiet_NaN();
        return inverse_ok;
    }
    int sendTarget(const JointTarget& target) override { ++sends; sent = target; return command("发送目标"); }
    int requestForceOff() override { return command("关闭力控开关"); }
    int setLimitAlarm() override { return command("限位报警"); }
    void beginZero(ControlTime now) override { calls.push_back("开始清零"); calibration.begin(now); }
    ZeroResult pollZero(ControlTime now) override {
        if (zero_pending) return ZeroResult::Pending;
        if (zero_failed) return ZeroResult::Failed;
        const ForceSample zero = {{0, 0, 0, 0, 0, 0}};
        return calibration.sample(now, zero, zero);
    }
    void cancelZero() override { calibration.cancel(); }
    void log(const char* message, int code = 0) override {
        logs.push_back(std::string(message) + "，代码=" + std::to_string(code));
        if (print_logs) std::cout << logs.back() << '\n';
    }
};

struct Rig {
    FakeHardware hardware;
    ControlStateMachine machine;
    ControlInputs inputs;
    ControlTime now{};
    Rig() : machine(hardware) {}
    void tick(int milliseconds = 1, bool exit = false) {
        now += std::chrono::milliseconds(milliseconds);
        machine.update(inputs, now, exit);
    }
    void advance(int milliseconds) { for (int i = 0; i < milliseconds; ++i) tick(); }
    void start() {
        inputs.force_on = false;
        tick();
        inputs.force_on = true;
        tick();
        advance(125);
        check(machine.isRunning(), "正常启动未进入力控运行状态");
    }
    void stop() {
        inputs.force_on = false;
        tick();
        advance(150);
    }
};

void testStartAndStop() {
    Rig r;
    r.inputs.force_on = true;
    r.advance(200);
    check(r.hardware.count("上使能") == 0, "程序启动时开关为高电平，不应自动使能");
    r.start();
    const std::vector<std::string>& calls = r.hardware.calls;
    const auto on = std::find(calls.begin(), calls.end(), "上使能");
    const auto open = std::find(calls.begin(), calls.end(), "开启跟踪");
    const auto send = std::find(calls.begin(), calls.end(), "发送目标");
    check(on < open && open < send, "启动接口顺序错误");
    check(r.machine.state() == ControlState::PositionControl, "默认应进入位置模式");
    const int sent = r.hardware.sends;
    r.inputs.force_on = false;
    r.tick();
    r.advance(50);
    check(r.hardware.count("下使能") == 0, "未达到连续停稳时间就下使能");
    r.advance(100);
    check(r.machine.state() == ControlState::Disabled, "正常停机未完成");
    check(r.hardware.sends == sent, "停止过程中仍发送了运动目标");
    check(r.hardware.count("关闭跟踪") == 1 && r.hardware.count("下使能") == 1,
          "正常停机重复或遗漏了硬件操作");
}

void testModeSynchronization() {
    Rig r;
    r.start();
    r.hardware.force = {{30, 25, 20, 0, 0, 2}};
    r.advance(200);
    r.hardware.actual.joints = {{12, 80, 240, -10, 0, 0, 0}};
    r.inputs.rotation_mode = true;
    const int sends = r.hardware.sends;
    r.tick();
    check(r.hardware.sends == sends, "切换同步当拍不应发送运动目标");
    const int inverses = r.hardware.inverses;
    r.advance(200);
    check(r.hardware.inverses == inverses, "旋转模式不应执行逆解");
    check(r.hardware.sent[0] == 12 && r.hardware.sent[1] == 80 && r.hardware.sent[2] == 240,
          "旋转模式的前三关节没有保持进入模式时的位置");
    check(r.hardware.sent[3] > -10, "末端旋转没有从真实角度开始增加");

    r.hardware.actual.pose = {{0.8, -0.3, 0.5, 1.2}};
    r.hardware.force.fill(0.0);
    r.inputs.rotation_mode = false;
    r.tick();
    r.tick();
    for (int i = 0; i < 4; ++i) {
        check(std::abs(r.hardware.inverse_pose[i] - r.hardware.actual.pose[i]) < 1e-12,
              "切回位置模式后仍追赶旧虚拟目标");
    }
    r.hardware.force[0] = 30;
    r.advance(200);
    r.hardware.actual.pose[0] = 0.6;
    r.hardware.force.fill(0.0);
    r.inputs.small_sensor = true;
    r.tick();
    r.tick();
    check(std::abs(r.hardware.inverse_pose[0] - 0.6) < 1e-12, "传感器切换没有重新同步真机");
}

void testCancelEveryStartStep() {
    const StartStep steps[] = {StartStep::CheckStopped, StartStep::ClearErrors, StartStep::ZeroSensor,
        StartStep::SetServoReady, StartStep::PowerOn, StartStep::OpenTracking,
        StartStep::WaitServoEnabled, StartStep::Synchronize};
    for (StartStep step : steps) {
        Rig r;
        r.tick();
        r.inputs.force_on = true;
        r.tick();
        for (int i = 0; i < 200 && r.machine.startStep() != step; ++i) r.tick();
        check(r.machine.startStep() == step, "测试未到达指定启动步骤");
        r.inputs.force_on = false;
        r.tick();
        r.advance(150);
        check(r.machine.state() == ControlState::Disabled, "启动中关闭开关没有完成清理");
        check(r.hardware.sends == 0, "取消启动后发送了目标");
        if (r.hardware.count("上使能") > 0) check(r.hardware.count("下使能") == 1, "取消启动遗留使能");
        if (r.hardware.count("开启跟踪") > 0) check(r.hardware.count("关闭跟踪") == 1, "取消启动遗留跟踪");
    }
}

void testStartupFailures() {
    const char* commands[] = {"清除错误", "伺服准备", "上使能", "开启跟踪"};
    for (const char* command : commands) {
        Rig r;
        r.hardware.fail_command = command;
        r.tick();
        r.inputs.force_on = true;
        r.tick();
        r.advance(400);
        check(r.machine.state() == ControlState::Fault, "启动失败没有进入故障状态");
        check(!r.machine.cleanupFailed(), "可以正常清理的启动故障被错误锁死");
        check(r.hardware.sends == 0, "启动故障后发送了运动目标");
        const int starts = r.hardware.count("上使能");
        r.advance(400);
        check(r.hardware.count("上使能") == starts, "开关保持开启时发生自动重启");
        r.hardware.fail_command.clear();
        r.start();
    }
    Rig r;
    r.hardware.servo_after_on = 1;
    r.tick(); r.inputs.force_on = true; r.tick(); r.advance(1400);
    check(r.machine.state() == ControlState::Fault && r.hardware.sends == 0, "使能等待超时未生效");
}

void testZeroFailures() {
    for (int kind = 0; kind < 2; ++kind) {
        Rig r;
        r.hardware.zero_pending = kind == 0;
        r.hardware.zero_failed = kind == 1;
        r.tick(); r.inputs.force_on = true; r.tick(); r.advance(1200);
        check(r.machine.state() == ControlState::Fault, "清零失败或超时没有进入故障");
        check(r.hardware.count("上使能") == 0, "清零失败后仍然上使能");
    }
}

void testCycleFailures() {
    for (int kind = 0; kind < 7; ++kind) {
        Rig r;
        r.start();
        switch (kind) {
        case 0: r.hardware.read_ok = false; break;
        case 1: r.hardware.force[0] = std::numeric_limits<double>::quiet_NaN(); break;
        case 2: r.hardware.inverse_ok = false; break;
        case 3: r.hardware.out_of_limit = true; break;
        case 4: r.hardware.nan_target = true; break;
        case 5: r.hardware.fail_command = "发送目标"; break;
        case 6: r.hardware.servo = 2; break;
        }
        const int sent = r.hardware.sends;
        r.tick();
        check(r.machine.state() == ControlState::Stopping, "周期故障没有立即禁止运行");
        check(r.hardware.sends == sent + (kind == 5 ? 1 : 0), "错误数据被发送给控制器");
        r.hardware.read_ok = true;
        r.hardware.servo = 3;
        r.advance(150);
        check(r.machine.state() == ControlState::Fault, "周期故障清理后未保留故障状态");
        check(r.hardware.sends == sent + (kind == 5 ? 1 : 0), "故障停机中继续发送目标");
    }
}

void testStopFailureLockout() {
    const char* commands[] = {"关闭跟踪", "下使能"};
    for (const char* command : commands) {
        Rig r;
        r.start();
        r.hardware.fail_command = command;
        r.stop();
        check(r.machine.cleanupFailed(), "停机接口失败未锁定故障");
        const int starts = r.hardware.count("上使能");
        r.inputs.force_on = true; r.tick(); r.advance(200);
        check(r.hardware.count("上使能") == starts, "停机失败被开关重新开启绕过");
        if (std::string(command) == "关闭跟踪") check(r.hardware.count("下使能") == 0, "停轨迹失败却直接断使能");
    }
    const int invalid_statuses[] = {2, 3, -1};
    for (int status : invalid_statuses) {
        Rig r;
        r.start();
        r.hardware.servo_after_off = status;
        r.stop();
        r.advance(1100);
        check(r.machine.cleanupFailed(), "报警、运行或未知状态被误判为下使能成功");
    }
}

void testStableWindowAndTimeout() {
    Rig r;
    r.start();
    r.inputs.force_on = false; r.tick(); r.advance(60);
    r.hardware.actual.joints[2] += 0.1;
    r.advance(60);
    check(r.hardware.count("下使能") == 0, "位置变化后没有重新累计停稳时间");
    r.advance(100);
    check(r.machine.state() == ControlState::Disabled, "重新停稳后未完成停机");

    Rig moving;
    moving.start(); moving.hardware.run = 1;
    moving.stop(); moving.advance(3100);
    check(moving.machine.cleanupFailed() && moving.hardware.count("下使能") == 0,
          "停稳超时仍断使能或未锁定故障");
}

void testZeroWhileRunning() {
    Rig r;
    r.start();
    const int sent = r.hardware.sends;
    r.inputs.zero_on = true;
    r.tick();
    check(r.machine.state() == ControlState::Stopping, "运行中清零没有先停机");
    r.advance(300);
    check(r.machine.state() == ControlState::Disabled && r.hardware.sends == sent,
          "运行中清零后自动恢复了运动");
    check(r.hardware.count("开始清零") == 2, "停机后没有执行额外清零");
    r.advance(200);
    check(r.hardware.count("开始清零") == 2, "清零开关保持高电平导致重复清零");
}

void testExitAndRearm() {
    Rig r;
    r.start();
    r.tick(1, true);
    r.advance(150);
    check(r.machine.exitFinished() && r.machine.exitSucceeded(), "退出请求未完成停机");
    Rig starting;
    starting.tick(); starting.inputs.force_on = true; starting.tick();
    starting.advance(20); starting.tick(1, true);
    check(starting.machine.exitFinished() && starting.hardware.count("上使能") == 0,
          "清零中退出还启动了伺服");
    Rig failed;
    failed.start(); failed.hardware.fail_command = "关闭跟踪";
    failed.tick(1, true); failed.advance(10);
    check(failed.machine.exitFinished() && !failed.machine.exitSucceeded(), "停机失败却报告正常退出");

    Rig switching;
    switching.start(); switching.inputs.force_on = false; switching.tick();
    switching.inputs.force_on = true; switching.tick(); switching.advance(200);
    check(switching.machine.state() == ControlState::Disabled, "停机期间打开开关导致自动重启");
    switching.start();
}

void testCalibration() {
    SensorCalibration zero;
    const ControlTime start{};
    ForceSample large = {{1, 2, 3, 4, 5, 6}};
    ForceSample small = {{6, 5, 4, 3, 2, 1}};
    zero.begin(start);
    check(zero.sample(start, large, small) == ZeroResult::Pending, "未等待预热就完成清零");
    for (int i = 0; i < 50; ++i) {
        const auto now = start + std::chrono::milliseconds(10 + i * 2);
        const ZeroResult result = zero.sample(now, large, small);
        check(result == (i == 49 ? ZeroResult::Complete : ZeroResult::Pending), "清零采样计数错误");
        if (i < 49) {
            check(zero.offset(false)[0] == 0, "未采样完成就修改零点");
            check(zero.sample(now, large, small) == ZeroResult::Pending, "同一时刻重复采样");
        }
    }
    check(zero.offset(false) == large && zero.offset(true) == small, "大小传感器零点计算错误");
    zero.begin(start);
    large[0] = std::numeric_limits<double>::infinity();
    check(zero.sample(start + std::chrono::milliseconds(10), large, small) == ZeroResult::Failed,
          "无效数据没有终止清零");
    check(zero.offset(false)[0] == 1, "失败清零破坏了原零点");
    zero.begin(start); zero.cancel();
    check(!zero.active() && zero.offset(false)[0] == 1, "取消清零没有保留原零点");
}

void demonstration() {
    Rig r;
    r.hardware.print_logs = true;
    r.start();
    r.inputs.rotation_mode = true; r.tick(); r.advance(5);
    r.inputs.rotation_mode = false; r.tick(); r.advance(5);
    r.stop();
}
}

int main(int argc, char** argv) {
    if (argc > 1 && std::string(argv[1]) == "--demo") { demonstration(); return 0; }
    struct Test { const char* name; void (*run)(); };
    const Test tests[] = {
        {"正常启停与上电高电平保护", testStartAndStop},
        {"双向模式切换与传感器切换同步", testModeSynchronization},
        {"逐个启动步骤中途取消", testCancelEveryStartStep},
        {"启动接口失败与使能超时", testStartupFailures},
        {"清零失败与清零超时", testZeroFailures},
        {"运行时读取、传感器、逆解、限位和发送故障", testCycleFailures},
        {"停机失败锁定与下使能状态校验", testStopFailureLockout},
        {"连续停稳窗口与停稳超时", testStableWindowAndTimeout},
        {"运行中清零先停机且不自动恢复", testZeroWhileRunning},
        {"程序退出与停机期间开关变化", testExitAndRearm},
        {"传感器清零分步采样与零点保留", testCalibration}
    };
    int failed = 0;
    for (const Test& test : tests) {
        try { test.run(); std::cout << "[通过] " << test.name << '\n'; }
        catch (const std::exception& error) {
            ++failed;
            std::cerr << "[失败] " << test.name << "：" << error.what() << '\n';
        }
    }
    std::cout << "测试组数=" << sizeof(tests) / sizeof(tests[0]) << "，失败=" << failed << '\n';
    return failed == 0 ? 0 : 1;
}
