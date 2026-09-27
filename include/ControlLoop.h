#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include <csignal>

#include "AsyncLogger.h"

// 收到退出请求后继续推进停机状态机，返回值表示是否正常完成退出。
bool RunControlLoop(AsyncLogger& logger, const volatile std::sig_atomic_t& exit_requested);

#endif
