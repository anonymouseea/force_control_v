#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <iostream>
#include <vector>
#include <cmath>
#include <thread> 
#include <chrono>
#include <csignal>
#include <atomic>
#include <sched.h>          
#include <sys/mman.h>       
#include <time.h>           
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <fstream>
#include <ctime>
#include <sstream>
#include <iomanip>
#include "Admittance.h" 
#include "nrcAPI.h"
#include "AsyncLogger.h"
#include "RobotUtils.h"
#include "ControlLoop.h"

#include <sys/stat.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

std::atomic<bool> g_running{true};
void handle_sigint(int) { g_running = false; }

AsyncLogger logger;

// 统一初始化入口
bool InitializeSystem() {
    logger.log("[初始化] 系统启动\n");
    // 智能启动
    if (NRC_GetControlInitComplete() != 1) {
        logger.log("[初始化] 检测到系统未初始化，执行系统启动流程\n");
        SystemStartup();
    } else {
        logger.log("[初始化] 检测到系统已在运行，跳过启动步骤\n");
    }
    //系统清错
    NRC_ClearAllError();

    //默认使用大量程传感器
    NRC_SetBoolVar(5, 0);

    //限位报警复位
    NRC_SetBoolVar(4, 0);

    // 日志，注释
    const std::string log_file = MakeLogFileName();
    ::mkdir("szl_log", 0777);
    logger.setLogFile(log_file);
    logger.log(std::string("[日志] 文件=") + log_file);

    signal(SIGINT, handle_sigint);

    if (!init_force_sensor_mapping()) {logger.log("[初始化] 初始化传感器地址失败\n");return false;}
    return true;
}


int main() 
{
    
    if (!InitializeSystem()) {logger.log("[初始化] 初始化系统失败，退出\n");return -1;}
    //增加了实时调度设置，确保控制循环的高精度执行
    if (!setup_realtime()) {logger.log("[错误] 设置实时调度失败，程序退出\n");return -1;}
    RunControlLoop(logger, g_running);
    return 0;
}
