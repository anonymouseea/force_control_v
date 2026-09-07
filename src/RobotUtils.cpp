#include "RobotUtils.h"

#include <cmath>
#include <chrono>
#include <thread>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <cstdio>
#include <sched.h>
#include <sys/mman.h>
#include <dirent.h>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static unsigned char* g_force_ptrsda[6] = {nullptr};
static unsigned char* g_force_ptrsxiao[6] = {nullptr};
static bool g_is_sensor_ready = false;
static SensorData g_sensor_offset = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static SensorData g_sensor_offset_xiao = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

bool setup_realtime() {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall failed");
        return false;
    }

    struct sched_param param;
    param.sched_priority = 80;
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed (Try sudo)");
        return false;
    }
    return true;
}

void SystemStartup() {
    std::cout << "库版本：" << NRC_GetNexMotionLibVersion() << std::endl;
    //启动控制系统
    NRC_StartController();
    //检测控制系统是否初始化完成
    while (NRC_GetControlInitComplete() != 1 ) {
        NRC_Delayms(100);   //延时100ms
    }
    //清除所有错误
    NRC_ClearAllError();
    std::cout << "----" << NRC_GetControlInitComplete() << std::endl;
    std::cout << "StartController Success" << std::endl;
    std::cout << "获取同步版本号" << NRC_GetSyncVersion() << std::endl;
    NRC_Delayms(200);
}

bool init_force_sensor_mapping() {
    unsigned short index = 0x6030;
    bool allSuccess = true;
    for(int i=0; i<6; ++i) {
        g_force_ptrsda[i] = NRC_GetPDOAddrMap(6, index, i + 1); // 大量程 Slave 6
        g_force_ptrsxiao[i] = NRC_GetPDOAddrMap(7, index, i + 1); // 小量程 Slave 7
        if(!g_force_ptrsda[i] || !g_force_ptrsxiao[i]) allSuccess = false;
    }
    if (allSuccess) g_is_sensor_ready = true;
    return allSuccess;
}

void zero_force_sensor(AsyncLogger& logger) {
    logger.log("开始传感器清零 (采样中，请勿触摸机械臂)...\n");

    for (int i = 0; i < 10; ++i) {
        if (g_is_sensor_ready) {
            float dummy = *reinterpret_cast<float*>(g_force_ptrsda[0]);
            (void)dummy;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    const int sample_count = 50;
    SensorData sum_da = {0, 0, 0, 0, 0, 0};
    SensorData sum_xiao = {0, 0, 0, 0, 0, 0};

    for (int i = 0; i < sample_count; ++i) {
        if (g_is_sensor_ready) {
            // 大量程传感器累加
            sum_da.fx += (double)*reinterpret_cast<float*>(g_force_ptrsda[0]);
            sum_da.fy += (double)*reinterpret_cast<float*>(g_force_ptrsda[1]);
            sum_da.fz += (double)*reinterpret_cast<float*>(g_force_ptrsda[2]);
            sum_da.mx += (double)*reinterpret_cast<float*>(g_force_ptrsda[3]);
            sum_da.my += (double)*reinterpret_cast<float*>(g_force_ptrsda[4]);
            sum_da.mz += (double)*reinterpret_cast<float*>(g_force_ptrsda[5]);

            // 小量程传感器累加
            sum_xiao.fx += (double)*reinterpret_cast<float*>(g_force_ptrsxiao[0]);
            sum_xiao.fy += (double)*reinterpret_cast<float*>(g_force_ptrsxiao[1]);
            sum_xiao.fz += (double)*reinterpret_cast<float*>(g_force_ptrsxiao[2]);
            sum_xiao.mx += (double)*reinterpret_cast<float*>(g_force_ptrsxiao[3]);
            sum_xiao.my += (double)*reinterpret_cast<float*>(g_force_ptrsxiao[4]);
            sum_xiao.mz += (double)*reinterpret_cast<float*>(g_force_ptrsxiao[5]);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2)); // 每 2ms 采一个点
    }

    // --- 大量程偏移量计算
    g_sensor_offset.fx = sum_da.fx / sample_count;
    g_sensor_offset.fy = sum_da.fy / sample_count;
    g_sensor_offset.fz = sum_da.fz / sample_count;
    g_sensor_offset.mx = sum_da.mx / sample_count;
    g_sensor_offset.my = sum_da.my / sample_count;
    g_sensor_offset.mz = sum_da.mz / sample_count;

    // --- 小量程偏移量计算
    g_sensor_offset_xiao.fx = sum_xiao.fx / sample_count;
    g_sensor_offset_xiao.fy = sum_xiao.fy / sample_count;
    g_sensor_offset_xiao.fz = sum_xiao.fz / sample_count;
    g_sensor_offset_xiao.mx = sum_xiao.mx / sample_count;
    g_sensor_offset_xiao.my = sum_xiao.my / sample_count;
    g_sensor_offset_xiao.mz = sum_xiao.mz / sample_count;

    logger.log("传感器清零完成。");
}

SensorData read_force_sensor_da_raw() {
    if (!g_is_sensor_ready) return {0,0,0,0,0,0};
    return {(double)*reinterpret_cast<float*>(g_force_ptrsda[0]), (double)*reinterpret_cast<float*>(g_force_ptrsda[1]), (double)*reinterpret_cast<float*>(g_force_ptrsda[2]),
            (double)*reinterpret_cast<float*>(g_force_ptrsda[3]), (double)*reinterpret_cast<float*>(g_force_ptrsda[4]), (double)*reinterpret_cast<float*>(g_force_ptrsda[5])};
}

SensorData read_force_sensor_xiao_raw() {
    if (!g_is_sensor_ready) return {0,0,0,0,0,0};
    return {(double)*reinterpret_cast<float*>(g_force_ptrsxiao[0]), (double)*reinterpret_cast<float*>(g_force_ptrsxiao[1]), (double)*reinterpret_cast<float*>(g_force_ptrsxiao[2]),
            (double)*reinterpret_cast<float*>(g_force_ptrsxiao[3]), (double)*reinterpret_cast<float*>(g_force_ptrsxiao[4]), (double)*reinterpret_cast<float*>(g_force_ptrsxiao[5])};
}

SensorData read_force_sensor_da() {
    SensorData r = read_force_sensor_da_raw();
    return {r.fx - g_sensor_offset.fx, r.fy - g_sensor_offset.fy, r.fz - g_sensor_offset.fz, r.mx - g_sensor_offset.mx, r.my - g_sensor_offset.my, r.mz - g_sensor_offset.mz};
}

SensorData read_force_sensor_xiao() {
    SensorData r = read_force_sensor_xiao_raw();
    // 小量程传感器 X,Y 轴方向与实际（或大量程）相反，做符号翻转
    return {-(r.fx - g_sensor_offset_xiao.fx),
            r.fy - g_sensor_offset_xiao.fy,
            r.fz - g_sensor_offset_xiao.fz,
            r.mx - g_sensor_offset_xiao.mx,
            r.my - g_sensor_offset_xiao.my,
            r.mz - g_sensor_offset_xiao.mz};
}

//mcs世界坐标系 acs关节坐标系
bool read_robot_full_state(MyRobotState& state) {
    NRC_Position mcs_position;
    NRC_Position acs_position;

    if (NRC_GetCurrentPos(NRC_COORD::NRC_MCS, mcs_position) != 0) {
        return false;
    }

    if (NRC_GetCurrentPos(NRC_COORD::NRC_ACS, acs_position) != 0) {
        return false;
    }

    MyRobotState new_state = {0};
    new_state.x = mcs_position.pos[0] / 1000.0;
    new_state.y = mcs_position.pos[1] / 1000.0;
    new_state.z = mcs_position.pos[2] / 1000.0;
    new_state.rz = mcs_position.pos[5];
    new_state.theta2 = acs_position.pos[0] * (M_PI / 180.0);
    new_state.theta4 = acs_position.pos[3] * (M_PI / 180.0);

    // 两种坐标均读取成功后才更新输出，避免返回部分有效的状态。
    state = new_state;
    return true;
}

bool perform_ik(NRC_Position& ref_acs, double x_m, double y_m, double z_m, double rz_rad, NRC_Position& res) {
    NRC_Position posMCS(NRC_COORD::NRC_MCS, x_m*1000.0, y_m*1000.0, z_m*1000.0, 3.14159, 0, rz_rad);
    return (NRC_MCStoACS(ref_acs, posMCS, res) == 0);
}




std::string MakeLogFileName() {
    int max_id = 0;
    DIR *dir;
    struct dirent *ent;
    
    // 遍历目录查找当前最大的序号
    if ((dir = opendir("szl_log")) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            std::string fname = ent->d_name;
            std::string prefix = "run_";
            std::string suffix = ".log";
            
            // 检查文件名是否以 run_ 开头，以 .log 结尾
            if (fname.length() > prefix.length() + suffix.length() &&
                fname.compare(0, prefix.length(), prefix) == 0 &&
                fname.compare(fname.length() - suffix.length(), suffix.length(), suffix) == 0) {
                
                std::string num_part = fname.substr(prefix.length(), fname.length() - prefix.length() - suffix.length());
                
                // 关键：如果包含 '_'，说明是旧的时间戳格式(run_YYYY_MM...)，忽略它
                if (num_part.find('_') != std::string::npos) {
                    continue;
                }
                
                // 尝试解析纯数字序号
                try {
                    int id = std::stoi(num_part);
                    if (id > max_id) max_id = id;
                } catch (...) {}
            }
        }
        closedir(dir);
    }
    
    std::ostringstream oss;
    oss << "szl_log/run_" << (max_id + 1) << ".log";
    return oss.str();
}
