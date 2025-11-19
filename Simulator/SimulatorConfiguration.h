//
// Created by fyh on 25-8-4.
//

#ifndef SIMULATORCONFIGURATION_H
#define SIMULATORCONFIGURATION_H
#pragma once

#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <locale>
#include "DynamicModel/TestVehicle.h"

struct SimulatorConfiguration {
    TrainInfo test_vehicle;
    std::wstring route_file;
    std::wstring ip;
    int port;
    int SIMULATION_INTERVAL_MS;

    // 指令相关
    long long simulation_start_time = 0;
    long long simulation_duration = 0;
    int trajectory_ID = 1;
    int trajectory_type = 1;
    int trajectory_ID_user2 = 2;
    int trajectory_type_user2 = 1;
    bool enable_second_user = false; // 0: 单用户；1: 双用户
};

/**
 * @brief 将指定格式的日期时间字符串转换为 2006 年 1 月 1 日 00:00 起的毫秒数
 * @param time_str 输入的时间字符串，格式为 "YYYY年M月D日H时m分S秒"
 * @return 2006-01-01 00:00:00 以来的毫秒数；解析失败返回 -1
 */

#endif //SIMULATORCONFIGURATION_H
