//
// Created by fyh on 25-8-4.
//

#ifndef SIMULATORCONFIGURATION_H
#define SIMULATORCONFIGURATION_H

#include <iostream>
#include <string>
#include <sstream>      // 用于字符串流
#include <iomanip>      // 用于 std::get_time
#include <chrono>       // 用于日期和时间处理
#include <locale>       // 用于处理中文字符

struct SimulatorConfiguration {
    TrainInfo test_vehicle;
    std::wstring route_file;
    std::wstring ip;
    int port;
    int SIMULATION_INTERVAL_MS;

    //指令相关
    long long simulation_start_time;
    long long simulation_duration;
    int trajectory_ID = 1;
    int trajectory_type = 1;
    int trajectory_ID_user2 = 2;
    int trajectory_type_user2 = 1;
};

/**
 * @brief 将指定格式的日期时间字符串转换为自2006年1月1日0时起的毫秒数
 * @param time_str 输入的时间字符串，格式为 "YYYY年M月D日H时m分S秒"
 * @return long long 自2006-01-01 00:00:00以来的毫秒数。如果解析失败则返回-1。
 */


#endif //SIMULATORCONFIGURATION_H
