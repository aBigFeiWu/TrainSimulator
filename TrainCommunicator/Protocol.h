//
// Created by fyh on 2025/7/23.
//

#ifndef PROTOCOL_H
#define PROTOCOL_H
#pragma once // 使用pragma once防止头文件重复包含

#include <cstdint> // 为了使用如 uint32_t 等标准整数类型

/**
 * @file Protocol.h
 * @brief 定义了通信协议中使用的数据结构。
 * * 使用 #pragma pack(push, 1) 来确保结构体成员之间没有内存对齐填充，
 * 这对于跨平台或不同编译器间的二进制数据交换至关重要。
 */

#pragma pack(push, 1)

// --- 网络通信的通用帧头 ---
struct NetworkFrameHeader {
    uint32_t frame_flag;
    uint32_t frame_number;
    uint32_t frame_length;
    uint32_t reserved;
};

// --- 每个用户的启动参数块（200 字节） ---
struct StartUserParams {
    unsigned int trajectory_id;
    unsigned int trajectory_type;
    double initial_user_pos_x;
    double initial_user_pos_y;
    double initial_user_pos_z;
    double reserved_after_pos[9];
    double initial_carrier_roll;
    double initial_carrier_azimuth;
    double initial_carrier_pitch;
    double reserved_after_pitch[9];
};

// --- 开始指令 ---
struct StartCommand {
    unsigned long long command_word;
    unsigned int reserved_after_cmd;
    unsigned int data_length;
    unsigned long long simulation_start_time;
    unsigned long long simulation_duration;
    StartUserParams users[2];
};

static_assert(sizeof(StartUserParams) == 200, "StartUserParams should remain 200 bytes");
static_assert(sizeof(StartCommand) == 32 + 2 * sizeof(StartUserParams), "StartCommand size should account for two users");

// --- 停止指令 ---
struct StopCommand {
    unsigned long long command_word;
    unsigned int data_length;
    unsigned int reserved_int;
    double reserved_doubles[27];
};

// --- 轨迹数据 ---
struct TrajectoryData {
    unsigned int trajectory_type;
    unsigned int trajectory_id;
    unsigned long long trajectory_data_seq_num;
    unsigned long long reserved_page4_ll;
    double trajectory_time;
    double user_pos_x;
    double user_pos_y;
    double user_pos_z;
    double user_vel_x;
    double user_vel_y;
    double user_vel_z;
    double user_acc_x;
    double user_acc_y;
    double user_acc_z;
    double user_jerk_x;
    double user_jerk_y;
    double user_jerk_z;
    unsigned long long reserved_page5_ll[3];
    double carrier_roll;
    double carrier_azimuth;
    double carrier_pitch;
    double carrier_angular_vel_x;
    double carrier_angular_vel_y;
    double carrier_angular_vel_z;
    double carrier_angular_acc_x;
    double carrier_angular_acc_y;
    double carrier_angular_acc_z;
    double carrier_angular_jerk_x;
    double carrier_angular_jerk_y;
    double carrier_angular_jerk_z;
};

static_assert(sizeof(TrajectoryData) == 248, "TrajectoryData size expected by UDP consumer");

// --- 双用户轨迹数据包 ---
struct DualTrajectoryData {
    TrajectoryData user1;
    TrajectoryData user2;
};
static_assert(sizeof(DualTrajectoryData) == 2 * sizeof(TrajectoryData), "DualTrajectoryData should be a simple concatenation of two user payloads");

#pragma pack(pop)
#endif //PROTOCOL_H
