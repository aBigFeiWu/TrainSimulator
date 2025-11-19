//
// Created by fyh on 2025/7/23.
//
#ifndef PROTOCOL_H
#define PROTOCOL_H
#pragma once

#include <cstdint>

/**
 * 网络注入协议（V2.4）结构定义，严格 1 字节对齐。
 */
#pragma pack(push, 1)

// 16B 网络帧头
struct NetworkFrameHeader {
    uint32_t frame_flag;    // 固定 0xA5A56666
    uint32_t frame_number;  // 协议要求固定 0
    uint32_t frame_length;  // payload 实际长度
    uint32_t reserved;      // 填 0
};

// 单个用户 200B 启动参数块
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

// 启动指令：32B 头 + N * 200B 用户块
struct StartCommand {
    uint64_t command_word;           // 0x000000000ABC0001（单）或 0x000000000ABC0002（双）
    uint32_t reserved_after_cmd;     // 4B 保留
    uint32_t data_length;            // 232（单用户）或 432（双用户）
    uint64_t simulation_start_time;  // 2006-01-01 至今的毫秒数
    uint64_t simulation_duration;    // 仿真持续时间
    StartUserParams users[2];
};

static_assert(sizeof(StartUserParams) == 200, "StartUserParams should remain 200 bytes");
static_assert(sizeof(StartCommand) == 32 + 2 * sizeof(StartUserParams), "StartCommand size should account for two users");

// 停止指令：固定 232B
struct StopCommand {
    uint64_t command_word;       // 0x000000000ABC0003
    uint32_t reserved_after_cmd; // 4B 保留
    uint32_t data_length;        // 固定 232
    double reserved_doubles[27]; // 216B 保留区
};
static_assert(sizeof(StopCommand) == 232, "StopCommand size must be 232 bytes per protocol");

// 轨迹数据：固定 248B
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

struct DualTrajectoryData {
    TrajectoryData user1;
    TrajectoryData user2;
};
static_assert(sizeof(DualTrajectoryData) == 2 * sizeof(TrajectoryData), "DualTrajectoryData should be a simple concatenation of two user payloads");

#pragma pack(pop)
#endif //PROTOCOL_H
