#pragma once
#include "MotionPlanner.h"
#include "TestVehicle.h"
#include <iostream>
#include <fstream>

// 自动驾驶状态
enum class TrainState {
    STOPPED,
    ACCELERATING,
    CRUISING,
    COASTING,
    BRAKING
};

// 新增：定义控制模式


class TrainController {
public:
    enum class ControlMode {
        AUTOMATIC,
        MANUAL
    };

    // 新增：手动控制的级别
    enum class ControlLevel {
        IDLE,
        CRUISE,
        TRACTION_1,
        TRACTION_2,
        TRACTION_3,
        BRAKE_1,
        BRAKE_2,
        BRAKE_3
    };

    TrainController(const TrainInfo& train_info, double track_length);
    ~TrainController();

    // --- 模式切换和控制接口 ---
    void setControlMode(ControlMode mode);
    void setControlLevel(ControlLevel level);

    // --- 核心更新函数 ---
    void update(double dt);

    // --- 数据获取接口 ---
    const KinematicState& getCurrentState() const;
    void printCurrentState() const;

private:
    void print_state() const;
    void update_state_machine(); // 自动驾驶的状态机
    static MotionConstraints create_constraints_from_info(const TrainInfo& train_info);

    // --- 状态变量 ---
    ControlMode m_control_mode = ControlMode::AUTOMATIC; // 默认启动为自动模式
    ControlLevel m_current_level = ControlLevel::IDLE;   // 手动模式的当前档位
    TrainState train_state_ = TrainState::STOPPED;       // 自动模式的当前状态

    KinematicState state_;
    MotionConstraints constraints_;
    MotionPlanner planner_;

    double track_length_;
    double station_position_;

    const double COASTING_BUFFER_DISTANCE = 500.0;
    mutable std::ofstream output_file_;
};