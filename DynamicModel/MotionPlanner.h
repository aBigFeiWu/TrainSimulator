#pragma once
#include "KinematicState.h"
#include "MotionConstraints.h"

class MotionPlanner {
public:
    MotionPlanner(const MotionConstraints& constraints);

    // --- 接口 ---
    void setTargetVelocity(double target_vel);     // 给自动模式使用
    void setTargetAcceleration(double target_accel); // 给手动模式使用

    // --- 更新函数 ---
    // 根据目标速度进行更新 (原始逻辑)
    void update_for_velocity(double dt, KinematicState& current_state);
    // 根据目标加速度进行更新 (新逻辑)
    void update_for_acceleration(double dt, KinematicState& current_state);

private:
    MotionConstraints constraints_;
    double target_velocity_ = 0.0;
    double target_acceleration_ = 0.0;
};