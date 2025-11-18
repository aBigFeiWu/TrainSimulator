#include "MotionPlanner.h"
#include <algorithm>
#include <cmath>

MotionPlanner::MotionPlanner(const MotionConstraints& constraints)
    : constraints_(constraints) {}

void MotionPlanner::setTargetVelocity(double target_vel) {
    target_velocity_ = std::clamp(target_vel, 0.0, constraints_.max_velocity);
}

void MotionPlanner::setTargetAcceleration(double target_accel) {
    target_acceleration_ = std::clamp(target_accel, constraints_.min_acceleration, constraints_.max_acceleration);
}

void MotionPlanner::update_for_velocity(double dt, KinematicState& current_state) {
    // 1. 计算速度误差
    double velocity_error = target_velocity_ - current_state.velocity;

    // 2. 估算减速到0所需的速度变化
    double time_to_stop_accel = current_state.acceleration / constraints_.max_jerk[2];
    double vel_change_to_stop_accel = 0.5 * current_state.acceleration * time_to_stop_accel;

    // 3. 决定理想加速度
    double target_accel = 0.0;
    if (std::abs(velocity_error) > std::abs(vel_change_to_stop_accel)) {
        target_accel = (velocity_error > 0) ? constraints_.max_acceleration : constraints_.min_acceleration;
    } else {
        target_accel = 0.0;
    }

    // 4. 计算加速度误差，并根据Jerk平滑过渡
    double accel_error = target_accel - current_state.acceleration;
    if (std::abs(accel_error) < constraints_.max_jerk[2] * dt) {
        current_state.acceleration = target_accel;
        current_state.jerk = 0;
    } else {
        double sign_a_err = (accel_error > 0) ? 1.0 : -1.0;
        current_state.jerk = sign_a_err * constraints_.max_jerk[2];
    }

    // 5. 积分更新状态
    current_state.position += current_state.velocity * dt + 0.5 * current_state.acceleration * dt * dt + (1.0/6.0) * current_state.jerk * dt * dt * dt;
    current_state.velocity += current_state.acceleration * dt + 0.5 * current_state.jerk * dt * dt;
    current_state.acceleration += current_state.jerk * dt;

    // 6. 施加约束
    current_state.acceleration = std::clamp(current_state.acceleration, constraints_.min_acceleration, constraints_.max_acceleration);
    current_state.velocity = std::clamp(current_state.velocity, 0.0, constraints_.max_velocity);
}

void MotionPlanner::update_for_acceleration(double dt, KinematicState& current_state) {
    // 1. 计算加速度误差
    double accel_error = target_acceleration_ - current_state.acceleration;

    // 2. 根据Jerk限制决定当前的jerk值
    if (std::abs(accel_error) < constraints_.max_jerk[2] * dt) {
        current_state.acceleration = target_acceleration_;
        current_state.jerk = 0;
    } else {
        double sign_a_err = (accel_error > 0) ? 1.0 : -1.0;
        current_state.jerk = sign_a_err * constraints_.max_jerk[2];
    }

    // 3. 积分更新状态
    current_state.position += current_state.velocity * dt + 0.5 * current_state.acceleration * dt * dt + (1.0/6.0) * current_state.jerk * dt * dt * dt;
    current_state.velocity += current_state.acceleration * dt + 0.5 * current_state.jerk * dt * dt;
    current_state.acceleration += current_state.jerk * dt;

    // 4. 施加约束
    current_state.acceleration = std::clamp(current_state.acceleration, constraints_.min_acceleration, constraints_.max_acceleration);
    current_state.velocity = std::clamp(current_state.velocity, 0.0, constraints_.max_velocity);

    if (current_state.velocity >= constraints_.max_velocity && current_state.acceleration > 0) {
        current_state.acceleration = 0;
        current_state.jerk = 0; // 同时也将Jerk清零
    }
}