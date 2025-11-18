//
// Created by fyh on 2025/7/22.
//

#ifndef MOTIONCONSTRAINTS_H
#define MOTIONCONSTRAINTS_H
#pragma once

// 描述列车能达到的物理极限
struct MotionConstraints {
    double max_velocity = 25.0;         // 最大速度 (m/s)
    double max_acceleration = 1.0;      // 最大加速度 (m/s^2)
    double min_acceleration = -1.0;     // 最大减速度 (m/s^2)

    // 最大加加速度 (舒适性约束), 分为3个档位
    // [0] -> 低档, [1] -> 中档, [2] -> 高档
    double max_jerk[3] = {0.25, 0.5, 1.0};
};
#endif //MOTIONCONSTRAINTS_H