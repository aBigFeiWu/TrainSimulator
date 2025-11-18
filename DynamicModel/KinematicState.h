//
// Created by fyh on 2025/7/22.
//

#ifndef KINEMATICSTATE_H
#define KINEMATICSTATE_H
#pragma once

// 描述列车某一时刻的运动状态
struct KinematicState {
    double position = 0.0;     // 位置 (m)
    double velocity = 0.0;     // 速度 (m/s)
    double acceleration = 0.0; // 加速度 (m/s^2)
    double jerk = 0.0;         // 加加速度 (m/s^3)
};
#endif //KINEMATICSTATE_H
