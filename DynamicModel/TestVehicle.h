//
// Created by fyh on 25-8-3.
//

#ifndef TESTVEHICLE_H
#define TESTVEHICLE_H

#include <string>

struct TrainInfo {
    std::wstring trainType;                 // 列车类型
    double maxSpeed;                    // 最大速度(km/h)
    double trainLong;                     // 列车长度(m)
    double resistanceCoefficients[3];         // 基本阻力公式
    double tractionAcceleration;        // 最大加速度(m/s²)
    double brakingAcceleration;         // 最大减速度(m/s²)
};

#endif //TESTVEHICLE_H