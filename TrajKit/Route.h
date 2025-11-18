//
// Created by fyh on 25-7-30.
//

#ifndef ROUTE_H
#define ROUTE_H
#pragma once
#include <string>
#include <vector>
#include "DataTypes.h"
#include "spline.h" // 假设 spline.h 在包含路径中

class Route {
public:
    Route() = default;

    // 从文件加载轨迹点并构建样条曲线，返回是否成功
    bool loadFromFile(const std::string& filename);

    // 获取总里程
    double getTotalDistance() const;

    // 根据走行距离 s 获取路径上的 ECEF 坐标
    ECEFPoint getPositionAt(double distance) const;

    // 根据走行距离 s 和当前速率 v(t)，获取三维速度矢量
    ECEFPoint getVelocityAt(double distance, double speed) const;

    // 根据走行距离 s, 速率 v(t), 切向加速度 a_t(t)，获取三维加速度矢量
    ECEFPoint getAccelerationAt(double distance, double speed, double tangential_accel) const;

    // 新增的 getJerkAt 函数
    ECEFPoint getJerkAt(double distance, double speed, double tangential_accel, double tangential_jerk) const;

    // 新增：检查路由是否已初始化
    bool isInitialized() const;

private:
    // 构建样条曲线的私有辅助函数
    void buildSplines();

    // 私有成员变量，封装内部状态
    std::vector<double> m_distances; // 每个点的走行距离 s
    std::vector<ECEFPoint> m_ecef_points; // 每个点的ECEF坐标

    tk::spline m_spline_x;
    tk::spline m_spline_y;
    tk::spline m_spline_z;

    double m_total_distance = 0.0;
    bool m_is_initialized = false;
};
#endif //ROUTE_H
