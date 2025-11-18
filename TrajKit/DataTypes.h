//
// Created by fyh on 25-7-30.
//

#ifndef DATATYPES_H
#define DATATYPES_H
#pragma once

// 用于表示大地坐标（经度, 纬度, 高程）
struct GeodeticPoint {
    double lon = 0.0; // 经度 (degrees)
    double lat = 0.0; // 纬度 (degrees)
    double alt = 0.0; // 高程 (meters)
};

// 用于表示地心固连 (ECEF) 坐标
struct ECEFPoint {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};
#endif //DATATYPES_H
