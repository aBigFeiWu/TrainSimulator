//
// Created by fyh on 25-7-30.
//

#ifndef CONSTANTS_H
#define CONSTANTS_H
#pragma once

namespace Constants {
    // WGS-84 椭球参数
    constexpr double WGS84_A = 6378137.0; // 椭球长半轴 (米)
    constexpr double WGS84_F = 1.0 / 298.257223563; // 扁率
    constexpr double WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F; // 第一偏心率的平方
}
#endif //CONSTANTS_H
