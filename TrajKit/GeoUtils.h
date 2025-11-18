//
// Created by fyh on 25-7-30.
//

#ifndef GEOUTILS_H
#define GEOUTILS_H
#pragma once
#include "DataTypes.h"

namespace GeoUtils {

    // 大地坐标 -> ECEF坐标
    ECEFPoint geodeticToEcef(const GeodeticPoint& geo);

    // ECEF坐标 -> 大地坐标
    GeodeticPoint ecefToGeodetic(const ECEFPoint& ecef);

    // 计算两个ECEF点之间的欧几里得距离
    double calculateDistance(const ECEFPoint& p1, const ECEFPoint& p2);

} // namespace GeoUtils
#endif //GEOUTILS_H
