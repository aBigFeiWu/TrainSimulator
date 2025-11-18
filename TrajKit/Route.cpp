#include "Route.h"
#include "GeoUtils.h"
#include <fstream>
#include <sstream>
#include <iostream>

bool Route::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return false;
    }

    m_distances.clear();
    m_ecef_points.clear();
    m_total_distance = 0.0;

    std::vector<GeodeticPoint> geo_points;
    std::string line;
    // 假设文件格式为: lon lat alt
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        GeodeticPoint p;
        if (!(iss >> p.lon >> p.lat >> p.alt)) {
            continue; // 跳过格式不正确的行
        }
        geo_points.push_back(p);
    }

    if (geo_points.size() < 4) { // not_a_knot 边界条件至少需要4个点
        std::cerr << "Error: Not enough data points to build spline (need at least 4 for not-a-knot)." << std::endl;
        return false;
    }

    // 转换到ECEF并计算走行距离
    ECEFPoint last_ecef_point{};
    for (size_t i = 0; i < geo_points.size(); ++i) {
        ECEFPoint current_ecef = GeoUtils::geodeticToEcef(geo_points[i]);
        if (i > 0) {
            m_total_distance += GeoUtils::calculateDistance(current_ecef, last_ecef_point);
        }
        m_ecef_points.push_back(current_ecef);
        m_distances.push_back(m_total_distance);
        last_ecef_point = current_ecef;
    }

    buildSplines();
    m_is_initialized = true;
    return true;
}

void Route::buildSplines() {
    std::vector<double> x, y, z;
    x.reserve(m_ecef_points.size());
    y.reserve(m_ecef_points.size());
    z.reserve(m_ecef_points.size());
    for (const auto& p : m_ecef_points) {
        x.push_back(p.x);
        y.push_back(p.y);
        z.push_back(p.z);
    }

    // !! 错误已修复：分离 set_boundary 和 set_points 调用 !!

    // 1. 先为每个样条设置边界条件
    // 使用 not_a_knot 边界条件通常对路径拟合更稳定
    tk::spline::bd_type b_type = tk::spline::not_a_knot;
    m_spline_x.set_boundary(b_type, 0.0, b_type, 0.0);
    m_spline_y.set_boundary(b_type, 0.0, b_type, 0.0);
    m_spline_z.set_boundary(b_type, 0.0, b_type, 0.0);

    // 2. 然后再设置数据点
    m_spline_x.set_points(m_distances, x);
    m_spline_y.set_points(m_distances, y);
    m_spline_z.set_points(m_distances, z);
}

double Route::getTotalDistance() const {
    return m_total_distance;
}

ECEFPoint Route::getPositionAt(double distance) const {
    if (!m_is_initialized) return {};
    return {m_spline_x(distance), m_spline_y(distance), m_spline_z(distance)};
}

ECEFPoint Route::getVelocityAt(double distance, double speed) const {
    if (!m_is_initialized) return {};
    // V = P'(s) * v(t)
    const double dx_ds = m_spline_x.deriv(1, distance);
    const double dy_ds = m_spline_y.deriv(1, distance);
    const double dz_ds = m_spline_z.deriv(1, distance);
    return {dx_ds * speed, dy_ds * speed, dz_ds * speed};
}

ECEFPoint Route::getAccelerationAt(double distance, double speed, double tangential_accel) const {
    if (!m_is_initialized) return {};
    // A = P'(s) * a_t(t) + P''(s) * v(t)^2
    const double dx_ds = m_spline_x.deriv(1, distance);
    const double dy_ds = m_spline_y.deriv(1, distance);
    const double dz_ds = m_spline_z.deriv(1, distance);

    const double d2x_ds2 = m_spline_x.deriv(2, distance);
    const double d2y_ds2 = m_spline_y.deriv(2, distance);
    const double d2z_ds2 = m_spline_z.deriv(2, distance);

    const double v2 = speed * speed;

    return {
        dx_ds * tangential_accel + d2x_ds2 * v2,
        dy_ds * tangential_accel + d2y_ds2 * v2,
        dz_ds * tangential_accel + d2z_ds2 * v2
    };
}

// 在 Route.cpp 文件末尾，添加以下函数实现

ECEFPoint Route::getJerkAt(double distance, double speed, double tangential_accel, double tangential_jerk) const {
    if (!m_is_initialized) return {};

    // P'(s): 样条的一阶导数 (切向)
    const double dx_ds = m_spline_x.deriv(1, distance);
    const double dy_ds = m_spline_y.deriv(1, distance);
    const double dz_ds = m_spline_z.deriv(1, distance);

    // P''(s): 样条的二阶导数 (曲率矢量)
    const double d2x_ds2 = m_spline_x.deriv(2, distance);
    const double d2y_ds2 = m_spline_y.deriv(2, distance);
    const double d2z_ds2 = m_spline_z.deriv(2, distance);

    // P'''(s): 样条的三阶导数
    const double d3x_ds3 = m_spline_x.deriv(3, distance);
    const double d3y_ds3 = m_spline_y.deriv(3, distance);
    const double d3z_ds3 = m_spline_z.deriv(3, distance);

    const double v = speed;
    const double a = tangential_accel;
    const double j = tangential_jerk;
    const double v3 = v * v * v;

    // J_x = X'''(s)v³ + 3X''(s)va + X'(s)j
    return {
        d3x_ds3 * v3 + 3 * d2x_ds2 * v * a + dx_ds * j,
        d3y_ds3 * v3 + 3 * d2y_ds2 * v * a + dy_ds * j,
        d3z_ds3 * v3 + 3 * d2z_ds2 * v * a + dz_ds * j
    };
}

// 新增的函数实现
bool Route::isInitialized() const {
    return m_is_initialized;
}
