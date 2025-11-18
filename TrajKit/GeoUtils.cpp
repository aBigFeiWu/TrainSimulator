#include "GeoUtils.h"
#include "Constants.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace GeoUtils {

    ECEFPoint geodeticToEcef(const GeodeticPoint& geo) {
        const double lon_rad = geo.lon * M_PI / 180.0;
        const double lat_rad = geo.lat * M_PI / 180.0;

        const double N = Constants::WGS84_A / std::sqrt(1.0 - Constants::WGS84_E2 * std::sin(lat_rad) * std::sin(lat_rad));

        ECEFPoint ecef;
        ecef.x = (N + geo.alt) * std::cos(lat_rad) * std::cos(lon_rad);
        ecef.y = (N + geo.alt) * std::cos(lat_rad) * std::sin(lon_rad);
        ecef.z = (N * (1.0 - Constants::WGS84_E2) + geo.alt) * std::sin(lat_rad);

        return ecef;
    }

    GeodeticPoint ecefToGeodetic(const ECEFPoint& ecef) {
        GeodeticPoint geo = {0.0, 0.0, 0.0};

        // Check for singularity at the poles
        if (ecef.x == 0.0 && ecef.y == 0.0) {
            return geo;
        }

        // --- Longitude Calculation ---
        // Directly computed from X and Y
        geo.lon = std::atan2(ecef.y, ecef.x);

        // --- Latitude Calculation (Iterative Method) ---
        double B = 0.0;
        double B0 = 0.0;
        double N = 0.0;
        const double e2 = Constants::WGS84_E2;
        const double p = std::sqrt(ecef.x * ecef.x + ecef.y * ecef.y);

        // Iterate until the change in latitude is negligible
        do {
            B0 = B;
            N = Constants::WGS84_A / std::sqrt(1.0 - e2 * std::sin(B) * std::sin(B));
            B = std::atan2((ecef.z + N * e2 * std::sin(B)), p);
        } while (std::abs(B0 - B) >= 1e-10);

        geo.lat = B;

        // --- Altitude Calculation ---
        // Recalculate N with the final latitude
        N = Constants::WGS84_A / std::sqrt(1.0 - e2 * std::sin(geo.lat) * std::sin(geo.lat));
        geo.alt = p / std::cos(geo.lat) - N;

        // --- Convert radians to degrees for final output ---
        geo.lon = geo.lon * 180.0 / M_PI;
        geo.lat = geo.lat * 180.0 / M_PI;

        return geo;
    }

    double calculateDistance(const ECEFPoint& p1, const ECEFPoint& p2) {
        const double dx = p1.x - p2.x;
        const double dy = p1.y - p2.y;
        const double dz = p1.z - p2.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

} // namespace GeoUtils