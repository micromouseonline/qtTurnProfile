/**
 * @file angles.h
 * @brief Pi and degree/radian conversion, shared by the whole project.
 */

#ifndef ANGLES_H
#define ANGLES_H

/// Pi. Kept at the value the project has always used so results are unchanged.
constexpr double PI = 3.14159265359;

/**
 * @brief Convert degrees to radians.
 * @param deg angle in degrees
 * @return angle in radians (double)
 */
constexpr double radians(double deg) {
    return deg * PI / 180.0;
}

/**
 * @brief Convert radians to degrees.
 * @param rad angle in radians
 * @return angle in degrees (double)
 */
constexpr double degrees(double rad) {
    return rad * 180.0 / PI;
}

#endif  // ANGLES_H
