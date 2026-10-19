/**
 * @file robot_compat.h
 * @brief Stand-ins for the firmware headers used by the robot profiles.
 *
 * The robot profiles in this directory are written to run unchanged in the
 * mazerunner32 firmware. There they include interfaces/isr.h and utils/core.h;
 * in the app they include this file instead. Only that include line differs.
 *
 * The firmware radians() and degrees() macros are not defined here because
 * they would clash with the app's functions of the same names (angles.h).
 * The robot profiles use RADIANS_PER_DEGREE and DEGREES_PER_RADIAN directly.
 */

#pragma once

/// Marks functions called from the control loop interrupt (firmware interfaces/isr.h)
#ifndef ISR_SAFE
#define ISR_SAFE
#endif

/// Same values as the firmware utils/core.h
constexpr float RADIANS_PER_DEGREE = 0.01745329f;
constexpr float DEGREES_PER_RADIAN = 57.2957795f;
constexpr float HALF_PI = 1.57079633f;
