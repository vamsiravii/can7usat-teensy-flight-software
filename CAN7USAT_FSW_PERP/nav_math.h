#pragma once

#include "nav_types.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define EARTH_RADIUS_M 6371000.0

// ----------------------------------------------------------------------------
// Angle Wrapping Utilities
// ----------------------------------------------------------------------------
inline float deg2rad(float deg) {
    return deg * (M_PI / 180.0f);
}

inline float rad2deg(float rad) {
    return rad * (180.0f / M_PI);
}

inline float wrap_360(float deg) {
    float res = fmodf(deg, 360.0f);
    if (res < 0.0f) res += 360.0f;
    return res;
}

inline float wrap_180(float deg) {
    float res = fmodf(deg + 180.0f, 360.0f);
    if (res < 0.0f) res += 360.0f;
    return res - 180.0f;
}

inline float wrap_pi(float rad) {
    float res = fmodf(rad + (float)M_PI, 2.0f * (float)M_PI);
    if (res < 0.0f) res += 2.0f * (float)M_PI;
    return res - (float)M_PI;
}

// ----------------------------------------------------------------------------
// 2D Distance & Bearing
// ----------------------------------------------------------------------------
inline float distance_xy(const LocalPoint& a, const LocalPoint& b) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    return sqrtf(dx * dx + dy * dy);
}

inline float bearing(const LocalPoint& from, const LocalPoint& to) {
    float dx = to.x - from.x;
    float dy = to.y - from.y;
    float angle_deg = rad2deg(atan2f(dx, dy));
    return wrap_360(angle_deg);
}

inline float heading_error(float current_heading, float desired_bearing) {
    return wrap_180(desired_bearing - current_heading);
}

// ----------------------------------------------------------------------------
// Equirectangular GPS to Local Coordinate Projection
// ----------------------------------------------------------------------------
inline LocalPoint gps_to_local(const GPSPoint& pt, const GPSPoint& origin) {
    double lat_rad = pt.lat * (M_PI / 180.0);
    double lon_rad = pt.lon * (M_PI / 180.0);
    double lat0_rad = origin.lat * (M_PI / 180.0);
    double lon0_rad = origin.lon * (M_PI / 180.0);

    double d_lat = lat_rad - lat0_rad;
    double d_lon = lon_rad - lon0_rad;

    float y = (float)(EARTH_RADIUS_M * d_lat);
    float x = (float)(EARTH_RADIUS_M * cos(lat0_rad) * d_lon);
    return LocalPoint(x, y);
}

inline GPSPoint local_to_gps(const LocalPoint& pt, const GPSPoint& origin) {
    double lat0_rad = origin.lat * (M_PI / 180.0);
    double d_lat = (double)pt.y / EARTH_RADIUS_M;
    double d_lon = (double)pt.x / (EARTH_RADIUS_M * cos(lat0_rad));
    double lat = (lat0_rad + d_lat) * (180.0 / M_PI);
    double lon = (origin.lon * (M_PI / 180.0) + d_lon) * (180.0 / M_PI);
    return GPSPoint(lat, lon);
}

// ----------------------------------------------------------------------------
// Aerodynamic Wind Crab Angle Compensation
// ----------------------------------------------------------------------------
inline WindCompResult calculate_wind_corrected_heading(
    float desired_course_deg,
    float airspeed,
    float wind_x,
    float wind_y
) {
    WindCompResult res;
    float chi_rad = deg2rad(desired_course_deg);

    // Crosswind component (perpendicular to course): n_d = [cos(chi), -sin(chi)]
    float w_perp = wind_x * cosf(chi_rad) - wind_y * sinf(chi_rad);

    // Along-track wind (parallel to course): u_d = [sin(chi), cos(chi)]
    float w_parallel = wind_x * sinf(chi_rad) + wind_y * cosf(chi_rad);

    res.crosswind_component = w_perp;
    res.along_track_wind = w_parallel;

    if (airspeed < 1e-4f) {
        res.heading_deg = desired_course_deg;
        res.crab_angle_deg = 0.0f;
        res.ground_speed_along_track = 0.0f;
        res.feasible = false;
        return res;
    }

    res.feasible = (fabsf(w_perp) <= (airspeed + 1e-5f));

    if (res.feasible) {
        float ratio = -w_perp / airspeed;
        if (ratio > 1.0f) ratio = 1.0f;
        if (ratio < -1.0f) ratio = -1.0f;

        float delta_rad = asinf(ratio);
        float delta_deg = rad2deg(delta_rad);

        res.crab_angle_deg = delta_deg;
        res.heading_deg = wrap_360(desired_course_deg + delta_deg);
        res.ground_speed_along_track = airspeed * cosf(delta_rad) + w_parallel;
    } else {
        // Crosswind exceeds airspeed: point fully into wind
        float delta_deg = (w_perp < 0.0f) ? 90.0f : -90.0f;
        res.crab_angle_deg = delta_deg;
        res.heading_deg = wrap_360(desired_course_deg + delta_deg);
        res.ground_speed_along_track = w_parallel;
    }

    return res;
}

// ----------------------------------------------------------------------------
// Circular Orbit Geometry
// ----------------------------------------------------------------------------
inline float calculate_tangent_heading(
    const LocalPoint& center,
    const LocalPoint& position,
    LoiterDirection direction
) {
    float dx = position.x - center.x;
    float dy = position.y - center.y;
    if (sqrtf(dx * dx + dy * dy) < 1e-4f) {
        return (direction == LoiterDirection::CLOCKWISE) ? 90.0f : 270.0f;
    }

    float radial_bearing = bearing(center, position);
    if (direction == LoiterDirection::CLOCKWISE) {
        return wrap_360(radial_bearing + 90.0f);
    } else {
        return wrap_360(radial_bearing - 90.0f);
    }
}
