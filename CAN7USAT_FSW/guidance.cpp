#include "guidance.h"
#include "nav_math.h"
#include "config.h"

// ============================================================================
// Wind Estimator
// ============================================================================
WindEstimator::WindEstimator(float a) : alpha(a), wind_x(0.0f), wind_y(0.0f), initialized(false) {}

void WindEstimator::reset() {
    wind_x = 0.0f;
    wind_y = 0.0f;
    initialized = false;
}

void WindEstimator::update(float raw_wx, float raw_wy) {
    if (!initialized) {
        wind_x = raw_wx;
        wind_y = raw_wy;
        initialized = true;
    } else {
        wind_x = alpha * raw_wx + (1.0f - alpha) * wind_x;
        wind_y = alpha * raw_wy + (1.0f - alpha) * wind_y;
    }
}

// ============================================================================
// PID Controller (Radian-Domain Input)
// ============================================================================
PIDController::PIDController(
    float _kp, float _ki, float _kd,
    float out_min, float out_max,
    float int_min, float int_max
) : kp(_kp), ki(_ki), kd(_kd),
    output_min(out_min), output_max(out_max),
    integral_min(int_min), integral_max(int_max),
    integral(0.0f), previous_error(0.0f), initialized(false) {}

void PIDController::reset() {
    integral = 0.0f;
    previous_error = 0.0f;
    initialized = false;
}

float PIDController::update(float error_rad, float dt) {
    if (dt <= 0.0f) return 0.0f;

    // Proportional
    float p_term = kp * error_rad;

    // Integral with anti-windup clamping
    integral += error_rad * dt;
    if (integral < integral_min) integral = integral_min;
    if (integral > integral_max) integral = integral_max;
    float i_term = ki * integral;

    // Derivative
    float d_term = 0.0f;
    if (!initialized) {
        initialized = true;
    } else {
        d_term = kd * ((error_rad - previous_error) / dt);
    }
    previous_error = error_rad;

    float output = p_term + i_term + d_term;
    if (output < output_min) output = output_min;
    if (output > output_max) output = output_max;

    return output;
}

// ============================================================================
// Waypoint Manager
// ============================================================================
WaypointManager::WaypointManager() : total_pts(0), current_idx(0), acceptance_radius(WAYPOINT_ACCEPTANCE_RAD) {}

void WaypointManager::init(const LocalPoint& start, const LocalPoint& target, int num_pts, float radius) {
    total_pts = (num_pts > 16) ? 16 : num_pts;
    if (total_pts < 1) total_pts = 1;
    current_idx = 0;
    acceptance_radius = radius;
    target_pt = target;

    for (int i = 1; i <= total_pts; i++) {
        float fraction = (float)i / (float)total_pts;
        waypoints[i - 1].x = start.x + fraction * (target.x - start.x);
        waypoints[i - 1].y = start.y + fraction * (target.y - start.y);
    }
}

void WaypointManager::reset() {
    current_idx = 0;
}

bool WaypointManager::update(const LocalPoint& current_pos) {
    if (isComplete()) return false;

    bool changed = false;
    while (!isComplete()) {
        LocalPoint wp = waypoints[current_idx];
        float dist = distance_xy(current_pos, wp);
        if (dist <= acceptance_radius) {
            current_idx++;
            changed = true;
            continue;
        }

        // Standard passing plane check: advance if vehicle has progressed past
        // the waypoint along the track vector toward the target
        float v_x = target_pt.x - wp.x;
        float v_y = target_pt.y - wp.y;
        float p_x = current_pos.x - wp.x;
        float p_y = current_pos.y - wp.y;
        float dot = p_x * v_x + p_y * v_y;
        if (dot > 0.0f) {
            current_idx++;
            changed = true;
            continue;
        }
        break;
    }
    return changed;
}

void WaypointManager::skipToFinalTarget() {
    current_idx = total_pts;
}

LocalPoint WaypointManager::getCurrentWaypoint() const {
    if (isComplete()) return target_pt;
    return waypoints[current_idx];
}

// ============================================================================
// Guidance Controller
// ============================================================================
GuidanceController::GuidanceController(WaypointManager* wm, PIDController* p)
    : waypoint_manager(wm), pid(p), wind_x(0.0f), wind_y(0.0f) {}

void GuidanceController::reset() {
    if (waypoint_manager) waypoint_manager->reset();
    if (pid) pid->reset();
}

GuidanceOutput GuidanceController::update(const LocalPoint& position, float heading, float altitude, float dt) {
    GuidanceOutput out;
    if (!waypoint_manager || !pid) return out;

    bool wp_reached = waypoint_manager->update(position);
    if (wp_reached) pid->reset();

    LocalPoint target = waypoint_manager->getTarget();
    float target_dist = distance_xy(position, target);

    if (waypoint_manager->isComplete()) {
        float desired_course = bearing(position, target);
        WindCompResult comp = calculate_wind_corrected_heading(desired_course, FORWARD_AIRSPEED, wind_x, wind_y);
        float err_deg = heading_error(heading, comp.heading_deg);
        float err_rad = deg2rad(err_deg);
        float steering = pid->update(err_rad, dt);

        out.steering = steering;
        out.desired_bearing = desired_course;
        out.heading_error = err_deg;
        out.distance_to_waypoint = target_dist;
        out.waypoint_number = waypoint_manager->getTotalWaypoints();
        out.waypoint_reached = wp_reached;
        out.target_reached = true;
        out.final_approach = (altitude <= FINAL_APPROACH_ALT || target_dist <= FINAL_APPROACH_RADIUS);
        out.desired_heading = comp.heading_deg;
        out.crab_angle = comp.crab_angle_deg;
        out.wind_feasible = comp.feasible;
        return out;
    }

    LocalPoint active_wp = waypoint_manager->getCurrentWaypoint();
    float desired_course = bearing(position, active_wp);
    WindCompResult comp = calculate_wind_corrected_heading(desired_course, FORWARD_AIRSPEED, wind_x, wind_y);
    float err_deg = heading_error(heading, comp.heading_deg);
    float err_rad = deg2rad(err_deg);
    float steering = pid->update(err_rad, dt);

    out.steering = steering;
    out.desired_bearing = desired_course;
    out.heading_error = err_deg;
    out.distance_to_waypoint = distance_xy(position, active_wp);
    out.waypoint_number = waypoint_manager->getCurrentIndex() + 1;
    out.waypoint_reached = wp_reached;
    out.target_reached = false;
    out.final_approach = (altitude <= FINAL_APPROACH_ALT && target_dist <= FINAL_APPROACH_RADIUS);
    out.desired_heading = comp.heading_deg;
    out.crab_angle = comp.crab_angle_deg;
    out.wind_feasible = comp.feasible;
    return out;
}

// ============================================================================
// Loiter Guidance
// ============================================================================
LoiterGuidance::LoiterGuidance()
    : center(TARGET_DEFAULT_EAST, TARGET_DEFAULT_NORTH),
      radius(LOITER_RADIUS),
      direction(LoiterDirection::CLOCKWISE),
      capture_tolerance(LOITER_CAPTURE_TOL),
      forward_speed(FORWARD_AIRSPEED),
      max_turn_rate(MAX_TURN_RATE_DEG_S),
      wind_x(0.0f), wind_y(0.0f),
      radial_dist(0.0f), radial_err(0.0f), desired_hdg(0.0f),
      phase(LoiterPhase::ORBIT_CAPTURE) {}

float LoiterGuidance::update(const LocalPoint& position, float current_heading) {
    radial_dist = distance_xy(center, position);
    radial_err = radial_dist - radius;

    float dir_sign = (direction == LoiterDirection::CLOCKWISE) ? 1.0f : -1.0f;
    bool is_tracking = fabsf(radial_err) <= capture_tolerance;
    phase = is_tracking ? LoiterPhase::ORBIT_TRACKING : LoiterPhase::ORBIT_CAPTURE;

    float pure_tangent = calculate_tangent_heading(center, position, direction);

    if (is_tracking) {
        float k_radial = 1.0f / radius;
        float corr_deg = rad2deg(atanf(k_radial * radial_err));
        if (corr_deg > 30.0f) corr_deg = 30.0f;
        if (corr_deg < -30.0f) corr_deg = -30.0f;

        float desired_course = (direction == LoiterDirection::CLOCKWISE)
            ? wrap_360(pure_tangent + corr_deg)
            : wrap_360(pure_tangent - corr_deg);

        WindCompResult comp = calculate_wind_corrected_heading(desired_course, forward_speed, wind_x, wind_y);
        desired_hdg = comp.heading_deg;

        // Nominal steady orbital feedforward command
        float nominal_turn_rate = rad2deg(forward_speed / radius);
        float feedforward = dir_sign * (nominal_turn_rate / max_turn_rate);

        float err = heading_error(current_heading, desired_hdg);
        float feedback = err / 45.0f;

        float cmd = feedforward + feedback;
        if (cmd > 1.0f) cmd = 1.0f;
        if (cmd < -1.0f) cmd = -1.0f;
        return cmd;
    } else {
        float desired_course = 0.0f;
        if (radial_dist > radius + capture_tolerance) {
            float theta_to_center = bearing(position, center);
            float ratio = radius / radial_dist;
            if (ratio > 1.0f) ratio = 1.0f;
            float alpha_deg = rad2deg(asinf(ratio));

            desired_course = (direction == LoiterDirection::CLOCKWISE)
                ? wrap_360(theta_to_center - alpha_deg)
                : wrap_360(theta_to_center + alpha_deg);
        } else {
            if (radial_dist < 1e-4f) {
                desired_course = current_heading;
            } else {
                float rad_b = bearing(center, position);
                float ratio = radial_dist / radius;
                if (ratio > 1.0f) ratio = 1.0f;
                float beta_deg = rad2deg(acosf(ratio));

                desired_course = (direction == LoiterDirection::CLOCKWISE)
                    ? wrap_360(rad_b + 90.0f - beta_deg)
                    : wrap_360(rad_b - 90.0f + beta_deg);
            }
        }

        WindCompResult comp = calculate_wind_corrected_heading(desired_course, forward_speed, wind_x, wind_y);
        desired_hdg = comp.heading_deg;

        float err = heading_error(current_heading, desired_hdg);
        float cmd = err / 45.0f;
        if (cmd > 1.0f) cmd = 1.0f;
        if (cmd < -1.0f) cmd = -1.0f;
        return cmd;
    }
}

// ============================================================================
// Energy Check
// ============================================================================
bool shouldEnterTerminalGuidance(
    float altitude,
    float target_distance,
    float wind_x,
    float wind_y,
    float desired_course_deg
) {
    if (altitude <= 0.0f || target_distance <= 0.0f) return false;

    // Glide range estimate: Rg = (Vg_along / Vz) * altitude
    WindCompResult comp = calculate_wind_corrected_heading(desired_course_deg, FORWARD_AIRSPEED, wind_x, wind_y);
    if (!comp.feasible || comp.ground_speed_along_track <= 0.0f) return false;

    float glide_range = (comp.ground_speed_along_track / PARAFOIL_DESCENT_RATE) * altitude;
    float threshold = target_distance + TERMINAL_MARGIN;

    bool energy_condition = (glide_range >= threshold);
    bool spatial_capture = (target_distance <= TERMINAL_CAPTURE_RADIUS);

    return (energy_condition && spatial_capture);
}

// ============================================================================
// Breakout Geometry Solver
// ============================================================================
BreakoutGeometry calculateBreakoutGeometry(
    const LocalPoint& position,
    float heading,
    const LocalPoint& target,
    float wind_x,
    float wind_y
) {
    BreakoutGeometry geom;
    geom.radial_distance = distance_xy(position, target);
    geom.target_bearing = bearing(position, target);
    geom.heading_error = heading_error(heading, geom.target_bearing);
    geom.glide_ratio = FORWARD_AIRSPEED / PARAFOIL_DESCENT_RATE;

    float omega_rad = deg2rad(MAX_TURN_RATE_DEG_S);
    geom.turn_radius = FORWARD_AIRSPEED / omega_rad;

    // Direct turn toward target bearing
    geom.turn_angle = fabsf(geom.heading_error);
    geom.turn_direction = (geom.heading_error >= 0.0f) ? 1.0f : -1.0f;

    geom.turn_duration = geom.turn_angle / MAX_TURN_RATE_DEG_S;
    geom.turn_arc_length = FORWARD_AIRSPEED * geom.turn_duration;

    // Kinematic turn endpoint in air frame
    float psi0 = deg2rad(heading);
    float psi1 = deg2rad(heading + geom.turn_direction * geom.turn_angle);

    float dx_turn = geom.turn_direction * geom.turn_radius * (cosf(psi0) - cosf(psi1));
    float dy_turn = -geom.turn_direction * geom.turn_radius * (sinf(psi0) - sinf(psi1));

    // Wind drift over turn
    float dx_drift = wind_x * geom.turn_duration;
    float dy_drift = wind_y * geom.turn_duration;

    geom.turn_endpoint = LocalPoint(position.x + dx_turn + dx_drift, position.y + dy_turn + dy_drift);
    geom.straight_distance = distance_xy(geom.turn_endpoint, target);
    geom.required_path_length = geom.turn_arc_length + geom.straight_distance;

    // Straight flight ground speed
    float straight_course = bearing(geom.turn_endpoint, target);
    WindCompResult comp_straight = calculate_wind_corrected_heading(straight_course, FORWARD_AIRSPEED, wind_x, wind_y);

    float t_straight = 0.0f;
    if (comp_straight.feasible && comp_straight.ground_speed_along_track > 0.0f) {
        t_straight = geom.straight_distance / comp_straight.ground_speed_along_track;
        geom.feasible = true;
    } else {
        t_straight = geom.straight_distance / FORWARD_AIRSPEED;
        geom.feasible = false;
    }

    float t_total = geom.turn_duration + t_straight;
    geom.minimum_required_altitude = PARAFOIL_DESCENT_RATE * t_total;
    geom.breakout_altitude = geom.minimum_required_altitude + BREAKOUT_ALT_MARGIN;

    return geom;
}
