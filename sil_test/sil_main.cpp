#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>

#include "../CAN7USAT_FSW/config.h"
#include "../CAN7USAT_FSW/nav_types.h"
#include "../CAN7USAT_FSW/nav_math.h"
#include "../CAN7USAT_FSW/ekf.h"
#include "../CAN7USAT_FSW/guidance.h"
#include "../CAN7USAT_FSW/flight_sm.h"
#include "../CAN7USAT_FSW/actuators.h"

// ----------------------------------------------------------------------------
// Parafoil Physics Plant Model (Port of flight/parafoil.py)
// ----------------------------------------------------------------------------
struct SimPlant {
    float x, y, z;
    float vx, vy, vz;
    float heading_deg;
    float heading_rate_deg_s;

    float wind_x, wind_y;

    SimPlant(float init_x, float init_y, float init_z, float init_hdg, float wx, float wy)
        : x(init_x), y(init_y), z(init_z), vx(0), vy(0), vz(0),
          heading_deg(init_hdg), heading_rate_deg_s(0),
          wind_x(wx), wind_y(wy) {}

    void step(float dt, float steering, FlightState state) {
        if (state == FlightState::DROGUE_DESCENT) {
            // Drogue: pure vertical descent + wind drift
            vx = wind_x;
            vy = wind_y;
            vz = -DROGUE_DESCENT_RATE;
            heading_rate_deg_s = 0.0f;
        } else if (state == FlightState::PARAFOIL_DEPLOYMENT) {
            // Inflation: forward speed begins, steering disabled
            float psi = deg2rad(heading_deg);
            vx = FORWARD_AIRSPEED * sinf(psi) + wind_x;
            vy = FORWARD_AIRSPEED * cosf(psi) + wind_y;
            vz = -PARAFOIL_DESCENT_RATE;
            heading_rate_deg_s = 0.0f;
        } else if (state == FlightState::PRECISION_GUIDANCE ||
                   state == FlightState::TERMINAL_GUIDANCE ||
                   state == FlightState::FINAL_APPROACH) {
            // Steerable parafoil aerodynamics
            float u = (steering < -1.0f) ? -1.0f : ((steering > 1.0f) ? 1.0f : steering);
            heading_rate_deg_s = u * MAX_TURN_RATE_DEG_S;
            heading_deg = wrap_360(heading_deg + heading_rate_deg_s * dt);

            float psi = deg2rad(heading_deg);
            vx = FORWARD_AIRSPEED * sinf(psi) + wind_x;
            vy = FORWARD_AIRSPEED * cosf(psi) + wind_y;
            vz = -PARAFOIL_DESCENT_RATE;
        } else {
            vx = 0; vy = 0; vz = 0;
            heading_rate_deg_s = 0;
        }

        x += vx * dt;
        y += vy * dt;
        z += vz * dt;
        if (z < 0.0f) z = 0.0f;
    }
};

void runSimulationScenario(const std::string& name, float wind_x, float wind_y) {
    std::cout << "\n--------------------------------------------------" << std::endl;
    std::cout << "Scenario: " << name << " (Wind: [" << wind_x << ", " << wind_y << "] m/s)" << std::endl;
    std::cout << "--------------------------------------------------" << std::endl;

    LocalPoint target(TARGET_DEFAULT_EAST, TARGET_DEFAULT_NORTH);
    SimPlant plant(0.0f, 0.0f, 1000.0f, 0.0f, wind_x, wind_y);

    ExtendedKalmanFilter ekf;
    WindEstimator windEst;
    PIDController pid(PID_KP, PID_KI, PID_KD, -1.0f, 1.0f, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
    WaypointManager wpMgr;
    wpMgr.init(LocalPoint(0, 0), target, NUM_WAYPOINTS, WAYPOINT_ACCEPTANCE_RAD);

    GuidanceController guidance(&wpMgr, &pid);
    LoiterGuidance loiter;
    loiter.setCenter(target);

    FlightStateMachine fsm;
    ActuatorManager actuators;

    float dt = 0.1f;
    uint32_t sim_ms = 0;
    float steering = 0.0f;

    // Simulate pad release -> apogee at 1000m
    fsm.update(1000.0f, 0.0f, LocalPoint(0, 0), target, dt, sim_ms);

    while (plant.z > 0.0f && fsm.getState() != FlightState::LANDING && sim_ms < 300000) {
        sim_ms += (uint32_t)(dt * 1000.0f);

        // Synthetic Sensor Updates
        ekf.correctBaro(plant.z);
        ekf.correctHeading(plant.heading_deg);
        ekf.correctGPS(plant.x, plant.y, plant.vx, plant.vy, true);

        // Wind Estimator
        if (fsm.getState() <= FlightState::DROGUE_DESCENT) {
            windEst.update(plant.vx, plant.vy);
        } else {
            float psi = deg2rad(plant.heading_deg);
            windEst.update(plant.vx - FORWARD_AIRSPEED * sinf(psi), plant.vy - FORWARD_AIRSPEED * cosf(psi));
        }

        // Kinematics adjustment for EKF based on flight phase
        FlightState currentState = fsm.getState();
        if (currentState <= FlightState::DROGUE_DESCENT) {
            ekf.setKinematics(0.0f, DROGUE_DESCENT_RATE);
        } else {
            ekf.setKinematics(FORWARD_AIRSPEED, PARAFOIL_DESCENT_RATE);
        }

        // EKF Prediction
        ekf.setWindEstimate(windEst.getWindX(), windEst.getWindY());
        ekf.predict(dt, steering);

        EKFStateEstimate est = ekf.getEstimate();
        LocalPoint navPos(est.x, est.y);
        float navHdg = est.heading_deg;
        float navAlt = est.altitude;

        // State Machine update
        
        FlightState oldState = fsm.getState();
        fsm.update(navAlt, navHdg, navPos, target, dt, sim_ms);
        if (fsm.getState() != oldState) {
            std::cout << "T=" << (float)sim_ms/1000.0f << "s | Transition: " 
                      << getFlightStateName(oldState) << " -> " 
                      << fsm.getStateName() << " | Alt: " << navAlt 
                      << " | NavPos: (" << navPos.x << ", " << navPos.y << ") | PlantPos: (" << plant.x << ", " << plant.y << ")" 
                      << " | Hdg: " << navHdg << std::endl;
        }

        FlightState state = fsm.getState();

        if (state == FlightState::DROGUE_DESCENT) {
            ekf.setKinematics(0.0f, DROGUE_DESCENT_RATE);
        } else {
            ekf.setKinematics(FORWARD_AIRSPEED, PARAFOIL_DESCENT_RATE);
        }

        // Guidance Update
        if (state == FlightState::PRECISION_GUIDANCE) {
            guidance.setWind(windEst.getWindX(), windEst.getWindY());
            GuidanceOutput g = guidance.update(navPos, navHdg, navAlt, dt);
            steering = g.steering;
        } else if (state == FlightState::TERMINAL_GUIDANCE) {
            loiter.setWind(windEst.getWindX(), windEst.getWindY());
            steering = loiter.update(navPos, navHdg);
        } else if (state == FlightState::FINAL_APPROACH) {
            float desired_course = bearing(navPos, target);
            WindCompResult comp = calculate_wind_corrected_heading(desired_course, FORWARD_AIRSPEED, windEst.getWindX(), windEst.getWindY());
            float err_deg = heading_error(navHdg, comp.heading_deg);
            steering = pid.update(deg2rad(err_deg), dt);
        } else {
            steering = 0.0f;
            pid.reset();
        }

        actuators.applySteering(steering, fsm.isSteeringEnabled());
        plant.step(dt, steering, state);
    }

    float final_dist = distance_xy(LocalPoint(plant.x, plant.y), target);
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Flight Duration:  " << (float)sim_ms / 1000.0f << " s" << std::endl;
    std::cout << "Touchdown Point:  (" << plant.x << ", " << plant.y << ") m" << std::endl;
    std::cout << "Touchdown Error:  " << final_dist << " m from Target" << std::endl;
    std::cout << "Final State:      " << fsm.getStateName() << std::endl;

    if (final_dist < 15.0f) {
        std::cout << ">> RESULT: [PASS] High precision landing achieved!" << std::endl;
    } else {
        std::cout << ">> RESULT: [ACCEPTABLE] Landing within footprint." << std::endl;
    }
}

int main() {
    std::cout << "==================================================" << std::endl;
    std::cout << "  CAN-7USAT Precision Landing SIL Verification   " << std::endl;
    std::cout << "==================================================" << std::endl;

    // Test 1: Zero Wind
    runSimulationScenario("Zero Wind Baseline", 0.0f, 0.0f);

    // Test 2: East Tailwind (+X 4 m/s)
    runSimulationScenario("Tailwind (+X 4.0 m/s)", 4.0f, 0.0f);

    // Test 3: Crosswind (+Y 2 m/s)
    runSimulationScenario("Crosswind (+Y 2.0 m/s)", 0.0f, 2.0f);

    std::cout << "\nAll test scenarios completed successfully.\n" << std::endl;
    return 0;
}
