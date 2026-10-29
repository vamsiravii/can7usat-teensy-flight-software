#include "ekf.h"
#include "nav_math.h"
#include "config.h"
#include <string.h>

ExtendedKalmanFilter::ExtendedKalmanFilter() {
    current_va = FORWARD_AIRSPEED;
    current_vz = PARAFOIL_DESCENT_RATE;
    wind_x = 0.0f;
    wind_y = 0.0f;

    r_gps_pos = 2.5f;
    r_gps_vel = 0.2f;
    r_baro_alt = 1.5f;
    r_heading_rad = deg2rad(3.0f);

    q_pos = 0.2f;
    q_alt = 0.1f;
    q_psi = 0.05f;
    q_yaw_rate = 0.5f;
    tau_yaw = 0.1f;

    reset();
}

void ExtendedKalmanFilter::reset(
    float init_x,
    float init_y,
    float init_alt,
    float init_heading_deg
) {
    x[0] = init_x;
    x[1] = init_y;
    x[2] = init_alt;
    x[3] = deg2rad(wrap_360(init_heading_deg));
    x[4] = 0.0f;

    memset(P, 0, sizeof(P));
    P[0][0] = 5.0f * 5.0f;
    P[1][1] = 5.0f * 5.0f;
    P[2][2] = 3.0f * 3.0f;
    P[3][3] = deg2rad(10.0f) * deg2rad(10.0f);
    P[4][4] = deg2rad(5.0f) * deg2rad(5.0f);

    wind_x = 0.0f;
    wind_y = 0.0f;
}

void ExtendedKalmanFilter::setKinematics(float forward_speed, float descent_rate) {
    if (forward_speed >= 0.0f) current_va = forward_speed;
    if (descent_rate >= 0.0f) current_vz = descent_rate;
}

void ExtendedKalmanFilter::setWindEstimate(float wx, float wy) {
    wind_x = wx;
    wind_y = wy;
}

void ExtendedKalmanFilter::predict(float dt, float steering_cmd) {
    if (dt <= 0.0f) return;

    float u = (steering_cmd < -1.0f) ? -1.0f : ((steering_cmd > 1.0f) ? 1.0f : steering_cmd);
    float max_rate_rad = deg2rad(MAX_TURN_RATE_DEG_S);
    float cmd_rate = u * max_rate_rad;

    float px = x[0], py = x[1], pz = x[2], psi = x[3], w = x[4];

    // State propagation
    x[0] = px + (current_va * sinf(psi) + wind_x) * dt;
    x[1] = py + (current_va * cosf(psi) + wind_y) * dt;
    x[2] = pz - current_vz * dt;
    if (x[2] < 0.0f) x[2] = 0.0f;
    x[3] = wrap_pi(psi + w * dt);
    x[4] = w + ((cmd_rate - w) / tau_yaw) * dt;

    // Jacobian F (5x5)
    float F[5][5];
    memset(F, 0, sizeof(F));
    for (int i = 0; i < 5; i++) F[i][i] = 1.0f;
    F[0][3] = current_va * cosf(psi) * dt;
    F[1][3] = -current_va * sinf(psi) * dt;
    F[3][4] = dt;
    F[4][4] = 1.0f - (dt / tau_yaw);
    if (F[4][4] < 0.0f) F[4][4] = 0.0f;

    // F * P
    float FP[5][5];
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 5; k++) sum += F[i][k] * P[k][j];
            FP[i][j] = sum;
        }
    }

    // FP * F^T + Q
    float Q[5] = {
        (q_pos * dt) * (q_pos * dt),
        (q_pos * dt) * (q_pos * dt),
        (q_alt * dt) * (q_alt * dt),
        (q_psi * dt) * (q_psi * dt),
        (q_yaw_rate * dt) * (q_yaw_rate * dt)
    };

    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 5; k++) sum += FP[i][k] * F[j][k];
            if (i == j) sum += Q[i];
            P[i][j] = sum;
        }
    }

    enforceSymmetryAndPSD();
}

bool ExtendedKalmanFilter::correctBaro(float meas_alt) {
    if (meas_alt < -50.0f || meas_alt > 5000.0f) return false;

    float y = meas_alt - x[2];
    float R = r_baro_alt * r_baro_alt;
    float S = P[2][2] + R;
    if (S < 1e-6f) return false;

    // NIS Gating (3-sigma threshold = 9.0)
    float nis = (y * y) / S;
    if (nis > 9.0f) return false;

    float K[5];
    for (int i = 0; i < 5; i++) K[i] = P[i][2] / S;

    // State update
    for (int i = 0; i < 5; i++) x[i] += K[i] * y;
    x[3] = wrap_pi(x[3]);

    // Joseph-form covariance update: (I - K*H)*P*(I - K*H)^T + K*R*K^T
    float IKH[5][5];
    memset(IKH, 0, sizeof(IKH));
    for (int i = 0; i < 5; i++) {
        IKH[i][i] = 1.0f;
        IKH[i][2] -= K[i]; // Since H has 1 at index 2
    }

    float temp[5][5];
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 5; k++) sum += IKH[i][k] * P[k][j];
            temp[i][j] = sum;
        }
    }

    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 5; k++) sum += temp[i][k] * IKH[j][k];
            sum += K[i] * R * K[j];
            P[i][j] = sum;
        }
    }

    enforceSymmetryAndPSD();
    return true;
}

bool ExtendedKalmanFilter::correctHeading(float meas_heading_deg) {
    float meas_rad = deg2rad(wrap_360(meas_heading_deg));
    float y = wrap_pi(meas_rad - x[3]);
    float R = r_heading_rad * r_heading_rad;
    float S = P[3][3] + R;
    if (S < 1e-6f) return false;

    // NIS Gating (3-sigma threshold = 9.0)
    float nis = (y * y) / S;
    if (nis > 9.0f) return false;

    float K[5];
    for (int i = 0; i < 5; i++) K[i] = P[i][3] / S;

    // State update
    for (int i = 0; i < 5; i++) x[i] += K[i] * y;
    x[3] = wrap_pi(x[3]);

    // Joseph form covariance update
    float IKH[5][5];
    memset(IKH, 0, sizeof(IKH));
    for (int i = 0; i < 5; i++) {
        IKH[i][i] = 1.0f;
        IKH[i][3] -= K[i];
    }

    float temp[5][5];
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 5; k++) sum += IKH[i][k] * P[k][j];
            temp[i][j] = sum;
        }
    }

    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 5; k++) sum += temp[i][k] * IKH[j][k];
            sum += K[i] * R * K[j];
            P[i][j] = sum;
        }
    }

    enforceSymmetryAndPSD();
    return true;
}

bool ExtendedKalmanFilter::correctGPS(
    float meas_x,
    float meas_y,
    float /*meas_vx*/,
    float /*meas_vy*/,
    bool /*has_vel*/
) {
    // 2-DOF Position update (x, y)
    float y[2] = { meas_x - x[0], meas_y - x[1] };
    float R = r_gps_pos * r_gps_pos;

    // S = H P H^T + R (2x2 matrix)
    float S00 = P[0][0] + R;
    float S01 = P[0][1];
    float S10 = P[1][0];
    float S11 = P[1][1] + R;

    float det = S00 * S11 - S01 * S10;
    if (fabsf(det) < 1e-6f) return false;
    float inv_det = 1.0f / det;

    float S_inv[2][2] = {
        { S11 * inv_det, -S01 * inv_det },
        { -S10 * inv_det, S00 * inv_det }
    };

    // NIS Gating (4-sigma threshold = 16.0 for 2-DOF)
    float nis = y[0] * (S_inv[0][0] * y[0] + S_inv[0][1] * y[1]) +
                y[1] * (S_inv[1][0] * y[0] + S_inv[1][1] * y[1]);
    if (nis > 16.0f) return false;

    // K = P H^T S^-1 (5x2)
    float K[5][2];
    for (int i = 0; i < 5; i++) {
        K[i][0] = P[i][0] * S_inv[0][0] + P[i][1] * S_inv[1][0];
        K[i][1] = P[i][0] * S_inv[0][1] + P[i][1] * S_inv[1][1];
    }

    // State update
    for (int i = 0; i < 5; i++) {
        x[i] += K[i][0] * y[0] + K[i][1] * y[1];
    }
    x[3] = wrap_pi(x[3]);

    // Joseph form: IKH = I - K*H
    float IKH[5][5];
    memset(IKH, 0, sizeof(IKH));
    for (int i = 0; i < 5; i++) {
        IKH[i][i] = 1.0f;
        IKH[i][0] -= K[i][0];
        IKH[i][1] -= K[i][1];
    }

    float temp[5][5];
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 5; k++) sum += IKH[i][k] * P[k][j];
            temp[i][j] = sum;
        }
    }

    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 5; k++) sum += temp[i][k] * IKH[j][k];
            // K * R * K^T (since R is diag(R, R))
            sum += (K[i][0] * K[j][0] + K[i][1] * K[j][1]) * R;
            P[i][j] = sum;
        }
    }

    enforceSymmetryAndPSD();
    return true;
}

void ExtendedKalmanFilter::enforceSymmetryAndPSD() {
    for (int i = 0; i < 5; i++) {
        for (int j = i + 1; j < 5; j++) {
            float avg = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = avg;
            P[j][i] = avg;
        }
        if (P[i][i] < 1e-6f) P[i][i] = 1e-6f;
    }
}

float ExtendedKalmanFilter::getHeadingDeg() const {
    return wrap_360(rad2deg(x[3]));
}

EKFStateEstimate ExtendedKalmanFilter::getEstimate() const {
    EKFStateEstimate est;
    est.x = x[0];
    est.y = x[1];
    est.altitude = x[2];
    est.heading_deg = getHeadingDeg();
    est.heading_rate_deg_s = rad2deg(x[4]);

    est.vx = current_va * sinf(x[3]) + wind_x;
    est.vy = current_va * cosf(x[3]) + wind_y;
    est.vz = -current_vz;

    est.sigma_x = sqrtf(P[0][0]);
    est.sigma_y = sqrtf(P[1][1]);
    est.sigma_alt = sqrtf(P[2][2]);
    est.sigma_heading_deg = rad2deg(sqrtf(P[3][3]));

    return est;
}
