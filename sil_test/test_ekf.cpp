
#include <iostream>
#include "CAN7USAT_FSW/ekf.h"
#include "CAN7USAT_FSW/nav_math.h"

int main() {
    ExtendedKalmanFilter ekf;
    ekf.reset(0, 0, 1000, 0);
    ekf.setKinematics(0, 17);
    for (int i = 0; i < 20; i++) {
        ekf.predict(0.1f, 0.0f);
        bool ok = ekf.correctGPS(0.0f, 0.0f);
        std::cout << "step " << i << " ok=" << ok << " x=" << ekf.getX() << " y=" << ekf.getY() << " z=" << ekf.getAltitude() << std::endl;
    }
    return 0;
}
