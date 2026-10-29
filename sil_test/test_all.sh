#!/usr/bin/env bash
set -e

echo "=================================================================="
echo "    CAN-7USAT Team 017 GNC Flight Software Test Suite & SIL      "
echo "=================================================================="

echo ""
echo ">>> STEP 1a: Compiling and Running FSW C++ Unit Tests <<<"
clang++ -std=c++17 -Wall -Wextra -DSERVO_USE_STANDARD_POSITIONAL=0 -DACTUATOR_CALIBRATED=0 -I./CAN7USAT_FSW \
    sil_test/unit_tests.cpp \
    CAN7USAT_FSW/ekf.cpp \
    CAN7USAT_FSW/guidance.cpp \
    CAN7USAT_FSW/flight_sm.cpp \
    CAN7USAT_FSW/actuators.cpp \
    -o sil_test/unit_runner

./sil_test/unit_runner

echo ""
echo ">>> STEP 1b: Compiling and Running Continuous-Rotation Spool Actuator Tests <<<"
clang++ -std=c++17 -Wall -Wextra -DSERVO_USE_STANDARD_POSITIONAL=0 -DACTUATOR_CALIBRATED=0 -I./CAN7USAT_FSW \
    sil_test/test_integrated_fsw_actuators.cpp \
    CAN7USAT_FSW/ekf.cpp \
    CAN7USAT_FSW/guidance.cpp \
    CAN7USAT_FSW/flight_sm.cpp \
    CAN7USAT_FSW/actuators.cpp \
    -o sil_test/test_spool_actuators

./sil_test/test_spool_actuators

echo ""
echo ">>> STEP 2: Compiling and Running High-Fidelity Flight Dynamics SIL <<<"
clang++ -std=c++17 -O3 -Wall -Wextra -DSERVO_USE_STANDARD_POSITIONAL=0 -I./CAN7USAT_FSW \
    sil_test/sim_harness.cpp \
    CAN7USAT_FSW/ekf.cpp \
    CAN7USAT_FSW/guidance.cpp \
    CAN7USAT_FSW/flight_sm.cpp \
    CAN7USAT_FSW/actuators.cpp \
    -o sil_test/sim_runner

./sil_test/sim_runner

echo ""
echo "=================================================================="
echo ">> ALL FLIGHT SOFTWARE CODES AND SIMULATIONS VERIFIED [PASS] <<"
echo "=================================================================="
