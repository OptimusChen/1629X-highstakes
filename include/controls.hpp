#pragma once

#include "pros/misc.h"
#include "pros/motors.h"

#include "pros/motor_group.hpp"

#define L_DRIVE_FRONT 16
#define L_DRIVE_MID 17
#define L_DRIVE_BACK 18

#define R_DRIVE_FRONT 13
#define R_DRIVE_MID 14
#define R_DRIVE_BACK 15

#define HOOKS 6

#define MOGO 1
#define CORNER_ARM 3
#define INTAKE_LIFT 2

#define ARM 5

#define VERTICAL 7
#define HORIZONTAL 8

#define INERTIAL_PORT 11

#define ARM_ROT 9

using namespace pros;

namespace controls {
    // MotorGroup left_motor_group;
    // MotorGroup right_motor_group;
    // Drivetrain drivetrain;
    // Imu inertial_sensor;
    // OdomSensors sensors;
    // ControllerSettings lateralController;
    // ControllerSettings angularController;
    // ExpoDriveCurve throttle_curve;
    // ExpoDriveCurve steer_curve;

    void clamp_mogo();
    void release_mogo();

    void raise_intake();
    void lower_intake();
    
    void intake();
    void stop_intake();

    void begin_intake(int duration, bool async);
}