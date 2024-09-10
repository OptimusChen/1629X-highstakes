#include "main.h"
#include "controls.hpp"

using namespace controls;

#define FAST 127
#define MID 100
#define ACCURATE 50

namespace auton {
    MotorGroup left_motor_group({-L_DRIVE_FRONT, -L_DRIVE_MID, -L_DRIVE_BACK});
    MotorGroup right_motor_group({R_DRIVE_FRONT, R_DRIVE_MID, R_DRIVE_BACK});

    Drivetrain drivetrain(&left_motor_group, &right_motor_group, 11, Omniwheel::NEW_275, 450, 2);

    Imu inertial_sensor(INERTIAL_PORT);

    pros::Rotation horizontal_encoder(-HORIZONTAL);
    pros::Rotation vertical_encoder(VERTICAL);

    lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 2.5f);
    lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 1.0f);

    OdomSensors sensors {
        &vertical_tracking_wheel, // vertical tracking wheel 1
        nullptr, // vertical tracking wheel 2
        &horizontal_tracking_wheel, // horizontal tracking wheel 1
        nullptr, // we don't have a second tracking wheel, so we set it to nullptr
        &inertial_sensor // inertial sensor
    };

    ControllerSettings lateralController {
        8, // kP
        0, // kI
        40, // kD
        3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
    };

    // turning PID
    ControllerSettings angularController {
        2, // proportional gain (kP)
        0, // integral gain (kI)
        10, // derivative gain (kD)
        3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
    };

    lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                        10, // minimum output where drivetrain will move out of 127
                                        1.019 // expo curve gain
    );

    // input curve for steer input during driver control
    lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                    0, // minimum output where drivetrain will move out of 127
                                    1 // expo curve gain
    );

    Chassis chassis(drivetrain, lateralController, angularController, sensors, &throttle_curve,  &steer_curve);

    void arcade(int leftY, int rightX) {
        chassis.arcade(rightX, leftY);
    }

    void calibrate_drivetrain() {
        chassis.calibrate(true);
    }

    Pose get_pose() {
        return chassis.getPose();
    }

    void four_ring_negative() {
        // chassis.setPose(0, 0, 0);
        // chassis.moveToPoint(10, 0, 1000000);

        // delay(1000000);

        // return;

        chassis.setPose(Pose(-54, 20, 270));

        chassis.moveToPoint(-22, 20, 2000, MoveToPointParams(false, MID), false);

        clamp_mogo();

        delay(500);

        intake();

        delay(1000);

        // chassis.turnToHeading(90, 10000, TurnToHeadingParams(AngularDirection::AUTO, FAST), false);
        chassis.turnToPoint(-28, 47, 2000, TurnToPointParams(true, AngularDirection::AUTO, ACCURATE), false);
        chassis.moveToPoint(-28, 40, 3000, MoveToPointParams(true, FAST), false);

        chassis.moveToPoint(-12, 10, 1000, MoveToPointParams(true, ACCURATE), false);

        return;

        delay(500);

        // chassis.moveToPose(-16, 43.5, 90, 4000, MoveToPoseParams(true, 0, 0, MID), false);

        chassis.turnToPoint(-3.5, 55, 2000, TurnToPointParams(true, AngularDirection::AUTO, MID), false);
        chassis.moveToPoint(-15, 54.5, 4000, MoveToPointParams(true, ACCURATE), false);

        // chassis.moveToPoint(-18, 43.5, 2000, MoveToPointParams(true, MID), false);
        // chassis.moveToPose(-10, -45.5, 60, 1000, MoveToPoseParams(false, 0, 0.6, ACCURATE), false);

        // chassis.moveToPoint(-5, 50, 2000, MoveToPointParams(true, MID), false);
        // chassis.moveToPose(-23.5, 23.5, 130, 2000, MoveToPoseParams(false, 0, 0.6, MID), false);

        // release_mogo();

        // 
    }
}