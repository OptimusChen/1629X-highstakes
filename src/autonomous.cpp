#include "main.h"
#include "controls.hpp"

using namespace controls;

#define FAST 127
#define MID 100
#define ACCURATE 50

namespace auton {
    MotorGroup left_motor_group({-L_DRIVE_FRONT, -L_DRIVE_MID, -L_DRIVE_BACK});
    MotorGroup right_motor_group({R_DRIVE_FRONT, R_DRIVE_MID, R_DRIVE_BACK});

    Drivetrain drivetrain(&left_motor_group, &right_motor_group, 10.8, Omniwheel::NEW_275, 450, 2);

    Imu inertial_sensor(INERTIAL_PORT);

    pros::Rotation horizontal_encoder(-HORIZONTAL);
    pros::Rotation vertical_encoder(VERTICAL);

    lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -3.25f - 0.125f);
    lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 1.25f);

    OdomSensors sensors {
        &vertical_tracking_wheel, // vertical tracking wheel 1
        nullptr, // vertical tracking wheel 2
        &horizontal_tracking_wheel, // horizontal tracking wheel 1
        nullptr, // we don't have a second tracking wheel, so we set it to nullptr
        &inertial_sensor // inertial sensor
    };

    ControllerSettings lateralController {
        7, // kP
        0.2, // kI
        25, // kD
        3, // anti windup
        1, // small error range, in inches
        100, // small error range timeout, in milliseconds
        3, // large error range, in inches
        500, // large error range timeout, in milliseconds
        20 // maximum acceleration (slew)
    };

    // turning PID
    ControllerSettings angularController {
        4.4, // proportional gain (kP)
        0, // integral gain (kI)
        30, // derivative gain (kD)
        3, // anti windup
        1, // small error range, in inches
        100, // small error range timeout, in milliseconds
        3, // large error range, in inches
        500, // large error range timeout, in milliseconds
        30 // maximum acceleration (slew)
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
        chassis.arcade(rightX, leftY / 1.5);
    }

    void calibrate_drivetrain() {
        chassis.calibrate(true);
    }

    Pose get_pose() {
        return chassis.getPose();
    }

    void blue_plus_side_awp() {
        chassis.setPose(Pose(58, -12.5, 0));

        chassis.moveToPoint(58, -0.5, 4000, MoveToPointParams(true, ACCURATE), false);

        outtake();
        delay(150);
        stop_intake();

        chassis.turnToHeading(270, 2000, TurnToHeadingParams(AngularDirection::AUTO, ACCURATE / 2), false);

        left_motor_group.move(-20);
        right_motor_group.move(-20);

        delay(400);

        left_motor_group.brake();
        right_motor_group.brake();

        begin_intake(1000, false, nullptr);

        raise_intake();

        begin_intake(250, true, [=]() {
            lower_intake();
        });

        chassis.moveToPoint(46, -4, 2000, MoveToPointParams(true, ACCURATE), false);

        begin_intake(500, false, nullptr);

        Task bruh2([=]() {
            c::motor_move_velocity(HOOKS, 20);

            delay(500);

            c::motor_move_velocity(HOOKS, 0);
        });

        chassis.moveToPose(25.8, -22.7, 45, 3000, MoveToPoseParams(false, 0, 0.6, FAST), false);

        clamp_mogo();

        intake(100);

        delay(1000);

        chassis.turnToHeading(180, 1000, TurnToHeadingParams(AngularDirection::CCW_COUNTERCLOCKWISE, MID), false);

        c::motor_move_velocity(HOOKS, 0);

        intake(100);

        chassis.moveToPoint(20.5, -55, 3000, MoveToPointParams(true, MID), false);

        delay(1500);

        Task bruh([=]() {
            delay(1000);
            release_mogo();
        });

        chassis.moveToPose(16.5, -7.5, 270, 4000, MoveToPoseParams(true, 0, 0.6, MID), false);
    }

    // red awp
    void red_plus_side_awp() {
        chassis.setPose(Pose(-58, -12.5, 0));
        
        chassis.moveToPoint(-58, -0.5, 4000, MoveToPointParams(true, ACCURATE), false);

        outtake();
        delay(150);
        stop_intake();

        chassis.turnToHeading(90, 2000, TurnToHeadingParams(AngularDirection::AUTO, ACCURATE / 2), false);

        // chassis.moveToPoint(-57.9, -0.5, 4000, MoveToPointParams(true, ACCURATE), false);

        begin_intake(1500, false, nullptr);

        raise_intake();

        chassis.swingToHeading(120, DriveSide::RIGHT, 500, SwingToHeadingParams(AngularDirection::AUTO, ACCURATE), false);

        begin_intake(250, true, [=]() {
            lower_intake();
        });

        chassis.moveToPoint(-46, -8, 2000, MoveToPointParams(true, ACCURATE), false);

        begin_intake(400, false, nullptr);

        Task bruh2([=]() {
            c::motor_move_velocity(HOOKS, 20);

            delay(1000);

            c::motor_move_velocity(HOOKS, 0);
        });

        // chassis.turnToHeading(315, 500, TurnToHeadingParams(AngularDirection::AUTO, MID), false);

        chassis.moveToPose(-26, -20, 335, 4000, MoveToPoseParams(false, 0, 0.3, FAST), false);

        clamp_mogo();

        intake();

        delay(1000);

        chassis.turnToHeading(180, 1000, TurnToHeadingParams(AngularDirection::CCW_COUNTERCLOCKWISE, MID), false);

        c::motor_move_velocity(HOOKS, 0);

        intake();

        chassis.moveToPoint(-20.5, -55, 2000, MoveToPointParams(true, MID), false);

        chassis.moveToPose(-16.5, -7.5, 90, 4000, MoveToPoseParams(true, 0, 0.6, MID), false);

        release_mogo();
    }

    void plus_side_wp() {
        chassis.setPose(Pose(-56, 12.5, 180));
        
        chassis.moveToPoint(-56, 0.5, 4000, MoveToPointParams(true, ACCURATE), false);

        outtake();
        delay(150);
        stop_intake();

        chassis.turnToHeading(90, 1000, TurnToHeadingParams(AngularDirection::AUTO, ACCURATE), false);

        begin_intake(750, false, nullptr);

        raise_intake();

        chassis.swingToHeading(60, DriveSide::RIGHT, 500, SwingToHeadingParams(AngularDirection::AUTO, ACCURATE), false);


        begin_intake(500, true, [=]() {
            lower_intake();
        });

        chassis.moveToPoint(-46, 2, 2000, MoveToPointParams(true, ACCURATE), false);

        begin_intake(250, false, nullptr);

        Task bruh2([=]() {
            c::motor_move_velocity(HOOKS, 20);

            delay(2000);

            c::motor_move_velocity(HOOKS, 0);
        });

        chassis.turnToHeading(240, 500, TurnToHeadingParams(AngularDirection::AUTO, MID), false);

        chassis.moveToPose(-23.5, 20, -135, 2000, MoveToPoseParams(false, 0, 0.6, MID), false);

        clamp_mogo();

        intake();

        delay(1000);

        chassis.turnToHeading(0, 1000, TurnToHeadingParams(AngularDirection::CCW_COUNTERCLOCKWISE, MID), false);

        c::motor_move_velocity(HOOKS, 0);
        release_mogo();

        intake();
        
        Task bruh1([=]() {
            delay(1000);


            c::motor_move_velocity(HOOKS, 0);
        });

        chassis.moveToPoint(-20.5, 50, 2000, MoveToPointParams(true, MID), false);

        chassis.swingToHeading(-70, DriveSide::LEFT, 1000, SwingToHeadingParams(AngularDirection::AUTO, MID), false);

        chassis.moveToPoint(-1.5, 50, 1000, MoveToPointParams(false, MID), false);

        delay(500);

        clamp_mogo();

        delay(150);

        intake();

        Task bruh3([=]() {
            delay(1000);


            c::motor_move_velocity(HOOKS, 0);
        });

        Task bruh([=]() {
            delay(2000);


            release_mogo();
        });

        chassis.moveToPose(13.5, -11.5, 90, 4000, MoveToPoseParams(true, 0, 0.6, ACCURATE), false);
    }

    void red_negative() {
        chassis.setPose(Pose(-54, 20, 270));

        chassis.moveToPoint(-22, 20, 2000, MoveToPointParams(false, MID), false);

        clamp_mogo();

        delay(500);

        intake();

        delay(1000);

        chassis.turnToPoint(-21.5, 47, 2000, TurnToPointParams(true, AngularDirection::AUTO, ACCURATE), false);
        chassis.moveToPoint(-21.5, 47, 3000, MoveToPointParams(true, FAST), false);

        chassis.turnToPoint(-23.5, 0, 1000, TurnToPointParams(true, AngularDirection::AUTO, ACCURATE), false);

        release_mogo();

        chassis.moveToPoint(-23.5, 0, 4000, MoveToPointParams(true, ACCURATE), false);
    }

    void blue_negative() {
        chassis.setPose(Pose(54, 20, 90));

        chassis.moveToPoint(22, 20, 2000, MoveToPointParams(false, MID), false);

        clamp_mogo();

        delay(500);

        intake();

        delay(1000);

        chassis.turnToPoint(21.5, 47, 2000, TurnToPointParams(true, AngularDirection::AUTO, ACCURATE), false);
        chassis.moveToPoint(21.5, 47, 3000, MoveToPointParams(true, FAST), false);

        chassis.turnToPoint(23.5, 0, 1000, TurnToPointParams(true, AngularDirection::AUTO, ACCURATE), false);

        release_mogo();

        chassis.moveToPoint(23.5, 0, 4000, MoveToPointParams(true, ACCURATE), false);
    }
}
