#include "main.h"
#include "arm.hpp"
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

    lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -3.0f);
    lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0.7f);

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
        40, // derivative gain (kD)
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

    /*
        ELIMS
    */

    void red_plus_side_sweep() {
        chassis.setPose(Pose(-58, -12.5, 0));

        chassis.moveToPoint(-58, -0.5, 4000, MoveToPointParams(true, ACCURATE), false);

        outtake();
        delay(150);
        stop_intake();

        chassis.turnToHeading(90, 2000, TurnToHeadingParams(AngularDirection::AUTO, ACCURATE / 2), false);

        left_motor_group.move(-20);
        right_motor_group.move(-20);

        delay(200);

        left_motor_group.brake();
        right_motor_group.brake();

        begin_intake(1000, false, nullptr);

        raise_intake();

        begin_intake(250, true, [=]() {
            lower_intake();
        });

        chassis.moveToPoint(-46, -4, 2000, MoveToPointParams(true, ACCURATE), false);

        begin_intake(500, false, nullptr);

        Task bruh2([=]() {
            c::motor_move_velocity(HOOKS, 20);

            delay(500);

            c::motor_move_velocity(HOOKS, 0);
        });

        chassis.moveToPose(-26.5, -22.7, 315, 2500, MoveToPoseParams(false, 0, 0.6, FAST), false);

        clamp_mogo();

        intake(100);

        delay(1000);

        chassis.turnToHeading(180, 1000, TurnToHeadingParams(AngularDirection::CCW_COUNTERCLOCKWISE, MID), false);

        c::motor_move_velocity(HOOKS, 0);

        intake(100);

        chassis.moveToPoint(-20.5, -55, 3000, MoveToPointParams(true, MID), false);

        delay(1500);

        activate_corner_arm();

        c::motor_move_velocity(HOOKS, 0);

        chassis.moveToPose(-63.5, -65, 270, 1500, MoveToPoseParams(true, 0, 0.6, FAST, FAST), false);

        chassis.turnToPoint(0, 0, 2000, TurnToPointParams(true, AngularDirection::CCW_COUNTERCLOCKWISE), false);

        left_motor_group.move(-127);
        right_motor_group.move(-127);

        delay(400);

        left_motor_group.brake();
        right_motor_group.brake();
    }

    void blue_plus_side_sweep() {
        chassis.setPose(Pose(58, -12.5, 0));

        chassis.moveToPoint(58, -0.5, 4000, MoveToPointParams(true, ACCURATE), false);

        outtake();
        delay(150);
        stop_intake();

        chassis.turnToHeading(270, 2000, TurnToHeadingParams(AngularDirection::AUTO, ACCURATE / 2), false);

        left_motor_group.move(-20);
        right_motor_group.move(-20);

        delay(200);

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

        chassis.moveToPose(26.5, -22.7, 45, 2500, MoveToPoseParams(false, 0, 0.6, FAST), false);

        clamp_mogo();

        intake(100);

        delay(1000);

        chassis.turnToHeading(180, 1000, TurnToHeadingParams(AngularDirection::CCW_COUNTERCLOCKWISE, MID), false);

        c::motor_move_velocity(HOOKS, 0);

        intake(100);

        chassis.moveToPoint(20.5, -55, 3000, MoveToPointParams(true, MID), false);

        delay(1500);

        activate_corner_arm();

        c::motor_move_velocity(HOOKS, 0);

        chassis.moveToPose(63.5, -65, 90, 1500, MoveToPoseParams(true, 0, 0.6, FAST, FAST), false);

        chassis.turnToPoint(0, 0, 2000, TurnToPointParams(true, AngularDirection::CCW_COUNTERCLOCKWISE), false);

        left_motor_group.move(-127);
        right_motor_group.move(-127);

        delay(400);

        left_motor_group.brake();
        right_motor_group.brake();
    }

    /*
    
    END ELIMS
    
    */

    // work
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

        chassis.moveToPose(26.5, -22.7, 45, 2500, MoveToPoseParams(false, 0, 0.6, FAST), false);

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
            // release_mogo();
        });

        raise_intake();

        chassis.moveToPose(24, 0, 0, 2250, MoveToPoseParams(true, 0, 0.6, MID), false);

        lower_intake();
    }

    // should work
    void red_plus_side_awp() {
        chassis.setPose(Pose(-58, -12.5, 0));

        chassis.moveToPoint(-58, -0.5, 4000, MoveToPointParams(true, ACCURATE), false);

        outtake();
        delay(150);
        stop_intake();

        chassis.turnToHeading(90, 2000, TurnToHeadingParams(AngularDirection::AUTO, ACCURATE / 2), false);

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

        chassis.moveToPoint(-46, -4, 2000, MoveToPointParams(true, ACCURATE), false);

        begin_intake(500, false, nullptr);

        Task bruh2([=]() {
            c::motor_move_velocity(HOOKS, 20);

            delay(500);

            c::motor_move_velocity(HOOKS, 0);
        });

        chassis.moveToPose(-26.5, -22.7, 315, 2500, MoveToPoseParams(false, 0, 0.6, FAST), false);

        clamp_mogo();

        intake(100);

        delay(1000);

        chassis.turnToHeading(180, 1000, TurnToHeadingParams(AngularDirection::CCW_COUNTERCLOCKWISE, MID), false);

        c::motor_move_velocity(HOOKS, 0);

        intake(100);

        chassis.moveToPoint(-20.5, -55, 3000, MoveToPointParams(true, MID), false);

        delay(1500);

        Task bruh([=]() {
            delay(1000);
            // release_mogo();
        });

        raise_intake();

        chassis.moveToPose(-24, 0, 0, 2250, MoveToPoseParams(true, 0, 0.6, MID), false);

        lower_intake();
    }

    void red_negative() {
        chassis.setPose(Pose(-54, 20, 270));

        chassis.moveToPoint(-22, 20, 2000, MoveToPointParams(false, MID), false);

        clamp_mogo();

        delay(500);

        intake();

        delay(500);

        chassis.turnToPoint(-22, 47, 1500, TurnToPointParams(true, AngularDirection::AUTO, ACCURATE), false);
        chassis.moveToPoint(-22, 47, 2000, MoveToPointParams(true, FAST), false);

        delay(1000);

        chassis.turnToPoint(-3.5, 57, 1000, TurnToPointParams(true, AngularDirection::AUTO), false);
        chassis.moveToPoint(0, 55, 2000, MoveToPointParams(true, MID), false);

        left_motor_group.move(-60);
        right_motor_group.move(-60);

        delay(250);

        left_motor_group.brake();
        right_motor_group.brake();

        delay(1000);

        chassis.turnToPoint(1, 45, 1000, TurnToPointParams(true, AngularDirection::AUTO), false);
        chassis.moveToPoint(1, 43, 2000, MoveToPointParams(true, MID), false);
        
        delay(1000);

        chassis.moveToPoint(-23, 47, 2000, MoveToPointParams(false, FAST), false);

        chassis.turnToPoint(-18, 0, 2000, TurnToPointParams(true, AngularDirection::AUTO, ACCURATE), false);

        raise_intake();

        chassis.moveToPoint(-18, 0, 4000, MoveToPointParams(true, MID), false);

        lower_intake();
    }

    void blue_negative() {
        chassis.setPose(Pose(54, 20, 90));

        chassis.moveToPoint(22, 20, 2000, MoveToPointParams(false, MID), false);

        clamp_mogo();

        delay(500);

        intake();

        delay(500);

        chassis.turnToPoint(22, 47, 1500, TurnToPointParams(true, AngularDirection::AUTO, ACCURATE), false);
        chassis.moveToPoint(22, 47, 2000, MoveToPointParams(true, FAST), false);

        delay(1000);

        chassis.turnToPoint(3.5, 57, 1000, TurnToPointParams(true, AngularDirection::AUTO), false);
        chassis.moveToPoint(-1, 55, 2000, MoveToPointParams(true, MID), false);

        left_motor_group.move(-60);
        right_motor_group.move(-60);

        delay(250);

        left_motor_group.brake();
        right_motor_group.brake();

        delay(1000);

        chassis.turnToPoint(-1, 45, 1000, TurnToPointParams(true, AngularDirection::AUTO), false);
        chassis.moveToPoint(-1, 43, 2000, MoveToPointParams(true, MID), false);
        
        delay(1000);

        chassis.moveToPoint(23, 47, 2000, MoveToPointParams(false, FAST), false);

        chassis.turnToPoint(18, 0, 2000, TurnToPointParams(true, AngularDirection::AUTO, ACCURATE), false);

        raise_intake();

        chassis.moveToPoint(18, 0, 4000, MoveToPointParams(true, MID), false);

        lower_intake();
    }

    /*
    

    ░██████╗██╗░░██╗██╗██╗░░░░░██╗░░░░░░██████╗
    ██╔════╝██║░██╔╝██║██║░░░░░██║░░░░░██╔════╝
    ╚█████╗░█████═╝░██║██║░░░░░██║░░░░░╚█████╗░
    ░╚═══██╗██╔═██╗░██║██║░░░░░██║░░░░░░╚═══██╗
    ██████╔╝██║░╚██╗██║███████╗███████╗██████╔╝
    ╚═════╝░╚═╝░░╚═╝╚═╝╚══════╝╚══════╝╚═════╝░
    
    */

    void skills() {
        Motor a(ARM);
        Rotation r(ARM_ROT);

        Arm arm(&a, &r);

        Task bruh2([&]() {
            while (true) {
                arm.tick({});

                delay(1);
            }
        });

        chassis.setPose(Pose(-60, 0, 90));

        begin_intake(500, false, nullptr);

        chassis.moveToPose(-50, -22, 310, 3000, MoveToPoseParams(false, 0, 0.6, MID), false); //Move to Mogo

        clamp_mogo();
        intake();

        chassis.turnToHeading(90, 2000, TurnToHeadingParams(AngularDirection::AUTO, FAST), false); //Turn to first ring

        chassis.moveToPoint(-23, -23, 2000, MoveToPointParams(true, MID), false); // ring 1
        chassis.moveToPoint(0, 0, 2000, MoveToPointParams(true, MID), false); //center ring
        chassis.moveToPoint(23, -23, 2000, MoveToPointParams(true, MID), false); // ring 2
        chassis.moveToPoint(23, -48, 2000, MoveToPointParams(true, MID), false); // ring 2
        //raise arm
        arm.prime();
        chassis.moveToPose(0, -60, 180, 2000, MoveToPoseParams(true, MID), false);
        //move to the wallstake
        //chassis.turnToHeading(180, 2000, TurnToHeadingParams(AngularDirection::AUTO, FAST), false);
        chassis.moveToPose(0, -70, 180, 1000, MoveToPoseParams(true, MID), false);

        chassis.turnToHeading(270, 2000, TurnToHeadingParams(AngularDirection::AUTO, FAST), false); //Turn to first ring

        // left_motor_group.move(80);
        // right_motor_group.move(80);

        // delay(200);

        // left_motor_group.brake();
        // right_motor_group.brake();
        //lokc in an score
        stop_intake();
        arm.score();
        delay(3000);
        intake();


        chassis.moveToPoint(0, -48, 2000, MoveToPointParams(false, MID), false); // ring 2
        /*chassis.moveToPoint(-20, -23.5, 2000, MoveToPointParams(true, MID), false);
        chassis.moveToPoint(-23.5, -51, 2000, MoveToPointParams(true, MID), false);
        chassis.moveToPoint(-63.5, -51, 2000, MoveToPointParams(true, MID), false);
        chassis.moveToPoint(-46, -61, 2000, MoveToPointParams(true, MID), false);*/
    }
}