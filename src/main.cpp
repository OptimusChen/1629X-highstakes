#include "main.h"
#include "pid.hpp"
#include "arm.hpp"
#include "controls.hpp"
#include "pros/misc.h"
#include "pros/motors.h"

#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <math.h>

#include "s.hpp"
#include "liblvgl/lvgl.h"

/*


    ██╗░░░██╗██████╗░  ███╗░░░███╗░█████╗░███╗░░░███╗  ██╗░██████╗  ░██████╗░░█████╗░██╗░░░██╗
    ██║░░░██║██╔══██╗  ████╗░████║██╔══██╗████╗░████║  ██║██╔════╝  ██╔════╝░██╔══██╗╚██╗░██╔╝
    ██║░░░██║██████╔╝  ██╔████╔██║██║░░██║██╔████╔██║  ██║╚█████╗░  ██║░░██╗░███████║░╚████╔╝░
    ██║░░░██║██╔══██╗  ██║╚██╔╝██║██║░░██║██║╚██╔╝██║  ██║░╚═══██╗  ██║░░╚██╗██╔══██║░░╚██╔╝░░
        ╚██████╔╝██║░░██║  ██║░╚═╝░██║╚█████╔╝██║░╚═╝░██║  ██║██████╔╝  ╚██████╔╝██║░░██║░░░██║░░░
    ░╚═════╝░╚═╝░░╚═╝  ╚═╝░░░░░╚═╝░╚════╝░╚═╝░░░░░╚═╝  ╚═╝╚═════╝░  ░╚═════╝░╚═╝░░╚═╝░░░╚═╝░░░

*/

using namespace pros;
using namespace pros::c;

using namespace controls;

Controller master (E_CONTROLLER_MASTER);

void screen_pose() {
    Imu inertial_sensor(INERTIAL_PORT);

    while (true) {
        lemlib::Pose pose = auton::get_pose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
        pros::lcd::print(3, "deg: %f", inertial_sensor.get_yaw()); // print the heading
        pros::lcd::print(4, "deg: %f", inertial_sensor.get_pitch()); // print the heading
        pros::delay(10);
    }
}

void auton_print() {
    while (true) {
        pros::lcd::print(0, "SELECTED: %d", sec::auton); // print the x position
        pros::delay(10);
    }
}   

float motor_temp(int port) {
    return motor_get_temperature(port);
}

void temps() {
    while (true) {
        pros::lcd::print(0, "left front: %.0f", motor_temp(L_DRIVE_FRONT));
        pros::lcd::print(1, "left mid: %.0f", motor_temp(L_DRIVE_MID));
        pros::lcd::print(2, "left back: %.0f", motor_temp(L_DRIVE_BACK));
        pros::lcd::print(3, "right front: %.0f", motor_temp(R_DRIVE_FRONT));
        pros::lcd::print(4, "right mid: %.0f", motor_temp(R_DRIVE_MID));
        pros::lcd::print(5, "right back: %.0f", motor_temp(R_DRIVE_BACK));

        pros::lcd::print(6, "intake: %.0f", motor_temp(HOOKS));
        pros::lcd::print(7, "arm: %.0f", motor_temp(ARM));
        pros::delay(10);
    }
}

void initialize() {
    lcd::initialize();
    sec::init();

    auton::calibrate_drivetrain();

    auton::skills();

    pros::Task intake_task([=]() {
        screen_pose();
    });
}

void disabled() {}

void competition_initialize() {}

// "Blue WP (+)","Blue Elims (+)","Blue (-)","\n","Red WP (+)","Red Elims (+)","Red (-)","\n","None","Skills"
void autonomous() {
    switch (sec::auton) {
        case 0:
            auton::blue_plus_side_awp();
            break;
        case 1:
            auton::blue_plus_side_sweep();
            break;
        case 2:
            auton::blue_negative();
            break;
        case 3:
            auton::red_plus_side_awp();
            break;
        case 4:
            auton::red_plus_side_sweep();
            break;
        case 5:
            auton::red_negative();
            break;
        case 6:
            break;
        case 7:
            auton::skills();
            break;
    }
}

void opcontrol() {
    auto mogo = ADIDigitalOut(MOGO);
    auto corner_arm = ADIDigitalOut(CORNER_ARM);
    auto lift_intake = ADIDigitalOut(INTAKE_LIFT);
    bool mogoActive = true;

    motor_set_gearing(HOOKS, E_MOTOR_GEAR_BLUE);

    Motor a(ARM);
    Rotation r(ARM_ROT);

    Arm arm(&a, &r);

    std::unordered_map<controller_digital_e_t, std::function<void()>> toggle_controls;
    std::unordered_map<controller_digital_e_t, std::pair<std::function<void(bool)>, std::function<void()>>> hold_controls;
    std::unordered_set<controller_digital_e_t> held;

    mogo.set_value(mogoActive);

    toggle_controls.emplace(E_CONTROLLER_DIGITAL_RIGHT, [&]() {
        mogoActive = !mogoActive;
        mogo.set_value(mogoActive);
    });

    hold_controls.emplace(E_CONTROLLER_DIGITAL_R1, std::make_pair(
        [&](bool firstActivation) {
            motor_move(HOOKS, 127);
        },
        [&]() {
            motor_brake(HOOKS);
        }
    ));

    hold_controls.emplace(E_CONTROLLER_DIGITAL_R2, std::make_pair(
        [&](bool firstActivation) {
            motor_move(HOOKS, -127);
        },
        [&]() {
            motor_brake(HOOKS);
        }
    ));

    toggle_controls.emplace(E_CONTROLLER_DIGITAL_L2, [&]() {
        arm.toggle_l2();
    });

    // on hold, the arm acts as a normal arm
    hold_controls.emplace(E_CONTROLLER_DIGITAL_L1, std::make_pair(
        [&](bool firstActivation) {
            if (firstActivation) return;
            arm.hold_l1();  
        },
        [&]() {
            arm.release_l1();
        }
    ));

    hold_controls.emplace(E_CONTROLLER_DIGITAL_L2, std::make_pair(
        [&](bool firstActivation) {
            if (firstActivation) return;
            arm.hold_l2();
        },
        [&]() {
            arm.release_l2();
        }
    ));

    hold_controls.emplace(E_CONTROLLER_DIGITAL_Y, std::make_pair(
        [&](bool firstActivation) {
            corner_arm.set_value(true);
        },
        [&]() {
            corner_arm.set_value(false);
        }
    ));

    hold_controls.emplace(E_CONTROLLER_DIGITAL_B, std::make_pair(
        [&](bool firstActivation) {
            raise_intake();
        },
        [&]() {
            lower_intake();
        }
    ));

    while (true) {
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        auton::arcade(leftY, rightX);

        // if arm PID is enabled recalculate the error and set voltage based off PID output
        arm.tick(held);

        for (auto control : toggle_controls) {
            if (master.get_digital_new_press(control.first) && !held.contains(control.first)) {
                control.second();
            }
        }

        for (auto control : hold_controls) {
            if (master.get_digital(control.first)) {
                control.second.first(!held.contains(control.first));
                held.insert(control.first);
            } else if (held.contains(control.first)) {
                control.second.second();
                held.erase(control.first);
            }
        }
        
        double battery = battery_get_capacity();

        auto drivetrainMotors = {L_DRIVE_FRONT, L_DRIVE_MID, L_DRIVE_BACK, R_DRIVE_FRONT, R_DRIVE_MID, R_DRIVE_BACK};

        double temperatureSum = 0.0;
        double hotspot = 0.0;
        int hotspotPort = 0;

        for (int port : drivetrainMotors) {
            double currentTemp = motor_get_temperature(port);

            temperatureSum += currentTemp;
            
            if (hotspot < currentTemp) {
                hotspot = currentTemp;
                hotspotPort = port;
            }
        }

        double averageTemperature = temperatureSum / drivetrainMotors.size();

        temperatureSum = 0.0;

        master.print(0, 0, "Battery: %.0f%c", battery, 37);
        // pros::delay(0.1);
        // master.print(1, 0, "Temps: %.0f°C", averageTemperature);
        // pros::delay(0.1);
        // master.print(2, 0, "Port %d: %.0f°C", hotspotPort, hotspot);

        pros::delay(1);
    }
}
