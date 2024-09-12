#include "main.h"
#include "pid.hpp"
#include "controls.hpp"
#include "pros/misc.h"
#include "pros/motors.h"


#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <math.h>


#include "s.hpp"
#include "liblvgl/lvgl.h"


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
       pros::lcd::print(0, "a: %d", sec::auton); // print the x position
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

   pros::Task intake_task([=]() {
       auton_print();
   });
}


void disabled() {}


void competition_initialize() {}


void autonomous() {
   switch (sec::auton) {
   }
}


float arm_pos() {
   Rotation armRotation(ARM_ROT);


   float measure = armRotation.get_angle() / 100.0f;


   if (measure > 350 || measure < 10) return 0;


   if (measure < 360) return measure;
  
   return measure - 360;
}


void opcontrol() {
   auto mogo = ADIDigitalOut(MOGO);
   auto corner_arm = ADIDigitalOut(CORNER_ARM);
   auto lift_intake = ADIDigitalOut(INTAKE_LIFT);
   bool mogoActive = false;


   motor_set_gearing(HOOKS, E_MOTOR_GEAR_BLUE);


   Motor arm = Motor(ARM, MotorGears::red, MotorUnits::degrees);
   Rotation armRotation(ARM_ROT);


   armRotation.reset_position();


   arm.set_brake_mode_all(motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);
   arm.tare_position();


   std::unordered_map<controller_digital_e_t, std::function<void()>> toggle_controls;
   std::unordered_map<controller_digital_e_t, std::pair<std::function<void(bool)>, std::function<void()>>> hold_controls;
   std::unordered_set<controller_digital_e_t> held;


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


   // initialize arm PID
   lib::PID armPid(
       1.5, // kP
       50, // kI
       10000 // kD
   );
   // arm targets
   const float rest = 0;
   const float load = 30;
   const float limit = 125;


   // arm error and target
   float armError = 0;
   float armTarget = rest;
   bool useArmPid = true;
   bool canControl = true;


   armPid.set_winduprange(5);
   armPid.reset();


   // on single press, set arm target to primed/rest position and enable PID
   toggle_controls.emplace(E_CONTROLLER_DIGITAL_L1, [&]() {
       armTarget = load;
       useArmPid = true;
   });


   toggle_controls.emplace(E_CONTROLLER_DIGITAL_L2, [&]() {
       armTarget = rest;
       useArmPid = true;
   });


   // on hold, the arm acts as a normal arm
   hold_controls.emplace(E_CONTROLLER_DIGITAL_L1, std::make_pair(
       [&](bool firstActivation) {
           if (firstActivation) return;
           if (!canControl) return;
           useArmPid = false;
           arm.move(127);
       },
       [&]() {
           arm.brake();
           armTarget = load;
           useArmPid = true;
       }
   ));


   hold_controls.emplace(E_CONTROLLER_DIGITAL_L2, std::make_pair(
       [&](bool firstActivation) {
           if (firstActivation) return;
           useArmPid = false;
           arm.move(-127);
       },
       [&]() {
           arm.brake();
           armTarget = rest;
           useArmPid = true;
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
       if (useArmPid == true) {
           armError = armTarget - arm_pos();
           arm.move(armPid.calculate(armError));
       }


       // if the arm is at the code limit, try to reset it backwards to stop it
       if (arm_pos() >= limit && (held.contains(E_CONTROLLER_DIGITAL_L1) || held.contains(E_CONTROLLER_DIGITAL_L2))) {
           canControl = false;
           useArmPid = false;


           arm.move_velocity(0);
       } else {
           canControl = true;
       }
      
       if (arm_pos() < 1 && arm_pos() > 0) {
           armTarget = rest;


           arm.tare_position();
           armPid.reset();
       }


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
       pros::delay(1);
       master.print(1, 0, "Temps: %.0f°C", averageTemperature);
       pros::delay(1);
       master.print(2, 0, "Port %d: %.0f°C", hotspotPort, hotspot);


       pros::delay(1);
   }
}
