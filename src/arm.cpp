// #include "arm.hpp"
// #include "pros/misc.h"

// using namespace pros;

// float Arm::arm_pos() {
//     Rotation armRotation(ARM_ROT);

//     float measure = armRotation.get_position() / 100.0f;

//     if (measure > 350 || measure < 10) return 0;

//     if (measure < 360) return measure;
//     return measure - 360;
// }