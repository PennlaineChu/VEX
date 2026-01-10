#pragma once

using namespace vex;

// ========== VEX Brain ==========
extern brain Brain;

// ========== Drive Motors ==========
extern motor L1;
extern motor L2;
extern motor L3;
extern motor R1;
extern motor R2;
extern motor R3;

// ========== Other Motors ==========
extern motor intake1;
extern motor intake2;
extern motor shooterUpper;
extern motor shooterLower;

// ========== Sensors ==========
extern optical OpticalFirst;
extern optical OpticalSecond;
extern inertial Inertial;
extern controller Controller1;
extern vex::vision Vision1;
extern vex::vision Vision2;

// ========== Tracking Wheel Rotation Sensors ==========
// NOTE: Tracking wheel rotation sensors are created by the Drive class
// Access them via: chassis.R_LeftTracker, chassis.R_RightTracker, chassis.R_BackTracker

// ========== Digital Outputs (Pneumatics/Lights) ==========
extern digital_out wing;
extern digital_out redlight;
extern digital_out whitelight;
extern digital_out intakeCylinder;
extern digital_out blockCylinder;
extern digital_out trackingWheel;

// ========== Initialization ==========
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
