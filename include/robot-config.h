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
extern motor intake;
extern motor intakedown;

// ========== Sensors ==========
extern optical Optical;
extern optical Optical_go;
extern inertial Inertial;
extern controller Controller1;
extern vex::vision Vision1;
extern vex::vision Vision2;

// ========== Digital Outputs (Pneumatics/Lights) ==========
extern digital_out wing;
extern digital_out redlight;
extern digital_out whitelight;
extern digital_out intakeCylander;
extern digital_out pushCylinder;
extern digital_out shooter;

// ========== Initialization ==========
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
