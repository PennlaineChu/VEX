#pragma once
#include "mock_vex.h"

// Mock robot configuration - mirrors your actual robot-config.h
using namespace vex;

// Global brain instance
extern brain Brain;

// Drive motors
extern motor L1, L2, L3;
extern motor R1, R2, R3;

// Other motors
extern motor intake;
extern motor intakedown;
extern motor hang1;

// Sensors
extern inertial Inertial;
extern optical Optical;
extern optical Optical_go;

// Controller
extern controller Controller1;

// Pneumatics
extern digital_out pushCylinder;
extern digital_out intakeCylander;
extern digital_out redlight;
extern digital_out whitelight;
extern digital_out shooter;
extern digital_out aligner;

// Vision sensors
extern vision Vision1;
extern vision Vision2;

// Competition object
extern competition Competition;

// VEXcode init function
void vexcodeInit(void);
