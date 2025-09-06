#include "mock_robot_config.h"

using namespace vex;

// Global brain instance
brain Brain;

// Drive motors - matching your actual configuration
motor L1 = motor(PORT1, true);   // reversed
motor L2 = motor(PORT2, false);
motor L3 = motor(PORT3, true);   // reversed
motor R1 = motor(PORT7, false);
motor R2 = motor(PORT8, true);   // reversed
motor R3 = motor(PORT9, false);

// Other motors
motor intake = motor(PORT14, true);      // reversed
motor intakedown = motor(PORT11, false);
motor hang1 = motor(PORT19, true);       // reversed

// Sensors
inertial Inertial = inertial(PORT12);
optical Optical = optical(PORT7);
optical Optical_go = optical(PORT16);

// Controller
controller Controller1 = controller();

// Pneumatics
digital_out pushCylinder = digital_out();
digital_out intakeCylander = digital_out();
digital_out redlight = digital_out();
digital_out whitelight = digital_out();
digital_out shooter = digital_out();
digital_out aligner = digital_out();

// Vision sensors
vision Vision1 = vision(PORT13, 50);
vision Vision2 = vision(PORT15, 50);

// Competition object
competition Competition;

// VEXcode init function
void vexcodeInit(void) {
    // Initialize any necessary components
}
