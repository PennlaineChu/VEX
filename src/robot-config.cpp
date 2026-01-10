#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// ========== Drive Motors (Not Used) ==========
motor L1 = motor(PORT3, ratio6_1, true);
motor L2 = motor(PORT3, ratio6_1, false);
motor L3 = motor(PORT3, ratio6_1, true);
motor R1 = motor(PORT3, ratio6_1, false);
motor R2 = motor(PORT3, ratio6_1, true);
motor R3 = motor(PORT3, ratio6_1, false);

// ========== IMU ==========
inertial Inertial = inertial(PORT18);

// ========== Controller ==========
controller Controller1 = controller(primary);

// ========== Intake Motors ==========
motor intake1 = motor(PORT2, ratio6_1, false);  
motor intake2 = motor(PORT11, ratio6_1, true);
motor shooterUpper = motor(PORT12, ratio6_1, true);
motor shooterLower = motor(PORT1, ratio6_1, false);

// ========== Distance Sensors ==========
distance DistanceLeft = distance(PORT16);
distance DistanceRight = distance(PORT6);
distance DistanceFront = distance(PORT14);
distance DistanceBack = distance(PORT15);

// ========== Optical Sensors ==========
optical OpticalFirst = optical(PORT4);
optical OpticalSecond = optical(PORT5);

// ========== Digital Outputs ==========
// ========== Tracking Wheel Rotation Sensors ==========
// NOTE: Tracking wheel rotation sensors are created by the Drive class constructor
// Do NOT define them here to avoid port conflicts
// The Drive class will create: R_LeftTracker, R_RightTracker, R_BackTracker

digital_out intakeCylinder = digital_out(Brain.ThreeWirePort.A);
digital_out blockCylinder = digital_out(Brain.ThreeWirePort.B);
digital_out trackingWheel = digital_out(Brain.ThreeWirePort.C);
digital_out redlight = digital_out(Brain.ThreeWirePort.D);
digital_out whitelight = digital_out(Brain.ThreeWirePort.E);
digital_out wing = digital_out(Brain.ThreeWirePort.F);


// 88
//  VEXcode generated functions
//  define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void)
{
  // nothing to initialize
}