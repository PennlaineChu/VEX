using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor L1;
extern motor L2;
extern motor L3;
extern motor R1;
extern motor R2;
extern motor R3;
extern optical Optical;
extern optical Optical_go;
extern inertial Inertial;
extern controller Controller1;
extern motor intake;
extern motor intakedown;
extern motor hang1;
extern digital_out redlight;
extern digital_out whitelight;
extern digital_out intakeCylander;
extern digital_out pushCylinder;
extern digital_out hookCylinder;
extern vex::vision Vision1;
extern vex::vision Vision2; 



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );