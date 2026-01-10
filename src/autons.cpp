#include "vex.h"
#include "note.h"

void cos_move_distance_smooth(double distance_in, double angle_deg, double turn_maxV, double drive_maxV);

bool downTaskForward = true;  // true = forward, false = reverse
bool upTaskForward   = true;  // true = forward, false = reverse

void driveTime(int ms)
{
  chassis.drive_with_voltage(12, 12);
  wait(ms, msec);
  chassis.DriveL.stop(brake);
  chassis.DriveR.stop(brake);
}


int info()
{
  while (1)
  {
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(0, 50, "X: %f", chassis.get_X_position()); 
    Brain.Screen.printAt(0, 70, "Y: %f", chassis.get_Y_position());
    Brain.Screen.printAt(0, 90, "Heading: %f", chassis.get_absolute_heading());
    Brain.Screen.printAt(0, 110, "ForwardTracker: %f", chassis.get_ForwardTracker_position());
    Brain.Screen.printAt(0, 130, "SidewaysTracker: %f", chassis.get_SidewaysTracker_position());
    task::sleep(20);
  }
  return 1;
}

void Right_43()
{
}
//----------------------------------------------------------------------

void Left_43()
{

}

//--------------------------------------------------------

void Right_7()
{

}

//--------------------------------------------------------

void Left_7()
{
}

//--------------------------------------------------------

void Solo()
{

}
//--------------------------------------------------------

void Skills()
{
  // Set PID constants
  default_constants();
  
  // Set starting position for chassis odometry
  chassis.set_coordinates(0, 0, 0);
  
  // Debug: Print raw rotation sensor values and converted positions
  // This will help verify if the unit conversion is correct
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Before movement:");
  Brain.Screen.setCursor(2, 1);
  Brain.Screen.print("Pos: X=%.2f Y=%.2f", chassis.get_X_position(), chassis.get_Y_position());
  
  // Use holonomic_turn_to_angle for turning in place with holonomic drive
  chassis.holonomic_drive_to_point(20, 20, 90);
  
  // Debug: Print final position
  Brain.Screen.setCursor(3, 1);
  Brain.Screen.print("After movement:");
  Brain.Screen.setCursor(4, 1);
  Brain.Screen.print("Pos: X=%.2f Y=%.2f", chassis.get_X_position(), chassis.get_Y_position());
  Brain.Screen.setCursor(5, 1);
  Brain.Screen.print("Expected: X=0.00 Y=10.00");
  Brain.Screen.setCursor(6, 1);
  Brain.Screen.print("If Y >> 10, units wrong!");

}

//--------------------------------------------------------
void blank1()
{ 
  // TUNING TEST PATH for set_drive_constants (HOLONOMIC DRIVE)
  // Set your constants here to test them
  chassis.set_drive_constants(10, 0.5, 0.005, 4, 20);
  chassis.set_heading_constants(10, 0.25, 0.007, 5, 20);
  
  // Set starting position
  chassis.set_coordinates(0, 0, 0);
  
  // Test 1: Drive forward 24 inches (Y+ direction, facing 0°)
  chassis.holonomic_drive_to_point(0, 24, 0);
  wait(500, msec);
  
  // Test 2: Drive backward 24 inches (return to start)
  chassis.holonomic_drive_to_point(0, 0, 0);
  wait(500, msec);
  
  // Test 3: Drive right 24 inches (X+ direction, facing 90°)
  chassis.holonomic_drive_to_point(24, 0, 90);
  wait(500, msec);
  
  // Test 4: Drive left 24 inches (return to start)
  chassis.holonomic_drive_to_point(0, 0, 0);
  wait(500, msec);
  
  // Test 5: Diagonal movement (X+, Y+)
  chassis.holonomic_drive_to_point(12, 12, 45);
  wait(500, msec);
  
  // Test 6: Return to start
  chassis.holonomic_drive_to_point(0, 0, 0);
}
//--------------------------------------------------------
void blank2()
{
  
}
//-------------------------------------------------------- 
void blank3()
{
  
}
//--------------------------------------------------------


void B_17022A()
{
  vex::color selectedTeamColor = vex::color::blue;
  task notetask(autonoteTask, 0);
  chassis.set_drive_constants(10, .5, 0.005, 4, 20);
  chassis.set_heading_constants(10, .25, 0.007, 5, 20);
  chassis.turn_to_angle(316);
  //--------------------------------------------------------
  chassis.set_drive_constants(12, 2.0, 0.005, 5, 5);
  chassis.set_heading_constants(12, 1.5, 0.005, 5, 10);
  chassis.drive_distance(3, 316, 6, 6);
  //----------------------------------------------------------------------

  // Reset motor positions
  L1.resetPosition(); L2.resetPosition(); L3.resetPosition();
  R1.resetPosition(); R2.resetPosition(); R3.resetPosition();
  set_robot_pose(0, 0, 0);

  vex::task odomTask(odometry_task);

  
  driveToXY(0, 20, 6, 1);
  wait(2, sec);
  turnToXY(10, 20, 4.0);
  wait(2, sec);
  driveToXY(10, 20, 6, 1);
  
}
//-------------------------------------------------------- 
// Removed duplicate blank3() function
//--------------------------------------------------------


// Removed duplicate B_17022A() function - using the one defined earlier

void odom_test()
{
}

void PIDtest()
{
}

void default_constants()
{
  chassis.set_drive_constants(12, 0.5, 0.005, 0, 10);
  chassis.set_heading_constants(12, 0.7, 0.007, 5, 40);
  chassis.set_turn_constants(12, .35, .001, 3, 90);
  chassis.set_swing_constants(12, .5, .001, 2, 15);
  chassis.set_drive_exit_conditions(.5, 0, 2000);
  chassis.set_turn_exit_conditions(4, 0, 3000);
  chassis.set_swing_exit_conditions(10, 0, 3000);
}

void odom_constants()
{
  default_constants();
  chassis.drive_max_voltage = 12;
  chassis.drive_settle_error = 3;
}

void drive_test()
{
  chassis.turn_to_angle(45);
  chassis.drive_distance(30);
  chassis.turn_to_angle(270);
  chassis.drive_distance(20);
  chassis.turn_to_angle(135);
  chassis.drive_distance(30);
  chassis.turn_to_angle(270);
  chassis.drive_distance(20);
  chassis.turn_to_angle(0);
}

void turn_test()
{
  chassis.turn_to_angle(5);
  chassis.turn_to_angle(30);
  chassis.turn_to_angle(90);
  chassis.turn_to_angle(225);
  chassis.turn_to_angle(0);
}

void swing_test()
{
  chassis.left_swing_to_angle(90);
  chassis.right_swing_to_angle(0);
}

void full_test()
{
  chassis.drive_distance(24);
  chassis.turn_to_angle(-45);
  chassis.drive_distance(-36);
  chassis.right_swing_to_angle(-90);
  chassis.drive_distance(24);
  chassis.turn_to_angle(0);
}

void tank_odom_test()
{
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.turn_to_point(24, 24);
  chassis.drive_to_point(24, 24);
  chassis.drive_to_point(0, 0);
  chassis.turn_to_angle(0);
}

void holonomic_odom_test()
{
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.holonomic_drive_to_point(0, 18, 90);
  chassis.holonomic_drive_to_point(18, 0, 180);
  chassis.holonomic_drive_to_point(0, 18, 270);
  chassis.holonomic_drive_to_point(0, 0, 0);
}
