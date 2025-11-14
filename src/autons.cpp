#include "vex.h"
#include "note.h"
#include "autoarm.h"
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
  // ========== CORRECTED COORDINATE MAP ==========
  // Field: 140.5" × 140.5"
  // Robot: 12.5" wide × 16" long (inertia at center: 6.25" from sides, 8" from front/back)
  // START: (22, 22.5) - Actual starting position on field (robot center)
  // Loader: (37.5, 8.25) - Loader position
  // Coordinate system: 0° = +Y (forward), 90° = +X (right), 180° = -Y, 270° = -X
  
  // ========== All coordinates for XY navigation ==========
  const double START_X = 22.0, START_Y = 22.5;
  
  // Phase 1: Drive to first position
  const double FIRST_POS_X = 41.05, FIRST_POS_Y = 33.5;
  
  // Phase 2: Approach 3 blocks (318° = -42° from +Y axis)
  const double BLOCKS_3_APPROACH_X = 38.67, BLOCKS_3_APPROACH_Y = 36.61; // 4" before blocks at 318°
  const double BLOCKS_3_X = 36.36, BLOCKS_3_Y = 38.70;                   // 3 blocks location
  
  // Phase 3: Score at middle goal
  const double MIDDLE_GOAL_APPROACH_X = 22.33, MIDDLE_GOAL_APPROACH_Y = 54.15; // 5" before goal
  const double MIDDLE_GOAL_X = 19.63, MIDDLE_GOAL_Y = 57.28;                   // Middle goal
  
  // Phase 4: Back up from goal and go to loader
  const double AFTER_BACKUP_X = 24.98, AFTER_BACKUP_Y = 51.34; // 8" backup from goal
  const double LOADER_X = 37.5, LOADER_Y = 8.25;               // Loader position
  
  // Phase 5: Load 3 blocks (push backward into loader at 178°)
  const double LOADER_ENTRY_X = 37.5, LOADER_ENTRY_Y = 16.25;  // 8" in front of loader
  const double LOADER_DEEP_X = 37.5, LOADER_DEEP_Y = 8.25;     // Deep in loader (29" total push)
  const double AFTER_BACKUP2_X = 37.5, AFTER_BACKUP2_Y = 15.25; // 7" backed out
  
  // Phase 6: Score at goal
  const double GOAL_APPROACH_X = 37.5, GOAL_APPROACH_Y = 27.25; // 5" before goal
  const double GOAL_X = 37.5, GOAL_Y = 32.25;                   // Final goal
  
  // Initialize position tracking with actual start position
  set_robot_pose(START_X, START_Y, 0.0);
  
  // ========== PHASE 1: Drive to first position ==========
  intakedown.spin(forward, 12, volt);
  driveToXY(FIRST_POS_X, FIRST_POS_Y, 8.0, 4.0);
  wait(0.05, sec);
  
  // ========== PHASE 2: Approach and intake 3 blocks ==========
  driveToXY(BLOCKS_3_APPROACH_X, BLOCKS_3_APPROACH_Y, 4.0, 4.0); // Approach slowly
  wait(0.05, sec);
  driveToXY(BLOCKS_3_X, BLOCKS_3_Y, 4.0, 4.0); // Drive onto blocks
  wait(0.05, sec);
  
  // ========== PHASE 3: Score at middle goal ==========
  driveToXY(MIDDLE_GOAL_APPROACH_X, MIDDLE_GOAL_APPROACH_Y, 8.0, 4.0); // Approach goal
  wait(0.05, sec);
  intake.spin(reverse, 12, volt);
  driveToXY(MIDDLE_GOAL_X, MIDDLE_GOAL_Y, 6.0, 4.0); // Push into goal
  intakedown.spin(reverse, 9, volt);
  wait(1.5, sec); // Outtake 3 blocks onto goal
  intake.stop();
  wait(0.05, sec);
  
  // ========== PHASE 4: Back up and go to loader ==========
  driveToXY(AFTER_BACKUP_X, AFTER_BACKUP_Y, 4.0, 4.0); // Back away from goal
  wait(0.05, sec);
  pushCylinder.set(false);
  wait(0.05, sec);
  
  driveToXY(LOADER_ENTRY_X, LOADER_ENTRY_Y, 10.0, 4.0); // Navigate to loader entry
  wait(0.05, sec);
  intakedown.spin(forward, 12, volt);
  wait(0.05, sec);
  
  // ========== PHASE 5: Load 3 blocks at loader ==========
  // Turn to face loader (backward = 180°)
  turnToXY(LOADER_DEEP_X, LOADER_DEEP_Y, 4.0);
  wait(0.05, sec);
  shooter.set(true);
  wait(0.05, sec);
  
  // Drive backward into loader (driveToXY will go backward since target is behind)
  driveToXY(LOADER_DEEP_X, LOADER_DEEP_Y, 10.0, 4.0);
  wait(0.05, sec);
  intakeCylander.set(true);
  wait(0.05, sec);
  
  // Load 3 blocks
  intake.spin(forward, 12, volt);
  wait(0.5, sec);
  intake.stop();
  wait(0.05, sec);
  
  // Back away from loader
  driveToXY(AFTER_BACKUP2_X, AFTER_BACKUP2_Y, 6.0, 4.0); // Drive forward out of loader
  intakeCylander.set(false);
  shooter.set(true);
  wait(0.05, sec);
  
  // ========== PHASE 6: Score at goal ==========
  driveToXY(GOAL_APPROACH_X, GOAL_APPROACH_Y, 8.0, 4.0); // Approach goal
  wait(0.05, sec);
  pushCylinder.set(true);
  wait(0.05, sec);
  
  driveToXY(GOAL_X, GOAL_Y, 6.0, 4.0); // Final push into goal
  wait(0.05, sec);
  
  intake.spin(forward, 12, volt);
  intakedown.spin(forward, 12, volt);
  wait(2.5, sec); // Score 3 blocks into goal
  intake.stop();
  intakedown.stop();
  
  // Display final position for debugging
  RobotPose final = get_robot_pose();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("End X:%.1f Y:%.1f", final.x, final.y);
}
//----------------------------------------------------------------------

void Left_43()
{
  chassis.set_drive_constants(12, 2.0, 0.005, 2, 10);
  chassis.set_heading_constants(12, 1.5, 0.005, 2, 10);
  chassis.turn_to_angle(300);
  wait(0.05,sec);
  intakedown.spin(forward, 12, volt);
  //intake.spin(forward, 12, volt);
  chassis.drive_distance(20, 300, 6, 6);
  wait(.05,sec);
  chassis.turn_to_angle(43); 
  chassis.drive_distance(4, 43, 4, 4);//preintake
  intake.spin(forward, 12, volt);
  chassis.drive_distance(3, 43, 4, 4);//intake 3 balls
  //shooter.set(true);
  wait(0.05,sec);
  chassis.drive_distance(22, 43, 6, 6);//middle goals
  pushCylinder.set(true);
  chassis.drive_distance(5, 43, 6, 6);//middle goals
  wait(1.5,sec);//outake middle goals
  intake.stop();
  wait(0.05,sec);
  chassis.drive_distance(-8, 43, 4, 4);//backback
  wait(0.05,sec);
  pushCylinder.set(false);
  wait(0.05,sec);
  chassis.turn_to_angle(240);
  wait(0.05,sec);
  chassis.drive_distance(32, 240, 10, 10);//to the loader
  wait(0.05,sec);
  chassis.turn_to_angle(182);
  wait(0.05,sec);
  chassis.drive_distance(12, 182, 10, 10);//into loader
  wait(0.05,sec);
  intakeCylander.set(true);
  wait(0.05,sec);
  intake.spin(forward, 12, volt);
  chassis.drive_distance(16, 182, 8, 8);
  wait(.5,sec);
  intake.stop();
  wait(.05,sec);
  chassis.drive_distance(-7, 182, 6, 6);
  intakeCylander.set(false);
  shooter.set(true);
  wait(0.05,sec);
  chassis.turn_to_angle(5); // to long goal
  wait(0.05,sec);
  chassis.drive_distance(12, 5, 6, 6);
  pushCylinder.set(true);
  wait(0.05,sec);
  shooter.set(true);
  wait(0.05,sec);
  cos_move_distance_smooth(4, 5, 6, 6);
  wait(0.05,sec);
  intake.spin(forward, 12, volt);
  wait(0.05,sec);
  intakedown.spin(forward, 12, volt);
  wait(2.5,sec);
  intake.stop();
  intakedown.stop();
}

//--------------------------------------------------------

void Right_7()
{
  vex::color selectedTeamColor = vex::color::red;

  cos_move_distance_smooth(16.5, 60, 6, 6); // mirrored angle
  wait(0.05,sec);
  intakedown.spin(forward, 11, volt);
  wait(0.05,sec);
  chassis.turn_to_angle(315); // mirrored angle
  wait(0.05,sec);
  cos_move_distance_smooth(17, 315, 5, 6);//intake 3 balls
  wait(0.8,sec);
  intakedown.spin(forward, 0, volt);
  cos_move_distance_smooth(12, 315, 8, 6);//middle goals
  wait(0.05,sec);
  intakedown.spin(reverse, 8, volt);
  wait(8,sec);
  cos_move_distance_smooth(-10, 315, 6, 6);//backback (mirrored)


}

//--------------------------------------------------------

void Left_7()
{
  vex::color selectedTeamColor = vex::color::red;
}

//--------------------------------------------------------

void Solo()
{
  vex::color selectedTeamColor = vex::color::red;
  chassis.set_drive_constants(12, 2.0, 0.005, 2, 10);
  chassis.set_heading_constants(12, 1.5, 0.005, 2, 10);
  chassis.turn_to_angle(300);
  wait(0.05,sec);
  intakedown.spin(forward, 12, volt);
  //intake.spin(forward, 12, volt);
  chassis.drive_distance(20, 300, 6, 6);
  wait(.05,sec);
  chassis.turn_to_angle(43); 
  chassis.drive_distance(4, 43, 4, 4);//preintake
  intake.spin(forward, 12, volt);
  chassis.drive_distance(3, 43, 4, 4);//intake 3 balls
  //shooter.set(true);
  wait(0.05,sec);
  chassis.drive_distance(22, 43, 6, 6);//middle goals
  pushCylinder.set(true);
  chassis.drive_distance(5, 43, 6, 6);//middle goals
  wait(1.5,sec);//outake middle goals
  intake.stop();
  wait(0.05,sec);
  chassis.drive_distance(-10, 43, 4, 4);//backback
  wait(0.05,sec);
  pushCylinder.set(false);
  wait(0.05,sec);

  chassis.turn_to_angle(110);
  wait(0.05,sec);
  chassis.drive_distance(50, 110, 10, 10);//to 2nd pile
  chassis.turn_to_angle(319); 
  chassis.drive_distance(4, 319, 4, 4);//preintake
  wait(0.05,sec);
  chassis.drive_distance(3, 319, 4, 4);//intake 3 balls
  //shooter.set(true);
  wait(0.05,sec);
  chassis.drive_distance(22, 319, 6, 6);//middle goals
  intake.spin(reverse, 12, volt);
  chassis.drive_distance(5, 319, 6, 6);//middle goals
  intakedown.spin(reverse, 9, volt);
  wait(1.5,sec);//outake middle goals
  intake.stop();
  wait(0.05,sec);
  chassis.drive_distance(-8, 319, 4, 4);//backback
  wait(0.05,sec);
  pushCylinder.set(false);

}
//--------------------------------------------------------

void Skills()
{
  chassis.set_drive_constants(12, 2.0, 0.005, 2, 10);
  chassis.set_heading_constants(12, 1.5, 0.005, 2, 10);
  chassis.turn_to_angle(300);
  wait(0.05,sec);
  intakedown.spin(forward, 12, volt);
  wait(0.05,sec);
  intake.spin(forward, 12, volt);
  chassis.drive_distance(20, 300, 6, 6);
  wait(.05,sec);
  chassis.turn_to_angle(43); 
  wait(0.05,sec);
  chassis.drive_distance(7.5, 43, 5, 5);//intake 3 balls
  wait(.3,sec);
  //shooter.set(true);
  pushCylinder.set(true);
  wait(0.05,sec);
  chassis.drive_distance(25.5, 43, 6, 6);//middle goals
  intake.spin(forward, 7, volt);
  wait(2.5,sec);//outake middle goals
  intake.stop();
  wait(0.05,sec);
  chassis.drive_distance(-15, 43, 4, 4);//backback
  wait(0.05,sec);
  pushCylinder.set(false);
  wait(0.05,sec);
  chassis.turn_to_angle(110);
  wait(0.05,sec);
  chassis.drive_distance(65, 110, 4, 4);//turn right
  wait(0.05,sec);
  chassis.turn_to_angle(300);
  wait(0.05,sec);
  chassis.drive_distance(10, 300, 4, 4);
  wait(.5,sec);
  intakedown.spin(reverse, 12, volt);
  wait(0.05,sec);
  intake.spin(reverse, 12, volt);
  wait(0.05,sec);
  chassis.drive_distance(-10, 300, 4, 4);
  wait(0.05,sec);
  chassis.turn_to_angle(3);
  wait(0.05,sec);
  chassis.drive_distance(60, 3, 6, 6);
  wait(0.05,sec);
  chassis.turn_to_angle(280);
  wait(0.05,sec);
  chassis.drive_distance(60, 280, 6, 6);
  wait(0.05,sec);
  chassis.turn_to_angle(180);
  pushCylinder.set(true);
  wait(0.05,sec);
  shooter.set(true);
  chassis.drive_distance(14, 180, 6, 6);
  

}
//--------------------------------------------------------
// NOTE: recordPath() is defined in main.cpp (lines 371-403)
// It's called from the autonomous selector (main.cpp line 1310)
//--------------------------------------------------------
void blank1()
{
  odom_test();
  wait(5, sec);
}
//--------------------------------------------------------
void blank2()
{
  // Set starting position and heading to 0
  L1.resetPosition(); L2.resetPosition(); L3.resetPosition();
  R1.resetPosition(); R2.resetPosition(); R3.resetPosition();
  set_robot_pose(22, 22.5, 0);

  vex::task odomTask(odometry_task);

  
  driveToXY(22, 54, 6, 1);
  wait(0.05, sec);
  turnToXY(-22, 80, 3);
  wait(0.05, sec);
  driveToXY(-22, 80, 6, 1);
  wait(0.05, sec);
  turnToXY(-13, 88, 3);
  wait(0.05, sec);
  driveToXY(-13, 88, 6, 1);
  
}
//-------------------------------------------------------- 
void blank3()
{
  // ========== SIMPLE MOTOR TEST - DO NOT DELETE ==========
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(10, 20, "Motor Test Starting...");
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("Motor Test");
  wait(1, sec);
  
  // Test: Drive forward for 3 seconds at 8 volts
  Brain.Screen.printAt(10, 40, "Driving FORWARD...");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("Forward 8V...");
  
  L1.spin(forward, 8, volt);
  L2.spin(forward, 8, volt);
  L3.spin(forward, 8, volt);
  R1.spin(forward, 8, volt);
  R2.spin(forward, 8, volt);
  R3.spin(forward, 8, volt);
  wait(3, sec);
  
  // Stop
  L1.stop(brake); 
  L2.stop(brake); 
  L3.stop(brake);
  R1.stop(brake); 
  R2.stop(brake); 
  R3.stop(brake);
  
  Brain.Screen.printAt(10, 60, "Stopped.");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("Stopped     ");
  wait(1, sec);
  
  // Test: Drive backward for 3 seconds
  Brain.Screen.printAt(10, 80, "Driving BACKWARD...");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("Backward 8V...");
  
  L1.spin(reverse, 8, volt);
  L2.spin(reverse, 8, volt);
  L3.spin(reverse, 8, volt);
  R1.spin(reverse, 8, volt);
  R2.spin(reverse, 8, volt);
  R3.spin(reverse, 8, volt);
  wait(3, sec);
  
  // Stop
  L1.stop(brake); 
  L2.stop(brake); 
  L3.stop(brake);
  R1.stop(brake); 
  R2.stop(brake); 
  R3.stop(brake);
  
  Brain.Screen.printAt(10, 100, "Test COMPLETE!");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("DONE!       ");
  Controller1.rumble("--");
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
  autoarm();
  //---------------------------------------------------------------------
  chassis.drive_distance(-3, 315, 6, 6); // 微退一點
  //---------------------------------------------------------------------
  chassis.set_drive_constants(12, 1.2, 0.005, 6, 20);
  chassis.set_heading_constants(12, 1.2, 0.007, 6, 20);
  chassis.drive_distance(-24.5, 290, 12, 12);
  chassis.set_drive_constants(12, 1, 0.005, 6, 5);
  chassis.set_heading_constants(12, 1, 0.007, 6, 5);
  chassis.drive_distance(-10, 290, 4.5, 4.5);
  intakeCylander = true;
  wait(0.4, sec);
  //-----------------------------------------------------------------
  chassis.set_drive_constants(12, 1, 0.005, 5, 20);
  chassis.set_heading_constants(12, 1, 0.005, 5, 20);
  chassis.turn_to_angle(180);
  //----------------------------------------------------------------
  intake.spin(forward, 12, volt);
  intakedown.spin(reverse, 12, volt);
  chassis.drive_distance(20, 180, 6, 6);
  wait(0.2, sec);
  chassis.drive_distance(-18, 180, 7, 7);
  wait(0.2, sec);
  chassis.turn_to_angle(135);
  chassis.set_drive_constants(4, 1, 0.005, 5, 20);
  chassis.set_heading_constants(4, 1, 0.005, 5, 20);
  chassis.drive_distance(13, 135, 4, 4);
  wait(0.3, sec);
  chassis.set_drive_constants(12, 1, 0.005, 5, 20);
  chassis.set_heading_constants(12, 1, 0.005, 5, 20);
  chassis.turn_to_angle(165);
  chassis.set_drive_constants(12, 1.2, 0.005, 5, 20);
  chassis.set_heading_constants(12, 1.2, 0.005, 5, 20);
  chassis.drive_distance(6, 165, 12, 12);
  //--------------eat 2 blue end ----------------
  chassis.set_drive_constants(12, 1.4, 0.005, 5, 35);
  chassis.set_heading_constants(12, 1.4, 0.005, 5, 35);
  chassis.drive_distance(-15, 145, 12, 12);
  intake.stop(brake);
  chassis.turn_to_angle(50);
  intake.spin(forward, 12, volt);
  wait(1, sec);
  //--------------------- wait --------------------------------
  chassis.drive_distance(7, 50, 7, 7);
  intake.stop(brake);
  intakedown.stop(brake);
}

void odom_test()
{
  // Reset encoders and set starting pose
  L1.resetPosition(); L2.resetPosition(); L3.resetPosition();
  R1.resetPosition(); R2.resetPosition(); R3.resetPosition();
  set_robot_pose(0, 0, 0);
  
  // Start odometry task
  vex::task odomTask(odometry_task);
  wait(0.5, sec);
  
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("ODOM TEST");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("Moving forward...");
  wait(1, sec);
  
  // Test 1: Drive forward 20" at 0° (should increase Y)
  driveToXY(0, 20, 6, 1);
  
  RobotPose pose1 = get_robot_pose();
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("After forward 20in:");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("X:%.2f Y:%.2f H:%.1f", pose1.x, pose1.y, pose1.heading);
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("Expected: X:0 Y:20");
  wait(3, sec);
  
  // Test 2: Turn to 90° and drive right 20" (should increase X)
  turnToXY(20, 20, 3);
  wait(0.5, sec);
  driveToXY(20, 20, 6, 1);
  
  RobotPose pose2 = get_robot_pose();
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("After right 20in:");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("X:%.2f Y:%.2f H:%.1f", pose2.x, pose2.y, pose2.heading);
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("Expected: X:20 Y:20");
  wait(3, sec);
  
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("TEST COMPLETE");
}

void PIDtest()
{
}

void default_constants()
{
  chassis.set_drive_constants(12, .5, 0.004, 5, 20);
  chassis.set_heading_constants(12, .25, 0.007, 5, 40);
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
