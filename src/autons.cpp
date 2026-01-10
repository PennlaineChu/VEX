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
  
  
     // ========== CORRECT COORDINATE MAP ==========
  // Field: 140.41" × 140.41" (0,0) is at top left corner, (140.41, 140.41) is at bottom right
  // Robot: 12.5" wide × 18" long (inertia at center: 6.25" from sides, 9" from front/back)
  // START: (88, 118.3) - Actual starting position on field (robot center)
  // Coordinate system: Top-left origin (0,0), 0° = -Y (up/forward), 90° = +X (right), 180° = +Y (down), 270° = -X (left)
  
  
  // Set initial pose: X, Y, and heading (0° = up/forward, 90° = right, 180° = down, 270° = left)
  set_robot_pose(86, 116.3, 0.0); // Robot starts facing up/forward (0°)
  intakedown.spin(forward, 12, volt);
  // ========== PHASE 1: Drive to intake position ==========
  // driveToXY automatically calculates and turns to face the target
  wait(0.05, sec);
  driveToXY(100, 105, 8.0, 8.0); //條歪一點 讓他可以三顆球吸起來
  wait(0.05, sec);
  intake.spin(forward, 10, volt);

  // ========== PHASE 2: Score 3 blocks ==========
  driveToXY(93.5, 95, 6.0, 6.0); // Approach slowly
  wait(0.1, sec);
  chassis.drive_distance(15, 315, 12.0, 6.0); // Drive to low goal
  intakedown.spin(reverse, 6, volt);
  intake.spin(reverse, 12, volt);
  wait(2, sec); //Score blocks 
  intakedown.stop();
  intake.stop();  
  // ========== PHASE 3: back up and load ==========
  chassis.drive_distance(-10, 315, 12, 6); // Backup
  wait(0.05, sec);
  driveToXAtHeading(115, 135, 8.0, 6.0); //to loader with x at 115, heading 135
  wait(0.05, sec);
  chassis.turn_to_angle(180); // Prepare to load
  intakeCylander.set(true);
  wait(0.1, sec);
  shooter.set(true); 
  intakedown.spin(forward, 10, volt);
  intake.spin(forward, 12, volt); 
  chassis.drive_distance(15, 180, 10.0, 4.0); // into goal
  wait(1, sec); // load
  intake.stop();
  wait(0.05, sec);
  
  // ========== PHASE 4: Back up and score long goal ==========
  chassis.drive_distance(-8, 180, 12.0, 4.0); 
  wait(0.05, sec);
  intakeCylander.set(false);
  //pushCylinder.set(false);
  chassis.turn_to_angle(2);
  wait(0.05, sec);
  
  chassis.drive_distance(15, 2, 10.0, 4.0); // to long goal prepare 
  //driveToXY(117.8, 103.7, 12.0, 6.0);
  pushCylinder.set(true); 
  wait(0.05, sec);
  //intake.spin(reverse, 6, volt);
  //intakedown.spin(reverse, 4, volt);
  intakedown.spin(forward, 10, volt);
  intake.spin(forward, 12, volt);
  wait(3, sec); // Score blocks
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
  // Set initial pose: X, Y, and heading (0° = up/forward, 90° = right, 180° = down, 270° = left)
  set_robot_pose(54.4, 116.3, 0.0); // Robot starts facing up/forward (0°)
  intakedown.spin(forward, 10, volt);
  
  // ========== PHASE 1: Drive to intake position ==========
  // driveToXY automatically calculates and turns to face the target
  wait(0.05, sec);
  driveToXY(36.4, 104, 8.0, 6.0); //should be safe/not too close to long goal
  wait(0.05, sec);
  

  // ========== PHASE 2: Score 3 blocks ==========
  chassis.turn_to_angle(45);
  intake.spin(forward, 10, volt);
  wait(0.05, sec);
  driveToXY(47, 92.5, 8.0, 6.0); // Approach slowly
  wait(0.1, sec);
  chassis.drive_distance(13, 47, 8.0, 6.0); // Drive to upper goal
  wait(0.05, sec);
  pushCylinder.set(true); 
  chassis.drive_distance(1.75, 47, 8, 6);
  wait(0.4, sec); //Score blocks 
  intakedown.stop();
  wait(0.05, sec);
  intake.stop(); 
  wait(0.05, sec); 
  // ========== PHASE 3: back up and load ==========
  pushCylinder.set(false); 
  chassis.drive_distance(-10, 45, 12, 6); // Backup
  wait(0.05, sec);
  intake.stop();
  driveToXY(26, 116, 10.0, 6.0); //to loader at X=25 while maintaining heading 225°
  chassis.turn_to_angle(180); // Prepare to load
  intakeCylander.set(true);
  wait(0.2, sec);
  intakedown.spin(forward, 10, volt);
  intake.spin(forward, 10, volt); 
  chassis.drive_distance(15, 180, 8.0, 4.0); // into goal
  wait(0.3, sec); // load
  intake.stop();
  wait(0.05, sec);
  
  // ========== PHASE 4: Back up and score long goal ==========
  chassis.drive_distance(-10, 180, 10.0, 4.0); 
  wait(0.05, sec);
  intakeCylander.set(false);
  chassis.turn_to_angle(355);
  wait(0.05, sec);
  chassis.drive_distance(14, 360, 8.0, 4.0); // to long goal prepare 
  intake.spin(reverse, 5, volt);
  shooter.set(true);
  pushCylinder.set(true);
  wait(0.1, sec);
  intake.spin(forward, 12, volt);
  intakedown.spin(forward, 10, volt);
  wait(3, sec); // Score blocks
  intake.stop();
  intakedown.stop();
  pushCylinder.set(false);
  
  // Display final position for debugging
  RobotPose final = get_robot_pose();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("End X:%.1f Y:%.1f", final.x, final.y);

}

//--------------------------------------------------------

void Right_7()
{

  vex::color selectedTeamColor = vex::color::red;
  set_robot_pose(86, 116.3, 0.0); // Robot starts facing up/forward (0°)
  intakedown.spin(forward, 10, volt);
  // ========== PHASE 1: Drive to intake position ==========
  // driveToXY automatically calculates and turns to face the target
  wait(0.05, sec);
  driveToXY(93.5, 93.5, 9.0, 8.0);
  wait(0.05, sec);
  intake.spin(forward, 12, volt);

  // ========== PHASE 2: intake 1 block ==========
  wait(0.05, sec);
  driveToXY(115, 76, 9.0, 6.0); // go intake, make sure don't cross alliance line
  wait(0.1, sec);
  driveToXYBackward(93.5, 93.5, 10.0, 6.0); // Drive to low goal backward
  /*intakedown.spin(reverse, 5, volt);
  intake.spin(reverse, 6, volt);
  wait(0.05, sec);
  chassis.turn_to_angle(320);
  chassis.drive_distance(13.5, 320, 10.0, 6.0); // Drive to low goal
  intakedown.spin(reverse, 7, volt);
  intake.spin(reverse, 12, volt);
  //wait(0.01, sec);
  chassis.drive_distance(2.5, 320, 10, 6);
  wait(0.01, sec);//Score blocks
  intakedown.stop();
  intake.stop();
  // ========== PHASE 3: back up and load ==========
  chassis.drive_distance(-10, 320, 10.0, 6); // Backup
  wait(0.05, sec);*/
  driveToXAtHeading(115, 135, 10.0, 8.0); //to loader with x at 115, heading 135
  chassis.turn_to_angle(180); // Prepare to load
  wait(0.1, sec);
  intakeCylander.set(true);
  intakedown.spin(forward, 12, volt);
  intake.spin(forward, 12, volt); 
  shooter.set(true);
  wait(0.2, sec);
  chassis.drive_distance(13.5, 180, 8.0, 4.0); // into goal
  // wait load
  intake.stop();
  wait(0.05, sec);
  // ========== PHASE 4: Back up and score long goal ==========
  chassis.drive_distance(-10, 180, 10.0, 4.0); 
  wait(0.05, sec);
  intakeCylander.set(false);
  chassis.turn_to_angle(5);
  wait(0.05, sec);
  chassis.drive_distance(14, 3, 10.0, 6.0); // to long goal prepare 
  intake.spin(reverse, 5, volt);
  wait(0.05, sec);
  pushCylinder.set(true);
  wait(0.05, sec);
  intake.spin(forward, 12, volt);
  intakedown.spin(forward, 10, volt);
  wait(3, sec); // Score blocks
  intake.stop();
  intakedown.stop();
  pushCylinder.set(false);

  // Display final position for debugging
  RobotPose final = get_robot_pose();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("End X:%.1f Y:%.1f", final.x, final.y);
  

}

//--------------------------------------------------------

void Left_7()
{
  vex::color selectedTeamColor = vex::color::red;
  set_robot_pose(54.4, 116.3, 0.0); // Robot starts facing up/forward (0°)
  intakedown.spin(forward, 12, volt);
  // ========== PHASE 1: Drive to intake position ==========
  // driveToXY automatically calculates and turns to face the target
  driveToXY(48, 92.5, 9.0, 8.0);
  wait(0.1, sec);
  intake.spin(forward, 12, volt);
  // ========== PHASE 2: intake 1 block ==========
  wait(0.05, sec);
  driveToXY(26, 75.5, 9.0, 6.0); // go intake, make sure don't cross alliance line
  wait(0.2, sec);
  driveToXYBackward(47, 93.5, 10.0, 6.0); // Drive to low goal backward
  /*intakedown.spin(reverse, 5, volt);
  intake.spin(reverse, 6, volt);
  wait(0.05, sec);
  chassis.turn_to_angle(320);
  chassis.drive_distance(13.5, 320, 10.0, 6.0); // Drive to low goal
  intakedown.spin(reverse, 7, volt);
  intake.spin(reverse, 12, volt);
  //wait(0.01, sec);
  chassis.drive_distance(2.5, 320, 10, 6);
  wait(0.01, sec);//Score blocks
  intakedown.stop();
  intake.stop();  
  // ========== PHASE 3: back up and load ==========
  chassis.drive_distance(-10, 320, 10.0, 6); // Backup
  wait(0.05, sec);*/
  driveToXAtHeading(24, 230, 10.0, 8.0); //to loader with x at 25.5, heading 110
  chassis.turn_to_angle(180); // Prepare to load
  wait(0.1, sec);
  intakeCylander.set(true);
  shooter.set(true);
  wait(0.2, sec);
  chassis.drive_distance(12, 180, 8.0, 4.0); // into goal
  // wait load
  intake.stop();
  wait(0.05, sec);
  // ========== PHASE 4: Back up and score long goal ==========
  chassis.drive_distance(-10, 180, 10.0, 4.0); 
  wait(0.05, sec);
  intakeCylander.set(false);
  chassis.turn_to_angle(355);
  wait(0.05, sec);
  chassis.drive_distance(14, 358, 10.0, 6.0); // to long goal prepare 
  intake.spin(reverse, 5, volt);
  wait(0.05, sec);
  pushCylinder.set(true);
  wait(0.05, sec);
  intake.spin(forward, 12, volt);
  intakedown.spin(forward, 10, volt);
  wait(3, sec); // Score blocks
  intake.stop();
  intakedown.stop();
  pushCylinder.set(false);

  // Display final position for debugging
  RobotPose final = get_robot_pose();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("End X:%.1f Y:%.1f", final.x, final.y);
}

//--------------------------------------------------------

void Solo()
{
  set_robot_pose(86.4, 117.5, 90); // Robot starts facing left (270°)
  // ========== PHASE 1: Load and score 3 from right loader ==========
  chassis.drive_distance(31.1, 80, 12.0, 6.0); //to 26, 116.5
  wait(0.05, sec);
  intakedown.spin(forward, 10, volt);
  intake.spin(forward, 10, volt);
  chassis.turn_to_angle(180); //face long goal
  intakeCylander.set(true); 
  shooter.set(true);
  wait(0.15, sec);
  chassis.drive_distance(18, 180, 9.0, 6.0); // into loader 1
  wait(.3, sec); //load 6 blocks
  chassis.drive_distance(-9, 180, 12.0, 6.0); // backup
  intakeCylander.set(false);
  wait(0.05, sec);
  chassis.turn_to_angle(0);
  wait(0.05, sec);
  
  chassis.drive_distance(13.5, 0, 10.0, 4.0); // to long goal prepare 
  
  wait(0.05, sec);
  shooter.set(true); 
  pushCylinder.set(true); 
  wait(0.05, sec);
  chassis.drive_distance(4, 0, 6, 6);
  wait(0.5, sec); // Score blocks
  intake.stop();
  pushCylinder.set(false);
  chassis.drive_distance(-7, 0, 6, 6); // backup

  // ========== PHASE 2: Score 6 to left upper goal ==========
  intake.spin(forward, 12, volt);
  driveToXY(110, 110, 8.0, 6.0);
  wait(0.05, sec);
  driveToXY(92, 92, 8.0, 6.0);
  wait(0.05, sec);
  driveToXY(46, 94.4, 8.0, 6.0);
  wait(0.15, sec);
  chassis.turn_to_angle(45);
  wait(0.05, sec);
  chassis.drive_distance(12, 45, 8.0, 6.0); // Drive to upper goal
  wait(0.05, sec);
  pushCylinder.set(true); 
  chassis.drive_distance(1.5, 45, 8, 6);
  wait(0.5, sec); //Score blocks 
  intake.stop(); 
  wait(0.05, sec); 
  pushCylinder.set(false); 
  chassis.drive_distance(-10, 45, 8, 6); // Backup
  wait(0.05, sec);

  // ========== PHASE 3: load and score 3 from left loader==========
  driveToXAtHeading(25, 225, 8.0, 6.0); //to loader at X=25 while maintaining heading 225°
  chassis.turn_to_angle(180); // Prepare to load
  intakedown.spin(forward, 10, volt);
  intakeCylander.set(true);
  intake.spin(forward, 10, volt); 
  chassis.drive_distance(15, 180, 6.0, 4.0); // into goal
  wait(0.3, sec); // load
  intake.stop();
  wait(0.05, sec);
  
  chassis.drive_distance(-12, 180, 4.0, 4.0);  //backup from loader
  wait(0.05, sec);
  intakeCylander.set(false);
  chassis.turn_to_angle(0);
  intakedown.stop();
  intake.stop();
  wait(0.05, sec);
  
  chassis.drive_distance(5, -3, 4.0, 4.0); // to long goal prepare 
  
  wait(0.05, sec);
  shooter.set(true); 
  //start to score
  chassis.drive_distance(7.5, 0, 6, 6);
  pushCylinder.set(true); 
  wait(0.05, sec);
  intakedown.spin(forward, 10, volt);
  intake.spin(forward, 10, volt);
  chassis.drive_distance(2.5, 0, 6, 6);
  wait(1.5, sec); // Score blocks
  pushCylinder.set(false);
  intake.stop();
  intakedown.stop();


}
//--------------------------------------------------------

void Skills()
{
  // Set initial pose: X, Y, and heading (0° = up/forward, 90° = right, 180° = down, 270° = left)
  // Robot is physically facing left, so set heading to 270°
  set_robot_pose(54, 117.5, 270); // Robot starts facing left (270°)
  // ========== PHASE 1: Load first 6 balls ==========
  //chassis.drive_distance(32.1, 280, 8.0, 6.0); //to 26, 116.5
  driveToXY(24, 116.5, 10.0, 6.0);
  wait(0.05, sec);
  intakedown.spin(forward, 12, volt);
  intake.spin(forward, 12, volt);
  chassis.turn_to_angle(182); //face long goal
  wait(0.05, sec);
  intakeCylander.set(true);
  shooter.set(true);
  wait(0.1, sec);
  chassis.drive_distance(17, 182, 12.0, 6.0); // into loader 1
  wait(1, sec); //load 6 blocks
  chassis.drive_distance(-9, 180, 12.0, 6.0); // backup
  intakeCylander.set(false);
  wait(0.05, sec);
  chassis.turn_to_angle(270); //for crossing
  // ========== PHASE 3: cross the field to score in long goal ==========
  driveToXY(8.1, 90, 8.0, 6.0); //cross the field
  wait(0.05, sec);
  intake.stop();
  intakedown.stop();
  driveToXY(10, 37, 8.0, 6.0); 
  wait(0.05, sec);
  driveToXY(18.4, 24.9, 8.0, 6.0); //to long goal
  wait(0.05, sec);
  chassis.turn_to_angle(183); //face long goal
  wait(0.05, sec);
  chassis.drive_distance(15.5, 181, 8.0, 4.0); // to long goal prepare 
  wait(0.05, sec);
  pushCylinder.set(true); 
  intakedown.spin(reverse, 12, volt);
  intake.spin(reverse, 12, volt);
  wait(0.1, sec);
  intakedown.spin(forward, 12, volt);
  intake.spin(forward, 14, volt);
  wait(0.1, sec);
  chassis.drive_distance(2.5, 182, 8, 6);
  wait(1, sec);
  pushCylinder.set(false); 
  chassis.drive_distance(-7, 180, 10.0, 4.0); // backup
  wait(0.05, sec);
  // ========== PHASE 4: Loader 2 and score 6 ==========
  chassis.turn_to_angle(2); //face loader
  wait(0.05, sec);
  chassis.drive_distance(5, 0, 10, 10);
  wait(0.05, sec);
  intakeCylander.set(true);
  wait(0.05, sec);
  chassis.drive_distance(16, 0, 11.0, 6.0); // into loader 2
  wait(0.7, sec); //load 6 blocks
  chassis.drive_distance(-12, 0, 10.0, 6.0); // backup
  intakeCylander.set(false);
  chassis.turn_to_angle(182); //face long goal
  wait(0.05, sec);
  chassis.drive_distance(8, 182, 8.0, 4.0); // to long goal prepare 
  wait(0.05, sec);
  pushCylinder.set(true); 
  chassis.drive_distance(2, 182, 8.0, 4.0);
  intakedown.spin(reverse, 12, volt);
  intake.spin(reverse, 12, volt);
  wait(0.2, sec);
  intakedown.spin(forward, 12, volt);
  intake.spin(forward, 14, volt);
  wait(3, sec); //score
  pushCylinder.set(false); 
  chassis.drive_distance(-10, 180, 8.0, 4.0); // backup
  wait(0.05, sec);
  chassis.drive_distance(10.4, 180, 15.0, 12.0); // push a bit
  wait(0.05, sec);
  chassis.drive_distance(-7, 180, 8.0, 4.0); // backup
  wait(0.05, sec);

  // ========== PHASE 4: Loader 3 ==========
  driveToXY(116.8, 25, 10.0, 6.0); //dont know if it would touch blue parking
  wait(0.05, sec);
  chassis.turn_to_angle(2); //face loader 3
  wait(0.05, sec);
  intakeCylander.set(true);
  wait(0.05, sec);
  chassis.drive_distance(21, 0, 10.0, 6.0); // into loader 3  
  wait(0.7, sec); //load 6 blocks
  chassis.drive_distance(-12, 0, 8.0, 6.0); // backup
  intakeCylander.set(false);
  chassis.turn_to_angle(90); //for crossing
  wait(0.05, sec);

  // ========== PHASE 5: Drive across the field to score in long goal ==========
  
  driveToXY(131, 48, 8.0, 6.0); // cross the field
  wait(0.05, sec);
  intake.stop();
  intakedown.stop();
  driveToXY(126.4, 104.4, 8.0, 6.0); 
  wait(0.05, sec);
  driveToXY(119, 115.5, 8.0, 6.0); // to long goal
  wait(0.05, sec);
  chassis.turn_to_angle(2); //face long goal
  wait(0.05, sec);
  chassis.drive_distance(22, 2, 8.0, 4.0); // to long goal prepare 
  wait(0.05, sec);
  pushCylinder.set(true); 
  intakedown.spin(reverse, 10, volt);
  intake.spin(reverse, 10, volt);
  wait(0.1, sec);
  intakedown.spin(forward, 10, volt);
  intake.spin(forward, 10, volt);
  wait(0.1, sec);
  chassis.drive_distance(2.5, 0, 6, 6);
  wait(0.8, sec);
  chassis.drive_distance(-6, 0, 6.0, 4.0); // backup
  wait(0.05, sec);
  chassis.drive_distance(7, 0, 15.0, 12.0); // push a bit
  wait(0.05, sec);
  chassis.drive_distance(-6, 0, 6.0, 4.0); // backup
  wait(0.05, sec);
  pushCylinder.set(false);

  // ========== PHASE 6: Loader 4 and score 6 ==========
  chassis.turn_to_angle(190); //face loader
  intakeCylander.set(true);
  chassis.drive_distance(19, 182, 10.0, 6.0); // into loader 4
  wait(1.5, sec); //load 6 blocks
  chassis.drive_distance(4, 182, 8.0, 6.0); // backup
  chassis.drive_distance(-12, 180, 8.0, 6.0); // backup
  intakeCylander.set(false);
  chassis.turn_to_angle(2); //face long goal
  wait(0.05, sec);
  chassis.drive_distance(7, 2, 6.0, 4.0); // to long goal prepare 
  wait(0.05, sec);
  pushCylinder.set(true); 
  chassis.drive_distance(5, 2, 8.0, 4.0);
  intakedown.spin(reverse, 12, volt);
  intake.spin(reverse, 12, volt);
  wait(0.2, sec);
  intakedown.spin(forward, 12, volt);
  intake.spin(forward, 14, volt);
  wait(3, sec); //score
  pushCylinder.set(false); 
  chassis.drive_distance(-7, 0, 6.0, 4.0); // backup
  wait(0.05, sec);
  intake.stop();
  intakedown.stop();
  pushCylinder.set(false);

  // ========== PHASE 7: parking ==========
  chassis.turn_to_angle(45); //backward face parking zone
  driveToXYBackward(100, 135, 8.0, 6.0); // to parking zone
  wait(0.05, sec);
  chassis.turn_to_angle(85); //face parking zone
  wait(0.05, sec);
  chassis.drive_distance(-45, 95, 12.0, 12.0); // into parking zone
  wait(0.05, sec);
  chassis.drive_distance(5, 90, 12.0, 12.0); // adjust position in parking zone
  wait(0.1, sec);
  chassis.drive_distance(-20, 90, 12.0, 12.0); // adjust position in parking zone
  wait(0.05, sec);
  chassis.drive_distance(5, 90, 12.0, 12.0); // adjust position in parking zone

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
