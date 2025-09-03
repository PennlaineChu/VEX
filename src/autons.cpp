#include "vex.h"
#include "note.h"
#include "autoarm.h"
void cos_move_distance_smooth(double distance_in, double angle_deg, double turn_maxV, double drive_maxV);

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

void RW_right()
{

  vex::color selectedTeamColor = vex::color::red;

  //--------------------------------------------------------
  chassis.set_drive_constants(12, 2.0, 0.005, 2, 10);
  chassis.set_heading_constants(12, 1.5, 0.005, 2, 10);
  cos_move_distance_smooth(25, 0, 10, 10);
  wait(0.05,sec);
  chassis.turn_to_angle(90);
  cos_move_distance_smooth(25, -90, 10, 10);
  //----------------------------------------------------------------------
 
}

void R3_right()
//=================================3顆===================================================
{

  vex::color selectedTeamColor = vex::color::red;
  chassis.set_drive_constants(12, .5, 0.005, 4, 20);
  chassis.set_heading_constants(12, .25, 0.007, 5, 20);
  chassis.turn_to_angle(313);
  //---------------------------------------------------------------------
  chassis.set_drive_constants(10, 1.2, 0.005, 6, 10);
  chassis.set_heading_constants(10, 1.2, 0.007, 6, 10);
  chassis.drive_distance(-34, 290, 7, 7);
  intakeCylander = true;
  wait(0.4, sec);
  //----------------------------------------------------------------
  task notetask(autonoteTask, 0);
  //----------------------------------------------------------------
  chassis.set_drive_constants(12, 1, 0.005, 5, 20);
  chassis.set_heading_constants(12, 1, 0.005, 5, 20);
  chassis.turn_to_angle(190);
  //----------------------------------------------------------------
  intake.spin(forward, 12, volt);
  intakedown.spin(reverse, 12, volt);
  chassis.drive_distance(19, 190, 8, 8);
  chassis.turn_to_angle(337);
  //------next---------
  chassis.set_drive_constants(12, 1.2, 0.005, 6, 20);
  chassis.set_heading_constants(12, 1.2, 0.007, 6, 20);
  chassis.drive_distance(33, 337, 12, 12);
  chassis.set_drive_constants(12, 1, 0.005, 6, 5);
  chassis.set_heading_constants(12, 1, 0.007, 6, 5);
  chassis.drive_distance(28, 337, 3, 3);
  chassis.drive_distance(-24, 340, 12, 12);
  wait(0.4, sec);
  chassis.turn_to_angle(90);
  intakedown.stop(brake);
  chassis.drive_distance(24, 75, 12, 12);
  intake.stop(brake);
  hang1.stop(brake);
}
void RW_left()
{
  wait(200, msec);
  vex::color selectedTeamColor = vex::color::red;
  task notetask(autonoteTask, 0);
  chassis.set_drive_constants(12, .5, 0.005, 4, 20);
  chassis.set_heading_constants(12, .25, 0.007, 5, 20);
  chassis.turn_to_angle(48);
  //--------------------------------------------------------
  chassis.set_drive_constants(12, 2.0, 0.005, 5, 5);
  chassis.set_heading_constants(12, 1.5, 0.005, 5, 10);
  chassis.drive_distance(3.5, 48, 10, 10);
  //----------------------------------------------------------------------
  autoarm();
  //----------------------------------------------------------------------
  chassis.drive_distance(-3, 48, 10, 10); // 微退一點
  //---------------------------------------------------------------------
  chassis.set_drive_constants(12, 1.2, 0.005, 6, 20);
  chassis.set_heading_constants(12, 1.2, 0.007, 6, 20);
  chassis.drive_distance(-24.5, 71, 12, 12);
  hang1.spin(forward, 12, volt);
  chassis.set_drive_constants(12, 1, 0.005, 6, 5);
  chassis.set_heading_constants(12, 1, 0.007, 6, 5);
  chassis.drive_distance(-10, 70, 5, 5);
  intakeCylander = true;
  hang1.stop(coast);
  wait(0.4, sec);
  //-----------------------------------------------------------------
  chassis.set_drive_constants(12, 0.8, 0.005, 5, 10);
  chassis.set_heading_constants(12, 0.8, 0.005, 5, 10);
  chassis.turn_to_angle(180);
  //----------------------------------------------------------------
  intake.spin(forward, 12, volt);
  intakedown.spin(reverse, 12, volt);
  chassis.drive_distance(20, 180, 8, 8);
  wait(0.2, sec);
  chassis.drive_distance(-16.5, 180, 8, 8);
  wait(0.2, sec);
  chassis.turn_to_angle(225);
  //--------------mid 2 red -----------------
  chassis.set_drive_constants(4, 1, 0.005, 5, 20);
  chassis.set_heading_constants(4, 1, 0.005, 5, 20);
  chassis.drive_distance(12, 225, 4, 4);
  wait(0.3, sec);
  chassis.set_drive_constants(12, 1, 0.005, 5, 20);
  chassis.set_heading_constants(12, 1, 0.005, 5, 20);
  chassis.turn_to_angle(195);
  chassis.set_drive_constants(12, 1.2, 0.005, 5, 20);
  chassis.set_heading_constants(12, 1.2, 0.005, 5, 20);
  chassis.drive_distance(6, 195, 12, 12);
  //--------------eat 2 red end----------------
  chassis.set_drive_constants(12, 1.4, 0.005, 5, 35);
  chassis.set_heading_constants(12, 1.4, 0.005, 5, 35);
  chassis.drive_distance(-20, 223, 12, 12);
  // intake.stop(coast);
  // chassis.turn_to_angle(36);
  intake.spin(forward, 12, volt);
  intakedown.spin(reverse, 12, volt);
}
void R5_left()
{
  vex::color selectedTeamColor = vex::color::red;
  task notetask(autonoteTask, 0);
  chassis.set_drive_constants(12, 0.5, 0.005, 4, 20);
  chassis.set_heading_constants(12, 0.25, 0.007, 5, 20);
  chassis.turn_to_angle(71);
  //---------------------------------------------------------------------
  chassis.set_drive_constants(12, 1.2, 0.005, 6, 20);
  chassis.set_heading_constants(12, 1.2, 0.007, 6, 20);
  chassis.drive_distance(-23.5, 71, 12, 12);
  chassis.set_drive_constants(12, 1, 0.005, 6, 5);
  chassis.set_heading_constants(12, 1, 0.007, 6, 5);
  chassis.drive_distance(-10.5, 70, 5, 5);
  intakeCylander = true;
  hang1.stop(coast);
  wait(0.4, sec);
  //-----------------------------------------------------------------
  chassis.set_drive_constants(12, 0.8, 0.005, 5, 10);
  chassis.set_heading_constants(12, 0.8, 0.005, 5, 10);
  chassis.turn_to_angle(180);
  //----------------------------------------------------------------
  intake.spin(forward, 12, volt);
  intakedown.spin(reverse, 12, volt);
  chassis.drive_distance(20, 180, 8, 8);
  wait(0.2, sec);
  chassis.drive_distance(-16.5, 180, 8, 8);
  wait(0.2, sec);
  chassis.turn_to_angle(225);
  //--------------mid 2 red -----------------
  chassis.set_drive_constants(4, 1, 0.005, 5, 20);
  chassis.set_heading_constants(4, 1, 0.005, 5, 20);
  chassis.drive_distance(11, 225, 4, 4);
  wait(0.3, sec);
  chassis.set_drive_constants(12, 1, 0.005, 5, 20);
  chassis.set_heading_constants(12, 1, 0.005, 5, 20);
  chassis.turn_to_angle(195);
  chassis.set_drive_constants(12, 1.2, 0.005, 5, 20);
  chassis.set_heading_constants(12, 1.2, 0.005, 5, 20);
  chassis.drive_distance(7.5, 195, 12, 12);
  //--------------eat 2 red end----------------
  chassis.set_drive_constants(12, 1.4, 0.005, 5, 35);
  chassis.set_heading_constants(12, 1.4, 0.005, 5, 35);
  chassis.drive_distance(-19, 215, 12, 12);
  intake.stop(coast);
  chassis.turn_to_angle(35);
  intake.spin(forward, 12, volt);
  //--------------------- --------------------------------
  chassis.drive_distance(30, 35, 3, 3);
  chassis.drive_distance(-20, 35, 7, 7);
  chassis.drive_distance(-28, 135, 7, 7);
  intake.stop(brake);
  intakedown.stop(brake);
}
void BW_left()
{
  vex::color selectedTeamColor = vex::color::blue;
  task notetask(autonoteTask, 0);
  chassis.set_drive_constants(10, .5, 0.005, 4, 20);
  chassis.set_heading_constants(10, .25, 0.007, 5, 20);
  chassis.turn_to_angle(47.5);
  //--------------------------------------------------------
  chassis.set_drive_constants(12, 2.0, 0.005, 5, 5);
  chassis.set_heading_constants(12, 1.5, 0.005, 5, 10);
  chassis.drive_distance(3, 47.5, 10, 10);
  //----------------------------------------------------------------------
  autoarm();
  //-------------------------------------------------------------------
  chassis.drive_distance(-3, 45, 10, 10); // 微退一點
  //---------------------------------------------------------------------
  chassis.set_drive_constants(12, 1.2, 0.005, 6, 20);
  chassis.set_heading_constants(12, 1.2, 0.007, 6, 20);
  chassis.drive_distance(-22.5, 70, 12, 12);
  hang1.spin(forward, 12, volt);
  chassis.set_drive_constants(12, 1, 0.005, 6, 5);
  chassis.set_heading_constants(12, 1, 0.007, 6, 5);
  chassis.drive_distance(-10, 70, 5, 5);
  intakeCylander = true;
  hang1.stop(coast);
  wait(0.4, sec);
  //-----------------------------------------------------------------
  chassis.set_drive_constants(12, 1, 0.005, 5, 20);
  chassis.set_heading_constants(12, 1, 0.005, 5, 20);
  chassis.turn_to_angle(180);
  //----------------------------------------------------------------
  intake.spin(forward, 12, volt);
  intakedown.spin(reverse, 12, volt);
  chassis.drive_distance(16, 180, 8, 8);
  chassis.turn_to_angle(23);
  //------next--------------------------------
  chassis.set_drive_constants(12, 1.2, 0.005, 6, 20);
  chassis.set_heading_constants(12, 1.2, 0.007, 6, 20);
  chassis.drive_distance(36, 23, 12, 12);
  chassis.set_drive_constants(12, 1, 0.005, 6, 5);
  chassis.set_heading_constants(12, 1, 0.007, 6, 5);
  chassis.drive_distance(25, 23, 3.5, 3.5);
  //-------------------------------------------
  chassis.drive_distance(-37, 20, 12, 12);
  chassis.turn_to_angle(135);
  intakedown.stop(brake);
  chassis.drive_distance(-27, 135, 12, 12);
  intake.stop(brake);
  hang1.stop(brake);
}
//--------------------------------------------------------------
void B3_left()
{
  vex::color selectedTeamColor = vex::color::blue;
  task notetask(autonoteTask, 0);
  chassis.set_drive_constants(12, .5, 0.005, 4, 20);
  chassis.set_heading_constants(12, .25, 0.007, 5, 20);
  chassis.turn_to_angle(47);
  //--------------------------------------------------------
  chassis.set_drive_constants(12, 1.2, 0.005, 6, 20);
  chassis.set_heading_constants(12, 1.2, 0.007, 6, 20);
  chassis.drive_distance(-22.5, 70, 12, 12);
  chassis.set_drive_constants(12, 1, 0.005, 6, 5);
  chassis.set_heading_constants(12, 1, 0.007, 6, 5);
  chassis.drive_distance(-10, 70, 5, 5);
  intakeCylander = true;
  wait(0.4, sec);
  //-----------------------------------------------------------------
  chassis.set_drive_constants(12, 1, 0.005, 5, 20);
  chassis.set_heading_constants(12, 1, 0.005, 5, 20);
  chassis.turn_to_angle(180);
  //----------------------------------------------------------------
  intake.spin(forward, 12, volt);
  intakedown.spin(reverse, 12, volt);
  chassis.drive_distance(18, 180, 8, 8);
  chassis.turn_to_angle(23);
  //------next--------------------------------
  chassis.set_drive_constants(12, 1.2, 0.005, 6, 20);
  chassis.set_heading_constants(12, 1.2, 0.007, 6, 20);
  chassis.drive_distance(36, 23, 12, 12);
  chassis.set_drive_constants(12, 1, 0.005, 6, 5);
  chassis.set_heading_constants(12, 1, 0.007, 6, 5);
  chassis.drive_distance(25, 23, 3.5, 3.5);
  //--------------------- --------------------------------
  chassis.drive_distance(-33, 20, 12, 12);
  chassis.turn_to_angle(135);
  intakedown.stop(brake);

  hang1.spinToPosition(-40, deg, false);

  chassis.drive_distance(-25, 135, 12, 12);
  intake.stop(brake);
  hang1.stop(hold);
}

//--------------------------------------------------------------

void BW_right()
{
  wait(200, msec);
  vex::color selectedTeamColor = vex::color::blue;
  task notetask(autonoteTask, 0);
  chassis.set_drive_constants(10, .5, 0.005, 4, 20);
  chassis.set_heading_constants(10, .25, 0.007, 5, 20);
  chassis.turn_to_angle(316);
  //--------------------------------------------------------
  chassis.set_drive_constants(12, 2.0, 0.005, 5, 5);
  chassis.set_heading_constants(12, 1.5, 0.005, 5, 10);
  chassis.drive_distance(3, 316, 10, 10);
  //----------------------------------------------------------------------
  autoarm();
  //---------------------------------------------------------------------
  chassis.drive_distance(-3, 315, 10, 10); // 微退一點
  //---------------------------------------------------------------------
  chassis.set_drive_constants(12, 1.2, 0.005, 6, 20);
  chassis.set_heading_constants(12, 1.2, 0.007, 6, 20);
  chassis.drive_distance(-24.5, 290, 12, 12);
  hang1.spin(forward, 12, volt);
  chassis.set_drive_constants(12, 1, 0.005, 6, 5);
  chassis.set_heading_constants(12, 1, 0.007, 6, 5);
  chassis.drive_distance(-10, 290, 5, 5);
  intakeCylander = true;
  hang1.stop(coast);
  wait(0.4, sec);
  //-----------------------------------------------------------------
  chassis.set_drive_constants(12, 1, 0.005, 5, 20);
  chassis.set_heading_constants(12, 1, 0.005, 5, 20);
  chassis.turn_to_angle(180);
  //----------------------------------------------------------------
  intake.spin(forward, 12, volt);
  intakedown.spin(reverse, 12, volt);
  chassis.drive_distance(20, 180, 8, 8);
  wait(0.2, sec);
  chassis.drive_distance(-19.5, 180, 8, 8);
  wait(0.2, sec);
  chassis.turn_to_angle(135);
  chassis.set_drive_constants(4, 1, 0.005, 5, 20);
  chassis.set_heading_constants(4, 1, 0.005, 5, 20);
  chassis.drive_distance(14, 135, 4, 4);
  wait(0.3, sec);
  chassis.set_drive_constants(12, 1, 0.005, 5, 20);
  chassis.set_heading_constants(12, 1, 0.005, 5, 20);
  chassis.turn_to_angle(165);
  chassis.set_drive_constants(12, 1.2, 0.005, 5, 20);
  chassis.set_heading_constants(12, 1.2, 0.005, 5, 20);
  chassis.drive_distance(6, 165, 12, 12);
  //--------------eat 2 blue----------------
  chassis.set_drive_constants(12, 1.4, 0.005, 5, 35);
  chassis.set_heading_constants(12, 1.4, 0.005, 5, 35);
  chassis.drive_distance(-20, 145, 12, 12);
  // intake.stop(brake);
  // chassis.turn_to_angle(322);
  intake.spin(forward, 12, volt);
  intakedown.spin(reverse, 12, volt);
  /* chassis.drive_distance(28, 322, 3.5, 3.5);
   wait(0.5,sec);
   chassis.drive_distance(-20, 322, 12, 12);
   hang1.spinToPosition(-30,deg,false);
   chassis.drive_distance(-28, 225, 12, 12);
   intake.stop(brake);
   intakedown.stop(brake);
   hang1.stop(hold);*/
}
//--------------------------------------------------------------
void B5_right() // B_solo
{
  //---------blue--------carry---------------------------
  vex::color selectedTeamColor = vex::color::blue;
  chassis.set_drive_constants(12, .5, 0.005, 4, 20);
  chassis.set_heading_constants(12, .25, 0.007, 5, 20);
  chassis.turn_to_angle(313);
  //--------------------------------------------------------
  chassis.set_drive_constants(12, 2.0, 0.005, 5, 5);
  chassis.set_heading_constants(12, 1.5, 0.005, 5, 10);
  chassis.drive_distance(3.5, 313, 10, 10);
  //----------------------------------------------------------------------
  autoarm();
  //----------------------------------------------------------------------
  chassis.drive_distance(-3, 312, 10, 10); // 微退一點
  //---------------------------------------------------------------------
  chassis.set_drive_constants(12, 1.2, 0.005, 6, 20);
  chassis.set_heading_constants(12, 1.2, 0.007, 6, 20);
  chassis.drive_distance(-24.5, 290, 12, 12);
  hang1.spin(forward, 12, volt);

  chassis.set_drive_constants(12, 1, 0.005, 6, 5);
  chassis.set_heading_constants(12, 1, 0.007, 6, 5);
  chassis.drive_distance(-10, 290, 6, 6);
  intakeCylander = true;
  hang1.stop(coast);
  wait(0.2, sec);
  //-----------------------------------------------------------------
  intake.spin(forward, 12, volt);
  intakedown.spin(reverse, 12, volt);
  chassis.turn_to_angle(135);
  //--------------mid 2 red -----------------
  chassis.set_drive_constants(4, 1, 0.005, 5, 20);
  chassis.set_heading_constants(4, 1, 0.005, 5, 20);
  chassis.drive_distance(15, 135, 8, 8);
  chassis.set_drive_constants(12, 1.2, 0.005, 5, 20);
  chassis.set_heading_constants(12, 1.2, 0.005, 5, 20);
  chassis.drive_distance(-15, 135, 7, 7);
  //---eat inside
  wait(0.1, sec);
  chassis.turn_to_angle(180);
  chassis.drive_distance(18, 180, 12, 12);
  chassis.drive_distance(-30, 160, 10, 10);
  chassis.drive_distance(-33, 185, 12, 12);
  intakeCylander = false;
  chassis.drive_distance(5, 185, 12, 12);
  chassis.turn_to_angle(225);
  chassis.drive_distance(-18, 230, 8, 8);
  intakeCylander = true;
  wait(0.3, sec);
  chassis.turn_to_angle(0);
  chassis.drive_distance(15, 0, 12, 12);
  wait(0.2, sec);
  chassis.turn_to_angle(160);
  hang1.spinToPosition(-30, deg, false);
  chassis.drive_distance(28, 160, 7.5, 7.5);
  intake.stop(brake);
  intakedown.stop(brake);
  //---------blue--------carry---------------------------
}

void skills()//red solo
{
  //---------red--------carry---------------------------
  vex::color selectedTeamColor = vex::color::red;
  chassis.set_drive_constants(12, .5, 0.005, 4, 20);
  chassis.set_heading_constants(12, .25, 0.007, 5, 20);
  chassis.turn_to_angle(47);
  //--------------------------------------------------------
  chassis.set_drive_constants(12, 2.0, 0.005, 5, 5);
  chassis.set_heading_constants(12, 1.5, 0.005, 5, 10);
  chassis.drive_distance(3.5, 47, 10, 10);
  //----------------------------------------------------------------------
  autoarm();
  //----------------------------------------------------------------------
  chassis.drive_distance(-3, 48, 10, 10); // 微退一點
  //---------------------------------------------------------------------
  chassis.set_drive_constants(12, 1.2, 0.005, 6, 20);
  chassis.set_heading_constants(12, 1.2, 0.007, 6, 20);
  chassis.drive_distance(-24.5, 71, 12, 12);
  hang1.spin(forward, 12, volt);
  chassis.set_drive_constants(12, 1, 0.005, 6, 5);
  chassis.set_heading_constants(12, 1, 0.007, 6, 5);
  chassis.drive_distance(-10, 70, 5, 5);
  intakeCylander = true;
  hang1.stop(coast);
  wait(0.2, sec);
  //-----------------------------------------------------------------
  intake.spin(forward, 12, volt);
  intakedown.spin(reverse, 12, volt);
  chassis.turn_to_angle(225);
  //--------------mid 2 red -----------------
  chassis.set_drive_constants(4, 1, 0.005, 5, 20);
  chassis.set_heading_constants(4, 1, 0.005, 5, 20);
  chassis.drive_distance(14, 225, 8, 8);
  chassis.set_drive_constants(12, 1.2, 0.005, 5, 20);
  chassis.set_heading_constants(12, 1.2, 0.005, 5, 20);
  //---------------------------------
  chassis.drive_distance(-16, 225, 8, 8);
  //---------------------------------
  wait(0.1, sec);
  chassis.turn_to_angle(180);
  chassis.drive_distance(18, 180, 12, 12);
  wait(0.1, sec);
  chassis.drive_distance(-18, 200, 10, 10);
  chassis.drive_distance(-42, 185, 12, 12);
  intakeCylander = false;
  chassis.drive_distance(5, 185, 12, 12);
  chassis.turn_to_angle(135);
  chassis.drive_distance(-17, 135, 7, 7);
  intakeCylander = true;
  wait(0.3, sec);
  chassis.turn_to_angle(0);
  chassis.drive_distance(13, 0, 12, 12);
  wait(0.3, sec);
  chassis.turn_to_angle(200);
  chassis.drive_distance(27, 200, 7.5, 7.5);
  intake.stop(brake);
  intakedown.stop(brake);
  //---------red--------carry---------------------------
}
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
  chassis.drive_distance(3, 316, 10, 10);
  //----------------------------------------------------------------------
  autoarm();
  //---------------------------------------------------------------------
  chassis.drive_distance(-3, 315, 10, 10); // 微退一點
  //---------------------------------------------------------------------
  chassis.set_drive_constants(12, 1.2, 0.005, 6, 20);
  chassis.set_heading_constants(12, 1.2, 0.007, 6, 20);
  chassis.drive_distance(-24.5, 290, 12, 12);
  hang1.spin(forward, 12, volt);
  chassis.set_drive_constants(12, 1, 0.005, 6, 5);
  chassis.set_heading_constants(12, 1, 0.007, 6, 5);
  chassis.drive_distance(-10, 290, 4.5, 4.5);
  intakeCylander = true;
  hang1.stop(coast);
  wait(0.4, sec);
  //-----------------------------------------------------------------
  chassis.set_drive_constants(12, 1, 0.005, 5, 20);
  chassis.set_heading_constants(12, 1, 0.005, 5, 20);
  chassis.turn_to_angle(180);
  //----------------------------------------------------------------
  intake.spin(forward, 12, volt);
  intakedown.spin(reverse, 12, volt);
  chassis.drive_distance(20, 180, 8, 8);
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
  hang1.spinToPosition(-170, deg, true);
  chassis.drive_distance(7, 50, 7, 7);
  intake.stop(brake);
  intakedown.stop(brake);
  hang1.stop(hold);
}

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
