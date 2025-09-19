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

void R_right()
{
//--------------------------------------------------------
  vex::color selectedTeamColor = vex::color::red;

  chassis.set_drive_constants(12, 2.0, 0.005, 2, 10);
  chassis.set_heading_constants(12, 1.5, 0.005, 2, 10);

  //shooter.set(true);
  cos_move_distance_smooth(16.5, 60, 10, 10); 
  wait(0.05,sec);
  intakedown.spin(forward, 12, volt);
  wait(0.05,sec);
  chassis.turn_to_angle(315);
  wait(0.05,sec);
  cos_move_distance_smooth(15, 315, 6, 6);//intake 3 balls
  wait(1,sec);
  intakedown.stop();
  cos_move_distance_smooth(10, 315, 8, 6);//low goals
  wait(0.05,sec);

  intake.spin(reverse, 12, volt);
  wait(0.05,sec);
  intakedown.spin(reverse, 12, volt);
  wait(1.5,sec);
  //intake.stop();
  wait(0.05,sec);
  intakedown.stop();

  wait(0.05,sec);
  cos_move_distance_smooth(-10, 315, 10, 10);//backback
  wait(0.05,sec);
  //intakedown.spin(forward, 10, volt);

  intakedown.spin(forward, 12, volt);
  wait(0.05,sec);
  chassis.turn_to_angle(140);
  wait(0.05,sec);
  cos_move_distance_smooth(16, 145, 10, 10);//to the loader
  wait(0.05,sec);
  chassis.turn_to_angle(180);
  wait(0.05,sec);
  cos_move_distance_smooth(4.5, 180, 10, 10);//into loader not sure!!!
  wait(0.05,sec);
  intakeCylander.set(true);
  wait(0.05,sec);
  chassis.drive_distance(7, 180, 12, 12);
  wait(0.05,sec);
  chassis.drive_distance(-2, 180, 10, 10);
  wait(0.05,sec);
  chassis.drive_distance(4, 180, 10, 10);
  wait(1.5,sec);
  cos_move_distance_smooth(-5, 180, 10, 10);
  intakeCylander.set(false);
  aligner.set(true);
  shooter.set(true);
  chassis.turn_to_angle(10);
  wait(0.05,sec);
  chassis.drive_distance(10, 10, 10, 10);
  wait(0.05,sec);
  shooter.set(true);
  intake.stop();
  wait(0.05,sec);
  cos_move_distance_smooth(10, 0, 10, 10);
  wait(0.05,sec);
  intake.spin(reverse, 10, volt);
  wait(0.05,sec);
  intakedown.spin(forward, 10, volt);
}
//----------------------------------------------------------------------

void R_left()
{
  vex::color selectedTeamColor = vex::color::red;

  chassis.set_drive_constants(12, 2.0, 0.005, 2, 10);
  chassis.set_heading_constants(12, 1.5, 0.005, 2, 10);

  //shooter.set(true);
  cos_move_distance_smooth(14.5, 300, 10, 10); // mirrored angle
  wait(0.05,sec);
  intakedown.spin(forward, 12, volt);
  wait(0.05,sec);
  chassis.turn_to_angle(46); // mirrored angle
  wait(0.05,sec);
  cos_move_distance_smooth(15, 45, 6, 6);//intake 3 balls
  wait(0.8,sec);
  cos_move_distance_smooth(13.8, 45, 8, 6);//middle goals
  wait(0.05,sec);
  intake.spin(forward, 12, volt);
  wait(1.5,sec);
  intake.stop();
  wait(0.05,sec);
  cos_move_distance_smooth(-10, 46, 10, 10);//backback (mirrored)
  wait(0.05,sec);
  chassis.turn_to_angle(230); // mirrored angle
  wait(0.05,sec);
  cos_move_distance_smooth(14, 230, 10, 10);//to the loader
  wait(0.05,sec);
  chassis.turn_to_angle(180);
  wait(0.05,sec);
  cos_move_distance_smooth(5, 180, 10, 10);//into loader not sure!!!
  wait(0.05,sec);
  intakeCylander.set(true);
  wait(0.05,sec);
  chassis.drive_distance(8.5, 180, 12, 12);
  wait(0.05,sec);
  chassis.drive_distance(-2, 180, 10, 10);
  wait(0.05,sec);
  intake.spin(forward, 12, volt);
  wait(0.05,sec);
  chassis.drive_distance(5.5, 180, 10, 10);
  wait(0.05,sec);
  intake.stop();
  wait(1.5,sec);
  cos_move_distance_smooth(-5, 180, 10, 10);
  intakeCylander.set(false);
  aligner.set(true);
  shooter.set(true);
  chassis.turn_to_angle(5); // to long goal
  wait(0.05,sec);
  chassis.drive_distance(10, 5, 10, 10);
  wait(0.05,sec);
  shooter.set(true);
  intake.stop();
  wait(0.05,sec);
  cos_move_distance_smooth(10, 0, 10, 10);
  wait(0.05,sec);
  intake.spin(forward, 10, volt);
  wait(0.05,sec);
  intakedown.spin(forward, 10, volt);
}

//--------------------------------------------------------

void R_right_final()
{
  vex::color selectedTeamColor = vex::color::red;
}

//--------------------------------------------------------

void R_left_final()
{
  vex::color selectedTeamColor = vex::color::red;
}

//--------------------------------------------------------

void R_solo()
{
  vex::color selectedTeamColor = vex::color::red;
}
//--------------------------------------------------------

void B_right()
{
  vex::color selectedTeamColor = vex::color::blue;

  chassis.set_drive_constants(12, 2.0, 0.005, 2, 10);
  chassis.set_heading_constants(12, 1.5, 0.005, 2, 10);

  //shooter.set(true);
  cos_move_distance_smooth(16.5, 60, 10, 10); 
  wait(0.05,sec);
  intakedown.spin(forward, 12, volt);
  wait(0.05,sec);
  chassis.turn_to_angle(315);
  wait(0.05,sec);
  cos_move_distance_smooth(15, 315, 6, 6);//intake 3 balls
  wait(1,sec);
  intakedown.stop();
  cos_move_distance_smooth(10, 315, 8, 6);//low goals
  wait(0.05,sec);

  intake.spin(reverse, 12, volt);
  wait(0.05,sec);
  intakedown.spin(reverse, 12, volt);
  wait(1.5,sec);
  //intake.stop();
  wait(0.05,sec);
  intakedown.stop();

  wait(0.05,sec);
  cos_move_distance_smooth(-10, 315, 10, 10);//backback
  wait(0.05,sec);
  //intakedown.spin(forward, 10, volt);


  intakedown.spin(forward, 12, volt);
  wait(0.05,sec);
  chassis.turn_to_angle(140);
  wait(0.05,sec);
  cos_move_distance_smooth(16, 145, 10, 10);//to the loader
  wait(0.05,sec);
  chassis.turn_to_angle(180);
  wait(0.05,sec);
  cos_move_distance_smooth(4.5, 180, 10, 10);//into loader not sure!!!
  wait(0.05,sec);
  intakeCylander.set(true);
  wait(0.05,sec);
  chassis.drive_distance(7, 180, 12, 12);
  wait(0.05,sec);
  chassis.drive_distance(-2, 180, 10, 10);
  wait(0.05,sec);
  chassis.drive_distance(4, 180, 10, 10);
  wait(1.5,sec);
  cos_move_distance_smooth(-5, 180, 10, 10);
  intakeCylander.set(false);
  aligner.set(true);
  shooter.set(true);
  chassis.turn_to_angle(10);
  wait(0.05,sec);
  chassis.drive_distance(10, 10, 10, 10);
  wait(0.05,sec);
  shooter.set(true);
  intake.stop();
  wait(0.05,sec);
  cos_move_distance_smooth(10, 0, 10, 10);
  wait(0.05,sec);
  intake.spin(reverse, 10, volt);
  wait(0.05,sec);
  intakedown.spin(forward, 10, volt);
}
//--------------------------------------------------------
void B_left()
{
  vex::color selectedTeamColor = vex::color::blue;

  chassis.set_drive_constants(12, 2.0, 0.005, 2, 10);
  chassis.set_heading_constants(12, 1.5, 0.005, 2, 10);

  //shooter.set(true);
  cos_move_distance_smooth(14.5, 290, 10, 10); // mirrored angle
  wait(0.05,sec);
  intakedown.spin(forward, 12, volt);
  wait(0.05,sec);
  chassis.turn_to_angle(44); // mirrored angle
  wait(0.05,sec);
  cos_move_distance_smooth(15, 44, 6, 6);//intake 3 balls
  wait(0.8,sec);
  cos_move_distance_smooth(11, 44, 8, 8);//middle goals
  wait(0.05,sec);
  intake.spin(forward, 12, volt);
  wait(1.5,sec);
  intake.stop();
  wait(0.05,sec);
  cos_move_distance_smooth(-10, 44, 10, 10);//backback (mirrored)
  wait(0.05,sec);
  chassis.turn_to_angle(230); // mirrored angle
  wait(0.05,sec);
  cos_move_distance_smooth(25, 230, 10, 10);//to the loader
  wait(0.05,sec);
  chassis.turn_to_angle(180);
  intakedown.stop();
  wait(0.05,sec);
  intakedown.spin(reverse, 12, volt);
  intakeCylander.set(true);
  intakedown.stop();
  wait(0.05,sec);
  intakedown.spin(forward, 12, volt);
  chassis.DriveL.spin(forward, 12, volt);
  chassis.DriveR.spin(forward, 12, volt);
  wait(1.5,sec);
  cos_move_distance_smooth(-7, 180, 10, 10);
  intakedown.stop();
  intakeCylander.set(false);
  aligner.set(true);
  shooter.set(true);
  chassis.turn_to_angle(3); // to long goal
  wait(0.05,sec);
  chassis.drive_distance(8, 3, 10, 10);
  wait(0.05,sec);
  shooter.set(true);
  //intake.stop();
  wait(0.05,sec);
  cos_move_distance_smooth(10, 0, 10, 10);
  wait(0.05,sec);
  intake.spin(forward, 12, volt);
  wait(0.05,sec);
  intakedown.spin(forward, 10, volt);
}
/*
void practice()
{
  vex::color selectedTeamColor = vex::color::blue;
  
  cos_move_distance_smooth(10, 0, 10, 10); 
  chassis.turn_to_angle(90); // mirrored angle
  cos_move_distance_smooth(10, 0, 10, 10); 
  chassis.turn_to_angle(90); // mirrored angle
  cos_move_distance_smooth(10, 0, 10, 10); 
  chassis.turn_to_angle(90); // mirrored angle
  cos_move_distance_smooth(10, 0, 10, 10);

  
}*/

//--------------------------------------------------------
void B_right_final()
{
  vex::color selectedTeamColor = vex::color::blue;   
}
//--------------------------------------------------------
void B_left_final()
{
  vex::color selectedTeamColor = vex::color::blue;
}
//-------------------------------------------------------- 
void B_solo()
{
  vex::color selectedTeamColor = vex::color::blue;
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
