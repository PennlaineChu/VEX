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
//--------------------------------------------------------
  chassis.set_drive_constants(12, 2.0, 0.005, 2, 10);
  chassis.set_heading_constants(12, 1.5, 0.005, 2, 10);
  chassis.turn_to_angle(60);
  wait(0.05,sec);
  intakedown.spin(forward, 12, volt);
  //intake.spin(forward, 12, volt);
  chassis.drive_distance(22, 60, 6, 6);
  wait(.05,sec);
  chassis.turn_to_angle(318); 
  chassis.drive_distance(4, 318, 4, 4);//preintake
  wait(0.05,sec);
  chassis.drive_distance(3, 318, 4, 4);//intake 3 balls
  //shooter.set(true);
  wait(0.05,sec);
  chassis.drive_distance(20, 318, 6, 6);//middle goals
  intake.spin(reverse, 12, volt);
  chassis.drive_distance(5, 318, 6, 6);//middle goals
  intakedown.spin(reverse, 9, volt);
  wait(1.5,sec);//outake middle goals
  intake.stop();
  wait(0.05,sec);
  chassis.drive_distance(-8, 318, 4, 4);//backback
  wait(0.05,sec);
  pushCylinder.set(false);
  wait(0.05,sec);
  chassis.turn_to_angle(120);
  wait(0.05,sec);
  chassis.drive_distance(33, 120, 10, 10);//to the loader
  wait(0.05,sec);
  intakedown.spin(forward, 12, volt);
  wait(0.05,sec);
  chassis.turn_to_angle(178);
  wait(0.05,sec);
  shooter.set(true);
  wait(0.05,sec);
  chassis.drive_distance(8, 178, 10, 10);//into loader
  wait(0.05,sec);
  intakeCylander.set(true);
  wait(0.05,sec);
  intake.spin(forward, 12, volt);
  chassis.drive_distance(21, 178, 8, 8);
  wait(.5,sec);
  intake.stop();
  wait(.05,sec);
  chassis.drive_distance(-7, 178, 6, 6);
  intakeCylander.set(false);
  shooter.set(true);
  wait(0.05,sec);
  chassis.turn_to_angle(360); // to long goal
  wait(0.05,sec);
  chassis.drive_distance(12, 360, 6, 6);
  pushCylinder.set(true);
  wait(0.05,sec);
  cos_move_distance_smooth(5, 360, 6, 6);
  wait(0.05,sec);
  intake.spin(forward, 12, volt);
  wait(0.05,sec);
  intakedown.spin(forward, 12, volt);
  wait(2.5,sec);
  intake.stop();
  intakedown.stop();
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
void blank1()
{
  /*vex::color selectedTeamColor = vex::color::blue;

  chassis.set_drive_constants(12, 2.0, 0.005, 2, 10);
  chassis.set_heading_constants(12, 1.5, 0.005, 2, 10);
  chassis.turn_to_angle(300);
  wait(0.05,sec);
  intakedown.spin(forward, 12, volt);
  intake.spin(forward, 12, volt);
  chassis.drive_distance(20, 300, 6, 6);
  wait(.05,sec);
  chassis.turn_to_angle(43); 
  wait(0.05,sec);
  chassis.drive_distance(8, 43, 6, 6);//intake 3 balls
  wait(.5,sec);
  //shooter.set(true);
  pushCylinder.set(true);
  wait(0.05,sec);
  chassis.drive_distance(26, 43, 6, 6);//middle goals
  intake.spin(forward, 7, volt);
  wait(2.5,sec);//outake middle goals
  intake.stop();
  wait(0.05,sec);
  chassis.drive_distance(-10, 43, 4, 4);//backback
  wait(0.05,sec);
  pushCylinder.set(false);
  wait(0.05,sec);
  chassis.turn_to_angle(240);
  wait(0.05,sec);
  chassis.drive_distance(32, 240, 6, 6);//to the loader
  wait(0.05,sec);
  chassis.turn_to_angle(182);
  wait(0.05,sec);
  chassis.drive_distance(14, 182, 10, 10);//into loader
  wait(0.05,sec);
  intakeCylander.set(true);
  wait(0.05,sec);
  intake.spin(forward, 10, volt);
  chassis.drive_distance(10, 182, 6, 6);
  wait(.7,sec);
  intake.stop();
  wait(.05,sec);
  chassis.drive_distance(-7, 182, 6, 6);
  intakeCylander.set(false);
  shooter.set(true);
  wait(0.05,sec);
  chassis.turn_to_angle(4); // to long goal
  wait(0.05,sec);
  chassis.drive_distance(11, 4, 6, 6);
  pushCylinder.set(true);
  wait(0.05,sec);
  shooter.set(true);
  wait(0.05,sec);
  cos_move_distance_smooth(5, 4, 6, 6);
  wait(0.05,sec);
  intake.spin(forward, 10, volt);
  wait(0.05,sec);
  intakedown.spin(forward, 10, volt);
  wait(2.5,sec);
  intake.stop();
  intakedown.stop();*/
}
/*
void practice()
{
  vex::color selectedTeamColor = vex::color::blue;
  
  cos_move_distance_smooth(10, 0, 6, 6); 
  chassis.turn_to_angle(90); // mirrored angle
  cos_move_distance_smooth(10, 0, 6, 6); 
  chassis.turn_to_angle(90); // mirrored angle
  cos_move_distance_smooth(10, 0, 6, 6); 
  chassis.turn_to_angle(90); // mirrored angle
  cos_move_distance_smooth(10, 0, 6, 6);

  
}*/

//--------------------------------------------------------
void blank2()
{
  vex::color selectedTeamColor = vex::color::blue;   
}
//--------------------------------------------------------
void blank3()
{
  vex::color selectedTeamColor = vex::color::blue;
}
//-------------------------------------------------------- 
void blank4()
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
  chassis.drive_distance(3, 316, 6, 6);
  //----------------------------------------------------------------------
  autoarm();
  //---------------------------------------------------------------------
  chassis.drive_distance(-3, 315, 6, 6); // 微退一點
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
