#pragma once
#include "JAR-Template/drive.h"

class Drive;

extern Drive chassis;

void default_constants();

void R_right();
void R_left();
void R_right_final();
void R_left_final();
void R_solo();
void B_right();
void B_left();
void B_right_final();
void B_left_final();
void B_solo();


void B_17022A();

void skills();
void PIDtest();
void autoarm();
void drive_test();
void turn_test();
void swing_test();
void full_test();
void odom_test();
void tank_odom_test();
void holonomic_odom_test();
