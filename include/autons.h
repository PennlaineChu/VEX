#pragma once
#include "JAR-Template/drive.h"

class Drive;

extern Drive chassis;

void default_constants();

void RW_right();
void R3_right();
void RW_left();
void R5_left();
void BW_right();
void B3_left();
void BW_left();
void B5_right();
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
