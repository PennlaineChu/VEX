/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#pragma once

// Standard C++ libraries
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// VEX V5 libraries
#include "v5.h"
#include "v5_vcs.h"

// Project headers
#include "robot-config.h"
#include "JAR-Template/odom.h"
#include "JAR-Template/drive.h"
#include "JAR-Template/util.h"
#include "JAR-Template/PID.h"
#include "autons.h"

// ========== Position Tracking Declarations ==========
/**
 * Robot pose structure for odometry tracking
 * Stores X, Y position in inches and heading in degrees (0-360)
 */
struct RobotPose {
  double x;        // X position in inches
  double y;        // Y position in inches
  double heading;  // Heading in degrees (0-360)
  
  RobotPose() : x(0.0), y(0.0), heading(0.0) {}
  RobotPose(double x_, double y_, double h_) : x(x_), y(y_), heading(h_) {}
};

// ========== XY Navigation Functions ==========
/**
 * Set the robot's starting pose (position and heading)
 * @param x X coordinate in inches
 * @param y Y coordinate in inches
 * @param heading Heading in degrees (0-360)
 */
void set_robot_pose(double x, double y, double heading);

/**
 * Get the current robot pose
 * @return RobotPose structure with current X, Y, and heading
 */
RobotPose get_robot_pose();

/**
 * Drive to a specific absolute X, Y coordinate
 * @param targetX Target X coordinate in inches
 * @param targetY Target Y coordinate in inches
 * @param maxV Maximum drive voltage (0-12V)
 * @param turnMaxV Maximum turn voltage (0-12V)
 */
void driveToXY(double targetX, double targetY, double maxV, double turnMaxV);

/**
 * Drive to a target X coordinate while maintaining a specific heading
 * Calculates the Y coordinate that results in the desired heading
 * @param targetX Target X coordinate in inches
 * @param targetHeading Desired heading in degrees (0° = -Y/up, 90° = +X/right)
 * @param maxV Maximum drive voltage (0-12V)
 * @param turnMaxV Maximum turn voltage (0-12V)
 */
void driveToXAtHeading(double targetX, double targetHeading, double maxV, double turnMaxV);

/**
 * Drive to a target (X, Y) coordinate while maintaining current heading
 * Moves backward (negative distance) to reach the target
 * Useful for backing into goals or moving backward while facing a specific direction
 * @param targetX Target X coordinate in inches
 * @param targetY Target Y coordinate in inches
 * @param maxV Maximum drive voltage (0-12V)
 * @param turnMaxV Maximum turn voltage (0-12V)
 */
void driveToXYBackward(double targetX, double targetY, double maxV, double turnMaxV);

/**
 * Turn to face a specific absolute X, Y coordinate
 * @param targetX Target X coordinate in inches
 * @param targetY Target Y coordinate in inches
 * @param turnMaxV Maximum turn voltage (0-12V)
 */
void turnToXY(double targetX, double targetY, double turnMaxV);

/**
 * Smooth cosine-velocity movement with distance and angle
 * @param distance_in Distance to travel in inches
 * @param angle_deg Target angle in degrees
 * @param turn_maxV Maximum turn voltage
 * @param drive_maxV Maximum drive voltage
 */
void cos_move_distance_smooth(double distance_in, double angle_deg, double turn_maxV, double drive_maxV);

/**
 * Fused movement with sensor fusion (IMU + encoders)
 * @param distance_in Distance to travel in inches
 * @param angle_deg Target angle in degrees
 * @param turn_maxV Maximum turn voltage
 * @param drive_maxV Maximum drive voltage
 */
void cos_move_distance_fused(double distance_in, double angle_deg, double turn_maxV, double drive_maxV);

// ========== Odometry Helper Functions ==========
/**
 * Update robot pose using encoder deltas and current heading
 * @param delta_left Left encoder delta in inches
 * @param delta_right Right encoder delta in inches
 * @param current_heading Current heading in degrees (0-360)
 */
void update_robot_pose(double delta_left, double delta_right, double current_heading);

/**
 * Get left side encoder position in inches
 * @return Left encoder position in inches
 */
double get_left_inches();

/**
 * Get right side encoder position in inches
 * @return Right encoder position in inches
 */
double get_right_inches();

/**
 * Test function to verify update_robot_pose calculations
 * Displays test results on controller screen
 */
void test_update_robot_pose();

/**
 * Background odometry task (runs continuously)
 * Updates robot pose using encoders and IMU
 * @return Task return code (always 0)
 */
int odometry_task();

// ========== Utility Macros ==========
/**
 * Wait until a condition is true
 * @param condition Boolean condition to wait for
 */
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

/**
 * Repeat a block of code a specified number of times
 * @param iterations Number of times to repeat
 */
#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
