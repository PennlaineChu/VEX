#include "vex.h"

void Odom::set_physical_distances(float ForwardTracker_center_distance, float SidewaysTracker_center_distance){
  this->ForwardTracker_center_distance = ForwardTracker_center_distance;
  this->SidewaysTracker_center_distance = SidewaysTracker_center_distance;
}

void Odom::set_position(float X_position, float Y_position, float orientation_deg, float ForwardTracker_position, float SidewaysTracker_position){
  this->ForwardTracker_position = ForwardTracker_position;
  this->SideWaysTracker_position = SidewaysTracker_position;
  this->X_position = X_position;
  this->Y_position = Y_position;
  this->orientation_deg = orientation_deg;
}

void Odom::update_position(float ForwardTracker_position, float SidewaysTracker_position, float orientation_deg){
  float Forward_delta = ForwardTracker_position-this->ForwardTracker_position;
  float Sideways_delta = SidewaysTracker_position-this->SideWaysTracker_position;
  this->ForwardTracker_position=ForwardTracker_position;
  this->SideWaysTracker_position=SidewaysTracker_position;
  float orientation_rad = to_rad(orientation_deg);
  float prev_orientation_rad = to_rad(this->orientation_deg);
  float orientation_delta_rad = orientation_rad-prev_orientation_rad;
  this->orientation_deg=orientation_deg;

  float local_X_position;
  float local_Y_position;

  if (orientation_delta_rad == 0) {
    local_X_position = Sideways_delta;
    local_Y_position = Forward_delta;
  } else {
    local_X_position = (2*sin(orientation_delta_rad/2))*((Sideways_delta/orientation_delta_rad)+SidewaysTracker_center_distance); 
    local_Y_position = (2*sin(orientation_delta_rad/2))*((Forward_delta/orientation_delta_rad)+ForwardTracker_center_distance);
  }

  float local_polar_angle;
  float local_polar_length;

  if (local_X_position == 0 && local_Y_position == 0){
    local_polar_angle = 0;
    local_polar_length = 0;
  } else {
    local_polar_angle = atan2(local_Y_position, local_X_position); 
    local_polar_length = sqrt(pow(local_X_position, 2) + pow(local_Y_position, 2)); 
  }

  float global_polar_angle = local_polar_angle - prev_orientation_rad - (orientation_delta_rad/2);

  float X_position_delta = local_polar_length*cos(global_polar_angle); 
  float Y_position_delta = local_polar_length*sin(global_polar_angle);

  X_position+=X_position_delta;
  Y_position+=Y_position_delta;
}

void Odom::set_three_wheel_distances(float LeftTracker_center_distance, float RightTracker_center_distance, float BackTracker_center_distance){
  this->LeftTracker_center_distance = LeftTracker_center_distance;
  this->RightTracker_center_distance = RightTracker_center_distance;
  this->BackTracker_center_distance = BackTracker_center_distance;
}

void Odom::set_three_wheel_position(float X_position, float Y_position, float orientation_deg, float LeftTracker_position, float RightTracker_position, float BackTracker_position){
  // CRITICAL: Explicitly set stored positions to the provided values
  // This ensures they start at 0.0 when initialized
  this->LeftTracker_position = LeftTracker_position;
  this->RightTracker_position = RightTracker_position;
  this->BackTracker_position = BackTracker_position;
  this->X_position = X_position;
  this->Y_position = Y_position;
  this->orientation_deg = orientation_deg;
  
  // Debug: Print what we're setting and verify
  Brain.Screen.setCursor(4, 1);
  Brain.Screen.print("Odom set: L=%.2f R=%.2f B=%.2f", LeftTracker_position, RightTracker_position, BackTracker_position);
  Brain.Screen.setCursor(5, 1);
  Brain.Screen.print("Odom stored: L=%.2f R=%.2f B=%.2f", this->LeftTracker_position, this->RightTracker_position, this->BackTracker_position);
}

void Odom::update_three_wheel_position(float LeftTracker_position, float RightTracker_position, float BackTracker_position, float orientation_deg){
  // Store old values for debug BEFORE updating
  float old_LeftTracker_position = this->LeftTracker_position;
  float old_RightTracker_position = this->RightTracker_position;
  float old_BackTracker_position = this->BackTracker_position;
  
  float Left_delta = LeftTracker_position - this->LeftTracker_position;
  float Right_delta = RightTracker_position - this->RightTracker_position;
  float Back_delta = BackTracker_position - this->BackTracker_position;
  
  // Update stored positions AFTER calculating deltas
  this->LeftTracker_position = LeftTracker_position;
  this->RightTracker_position = RightTracker_position;
  this->BackTracker_position = BackTracker_position;
  
  float orientation_rad = to_rad(orientation_deg);
  float prev_orientation_rad = to_rad(this->orientation_deg);
  float orientation_delta_rad = orientation_rad - prev_orientation_rad;
  this->orientation_deg = orientation_deg;
  
  // Calculate forward movement (average of left and right trackers)
  float forward_delta = (Left_delta + Right_delta) / 2.0;
  
  // Calculate sideways movement from back tracker
  float sideways_delta = Back_delta;
  
  // Calculate local position changes
  // For 3-wheel odometry, we use the orientation from the gyro, not from wheel differences
  // The left/right difference is used to verify, but gyro is primary source
  float local_X_position;
  float local_Y_position;
  
  if (fabs(orientation_delta_rad) < 0.001) {  // Use small epsilon instead of == 0
    // No rotation, simple translation
    local_X_position = sideways_delta;
    local_Y_position = forward_delta;
  } else {
    // Account for arc movement when rotating
    // The trackers move in arcs when the robot rotates
    local_X_position = (2 * sin(orientation_delta_rad / 2)) * ((sideways_delta / orientation_delta_rad) + BackTracker_center_distance);
    local_Y_position = (2 * sin(orientation_delta_rad / 2)) * ((forward_delta / orientation_delta_rad) + ((LeftTracker_center_distance + RightTracker_center_distance) / 2.0));
  }
  
  // Convert to polar coordinates
  float local_polar_angle;
  float local_polar_length;
  
  if (fabs(local_X_position) < 0.001 && fabs(local_Y_position) < 0.001) {  // Use epsilon
    local_polar_angle = 0;
    local_polar_length = 0;
  } else {
    local_polar_angle = atan2(local_Y_position, local_X_position);
    local_polar_length = sqrt(pow(local_X_position, 2) + pow(local_Y_position, 2));
  }
  
  // Transform to global coordinates
  float global_polar_angle = local_polar_angle - prev_orientation_rad - (orientation_delta_rad / 2);
  
  float X_position_delta = local_polar_length * cos(global_polar_angle);
  float Y_position_delta = local_polar_length * sin(global_polar_angle);
  
  X_position += X_position_delta;
  Y_position += Y_position_delta;
  
  // Debug: Print odometry update info periodically (every 20 updates = ~100ms)
  static int debug_count = 0;
  static int total_updates = 0;  // Track total number of updates to verify task is running
  debug_count++;
  total_updates++;
  
  if (debug_count % 20 == 0) {
    Brain.Screen.setCursor(9, 1);
    Brain.Screen.print("Updates: %d | Ld=%.2f Rd=%.2f Fd=%.2f", total_updates, Left_delta, Right_delta, forward_delta);
    Brain.Screen.setCursor(10, 1);
    Brain.Screen.print("Old: L=%.2f R=%.2f B=%.2f", old_LeftTracker_position, old_RightTracker_position, old_BackTracker_position);
    Brain.Screen.setCursor(11, 1);
    Brain.Screen.print("New: L=%.2f R=%.2f B=%.2f", LeftTracker_position, RightTracker_position, BackTracker_position);
    Brain.Screen.setCursor(12, 1);
    Brain.Screen.print("Local: X=%.2f Y=%.2f", local_X_position, local_Y_position);
    Brain.Screen.setCursor(13, 1);
    Brain.Screen.print("Delta: X=%.2f Y=%.2f", X_position_delta, Y_position_delta);
    Brain.Screen.setCursor(14, 1);
    Brain.Screen.print("Pos: X=%.2f Y=%.2f", X_position, Y_position);
  }
}