#include "vex.h"
#include "robot-config.h"
#include <cmath>
#include <cstdlib>
Drive::Drive(enum::drive_setup drive_setup, motor_group DriveL, motor_group DriveR, int gyro_port, float wheel_diameter, float wheel_ratio, float gyro_scale, int DriveLF_port, int DriveRF_port, int DriveLB_port, int DriveRB_port, int ForwardTracker_port, float ForwardTracker_diameter, float ForwardTracker_center_distance, int SidewaysTracker_port, float SidewaysTracker_diameter, float SidewaysTracker_center_distance, int LeftTracker_port, float LeftTracker_diameter, float LeftTracker_center_distance, int RightTracker_port, float RightTracker_diameter, float RightTracker_center_distance, int BackTracker_port, float BackTracker_diameter, float BackTracker_center_distance) :
  wheel_diameter(wheel_diameter),
  wheel_ratio(wheel_ratio),
  gyro_scale(gyro_scale),
  drive_in_to_deg_ratio(wheel_ratio/360.0*M_PI*wheel_diameter),
  ForwardTracker_center_distance(ForwardTracker_center_distance),
  ForwardTracker_diameter(ForwardTracker_diameter),
  ForwardTracker_in_to_deg_ratio(M_PI*ForwardTracker_diameter/360.0),
  SidewaysTracker_center_distance(SidewaysTracker_center_distance),
  SidewaysTracker_diameter(SidewaysTracker_diameter),
  SidewaysTracker_in_to_deg_ratio(M_PI*SidewaysTracker_diameter/360.0),
  LeftTracker_center_distance(LeftTracker_center_distance),
  LeftTracker_diameter(LeftTracker_diameter),
  LeftTracker_in_to_deg_ratio(M_PI*LeftTracker_diameter/360.0),
  RightTracker_center_distance(RightTracker_center_distance),
  RightTracker_diameter(RightTracker_diameter),
  RightTracker_in_to_deg_ratio(M_PI*RightTracker_diameter/360.0),
  BackTracker_center_distance(BackTracker_center_distance),
  BackTracker_diameter(BackTracker_diameter),
  BackTracker_in_to_deg_ratio(M_PI*BackTracker_diameter/360.0),
  drive_setup(drive_setup),
  DriveL(DriveL),
  DriveR(DriveR),
  Gyro(inertial(gyro_port)),
  DriveLF((DriveLF_port < 0) ? -DriveLF_port : DriveLF_port, ratio6_1, is_reversed(DriveLF_port)),
  DriveRF((DriveRF_port < 0) ? -DriveRF_port : DriveRF_port, ratio6_1, is_reversed(DriveRF_port)),
  DriveLB((DriveLB_port < 0) ? -DriveLB_port : DriveLB_port, ratio6_1, is_reversed(DriveLB_port)),
  DriveRB((DriveRB_port < 0) ? -DriveRB_port : DriveRB_port, ratio6_1, is_reversed(DriveRB_port)),
  R_ForwardTracker(ForwardTracker_port),
  R_SidewaysTracker(SidewaysTracker_port),
  R_LeftTracker(LeftTracker_port),
  R_RightTracker(RightTracker_port),
  R_BackTracker(BackTracker_port),
  E_ForwardTracker(ThreeWire.Port[to_port(ForwardTracker_port)]),
  E_SidewaysTracker(ThreeWire.Port[to_port(SidewaysTracker_port)])
{
  if (drive_setup != ZERO_TRACKER_NO_ODOM){
    if (drive_setup == HOLONOMIC_THREE_ROTATION){
      odom.set_three_wheel_distances(LeftTracker_center_distance, RightTracker_center_distance, BackTracker_center_distance);
      // Initialize odometry heading tracking variables
      odometry_heading_offset = 0.0;
      initial_left_tracker_pos = 0.0;
      initial_right_tracker_pos = 0.0;
      odometry_heading_initialized = false;
    } else if (drive_setup == TANK_ONE_ENCODER || drive_setup == TANK_ONE_ROTATION || drive_setup == ZERO_TRACKER_ODOM){
      odom.set_physical_distances(ForwardTracker_center_distance, 0);
    } else {
      odom.set_physical_distances(ForwardTracker_center_distance, SidewaysTracker_center_distance);
    }
  }
}

void Drive::drive_with_voltage(float leftVoltage, float rightVoltage){
  DriveL.spin(fwd, leftVoltage, volt);
  DriveR.spin(fwd, rightVoltage,volt);
}

void Drive::set_turn_constants(float turn_max_voltage, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  this->turn_max_voltage = turn_max_voltage;
  this->turn_kp = turn_kp;
  this->turn_ki = turn_ki;
  this->turn_kd = turn_kd;
  this->turn_starti = turn_starti;
} 

void Drive::set_drive_constants(float drive_max_voltage, float drive_kp, float drive_ki, float drive_kd, float drive_starti){
  this->drive_max_voltage = drive_max_voltage;
  this->drive_kp = drive_kp;
  this->drive_ki = drive_ki;
  this->drive_kd = drive_kd;
  this->drive_starti = drive_starti;
} 

void Drive::set_heading_constants(float heading_max_voltage, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  this->heading_max_voltage = heading_max_voltage;
  this->heading_kp = heading_kp;
  this->heading_ki = heading_ki;
  this->heading_kd = heading_kd;
  this->heading_starti = heading_starti;
}

void Drive::set_swing_constants(float swing_max_voltage, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  this->swing_max_voltage = swing_max_voltage;
  this->swing_kp = swing_kp;
  this->swing_ki = swing_ki;
  this->swing_kd = swing_kd;
  this->swing_starti = swing_starti;
} 

void Drive::set_turn_exit_conditions(float turn_settle_error, float turn_settle_time, float turn_timeout){
  this->turn_settle_error = turn_settle_error;
  this->turn_settle_time = turn_settle_time;
  this->turn_timeout = turn_timeout;
}

void Drive::set_drive_exit_conditions(float drive_settle_error, float drive_settle_time, float drive_timeout){
  this->drive_settle_error = drive_settle_error;
  this->drive_settle_time = drive_settle_time;
  this->drive_timeout = drive_timeout;
}

void Drive::set_swing_exit_conditions(float swing_settle_error, float swing_settle_time, float swing_timeout){
  this->swing_settle_error = swing_settle_error;
  this->swing_settle_time = swing_settle_time;
  this->swing_timeout = swing_timeout;
}

float Drive::get_absolute_heading(){ 
  return( reduce_0_to_360( Gyro.rotation()*360.0/gyro_scale ) ); 
}

float Drive::get_left_position_in(){
  return( DriveL.position(deg)*drive_in_to_deg_ratio );
}

float Drive::get_right_position_in(){
  return( DriveR.position(deg)*drive_in_to_deg_ratio );
}

void Drive::turn_to_angle(float angle){
  turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_max_voltage){
  turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout){
  turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_angle(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  desired_heading = angle;
  PID turnPID(reduce_negative_180_to_180(angle - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while(turnPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = turnPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    drive_with_voltage(output, -output);
    task::sleep(10);
  }
  DriveL.stop(hold);
  DriveR.stop(hold);
}

void Drive::drive_distance(float distance){
  drive_distance(distance, desired_heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading){
  drive_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage){
  drive_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  drive_distance(distance, heading, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_distance(float distance, float heading, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  desired_heading = heading;
  PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(heading - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
  float average_position = start_average_position;
  while(drivePID.is_settled() == false){
    average_position = (get_left_position_in()+get_right_position_in())/2.0;
    float drive_error = distance+start_average_position-average_position;
    float heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);
    float heading_output = headingPID.compute(heading_error);

    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    // Early exit: only if error is very small (< 0.5 inches), meaning we're at the target
    // Don't exit based on output alone, as low output could mean low PID constants
    if(std::fabs(drive_error) < 0.5){
      break;
    }

    drive_with_voltage(drive_output+heading_output, drive_output-heading_output);
    task::sleep(10);
  }
  DriveL.stop(hold);
  DriveR.stop(hold);
}

void Drive::left_swing_to_angle(float angle){
  left_swing_to_angle(angle, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}

void Drive::left_swing_to_angle(float angle, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_timeout, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  desired_heading = angle;
  PID swingPID(reduce_negative_180_to_180(angle - get_absolute_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
  while(swingPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = swingPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    DriveL.spin(fwd, output, volt);
    DriveR.stop(hold);
    task::sleep(10);
  }
  DriveL.stop(hold);
  DriveR.stop(hold);
}

void Drive::right_swing_to_angle(float angle){
  right_swing_to_angle(angle, swing_max_voltage, swing_settle_error, swing_settle_time, swing_timeout, swing_kp, swing_ki, swing_kd, swing_starti);
}

void Drive::right_swing_to_angle(float angle, float swing_max_voltage, float swing_settle_error, float swing_settle_time, float swing_timeout, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  desired_heading = angle;
  PID swingPID(reduce_negative_180_to_180(angle - get_absolute_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
  while(swingPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = swingPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    DriveR.spin(reverse, output, volt);
    DriveL.stop(hold);
    task::sleep(10);
  }
  DriveL.stop(hold);
  DriveR.stop(hold);
}

float Drive::get_ForwardTracker_position(){
  if (drive_setup==ZERO_TRACKER_ODOM){
    return(get_right_position_in());
  }
  if (drive_setup==TANK_ONE_ENCODER || drive_setup == TANK_TWO_ENCODER || drive_setup == HOLONOMIC_TWO_ENCODER){
    return(E_ForwardTracker.position(deg)*ForwardTracker_in_to_deg_ratio);
  }else{
    return(R_ForwardTracker.position(deg)*ForwardTracker_in_to_deg_ratio);
  }
}

float Drive::get_SidewaysTracker_position(){
  if (drive_setup==TANK_ONE_ENCODER || drive_setup == TANK_ONE_ROTATION || drive_setup == ZERO_TRACKER_ODOM){
    return(0);
  }else if (drive_setup == TANK_TWO_ENCODER || drive_setup == HOLONOMIC_TWO_ENCODER){
    return(E_SidewaysTracker.position(deg)*SidewaysTracker_in_to_deg_ratio);
  }else{
    return(R_SidewaysTracker.position(deg)*SidewaysTracker_in_to_deg_ratio);
  }
}

float Drive::get_LeftTracker_position(){
  if (drive_setup == HOLONOMIC_THREE_ROTATION){
    return(R_LeftTracker.position(deg)*LeftTracker_in_to_deg_ratio);
  }
  return(0);
}

float Drive::get_RightTracker_position(){
  if (drive_setup == HOLONOMIC_THREE_ROTATION){
    // Negate the right tracker reading to match left tracker direction
    // When moving forward, both left and right should have the same sign
    return(-R_RightTracker.position(deg)*RightTracker_in_to_deg_ratio);
  }
  return(0);
}

float Drive::get_BackTracker_position(){
  if (drive_setup == HOLONOMIC_THREE_ROTATION){
    return(-R_BackTracker.position(deg)*BackTracker_in_to_deg_ratio);
  }
  return(0);
}

float Drive::get_odometry_heading(){
  // Calculate heading from left and right tracking wheels
  // The difference between right and left wheel positions gives us the rotation
  // heading = initial_heading + (right_delta - left_delta) / track_width
  if (drive_setup == HOLONOMIC_THREE_ROTATION){
    float left_pos = get_LeftTracker_position();
    float right_pos = get_RightTracker_position();
    float track_width = LeftTracker_center_distance + RightTracker_center_distance;  // 5.625 + 5.625 = 11.25 inches
    
    // Initialize on first call using current gyro heading
    if (!odometry_heading_initialized && track_width != 0) {
      initial_left_tracker_pos = left_pos;
      initial_right_tracker_pos = right_pos;
      odometry_heading_offset = get_absolute_heading();  // Use gyro as initial reference
      odometry_heading_initialized = true;
    }
    
    if (track_width != 0 && odometry_heading_initialized) {
      // Calculate rotation angle in radians from wheel difference
      // When robot rotates, right wheel moves forward and left moves backward (or vice versa)
      float right_delta = right_pos - initial_right_tracker_pos;
      float left_delta = left_pos - initial_left_tracker_pos;
      float rotation_rad = (right_delta - left_delta) / track_width;
      // Convert to degrees and add to initial heading
      float heading_deg = odometry_heading_offset + to_deg(rotation_rad);
      // Normalize to 0-360
      while(heading_deg < 0) heading_deg += 360.0;
      while(heading_deg >= 360.0) heading_deg -= 360.0;
      return heading_deg;
    }
  }
  // Fallback to gyro if not 3-wheel setup or not initialized
  return get_absolute_heading();
}

void Drive::position_track(){
  while(1){
    if (drive_setup == HOLONOMIC_THREE_ROTATION){
      odom.update_three_wheel_position(get_LeftTracker_position(), get_RightTracker_position(), get_BackTracker_position(), get_absolute_heading());
    } else {
      odom.update_position(get_ForwardTracker_position(), get_SidewaysTracker_position(), get_absolute_heading());
    }
    task::sleep(5);
  }
}

void Drive::set_heading(float orientation_deg){
  Gyro.setRotation(orientation_deg*gyro_scale/360.0, deg);
}

void Drive::set_coordinates(float X_position, float Y_position, float orientation_deg){
  // Stop existing odometry task if running
  // Note: stop() is safe to call even if task is not running
  odom_task.stop();
  
  if (drive_setup == HOLONOMIC_THREE_ROTATION){
    // CRITICAL: Reset rotation sensors to 0 before setting coordinates
    // This ensures the odometry starts tracking from the current position
    R_LeftTracker.resetPosition();
    R_RightTracker.resetPosition();
    R_BackTracker.resetPosition();
    
    // Wait a moment for sensors to reset
    task::sleep(50);  // Increased wait time to ensure sensors reset
    
    // Verify sensors are actually at 0 before proceeding
    // Read current tracker positions (should be 0 after reset)
    float current_L = get_LeftTracker_position();
    float current_R = get_RightTracker_position();
    float current_B = get_BackTracker_position();
    
    // Debug: Print what we're setting
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Setting coords: X=%.2f Y=%.2f", X_position, Y_position);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Tracker after reset: L=%.2f R=%.2f B=%.2f", current_L, current_R, current_B);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Setting stored to: 0.0, 0.0, 0.0");
    
    // Now set odometry with zeroed tracker positions
    // IMPORTANT: Use 0.0, not the current positions, to ensure stored positions start at 0
    odom.set_three_wheel_position(X_position, Y_position, orientation_deg, 0.0, 0.0, 0.0);
    
    // Reset odometry heading tracking when coordinates are set
    initial_left_tracker_pos = 0.0;
    initial_right_tracker_pos = 0.0;
    odometry_heading_offset = orientation_deg;
    odometry_heading_initialized = true;
    
    // CRITICAL: Wait a bit more and verify stored positions are still 0 before starting task
    // This prevents the task from starting before initialization is complete
    task::sleep(100);  // Additional delay to ensure everything is set
    
    // Verify stored positions are 0 (they should be, but double-check)
    // Note: We can't directly access private members, but we can verify by checking
    // that the first update will have correct deltas
  } else {
    odom.set_position(X_position, Y_position, orientation_deg, get_ForwardTracker_position(), get_SidewaysTracker_position());
  }
  set_heading(orientation_deg);
  
  // Start odometry task AFTER everything is initialized and verified
  odom_task = task(position_track_task);
}

float Drive::get_X_position(){
  return(odom.X_position);
}

float Drive::get_Y_position(){
  return(odom.Y_position);
}

void Drive::drive_to_point(float X_position, float Y_position){
  drive_to_point(X_position, Y_position, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, float drive_max_voltage, float heading_max_voltage){
  drive_to_point(X_position, Y_position, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  drive_to_point(X_position, Y_position, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  PID drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  while(drivePID.is_settled() == false){
    float drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
    float heading_error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);

    float heading_scale_factor = cos(to_rad(heading_error));
    drive_output*=heading_scale_factor;
    heading_error = reduce_negative_90_to_90(heading_error);
    float heading_output = headingPID.compute(heading_error);
    
    if (drive_error<drive_settle_error) { heading_output = 0; }

    drive_output = clamp(drive_output, -fabs(heading_scale_factor)*drive_max_voltage, fabs(heading_scale_factor)*drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_with_voltage(drive_output+heading_output, drive_output-heading_output);
    task::sleep(10);
  }
  desired_heading = get_absolute_heading();
  DriveL.stop(hold);
  DriveR.stop(hold);
}

void Drive::turn_to_point(float X_position, float Y_position){
  turn_to_point(X_position, Y_position, 0, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg){
  turn_to_point(X_position, Y_position, extra_angle_deg, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout){
  turn_to_point(X_position, Y_position, extra_angle_deg, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  PID turnPID(reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while(turnPID.is_settled() == false){
    float error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading() + extra_angle_deg);
    float output = turnPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    drive_with_voltage(output, -output);
    task::sleep(10);
  }
  desired_heading = get_absolute_heading();
  DriveL.stop(hold);
  DriveR.stop(hold);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position){
  holonomic_drive_to_point(X_position, Y_position, get_absolute_heading(), drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position, float angle){
  holonomic_drive_to_point(X_position, Y_position, angle, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_max_voltage, float heading_max_voltage){
  holonomic_drive_to_point(X_position, Y_position, angle, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  holonomic_drive_to_point(X_position, Y_position, angle, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_drive_to_point(float X_position, float Y_position, float angle, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  desired_heading = angle;
  // Calculate target heading in VEX coordinate system
  // VEX Inertial: 0°=North(+Y), 90°=East(+X), 180°=South(-Y), 270°=West(-X)
  // atan2(y,x): 0°=+X(East), 90°=+Y(North), 180°=-X(West), 270°=-Y(South)
  // Conversion: VEX_angle = 90° - atan2(y,x)
  // Verification:
  //   (10,0) East:  atan2(0,10)=0°  → 90°-0°  = 90° (East) ✓
  //   (0,10) North: atan2(10,0)=90° → 90°-90° = 0°  (North) ✓
  //   (1,√3) 30°:   atan2(√3,1)=60° → 90°-60° = 30° ✓
  //   (-10,0) West: atan2(0,-10)=180° → 90°-180° = -90° = 270° (West) ✓
  //   (0,-10) South: atan2(-10,0)=-90° → 90°-(-90°) = 180° (South) ✓
  float dx = X_position - get_X_position();
  float dy = Y_position - get_Y_position();
  float target_heading_deg = 90.0 - to_deg(atan2(dy, dx));  // Convert atan2 to VEX coordinates
  while(target_heading_deg < 0.0) target_heading_deg += 360.0;
  while(target_heading_deg >= 360.0) target_heading_deg -= 360.0;
  
  // Normalize desired heading to 0-360
  float desired_heading_deg = reduce_0_to_360(angle);
  
  PID drivePID(hypot(dx, dy), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID turnPID(reduce_negative_180_to_180(desired_heading_deg - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  
  int loop_count = 0;
  // Continue loop if either drive or turn needs to run
  // This allows turning in place when at the same position
  while(!(drivePID.is_settled()) || fabs(reduce_negative_180_to_180(desired_heading_deg - get_absolute_heading())) > 1.0){
    float drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
    
    // Recalculate direction to target each iteration (for driving)
    dx = X_position - get_X_position();
    dy = Y_position - get_Y_position();
    
    // Direction to target in VEX heading space (used only for driving direction)
    target_heading_deg = 90.0 - to_deg(atan2(dy, dx));  // Convert atan2 to VEX coordinates
    target_heading_deg = reduce_0_to_360(target_heading_deg);
    
    // Convert field-relative target direction to robot-relative movement
    // VEX: 0°=North(+Y), 90°=East(+X)
    // Robot-relative: forward is direction robot faces, strafe is perpendicular
    float robot_heading_deg = get_absolute_heading();
    float relative_angle_rad = to_rad(target_heading_deg - robot_heading_deg);
    
    // Turn error uses desired_heading (angle parameter), not direction to target
    float turn_error = reduce_negative_180_to_180(desired_heading_deg - robot_heading_deg);

    float drive_output = drivePID.compute(drive_error);
    float turn_output = turnPID.compute(turn_error);

    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    turn_output = clamp(turn_output, -heading_max_voltage, heading_max_voltage);

    // Decompose drive_output into forward and strafe components (robot-relative)
    // forward: movement in direction robot is facing (positive = forward)
    // strafe: movement perpendicular to robot (positive = right)
    float forward = drive_output * cos(relative_angle_rad);
    float strafe = drive_output * sin(relative_angle_rad);

    // Print debug info every 10 loops (100ms)
    if (loop_count % 10 == 0) {
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1, 1);
      Brain.Screen.print("Pos: X=%.2f Y=%.2f", get_X_position(), get_Y_position());
      Brain.Screen.setCursor(2, 1);
      Brain.Screen.print("Target: X=%.2f Y=%.2f", X_position, Y_position);
      Brain.Screen.setCursor(3, 1);
      Brain.Screen.print("dx=%.2f dy=%.2f", dx, dy);
      Brain.Screen.setCursor(4, 1);
      Brain.Screen.print("Robot H:%.1f Target H:%.1f", robot_heading_deg, target_heading_deg);
      Brain.Screen.setCursor(5, 1);
      Brain.Screen.print("Forward:%.2f Strafe:%.2f", forward, strafe);
      Brain.Screen.setCursor(6, 1);
      Brain.Screen.print("Drive:%.2f Turn:%.2f", drive_output, turn_output);
      // Debug: Show raw tracker positions to verify odometry
      if (drive_setup == HOLONOMIC_THREE_ROTATION) {
        Brain.Screen.setCursor(7, 1);
        Brain.Screen.print("L:%.2f R:%.2f B:%.2f", 
          get_LeftTracker_position(), 
          get_RightTracker_position(), 
          get_BackTracker_position());
        // Show raw rotation sensor values
        Brain.Screen.setCursor(8, 1);
        Brain.Screen.print("Raw L:%.0f R:%.0f B:%.0f deg", 
          R_LeftTracker.position(deg), 
          R_RightTracker.position(deg), 
          R_BackTracker.position(deg));
      }
    }
    loop_count++;

    // Apply to motors (matching control_holonomic pattern: throttle=forward, strafe=strafe)
    // LF: forward + turn + strafe
    // RF: forward - turn - strafe
    // LB: forward + turn - strafe
    // RB: forward - turn + strafe
    DriveLF.spin(fwd, forward + turn_output + strafe, volt);
    DriveRF.spin(fwd, forward - turn_output - strafe, volt);
    DriveLB.spin(fwd, forward + turn_output - strafe, volt);
    DriveRB.spin(fwd, forward - turn_output + strafe, volt);
    
    task::sleep(10);
  }
  DriveLF.stop(hold);
  DriveLB.stop(hold);
  DriveRB.stop(hold);
  DriveRF.stop(hold);
}

void Drive::holonomic_turn_to_angle(float angle){
  holonomic_turn_to_angle(angle, heading_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_turn_to_angle(float angle, float turn_max_voltage){
  holonomic_turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_turn_to_angle(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout){
  holonomic_turn_to_angle(angle, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::holonomic_turn_to_angle(float angle, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  desired_heading = angle;
  PID turnPID(reduce_negative_180_to_180(angle - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  
  while(turnPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    float output = turnPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    
    // For holonomic drive, turn in place using all 4 motors:
    // LF and LB: +turn (left side forward)
    // RF and RB: -turn (right side backward)
    // This creates pure rotation without translation
    DriveLF.spin(fwd, output, volt);
    DriveLB.spin(fwd, output, volt);
    DriveRF.spin(fwd, -output, volt);
    DriveRB.spin(fwd, -output, volt);
    
    task::sleep(10);
  }
  
  DriveLF.stop(hold);
  DriveLB.stop(hold);
  DriveRF.stop(hold);
  DriveRB.stop(hold);
}

void Drive::control_arcade(){
  float throttle = deadband(controller(primary).Axis3.value(), 5);
  float turn = deadband(controller(primary).Axis1.value(), 5);
  DriveL.spin(fwd, to_volt(throttle+turn), volt);
  DriveR.spin(fwd, to_volt(throttle-turn), volt);
}

void Drive::control_holonomic(){
  float throttle = deadband(controller(primary).Axis3.value(), 7);
  float turn = deadband(controller(primary).Axis1.value(), 7);
  float strafe = deadband(controller(primary).Axis4.value(), 7);
  
  float field_throttle = throttle;
  float field_strafe = strafe;
  
  // Field-centric control: rotate joystick inputs by negative of robot heading
  // Use odometry tracking wheels for heading (more accurate for field-centric)
  float heading_deg = 0;
  bool heading_valid = false;

  if(Inertial.installed() && !Inertial.isCalibrating()){
    heading_deg = Inertial.heading(degrees);
    heading_valid = true;
  }
  
  // Apply field-centric rotation if we have a valid heading
  if(heading_valid){
    // Convert to radians and negate for field-centric rotation
    float heading_rad = to_rad(-heading_deg);
    
    // Rotate throttle and strafe by the robot's heading
    // This makes the robot move relative to the field, not the robot's orientation
    float cos_h = cos(heading_rad);
    float sin_h = sin(heading_rad);
    field_throttle = throttle * cos_h - strafe * sin_h;
    field_strafe = throttle * sin_h + strafe * cos_h;
  }
  
  // Calculate motor voltages
  float lf_voltage = to_volt(field_throttle+turn+field_strafe);
  float rf_voltage = to_volt(field_throttle-turn-field_strafe);  
  float lb_voltage = to_volt(field_throttle+turn-field_strafe);
  float rb_voltage = to_volt(field_throttle-turn+field_strafe);
  
  // Always apply voltages to motors (even if zero) to keep them active and prevent blue/coast state
  DriveLF.spin(fwd, lf_voltage, volt);
  DriveRF.spin(fwd, rf_voltage, volt);
  DriveLB.spin(fwd, lb_voltage, volt);
  DriveRB.spin(fwd, rb_voltage, volt);
}

void Drive::control_tank(float percentage = 100){
  float leftthrottle = deadband(controller(primary).Axis3.value(), 5) * percentage / 100;
  float rightthrottle = deadband(controller(primary).Axis2.value(), 5) * percentage / 100;
  DriveL.spin(fwd, to_volt(leftthrottle), volt);
  DriveR.spin(fwd, to_volt(rightthrottle), volt);
  if(leftthrottle == 0)
    DriveL.stop(coast);
  if(rightthrottle == 0)
    DriveR.stop(coast);
}


int Drive::position_track_task(){
  chassis.position_track();
  return(0);
}

