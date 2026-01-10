#include "vex.h"
#include "note.h"
#include <cmath>
#include <algorithm>
extern vex::color selectedTeamColor;
using namespace vex;
competition Competition;


Drive chassis(
    // Specify your drive setup below. There are eight options:
    // ZERO_TRACKER_NO_ODOM, ZERO_TRACKER_ODOM, TANK_ONE_ENCODER, TANK_ONE_ROTATION, TANK_TWO_ENCODER, TANK_TWO_ROTATION, HOLONOMIC_TWO_ENCODER, HOLONOMIC_TWO_ROTATION, and HOLONOMIC_THREE_ROTATION
    // For holonomic drive with 3 tracking wheels (left, right, back), use HOLONOMIC_THREE_ROTATION:
    HOLONOMIC_THREE_ROTATION,
    // Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
    // For holonomic drives, these can be empty motor groups since we use individual motors:
    // Left Motors:
    motor_group(),
    // Right Motors:
    motor_group(),
    // Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
    PORT18,
    // Input your wheel diameter: 3.25 inches
    3.25,
    // External gear ratio: USER MEASURED - 360° motor = 3.25*PI*0.75 inches
    // This means 1 motor rotation = 0.75 wheel rotations, so ratio = 0.75
    1,
    // Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
    // For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
    360,
    /*---------------------------------------------------------------------------*/
    /*                                  PAUSE!                                   */
    /*                                                                           */
    /*  The rest of the drive constructor is for robots using POSITION TRACKING. */
    /*  If you are not using position tracking, leave the rest of the values as  */
    /*  they are.                                                                */
    /*---------------------------------------------------------------------------*/
    // FOR HOLONOMIC DRIVES: Input your drive motors by position.
    // LF:      //RF:
    PORT19, -PORT9,  // Top right motor - reversed
    // LB:      //RB:
    PORT20, -PORT10,  // Bottom right motor reversed (negative port)
    // Legacy tracker ports (not used for HOLONOMIC_THREE_ROTATION, but required by constructor):
    // Use valid ports (PORT4 and PORT10) for unused trackers to avoid memory permission errors
    PORT3, 0.0, 0.0,  // Forward tracker (not used, but needs valid port)
    PORT3, 0.0, 0.0,  // Sideways tracker (not used, but needs valid port)
    // 3-Wheel Odometry Tracking Wheels:
    PORT17,   // Left tracker port (5.625" from center, 3.25" wheel)
    3.25,    // Left tracker diameter (inches)
    5.625,   // Left tracker center distance from center (inches)
    PORT7,   // Right tracker port (5.625" from center, 3.25" wheel)
    3.25,    // Right tracker diameter (inches)
    5.625,   // Right tracker center distance from center (inches)
    PORT8,   // Back tracker port (4.25" from center, 2" wheel)
    2.0,     // Back tracker diameter (inches)
    4.25     // Back tracker center distance from center (inches)
);
// ---- helpers ----
static inline double clampd(double v, double lo, double hi){ return v<lo?lo:(v>hi?hi:v); }
static inline double sgn(double x){ return (x>=0)?1.0:-1.0; }
static inline double wrap180(double a){ while(a>180)a-=360; while(a<-180)a+=360; return a; }
static const double PI = 3.14159265358979323846;

// Robot wheel parameters (CRITICAL: Must match Drive chassis!)
// Wheels: 3.25 inch diameter
// Gearing: USER MEASURED - 360° motor rotation = 3.25*PI*0.75 inches
// This means 1 motor rotation = 0.75 wheel rotations, so gear ratio = 0.75
static const double WHEEL_DIAM_IN  = 3.25;
static const double EXT_GEAR_RATIO = 1;  // USER MEASURED: 360° motor = 0.75 wheel circumference
static const double WHEEL_CIRC_IN  = PI * WHEEL_DIAM_IN;

// Pre-calculated constant for better precision (reduces floating-point rounding errors)
static const double DEG_TO_INCHES = (EXT_GEAR_RATIO * WHEEL_CIRC_IN) / 360.0;

// Movement constants
static const double MIN_DRIVE_VELOCITY = 3.5;  // Minimum voltage to overcome motor static friction

// Robot dimensions
static const double ROBOT_WIDTH_IN  = 12.5; // Robot width
static const double ROBOT_LENGTH_IN = 16.0; // Robot length
static const double TRACK_WIDTH_IN  = 10; // Distance between left and right wheels (center to center)

// ========== Position Tracking Structure ==========
// RobotPose struct is now defined in vex.h so it can be used in autons.cpp

// Global robot pose
static RobotPose robot_pose;

// Global odometry task handle (for task management)
static vex::task* odometry_task_handle = nullptr;

// Helper function to start odometry task (stops existing one first)
static void start_odometry_task() {
  if(odometry_task_handle != nullptr) {
    odometry_task_handle->stop();
    delete odometry_task_handle;
    odometry_task_handle = nullptr;
  }
  odometry_task_handle = new vex::task(odometry_task);
}

static double deg_to_inches(double posDeg){
    double rotations = posDeg / 360.0;
    double dist_to_targetInches = rotations * (WHEEL_CIRC_IN * EXT_GEAR_RATIO);
    return dist_to_targetInches;
}

// Get left side travel dist_to_target (signed)
// Optimized: fewer operations, better precision
// Made non-static so test function can call it
double get_left_inches(){
  double l_deg = (L1.position(vex::deg)+L2.position(vex::deg)+L3.position(vex::deg))/3.0;
  return l_deg * DEG_TO_INCHES;
}

// Get right side travel dist_to_target (signed)
// Optimized: fewer operations, better precision
// Made non-static so test function can call it
double get_right_inches(){
  double r_deg = (R1.position(vex::deg)+R2.position(vex::deg)+R3.position(vex::deg))/3.0;
  return r_deg * DEG_TO_INCHES;
}

// Get average travel dist_to_target (absolute value)
// Optimized: fewer operations, better precision
static double get_avg_inches(){
  double l_deg = std::fabs(L1.position(vex::deg)+L2.position(vex::deg)+L3.position(vex::deg))/3.0;
  double r_deg = std::fabs(R1.position(vex::deg)+R2.position(vex::deg)+R3.position(vex::deg))/3.0;
  return (l_deg + r_deg) * 0.5 * DEG_TO_INCHES;
}

// Calculate heading from motor encoders (differential drive)
// Based on the difference between left and right wheel travel
static double get_encoder_heading_delta(double left_inches, double right_inches){
  // Arc length difference divided by track width gives angle in radians
  double delta_heading_rad = (right_inches - left_inches) / TRACK_WIDTH_IN;
  double delta_heading_deg = delta_heading_rad * 180.0 / PI;
  return delta_heading_deg;
}

// IMPROVED Sensor fusion: combine IMU heading with encoder heading
static double get_fused_heading(double imu_heading, double encoder_heading_delta, double prev_fused_heading){
  // IGNORE encoder heading if change is tiny (encoder noise/gear backlash)
  if(std::fabs(encoder_heading_delta) < 0.1) {
    return imu_heading;  // Trust IMU when barely moving
  }
  
  // Calculate what the encoder thinks the heading should be
  double encoder_predicted = prev_fused_heading + encoder_heading_delta;
  // Normalize to [0, 360)
  encoder_predicted = std::fmod(encoder_predicted + 360.0, 360.0);
  
  // Simplified fusion: use encoder prediction as base, then correct towards IMU
  // This is equivalent to: fused = encoder_predicted, then correct 3% towards IMU
  double diff = wrap180(imu_heading - encoder_predicted);
  double fused = encoder_predicted + diff * 0.03;
  
  // Keep fused in [0, 360)
  fused = std::fmod(fused + 360.0, 360.0);
  
  return fused;
}

// Update robot position using odometry (Monte Carlo-like approach)
// Made non-static so test function can call it
void update_robot_pose(double delta_left, double delta_right, double current_heading){
  // Average dist_to_target traveled
  double delta_center = (delta_left + delta_right) / 2.0;
  
  // CRITICAL: Detect in-place turning (wheels moving in opposite directions)
  // When turning in place, delta_left and delta_right have opposite signs
  // If they're roughly equal magnitude but opposite, delta_center ≈ 0
  // But we should also check if the magnitudes are similar to avoid position drift
  double abs_left = std::fabs(delta_left);
  double abs_right = std::fabs(delta_right);
  bool is_turning_in_place = (delta_left * delta_right < 0) &&  // Opposite signs
                              (std::fabs(abs_left - abs_right) < std::max(abs_left, abs_right) * 0.3);  // Similar magnitudes (within 30%)
  
  // If turning in place, don't update X/Y position (only update heading)
  // This prevents position drift during turns
  if(is_turning_in_place && std::fabs(delta_center) < 0.1) {
    // Only update heading, skip X/Y update
    while(current_heading < 0.0) current_heading += 360.0;
    while(current_heading >= 360.0) current_heading -= 360.0;
    robot_pose.heading = current_heading;
    return;
  }
  
  // CRITICAL: Normalize heading to 0-360 before using it
  while(current_heading < 0.0) current_heading += 360.0;
  while(current_heading >= 360.0) current_heading -= 360.0;
  
  // Convert heading to radians for calculation
  double heading_rad = current_heading * PI / 180.0;
  
  // Update x and y position using current heading
  // Coordinate system: Top-left origin (0,0), bottom-right (140.41, 140.41)
  // 0° = -Y (up/forward), 90° = +X (right), 180° = +Y (down), 270° = -X (left)
  // At 0°: X = 0, Y = -1 → X = sin(0°) = 0, Y = -cos(0°) = -1 ✓
  // At 90°: X = +1, Y = 0 → X = sin(90°) = 1, Y = -cos(90°) = 0 ✓
  // CORRECT: X = +sin(heading), Y = -cos(heading)
  robot_pose.x += delta_center * std::sin(heading_rad);
  robot_pose.y -= delta_center * std::cos(heading_rad);
  robot_pose.heading = current_heading;  // Store normalized heading
}

static void set_drive_volt(double leftV, double rightV){
  L1.spin(vex::fwd, leftV, vex::volt); L2.spin(vex::fwd, leftV, vex::volt); L3.spin(vex::fwd, leftV, vex::volt);
  R1.spin(vex::fwd, rightV, vex::volt); R2.spin(vex::fwd, rightV, vex::volt); R3.spin(vex::fwd, rightV, vex::volt);
}

// CRITICAL: Track IMU offset so we can reset heading to any value
// When we call set_robot_pose(0, 0, 0), we store the current IMU reading as the offset
// Then heading = (IMU - offset), normalized to 0-360
static double imu_heading_offset = 0.0;

// Initialize robot pose (non-static so it can be called from autons.cpp)
void set_robot_pose(double x, double y, double heading){
  // Normalize heading to 0-360
  while(heading < 0.0) heading += 360.0;
  while(heading >= 360.0) heading -= 360.0;
  
  robot_pose.x = x;
  robot_pose.y = y;
  robot_pose.heading = heading;
  
  // CRITICAL: Set both IMU objects directly to the desired heading
  // Both Inertial and chassis.Gyro are the same physical sensor (PORT20) but different objects
  // We need to sync both so they agree on the heading
  vex::wait(50, vex::msec);  // Wait for IMU to stabilize
  Inertial.setHeading(heading, vex::degrees);  // Set Inertial directly
  chassis.set_heading(heading);  // Set chassis Gyro directly
  
  // Since we set both directly, the offset should be 0
  // But we keep the offset system for compatibility with existing code
  imu_heading_offset = 0.0;
  
  // DEBUG: Show heading set
  Brain.Screen.printAt(10, 160, "HEADING SET: %.0f deg (offset=0)", heading);
}

// Get current robot pose (non-static so it can be called from autons.cpp)
RobotPose get_robot_pose(){
  return robot_pose;
}

// ========== TEST FUNCTION: Verify update_robot_pose calculation ==========
// This function tests if X/Y calculation is correct by simulating movements
void test_update_robot_pose() {
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("TESTING...");
  wait(0.5, sec);
  
  // Save current pose
  RobotPose saved_pose = robot_pose;
  
  // Test 1: Move up 10" at 0° (0° = -Y up/forward)
  robot_pose.x = 0.0;
  robot_pose.y = 0.0;
  robot_pose.heading = 0.0;
  update_robot_pose(10.0, 10.0, 0.0);  // Positive deltas = forward movement
  double expected_x_0 = 0.0;
  double expected_y_0 = -10.0;
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("T1: 0deg +10in");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("X:%.2f Y:%.2f", robot_pose.x, robot_pose.y);
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("Exp: X:%.2f Y:%.2f", expected_x_0, expected_y_0);
  wait(2, sec);
  
  // Test 2: Move right 10" at 90° (90° = +X right) - CRITICAL TEST
  robot_pose.x = 0.0;
  robot_pose.y = 0.0;
  robot_pose.heading = 90.0;
  update_robot_pose(10.0, 10.0, 90.0);  // Positive deltas = forward movement
  double expected_x_90 = 10.0;  // Should be +10 if working correctly
  double expected_y_90 = 0.0;
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("T2: 90deg +10in");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("X:%.2f Y:%.2f", robot_pose.x, robot_pose.y);
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("Exp: X:%.2f Y:%.2f", expected_x_90, expected_y_90);
  wait(2, sec);
  
  // Test 3: Move down 10" at 180° (180° = +Y down)
  robot_pose.x = 0.0;
  robot_pose.y = 0.0;
  robot_pose.heading = 180.0;
  update_robot_pose(-10.0, -10.0, 180.0);  // Negative deltas = backward movement
  double expected_x_180 = 0.0;
  double expected_y_180 = 10.0;
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("T3: 180deg -10in");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("X:%.2f Y:%.2f", robot_pose.x, robot_pose.y);
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("Exp: X:%.2f Y:%.2f", expected_x_180, expected_y_180);
  wait(2, sec);
  
  // Test 4: Move left 10" at 270° (270° = -X left)
  robot_pose.x = 0.0;
  robot_pose.y = 0.0;
  robot_pose.heading = 270.0;
  update_robot_pose(-10.0, -10.0, 270.0);  // Negative deltas = backward movement
  double expected_x_270 = -10.0;
  double expected_y_270 = 0.0;
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("T4: 270deg -10in");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("X:%.2f Y:%.2f", robot_pose.x, robot_pose.y);
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("Exp: X:%.2f Y:%.2f", expected_x_270, expected_y_270);
  wait(2, sec);
  
  // Test 5: Move at 45° (diagonally up-right)
  robot_pose.x = 0.0;
  robot_pose.y = 0.0;
  robot_pose.heading = 45.0;
  update_robot_pose(10.0, 10.0, 45.0);  // Positive deltas = forward movement
  double expected_x_45 = 10.0 * std::sin(45.0 * PI / 180.0);
  double expected_y_45 = -10.0 * std::cos(45.0 * PI / 180.0);
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("T5: 45deg +10in");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("X:%.2f Y:%.2f", robot_pose.x, robot_pose.y);
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("Exp: X:%.2f Y:%.2f", expected_x_45, expected_y_45);
  wait(2, sec);
  
  // Test 6: Actual encoder test
  double curr_l = get_left_inches();
  double curr_r = get_right_inches();
  double delta_l = curr_l;
  double delta_r = curr_r;
  double heading_now = robot_pose.heading;
  update_robot_pose(delta_l, delta_r, heading_now);
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("T6: ACTUAL");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("dL:%.2f dR:%.2f", delta_l, delta_r);
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("H:%.1f X:%.2f Y:%.2f", heading_now, robot_pose.x, robot_pose.y);
  wait(2, sec);
  
  // Restore saved pose
  robot_pose = saved_pose;
  
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("TEST COMPLETE");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("Check results above");
}

// Display robot pose on controller
// Optimized for minimal blocking - caller controls frequency
void display_robot_pose(){
  Controller1.Screen.clearScreen();  // Clear screen first to prevent overlapping text
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("X:%.1f Y:%.1f", robot_pose.x, robot_pose.y);
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("H:%.1f", robot_pose.heading);
  
  // DEBUG: Show encoder values (less frequently)
  static int debug_count = 0;
  debug_count++;
  if(debug_count % 5 == 0) {  // Update every 2.5 seconds
    double l_inches = get_left_inches();
    double r_inches = get_right_inches();
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("L:%.1f R:%.1f", l_inches, r_inches);
  }
}

// Display robot pose on brain
static void display_robot_pose_brain(){
  Brain.Screen.setPenColor(vex::white);
  Brain.Screen.printAt(10, 220, false, "Pose: X=%.1f Y=%.1f H=%.1f", 
                       robot_pose.x, robot_pose.y, robot_pose.heading);
}

// ========== PATH RECORDING SYSTEM ==========

// Waypoint structure for recording path
struct Waypoint {
  double x;
  double y;
  double heading;
  double dist_to_target;  // Total dist_to_target traveled from start
  
  Waypoint() : x(0.0), y(0.0), heading(0.0), dist_to_target(0.0) {}
  Waypoint(double x_, double y_, double h_, double d_) : x(x_), y(y_), heading(h_), dist_to_target(d_) {}
};

// Global recording state
static std::vector<Waypoint> recordedWaypoints;
static bool isRecording = false;
static double totalDistanceTraveled = 0.0;
// Encoder-based position tracking for recording
static double start_left_encoder = 0.0;
static double start_right_encoder = 0.0;
static double start_x = 0.0;
static double start_y = 0.0;
static double start_heading_value = 0.0;
// Current encoder-based position (updated incrementally)
static double encoder_based_x = 0.0;
static double encoder_based_y = 0.0;
static double encoder_based_heading = 0.0;
static double prev_record_left = 0.0;
static double prev_record_right = 0.0;

// Forward declarations for recording functions
static void printRecordedPath();
void recordPath();  // Main recording function

// Background task to continuously update odometry (non-static so autons can use it)
int odometry_task() {
  double prev_left = 0.0;
  double prev_right = 0.0;
  bool encoders_reset = false;  // Track if encoders were reset
  
  while(true) {
    // ALWAYS update position (not just when recording!)
      // Get current encoder positions
      double curr_left = get_left_inches();
      double curr_right = get_right_inches();
    
    // CRITICAL: Detect if encoders were reset (position jumps to near zero)
    // If encoders were reset, reset our prev values to avoid huge deltas
    if(encoders_reset || (std::fabs(curr_left) < 0.1 && std::fabs(curr_right) < 0.1 && 
                          (std::fabs(prev_left) > 1.0 || std::fabs(prev_right) > 1.0))) {
      // Encoders were reset - reset our tracking
      prev_left = curr_left;
      prev_right = curr_right;
      encoders_reset = false;
      continue;  // Skip this iteration to avoid huge delta
    }
      
      // Calculate deltas
      double delta_left = curr_left - prev_left;
      double delta_right = curr_right - prev_right;
      
    // Safety check: If delta is unreasonably large (>10 inches in one cycle), encoders were probably reset
    if(std::fabs(delta_left) > 10.0 || std::fabs(delta_right) > 10.0) {
      // Encoders were reset - reset our tracking
      prev_left = curr_left;
      prev_right = curr_right;
      continue;  // Skip this iteration
    }
    
    // Update total dist_to_target traveled
      double delta_dist = (std::fabs(delta_left) + std::fabs(delta_right)) / 2.0;
      totalDistanceTraveled += delta_dist;
      
    // PURE IMU heading with OFFSET (so set_robot_pose can reset the zero point)
    // Read raw IMU, subtract the offset set by set_robot_pose()
    double raw_imu = Inertial.heading();
    double adjusted_heading = raw_imu - imu_heading_offset;
    
    // Normalize to 0-360
    while(adjusted_heading < 0.0) adjusted_heading += 360.0;
    while(adjusted_heading >= 360.0) adjusted_heading -= 360.0;
    
    // DEBUG: Only write to Brain.Screen when recording (reduces interference with driver control)
    // Reduced frequency to prevent screen update conflicts
    static int debug_counter = 0;
    debug_counter++;
    if(isRecording && (debug_counter % 100 == 0)) {  // Only when recording, less frequently
      double abs_heading = (adjusted_heading > 180) ? (360 - adjusted_heading) : adjusted_heading;
      if(abs_heading > 50) {
        Brain.Screen.printAt(10, 220, "Odom: Raw=%.0f Off=%.0f Adj=%.0f  ", 
                             raw_imu, imu_heading_offset, adjusted_heading);
      }
    }
    
    // Update robot position with adjusted heading
    update_robot_pose(delta_left, delta_right, adjusted_heading);
      
      // Store for next iteration
      prev_left = curr_left;
      prev_right = curr_right;
      
    // Display current pose on brain (only when recording)
    if(isRecording) {
      Brain.Screen.setPenColor(vex::white);
      Brain.Screen.printAt(10, 20, false, "X: %.2f  Y: %.2f", robot_pose.x, robot_pose.y);
      Brain.Screen.printAt(10, 40, false, "Heading: %.1f  Dist: %.2f", robot_pose.heading, totalDistanceTraveled);
      Brain.Screen.printAt(10, 60, false, "Points: %d", (int)recordedWaypoints.size());
    }
    
    vex::wait(10, vex::msec);
  }
  return 0;
}

// Start recording path
static void startRecording(double start_x_val, double start_y_val, double start_heading_val) {
  // Reset everything
  recordedWaypoints.clear();
  totalDistanceTraveled = 0.0;
  
  // Reset encoders
  L1.resetPosition(); L2.resetPosition(); L3.resetPosition();
  R1.resetPosition(); R2.resetPosition(); R3.resetPosition();
  
  // Wait a bit for encoders to reset
  vex::wait(50, vex::msec);
  
  // Store starting encoder positions (these will be 0 after reset, but we store them anyway)
  start_left_encoder = get_left_inches();
  start_right_encoder = get_right_inches();
  start_x = start_x_val;
  start_y = start_y_val;
  start_heading_value = start_heading_val;
  
  // Initialize encoder-based position tracking
  encoder_based_x = start_x_val;
  encoder_based_y = start_y_val;
  encoder_based_heading = start_heading_val;
  prev_record_left = start_left_encoder;
  prev_record_right = start_right_encoder;
  
  // Set starting position (for odometry task if needed)
  set_robot_pose(start_x_val, start_y_val, start_heading_val);
  
  // Record starting waypoint
  recordedWaypoints.push_back(Waypoint(start_x_val, start_y_val, start_heading_val, 0.0));
  
  // Start recording
  isRecording = true;
  
  // Start background odometry task (stop existing one if any)
  start_odometry_task();
  
  Brain.Screen.clearScreen();
  Brain.Screen.setPenColor(vex::green);
  Brain.Screen.printAt(10, 100, "RECORDING PATH...");
  Brain.Screen.setPenColor(vex::white);
  Brain.Screen.printAt(10, 120, "A: Record waypoint");
  Brain.Screen.printAt(10, 140, "B: Finish recording");
}

// Record current position as waypoint
// Calculates X, Y directly from motor encoder positions, not from odometry task
static void recordWaypoint() {
  if(!isRecording) return;
  
  // Get current encoder positions
  double curr_left = get_left_inches();
  double curr_right = get_right_inches();
  
  // Calculate encoder deltas since last recording
  double delta_left = curr_left - prev_record_left;
  double delta_right = curr_right - prev_record_right;
  
  // Get current heading from IMU (with offset applied)
  double raw_imu = Inertial.heading();
  double adjusted_heading = raw_imu - imu_heading_offset;
  while(adjusted_heading < 0.0) adjusted_heading += 360.0;
  while(adjusted_heading >= 360.0) adjusted_heading -= 360.0;
  
  // Update encoder-based position using the same logic as update_robot_pose
  // Calculate average distance traveled (center of robot)
  double delta_center = (delta_left + delta_right) / 2.0;
  
  // Detect in-place turning (wheels moving in opposite directions)
  double abs_left = std::fabs(delta_left);
  double abs_right = std::fabs(delta_right);
  bool is_turning_in_place = (delta_left * delta_right < 0) &&  // Opposite signs
                              (std::fabs(abs_left - abs_right) < std::max(abs_left, abs_right) * 0.3);
  
  // If turning in place, only update heading, not X/Y
  if(is_turning_in_place && std::fabs(delta_center) < 0.1) {
    encoder_based_heading = adjusted_heading;
  } else {
    // Normal movement: update X, Y position using current heading
    // Coordinate system: Top-left origin (0,0), bottom-right (140.41, 140.41)
    // 0° = -Y (up/forward), 90° = +X (right), 180° = +Y (down), 270° = -X (left)
    double heading_rad = adjusted_heading * PI / 180.0;
    encoder_based_x += delta_center * std::sin(heading_rad);
    encoder_based_y -= delta_center * std::cos(heading_rad);
    encoder_based_heading = adjusted_heading;
  }
  
  // Update total distance traveled
  totalDistanceTraveled += (std::fabs(delta_left) + std::fabs(delta_right)) / 2.0;
  
  // Save waypoint using encoder-based position
  recordedWaypoints.push_back(Waypoint(encoder_based_x, encoder_based_y, encoder_based_heading, totalDistanceTraveled));
  
  // Update previous encoder positions for next calculation
  prev_record_left = curr_left;
  prev_record_right = curr_right;
  
  // Feedback - RUMBLE with delay to ensure it executes
  Controller1.rumble(".");  // Single short pulse (more reliable)
  vex::wait(50, vex::msec);  // Small delay to let rumble execute
  
  // Visual feedback
  Brain.Screen.setPenColor(vex::green);
  Brain.Screen.printAt(10, 180, false, "Waypoint %d: X=%.1f Y=%.1f H=%.1f", 
                       (int)recordedWaypoints.size(), encoder_based_x, encoder_based_y, encoder_based_heading);
  Brain.Screen.setPenColor(vex::white);
}

// Stop recording and print results
static void stopRecording() {
  if(!isRecording) return;
  
  isRecording = false;
  
  // Final feedback - RUMBLE with delay to ensure it executes
  Controller1.rumble("-");  // Single long pulse (more reliable)
  vex::wait(100, vex::msec);  // Delay to let rumble execute
  
  // Visual feedback
  Brain.Screen.clearScreen();
  Brain.Screen.setPenColor(vex::green);
  Brain.Screen.printAt(10, 20, "RECORDING COMPLETE!");
  Brain.Screen.setPenColor(vex::white);
  Brain.Screen.printAt(10, 40, "Total waypoints: %d", (int)recordedWaypoints.size());
  
  vex::wait(1, vex::sec);
  
  // Print recorded path
  printRecordedPath();
}

// Print recorded path to terminal/brain
static void printRecordedPath() {
  Brain.Screen.clearScreen();
  Brain.Screen.setPenColor(vex::cyan);
  Brain.Screen.printAt(10, 20, "RECORDED PATH:");
  Brain.Screen.setPenColor(vex::white);
  
  int line = 40;
  for(size_t i = 0; i < recordedWaypoints.size() && i < 10; i++) {
    Waypoint& wp = recordedWaypoints[i];
    Brain.Screen.printAt(10, line, false, "%d: X=%.1f Y=%.1f H=%.0f D=%.1f", 
                         (int)i, wp.x, wp.y, wp.heading, wp.dist_to_target);
    line += 20;
  }
  
  // Also print to console for easy copy-paste
  printf("\n========== RECORDED PATH ==========\n");
  printf("// Total waypoints: %d\n", (int)recordedWaypoints.size());
  printf("// Format: X, Y, Heading, Distance\n\n");
  
  for(size_t i = 0; i < recordedWaypoints.size(); i++) {
    Waypoint& wp = recordedWaypoints[i];
    printf("// Waypoint %d\n", (int)i);
    printf("const double WP%d_X = %.2f;\n", (int)i, wp.x);
    printf("const double WP%d_Y = %.2f;\n", (int)i, wp.y);
    printf("const double WP%d_H = %.1f;\n", (int)i, wp.heading);
    printf("const double WP%d_D = %.2f; // Total dist_to_target from start\n\n", (int)i, wp.dist_to_target);
  }
  
  printf("========== END RECORDED PATH ==========\n\n");
}

// Main recording function - call this from autonomous selector
void recordPath() {
  // Starting position
  const double START_X = 22.0;
  const double START_Y = 22.5;
  const double START_HEADING = Inertial.heading();  // Use current IMU heading as start
  
  // Start recording
  startRecording(START_X, START_Y, START_HEADING);
  
  // Display instructions on controller
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("PATH RECORDING MODE");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("Drive manually");
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("A=Save B=Finish");
  
  // Wait for user input
  bool lastButtonA = false;
  bool lastButtonB = false;
  
  // Show recording instructions on controller
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("RECORDING...");
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("A=Save B=Stop");
  
  while(isRecording) {
    // ========== ENABLE MANUAL DRIVING DURING RECORDING ==========
    // Read joystick inputs and control robot
    chassis.control_holonomic();  // Enable holonomic drive control
    
    // Button A: Record waypoint (with debounce)
    bool currentButtonA = Controller1.ButtonA.pressing();
    if(currentButtonA && !lastButtonA) {
      recordWaypoint();  // This function already rumbles and waits
      // Show feedback on controller
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print("Saved! Pts:%d", (int)recordedWaypoints.size());
      Controller1.Screen.setCursor(2, 1);
      Controller1.Screen.print("A=Save B=Stop");
      vex::wait(250, vex::msec); // Debounce (reduced since rumble already waits)
    }
    lastButtonA = currentButtonA;
    
    // Button B: Finish recording (with debounce)
    bool currentButtonB = Controller1.ButtonB.pressing();
    if(currentButtonB && !lastButtonB) {
      stopRecording();  // This function already rumbles and waits
      // Show feedback on controller
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print("STOPPED!");
      Controller1.Screen.setCursor(2, 1);
      Controller1.Screen.print("Total:%d pts", (int)recordedWaypoints.size());
      // No additional wait needed - stopRecording already handles it
    }
    lastButtonB = currentButtonB;
    
    vex::wait(20, vex::msec);
  }
  
  // Keep results on screen for viewing
  vex::wait(10, vex::sec);
}

// 平滑版：cos 加減速 + IMU 保角（強化：加入 heading I 項，較快加速）
void cos_move_distance_smooth(double distance_in, double angle_deg, double turn_maxV, double drive_maxV){
  double D = std::fabs(distance_in);
  if (D <= 0.0) return;

  // Reset ALL motor positions (get_avg_inches uses all 6)
  L1.resetPosition(); L2.resetPosition(); L3.resetPosition();
  R1.resetPosition(); R2.resetPosition(); R3.resetPosition();
  
  L1.setStopping(vex::brake); L2.setStopping(vex::brake); L3.setStopping(vex::brake);
  R1.setStopping(vex::brake); R2.setStopping(vex::brake); R3.setStopping(vex::brake);

  const double dir   = sgn(distance_in);
  const double Vmax  = clampd(drive_maxV, 0.0, 12.0);
  const double MIN_V = std::min(MIN_DRIVE_VELOCITY, Vmax);

  // 速度輪廓參數 (SMOOTHED for less jerky motion)
  // acc: 加速段長度, dec: 減速段長度, cruise: 等速段長度
  // Longer acc/dec = smoother transitions, less noticeable phases
  double acc = D * 0.35, dec = D * 0.45;  // Was 0.22/0.41, now smoother
  if (acc + dec > D) { double s = D / (acc + dec); acc *= s; dec *= s; }
  const double cruise = std::max(0.0, D - acc - dec);

 
  const int    dt_ms = 10;
  const double kH    = 0.17;
  const double kHi   = 0.003;
  const double i_cap = turn_maxV * 0.5;
  const double settle_in = 0.30;

  double h_i = 0.0;
  double last_s = 0.0;
  int    settle_timer = 0;

 
  const double trimV = 0.0;

  while (true){
    double s = get_avg_inches();
    if (s >= D) break;

    
    double v;
    if (s < acc){
      double x = s / std::max(1e-6, acc);
      v = Vmax * 0.5 * (1.0 - std::cos(PI * x));
    } else if (s < acc + cruise){
      v = Vmax;
    } else {
      double x = (s - (D - dec)) / std::max(1e-6, dec);
      v = Vmax * 0.5 * (1.0 + std::cos(PI * x));
    }
    v = std::max(v, MIN_V) * dir;

 
    double h_err = wrap180(angle_deg - Inertial.heading());
    h_i += h_err * (dt_ms / 1000.0) * kHi;
    h_i  = clampd(h_i, -i_cap, i_cap);

    double turnV = clampd(kH * h_err + h_i, -turn_maxV, turn_maxV);


    double leftV  = clampd(v + turnV - trimV, -Vmax, Vmax);
    double rightV = clampd(v - turnV + trimV, -Vmax, Vmax);
    set_drive_volt(leftV, rightV);

    /*Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("current travel: %.2f in", s);
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("target travel: %.2f in", D);
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("H: %.2f", Inertial.heading());*/

    double ds = std::fabs(s - last_s);
    if ((D - s) <= settle_in && ds < 0.02) {
      settle_timer += dt_ms;
      if (settle_timer >= 150) break;
    } else {
      settle_timer = 0;
    }
    last_s = s;

    vex::wait(dt_ms, vex::msec);
  }

  set_drive_volt(0, 0);
  L1.stop(); L2.stop(); L3.stop();
  R1.stop(); R2.stop(); R3.stop();
}

// ========== HELPER: Cosine Velocity Profile (Reusable) ==========
static double cosineVelocity(double traveled, double totalDistance, double Vmax) {
    // Less aggressive deceleration - shorter decel phase
    double acc = totalDistance * 0.20;  // 20% acceleration
    double dec = totalDistance * 0.40;  // Reduced from 70% to 40% (less aggressive)
    double cruise = std::max(0.0, totalDistance - acc - dec);
    
    const double MIN_V = std::min(MIN_DRIVE_VELOCITY, Vmax);
    
    if(traveled < acc) {
        // Acceleration phase: cosine ramp up
        Controller1.Screen.setCursor(3, 1);
        //Controller1.Screen.print("Acc");
        double x = traveled / std::max(1e-6, acc);
        return Vmax * 0.5 * (1.0 - std::cos(PI * x));
    } else if(traveled < acc + cruise) {
        Controller1.Screen.setCursor(3, 1);
        //Controller1.Screen.print("Cruise");
        // Cruise phase: constant speed
        return Vmax;
    } else {
        // Deceleration phase: gentler cosine ramp down (no squaring)
        Controller1.Screen.setCursor(3, 1);
        //Controller1.Screen.print("Dec");
        double x = (traveled - (totalDistance - dec)) / std::max(1e-6, dec);
        // Simple cosine deceleration (not squared) - gentler curve
        double v = Vmax * 0.5 * (1.0 + std::cos(PI * x));
        // Ensure minimum velocity when still far from target
        double remaining = totalDistance - traveled;
        if(remaining > 1.0) {
            // When >1" away, maintain at least 30% of max velocity
            v = std::max(v, Vmax * 0.3);
        }
        return std::max(v, 0.0);
    }
}

// ========== ENHANCED VERSION with Sensor Fusion & Position Tracking ==========
// This version uses 90% IMU + 10% encoder heading and maintains x,y position
void cos_move_distance_fused(double distance_in, double angle_deg, double turn_maxV, double drive_maxV){
  double D = std::fabs(distance_in);
  if (D <= 0.0) return;

  // Reset ALL motor positions (get_avg_inches uses all 6)
  L1.resetPosition(); L2.resetPosition(); L3.resetPosition();
  R1.resetPosition(); R2.resetPosition(); R3.resetPosition();
  
  L1.setStopping(vex::coast); L2.setStopping(vex::coast); L3.setStopping(vex::coast);
  R1.setStopping(vex::coast); R2.setStopping(vex::coast); R3.setStopping(vex::coast);

  const double dir   = sgn(distance_in);
  const double Vmax  = clampd(drive_maxV, 0.0, 12.0);
  const double MIN_V = std::min(MIN_DRIVE_VELOCITY, Vmax);

  // Velocity profile parameters (SMOOTHED for less jerky motion)
  double acc = D * 0.35, dec = D * 0.45;  // Was 0.22/0.41, now smoother
  if (acc + dec > D) { double s = D / (acc + dec); acc *= s; dec *= s; }
  const double cruise = std::max(0.0, D - acc - dec);

  const int    dt_ms = 10;
  const double kH    = 0.17;
  const double kHi   = 0.003;
  const double i_cap = turn_maxV * 0.5;
  const double settle_in = 0.30;

  double h_i = 0.0;
  double last_s = 0.0;
  int    settle_timer = 0;
  const double trimV = 0.0;

  // For sensor fusion and position tracking
  double prev_left = 0.0;
  double prev_right = 0.0;
  
  // Start with current adjusted heading (raw IMU - offset)
  double raw_imu_init = Inertial.heading();
  double fused_heading = raw_imu_init - imu_heading_offset;
  while(fused_heading < 0.0) fused_heading += 360.0;
  while(fused_heading >= 360.0) fused_heading -= 360.0;

  while (true){
    double s = get_avg_inches();
    if (s >= D) break;

    // Calculate velocity based on profile (using helper function)
    double v = cosineVelocity(s, D, Vmax) * dir;

    // ========== SENSOR FUSION ==========
    // Get current encoder positions
    double curr_left = get_left_inches();
    double curr_right = get_right_inches();
    
    // Calculate deltas since last iteration
    double delta_left = curr_left - prev_left;
    double delta_right = curr_right - prev_right;
    
    // Calculate encoder-based heading change
    double encoder_heading_delta = get_encoder_heading_delta(delta_left, delta_right);
    
    // Get IMU heading with offset applied
    double raw_imu = Inertial.heading();
    double adjusted_imu = raw_imu - imu_heading_offset;
    while(adjusted_imu < 0.0) adjusted_imu += 360.0;
    while(adjusted_imu >= 360.0) adjusted_imu -= 360.0;
    
    // Update fused heading using encoder delta first
    fused_heading += encoder_heading_delta;
    while(fused_heading < 0.0) fused_heading += 360.0;
    while(fused_heading >= 360.0) fused_heading -= 360.0;
    
    // Then correct 3% towards IMU (simpler fusion approach)
    double diff = wrap180(adjusted_imu - fused_heading);
    fused_heading += diff * 0.03;
    while(fused_heading < 0.0) fused_heading += 360.0;
    while(fused_heading >= 360.0) fused_heading -= 360.0;
    
    // Update robot position using fused heading
    update_robot_pose(delta_left, delta_right, fused_heading);
    
    // Store current positions for next iteration
    prev_left = curr_left;
    prev_right = curr_right;

    // ========== HEADING CONTROL with AVERAGED TARGET & REAL ==========
    // Calculate error using target angle
    double target_err = wrap180(angle_deg - fused_heading);
    // Use fused_heading (which is already adjusted) for both calculations
    double real_err = target_err;  // Simplified - just use target error
    
    // Average the target and real heading errors for PID input
    double h_err = (target_err + real_err) / 2.0;
    
    // PID calculation
    h_i += h_err * (dt_ms / 1000.0) * kHi;
    h_i  = clampd(h_i, -i_cap, i_cap);
    double turnV = clampd(kH * h_err + h_i, -turn_maxV, turn_maxV);

    // Calculate motor voltages
    double leftV  = clampd(v + turnV - trimV, -Vmax, Vmax);
    double rightV = clampd(v - turnV + trimV, -Vmax, Vmax);
    set_drive_volt(leftV, rightV);

    // Optional: Display position tracking info
    /*Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("X: %.2f Y: %.2f", robot_pose.x, robot_pose.y);
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("H_fused: %.1f IMU: %.1f", fused_heading, imu_heading);
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("Distance: %.2f / %.2f", s, D);*/

    // Settle detection
    double ds = std::fabs(s - last_s);
    if ((D - s) <= settle_in && ds < 0.02) {
      settle_timer += dt_ms;
      if (settle_timer >= 150) break;
    } else {
      settle_timer = 0;
    }
    last_s = s;

    vex::wait(dt_ms, vex::msec);
  }

  set_drive_volt(0, 0);
  L1.stop(); L2.stop(); L3.stop();
  R1.stop(); R2.stop(); R3.stop();
}

// ========== DRIVE TO ABSOLUTE X,Y COORDINATES ==========
// Navigate to a specific point on the field with sensor fusion
// Coordinate system: Top-left origin (0,0), bottom-right (140.41, 140.41)
// 0° = -Y (up/forward), 90° = +X (right), 180° = +Y (down), 270° = -X (left)
// Strategy: First turn to face target, then drive straight to prevent drift
void driveToXY(double targetX, double targetY, double maxV, double turnMaxV) {
    // ========== STEP 1: Calculate target heading (theta) ==========
    double dx = targetX - robot_pose.x;
    double dy = targetY - robot_pose.y;
    double targetHeading = std::atan2(dx, -dy) * 180.0 / PI;
    
    // Normalize to 0-360
    while(targetHeading < 0.0) targetHeading += 360.0;
    while(targetHeading >= 360.0) targetHeading -= 360.0;
        
    // ========== STEP 2: Turn to face target heading ==========
    chassis.turn_to_angle(targetHeading);
    vex::wait(50, vex::msec);  // Small delay after turning
    
    // ========== STEP 3: Drive straight to target ==========
    // Calculate distance to target
    double distance = std::sqrt(dx * dx + dy * dy);
    
    // Use chassis.drive_distance to drive straight while maintaining heading
    // This will maintain the targetHeading we just turned to
    chassis.drive_distance(distance, targetHeading, maxV, turnMaxV);
    
}

// ========== DRIVE TO TARGET X COORDINATE AT SPECIFIED HEADING ==========
// Drive to a target X coordinate while maintaining a specific heading
// Calculates the Y coordinate that results in the desired heading
void driveToXAtHeading(double targetX, double targetHeading, double maxV, double turnMaxV) {
    // Calculate the Y coordinate that results in the desired heading
    // heading = atan2(dx, -dy) * 180 / PI
    // So: tan(heading * PI / 180) = dx / (-dy)
    // So: -dy = dx / tan(heading * PI / 180)
    // So: dy = -dx / tan(heading * PI / 180)
    // So: targetY = robot_pose.y + dy = robot_pose.y - dx / tan(heading * PI / 180)
    
    double dx = targetX - robot_pose.x;
    double heading_rad = targetHeading * PI / 180.0;
    
    // Handle vertical headings (90° or 270°) where tan is undefined
    if(std::fabs(std::cos(heading_rad)) < 0.001) {
        // Heading is approximately 90° or 270° (moving purely in Y direction)
        // In this case, we can't reach a specific X while maintaining this heading
        // Just drive straight at the heading
        double distance = std::fabs(dx);  // Use a reasonable distance
        chassis.turn_to_angle(targetHeading);
        vex::wait(50, vex::msec);
        chassis.drive_distance(distance, targetHeading, maxV, turnMaxV);
        return;
    }
    
    // Calculate target Y coordinate
    double dy = -dx / std::tan(heading_rad);
    double targetY = robot_pose.y + dy;
    
    // Now drive to the calculated X,Y coordinate
    driveToXY(targetX, targetY, maxV, turnMaxV);
}

// ========== DRIVE TO TARGET XY COORDINATE BACKWARDS ==========
// Drive to a target (X, Y) coordinate while maintaining current heading
// Moves backward (negative distance) to reach the target
// Useful for backing into goals or moving backward while facing a specific direction
void driveToXYBackward(double targetX, double targetY, double maxV, double turnMaxV) {
    // ========== STEP 1: Calculate distance to target ==========
    double dx = targetX - robot_pose.x;
    double dy = targetY - robot_pose.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    // ========== STEP 2: Get current heading ==========
    // Use current robot heading - don't change it
    double currentHeading = robot_pose.heading;
    
    // ========== STEP 3: Drive backwards to target ==========
    // Drive backward (negative distance) while maintaining current heading
    chassis.drive_distance(-distance, currentHeading, maxV, turnMaxV);
}

// ========== DRIVE BACKWARD TO TARGET X COORDINATE AT SPECIFIED HEADING ==========
// Drive backward to a target X coordinate while maintaining a specific heading
// Calculates the Y coordinate that results in the desired heading, then drives backward
// Useful for backing up to a specific X position while facing a direction
void driveToXAtHeadingBackward(double targetX, double targetHeading, double maxV, double turnMaxV) {
    // Calculate the Y coordinate that results in the desired heading
    // heading = atan2(dx, -dy) * 180 / PI
    // So: tan(heading * PI / 180) = dx / (-dy)
    // So: -dy = dx / tan(heading * PI / 180)
    // So: dy = -dx / tan(heading * PI / 180)
    // So: targetY = robot_pose.y + dy = robot_pose.y - dx / tan(heading * PI / 180)
    
    double dx = targetX - robot_pose.x;
    double heading_rad = targetHeading * PI / 180.0;
    
    // Handle vertical headings (90° or 270°) where tan is undefined
    if(std::fabs(std::cos(heading_rad)) < 0.001) {
        // Heading is approximately 90° or 270° (moving purely in Y direction)
        // In this case, we can't reach a specific X while maintaining this heading
        // Just drive backward straight at the heading
        double distance = std::fabs(dx);  // Use a reasonable distance
        chassis.turn_to_angle(targetHeading);
        vex::wait(50, vex::msec);
        chassis.drive_distance(-distance, targetHeading, maxV, turnMaxV);
        return;
    }
    
    // Calculate target Y coordinate
    double dy = -dx / std::tan(heading_rad);
    double targetY = robot_pose.y + dy;
    
    // Now drive backward to the calculated X,Y coordinate
    driveToXYBackward(targetX, targetY, maxV, turnMaxV);
}

// ========== TURN TO FACE X,Y COORDINATES ==========
// Turn to face a specific point (useful before grabbing objects)
void turnToXY(double targetX, double targetY, double turnMaxV) {
    const double settle_error = 2.0;  // degrees - settle when within 2°
    const int dt_ms = 10;
    int settle_timer = 0;
    int loop_count = 0;
    const int MAX_LOOPS = 1000;  // 10 second timeout
    
    // Track heading for "no movement" detection
    double prev_heading = robot_pose.heading;
    int no_turn_timer = 0;
    const double MIN_TURN_THRESHOLD = 0.5;  // degrees - must turn at least 0.5° per 200ms
    
    // PID state - reduced gains to prevent overshoot
    double h_i = 0.0;
    double prev_error = 0.0;
    const double kH = 0.07;  // Further reduced proportional gain (was 0.10)
    const double kHi = 0.001;  // Reduced integral gain (was 0.002)
    const double kHd = 0.08;  // Increased derivative gain for better damping
    const double i_cap = turnMaxV * 0.25;
    
    L1.setStopping(vex::brake); L2.setStopping(vex::brake); L3.setStopping(vex::brake);
    R1.setStopping(vex::brake); R2.setStopping(vex::brake); R3.setStopping(vex::brake);
    
    while(true) {
        loop_count++;
        
        // Timeout check
        if(loop_count > MAX_LOOPS) {
            set_drive_volt(0, 0);
            Controller1.Screen.clearScreen();
            Controller1.Screen.setCursor(1, 1);
            Controller1.Screen.print("TURN TIMEOUT");
            break;
        }
        
        // Calculate target heading (same as driveToXY)
        double dx = targetX - robot_pose.x;
        double dy = targetY - robot_pose.y;
        // Coordinate system: Top-left origin (0,0), bottom-right (140.41, 140.41)
        // 0° = -Y (up/forward), 90° = +X (right), 180° = +Y (down), 270° = -X (left)
        // Convert from atan2 (0°=+X, 90°=+Y) to our system (0°=-Y, 90°=+X)
        double targetHeading = std::atan2(dx, -dy) * 180.0 / PI;
        
        // Normalize targetHeading to 0-360
        while(targetHeading < 0.0) targetHeading += 360.0;
        while(targetHeading >= 360.0) targetHeading -= 360.0;
        
        // Get current heading and normalize to 0-360
        double current_heading = robot_pose.heading;
        while(current_heading < 0.0) current_heading += 360.0;
        while(current_heading >= 360.0) current_heading -= 360.0;
        
        // Calculate error and wrap to -180 to 180 using wrap180 function
        double headingError = wrap180(targetHeading - current_heading);
        
        // ========== NO TURN DETECTION ==========
        // Check if robot is actually turning (heading changing)
        double heading_change = std::fabs(wrap180(current_heading - prev_heading));
        
        // Check every 200ms (20 loops) if robot is turning
        if(loop_count % 20 == 0) {
            if(heading_change < MIN_TURN_THRESHOLD && std::fabs(headingError) > 3.0) {
                // Robot not turning and still has significant error
                no_turn_timer += 200;  // Add 200ms
                if(no_turn_timer >= 500) {  // Not turning for 500ms
                    set_drive_volt(0, 0);
                    Controller1.Screen.clearScreen();
                    Controller1.Screen.setCursor(1, 1);
                    Controller1.Screen.print("STOP: NO TURN");
                    Controller1.Screen.setCursor(2, 1);
                    Controller1.Screen.print("Err:%.1f H:%.1f", headingError, current_heading);
                    vex::wait(500, vex::msec);
                    break;
                }
            } else {
                no_turn_timer = 0;  // Reset timer if turning
            }
            prev_heading = current_heading;
        }
        
        // DEBUG: Print target and current heading (controller only)
        static int debug_counter = 0;
        if(debug_counter % 20 == 0) {  // Update every 200ms
            Controller1.Screen.setCursor(1, 1);
            Controller1.Screen.print("T:%.0f N:%.0f E:%.0f", targetHeading, current_heading, headingError);
            Controller1.Screen.setCursor(2, 1);
            Controller1.Screen.print("dx:%.1f dy:%.1f", dx, dy);
            Controller1.Screen.setCursor(3, 1);
            Controller1.Screen.print("X:%.1f Y:%.1f", robot_pose.x, robot_pose.y);
        }
        debug_counter++;
        
        // Settle detection - use absolute error
        double abs_error = std::fabs(headingError);
        if(abs_error < settle_error) {
            settle_timer += dt_ms;
            if(settle_timer >= 150) {  // Stable for 150ms
                Controller1.Screen.clearScreen();
                Controller1.Screen.setCursor(1, 1);
                Controller1.Screen.print("SETTLED");
                Controller1.Screen.setCursor(2, 1);
                Controller1.Screen.print("E:%.1f", headingError);
                break;
            }
        } else {
            settle_timer = 0;
        }
        
        // Simple PID - no deadband, no complex scaling
        double p_term = kH * headingError;
        h_i += headingError * (dt_ms / 1000.0) * kHi;
        h_i = clampd(h_i, -i_cap, i_cap);
        double turnV = clampd(p_term + h_i, -turnMaxV, turnMaxV);
        
        // Simple scaling when close - reduce power gradually
        if(std::fabs(headingError) < 5.0) {
            double scale = std::fabs(headingError) / 5.0;  // 0.0 to 1.0
            scale = std::max(0.3, scale);  // Minimum 30% power
            turnV *= scale;
        }
        
        // Cap when very close
        if(std::fabs(headingError) < 2.0) {
            turnV = clampd(turnV, -0.4, 0.4);
        }
        
        // DEBUG: Show turn voltage
        if(debug_counter % 20 == 0) {
            Controller1.Screen.setCursor(2, 1);
            Controller1.Screen.print("V:%.2f", turnV);
        }
        
        set_drive_volt(turnV, -turnV);
        vex::wait(dt_ms, vex::msec);
    }
    
    set_drive_volt(0, 0);
    L1.stop(); L2.stop(); L3.stop();
    R1.stop(); R2.stop(); R3.stop();
}

int current_auton_selection = 5;
bool auto_started = false;
bool airspace = false;
bool ran_auton = false; // 是否已經跑auto模式


void cylinderSwitch()
{
  intakeCylinder = !intakeCylinder;
}
void intakeCylinderon()
{
  airspace = !airspace;
  intakeCylinder = airspace;
}
void intakeCylinderoff()
{
  intakeCylinder = false;
}
void blockswitch()
{
  blockCylinder = !blockCylinder;
}
void park(){
  trackingWheel = true;
}



// ========== Color Sorting Background Task ==========
// Color sorting modes
enum ColorSortMode {
  COLOR_SORT_LID = 1,      // Mode 1: Close/open lid through blockCylinder
  COLOR_SORT_SHOOTER = 2,  // Mode 2: Spin shooterLower forward if wrong color
  COLOR_SORT_INTAKE = 3    // Mode 3: Stop intake1 if wrong color
};

// Global flags for color sorting control
static bool colorSortEnabled = false;
static vex::color targetColor = vex::color::black;
static ColorSortMode colorSortMode = COLOR_SORT_SHOOTER;

// Enable/disable color sorting with mode selection
// mode: 1 = lid control, 2 = shooter control, 3 = intake control
void enableColorSort(vex::color color, int mode = 1) {
  colorSortEnabled = true;
  targetColor = color;
  colorSortMode = (ColorSortMode)mode;
}

void disableColorSort() {
  colorSortEnabled = false;
}

// Background task for color sorting - runs continuously, only active when enabled
// Start this once in autonomous: task colorSort(ColorSortTask);
// Then use enableColorSort(vex::color::red, 1) for lid mode
// Or enableColorSort(vex::color::red, 2) for shooter mode
// Or enableColorSort(vex::color::red, 3) for intake mode
// Use disableColorSort() to turn it off
int ColorSortTask() {
  while(true) {
    // Only process if enabled
    if(colorSortEnabled && targetColor != vex::color::black) {
      if(OpticalFirst.isNearObject()) {
        vex::color detectedColor = OpticalFirst.color();
        bool isWrongColor = false;
        
        // Determine if wrong color
        if(targetColor == vex::color::red) { //want red
          isWrongColor = (detectedColor == vex::color::blue); //see blue
        } else if(targetColor == vex::color::blue) { //want blue
          isWrongColor = (detectedColor != vex::color::blue); //see red
        }
        
        // Handle based on mode
        switch(colorSortMode) {
          case COLOR_SORT_LID: {
            // Mode 1: Close/open lid through blockCylinder
            if(isWrongColor) {
              blockCylinder = false; // Close lid to block wrong color
            } else {
              blockCylinder = true; // Open lid to allow correct color
            }
            break;
          }
          
          case COLOR_SORT_SHOOTER: {
            // Mode 2: If wrong color, spin shooterLower forward (should be reverse)
            if(isWrongColor) {
              shooterLower.spin(forward, 12, volt); // Spin forward to reverse direction
              blockCylinder = false; // Close lid to block block from falling out
            } else {
              // Correct color - let shooter run normally (don't interfere)
              // shooterLower will be controlled by other code
            }
            break;
          }
          
          case COLOR_SORT_INTAKE: {
            // Mode 3: If wrong color, stop intake1 (should be scoring/reverse)
            if(isWrongColor) {
              intake1.stop(brake); // Stop intake when wrong color
            } else {
              // Correct color - let intake run normally (don't interfere)
              // intake1 will be controlled by other code
            }
            break;
          }
        }
      }
    }
    wait(50, msec);
  }
  return 0;
}

   

struct Rect { int x, y, w, h; };

static inline void fillRect(const Rect& r, vex::color fill, vex::color pen = vex::transparent) {
  Brain.Screen.setFillColor(fill);
  Brain.Screen.setPenColor(pen);
  Brain.Screen.drawRectangle(r.x, r.y, r.w, r.h);
}
static inline void strokeRect(const Rect& r, vex::color pen) {
  Brain.Screen.setFillColor(vex::transparent);
  Brain.Screen.setPenColor(pen);
  Brain.Screen.drawRectangle(r.x, r.y, r.w, r.h, vex::transparent);
}
void drawLogo() {
  static const char* imageColors[] = {
      "#fdbe00", "#fdbf00", "#fcbe00", "#ffb600", "#fcbd00", "#fdbd00", "#ffbe00", "#ffb900", "#fcbf00", "#fbbf00", "#ff8000", "#fabe00", "#ffcc00", "#ffbf00", "#ffbc00", "#fbbc00", "#fbbe00", "#ffbb00", "#ffff00", "#ffc100", "#ffc200", "#ffba00", "#f9c100", "#ffc600", "#fbbd00", "#ffbd00", "#fabf00", "#fac100", "#ffc400", "#ffb300", "#ffaa00", "#ffc300", "#ffb800", "#f9be00", "#fabc00", "#f9bf00", "#f9bc00", "#fabd00", 
  };

  static const int imageIndices[] = {
      -1, 0, 1, 2, 3, -1, 4, 1, 5, 6, -1, 7, 8, 1, 5, 9, 10, -1, 11, 0, 1, 0, 2, 12, -1, 1, 5, 13, -1, 14, 1, 0, 14, -1, 2, 15, -1, 16, 2, 17, -1, 14, 2, 18, -1, 19, 2, 9, -1, 18, 2, 4, 7, -1, 2, 20, -1, 2, 8, -1, 16, 2, 21, -1, 22, 2, 10, -1, 19, 2, -1, 18, 2, 3, -1, 2, 5, 23, -1, 2, 13, -1, 16, 2, 4, -1, 20, 2, 0, -1, 2, 14, -1, 18, 2, 24, -1, 2, 5, -1, 2, 4, -1, 16, 2, 25, -1, 2, 26, -1, 27, 2, 0, -1, 18, 2, 3, -1, 2, -1, 16, 24, 16, 2, 1, -1, 24, 9, 15, 4, 1, 2, 5, -1, 14, 2, 5, 9, 16, 11, 16, 2, 4, -1, 8, 2, 8, 9, 4, 2, 24, -1, 9, 5, 2, 4, -1, 26, 24, 2, -1, 28, 2, 1, -1, 8, 2, 8, -1, 9, 2, 9, -1, 18, 2, 14, -1, 2, 16, -1, 26, 2, 3, -1, 4, 2, 16, -1, 10, 2, -1, 13, 9, 2, 9, -1, 8, 2, 8, 18, 2, -1, 19, 2, 0, -1, 24, 2, 0, -1, 26, 2, 13, -1, 2, -1, 13, 0, 2, -1, 29, 2, 8, 2, 0, 12, -1, 19, 2, 24, 13, 2, 26, -1, 2, -1, 2, 0, 13, 12, -1, 3, 2, 30, -1, 26, 2, 5, -1, 10, 16, 2, 14, -1, 14, 2, 26, -1, 20, 2, 4, 8, 2, 13, -1, 1, 2, -1, 24, 2, -1, 21, 2, 26, -1, 14, 0, 2, 0, -1, 13, 8, 2, 16, -1, 14, 2, 12, 0, 2, -1, 8, 2, -1, 17, 2, 8, -1, 13, 2, -1, 27, 2, 0, -1, 31, 0, 2, 16, 7, -1, 14, 2, 16, -1, 24, 2, -1, 2, 18, -1, 4, 2, 18, -1, 13, 2, 8, -1, 13, 8, 2, 27, -1, 25, 24, 2, 8, 4, 12, -1, 14, 2, 16, 18, 1, 2, -1, 4, 2, -1, 32, 2, 16, -1, 13, 2, -1, 28, 2, 16, 0, 32, -1, 29, 0, 2, 16, 19, -1, 14, 2, 9, 4, 2, 17, -1, 16, 2, -1, 0, 2, 8, -1, 13, 2, 11, -1, 2, 4, 9, 18, -1, 0, 2, 23, -1, 10, 31, 33, 2, 5, 24, 2, -1, 8, 2, -1, 2, 26, 31, 19, 14, 2, -1, 10, 31, 20, 9, 2, 0, -1, 34, 2, 16, 5, 20, -1, 2, 24, 14, -1, 24, 2, 24, 23, 2, 8, -1, 6, 2, 16, -1, 8, 2, 0, -1, 35, 2, 28, -1, 2, -1, 16, 2, 4, 15, 18, -1, 2, 29, -1, 16, 2, 9, -1, 2, 8, -1, 18, 2, 24, -1, 2, 5, -1, 13, 2, 13, -1, 2, 16, -1, 2, 34, -1, 2, 0, 2, 16, -1, 19, 14, 27, 11, 2, 5, -1, 2, 16, 4, 24, 9, 8, 2, 16, 18, -1, 4, 2, 15, 11, 2, 0, -1, 14, 2, -1, 8, 2, -1, 2, -1, 16, 2, 14, -1, 30, 16, 2, 8, -1, 15, 2, 9, -1, 18, 2, 20, -1, 2, -1, 0, 2, -1, 16, 2, -1, 16, 2, 0, 13, -1, 2, 19, -1, 18, 2, -1, 2, -1, 13, 24, 2, -1, 16, 2, 14, -1, 0, 2, 19, -1, 13, 2, 9, -1, 18, 2, 17, -1, 10, 2, -1, 19, 0, 2, -1, 16, 2, 16, 22, -1, 15, 0, 2, 7, -1, 13, 0, 2, 0, 18, -1, 18, 2, 36, -1, 21, 2, -1, 13, 2, 16, 2, 8, 24, 8, -1, 16, 2, 16, 4, 13, -1, 18, 15, 0, 1, 24, 8, 2, 5, 19, -1, 12, 8, 2, 8, 20, -1, 18, 2, 24, 26, 18, -1, 18, 15, 9, 2, 1, -1, 13, 33, 2, 4, 9, 35, 32, -1, 16, 2, 13, -1, 4, 2, 15, -1, 27, 2, 4, 24, 12, -1, 2, 11, -1, 2, 8, 2, 9, 13, 18, -1, 25, 0, 2, 9, 2, 7, -1, 2, 17, -1, 2, -1, 5, 2, 0, 3, -1, 2, 16, -1, 8, 2, 24, -1, 23, 2, 24, 30, -1, 2, 17, -1, 16, 2, -1, 5, 2, 13, -1, 2, 16, -1, 8, 2, 13, -1, 9, 2, 29, -1, 2, 17, -1, 2, -1, 5, 2, 13, -1, 2, 16, -1, 8, 2, 16, 25, -1, 18, 4, 2, -1, 2, 17, -1, 18, 2, 30, -1, 5, 2, 0, -1, 2, 16, -1, 8, 2, 8, 18, -1, 13, 2, 11, -1, 2, 17, -1, 17, 2, 13, -1, 5, 2, 13, -1, 2, 16, -1, 8, 2, -1, 4, 2, -1, 2, 17, -1, 14, 2, 24, -1, 5, 2, 0, -1, 2, 16, -1, 8, 2, -1, 4, 2, 17, -1, 2, 17, -1, 26, 2, 8, -1, 5, 2, 18, -1, 2, 16, -1, 8, 2, 19, -1, 37, 2, 4, -1, 2, 17, -1, 16, 2, 5, 11, 17, 13, 29, 33, 9, 5, 2, 1, -1, 5, 2, 33, -1, 2, 16, -1, 8, 2, 1, 24, 2, 8, -1, 19, 2, 1, 19, 2, 0, -1, 2, 17, -1, 4, 2, 25, 30, 2, 8, 2, -1, 5, 2, 23, 14, 2, 0, -1, 2, 1, -1, 8, 2, 8, -1, 11, 2, 5, -1, 13, 2, 0, -1, 2, 16, -1, 2, 9, 13, -1, 1, 2, 9, 13, 1, 2, 8, -1, 5, 2, -1, 19, 2, 4, -1, 2, 23, -1, 8, 2, 8, -1, 2, -1, 2, 5, -1, 13, 2, -1, 2, -1, 0, 2, 33, 37, 5, 2, 0, -1, 5, 2, -1, 30, 2, -1, 2, -1, 8, 2, 8, -1, 2, -1, 2, 1, -1, 0, 2, -1, 2, -1, 19, 16, 2, -1, 5, 2, -1, 2, 24, -1, 2, -1, 8, 2, 8, -1, 2, -1, 16, 2, 5, -1, 2, -1, 2, -1, 21, 14, 2, 3, -1, 5, 2, -1, 2, -1, 2, -1, 8, 2, 8, -1, 2, -1, 0, 2, 0, -1, 16, 2, 16, -1, 2, -1, 24, 34, 1, 2, 22, -1, 5, 2, -1, 2, -1, 2, -1, 8, 2, 8, -1, 2, 16, -1, 2, 5, -1, 29, 2, 5, -1, 2, -1, 29, 2, 15, 8, 2, 8, 2, -1, 5, 2, -1, 2, -1, 2, -1, 8, 2, 8, -1, 2, 5, -1, 2, 5, -1, 8, 2, 24, -1, 2, -1, 6, 2, 11, 2, 4, -1, 13, 2, 5, -1, 5, 2, -1, 2, -1, 2, -1, 8, 2, 8, -1, 26, 2, 11, -1, 2, 16, 9, 2, 13, -1, 2, 13, -1, 9, 2, 36, 24, 2, 14, -1, 5, 2, 9, -1, 5, 2, -1, 13, 2, 0, -1, 2, -1, 8, 2, 5, 24, 2, 18, -1, 2, -1, 2, 17, -1, 2, 0, 24, 2, -1, 10, 0, 2, -1, 5, 2, -1, 4, 2, 16, -1, 2, 16, -1, 8, 2, 1, -1, 2, 6, -1, 2, 17, -1, 1, 2, 8, 2, 7, -1, 0, 2, -1, 5, 2, 0, 2, 13, -1, 2, 16, -1, 8, 2, 0, 30, -1, 2, 8, -1, 2, 17, -1, 2, 8, 1, 19, -1, 2, -1, 5, 2, -1, 2, 16, -1, 8, 2, 19, -1, 2, 0, 19, -1, 2, 17, -1, 2, -1, 10, 4, 2, 13, -1, 5, 2, 13, -1, 2, 16, -1, 8, 2, 5, 0, 28, -1, 2, 16, 14, -1, 2, 17, -1, 16, 2, 0, -1, 16, 2, 25, -1, 5, 2, 4, -1, 2, 16, -1, 8, 2, 9, -1, 2, 25, -1, 2, 17, -1, 2, 0, -1, 26, 2, 15, -1, 5, 2, 31, -1, 2, 16, -1, 8, 2, 14, -1, 2, 13, -1, 2, 17, -1, 14, 2, 4, -1, 13, 2, 8, -1, 5, 2, 27, -1, 2, 16, -1, 8, 2, 8, -1, 2, 1, 2, -1, 2, 17, -1, 26, 2, 17, -1, 23, 2, 0, -1, 5, 2, 11, -1, 2, 16, -1, 8, 2, 4, 2, 14, -1, 2, 19, -1, 19, 2, -1, 2, 17, -1, 4, 2, 27, -1, 13, 2, 8, -1, 5, 2, 16, 3, -1, 2, 16, -1, 8, 2, -1, 2, 0, -1, 2, 19, -1, 0, 2, 10, -1, 2, 11, 30, -1, 1, 2, 16, -1, 17, 2, -1, 5, 2, 24, 9, 19, 10, -1, 2, 7, -1, 8, 2, 8, -1, 16, 2, 4, -1, 2, 19, 37, 26, 16, 2, 14, -1, 2, -1, 24, 2, -1, 5, 2, 0, -1, 5, 2, -1, 2, -1, 8, 2, 8, -1, 5, 2, 0, -1, 2, 19, 14, 2, 1, 5, 2, 8, -1, 2, -1, 2, 4, 13, -1, 0, 2, -1, 5, 2, -1, 2, -1, 8, 2, 4, -1, 2, -1, 2, 19, -1, 11, 2, 19, -1, 24, 2, 0, -1, 2, -1, 8, 2, 33, 16, 2, -1, 18, 2, 31, -1, 5, 2, -1, 2, -1, 8, 2, -1, 9, 2, 28, -1, 2, 19, -1, 19, 4, 8, -1, 11, 2, 1, -1, 2, -1, 2, 35, 8, 2, -1, 18, 16, 2, 8, 5, 2, 26, -1, 5, 2, -1, 2, -1, 8, 2, 8, -1, 14, 2, 14, -1, 2, 19, -1, 32, -1, 13, 2, 16, -1, 2, -1, 30, 2, 0, 2, -1, 13, 2, 24, 1, 0, 2, -1, 5, 2, -1, 2, -1, 8, 2, 8, -1, 13, 2, 9, -1, 2, 19, -1, 2, 16, -1, 2, -1, 13, 2, 16, 2, -1, 15, 2, 8, -1, 5, 2, -1, 2, -1, 8, 2, 4, -1, 10, 2, -1, 2, 19, -1, 10, -1, 0, 2, 24, -1, 2, -1, 15, 2, 16, -1, 8, 2, 11, -1, 5, 2, -1, 2, -1, 8, 2, -1, 2, 9, -1, 2, 19, -1, 2, 5, 7, -1, 8, 2, 16, -1, 2, 4, 29, -1, 2, 4, -1, 0, 2, 8, -1, 5, 2, -1, 2, 1, -1, 8, 2, 1, -1, 5, 2, -1, 2, 19, -1, 28, 2, 0, 2, 9, -1, 2, 17, -1, 5, 2, 19, -1, 5, 2, 8, -1, 5, 2, -1, 2, 16, -1, 8, 2, 5, -1, 2, 8, -1, 2, 19, -1, 2, 0, -1, 2, 17, -1, 16, 2, -1, 0, 2, 0, -1, 5, 2, -1, 2, 16, -1, 8, 2, 5, -1, 2, -1, 2, 19, -1, 27, 2, 0, -1, 2, 17, -1, 8, 2, 0, -1, 1, 2, -1, 5, 2, -1, 2, 16, -1, 8, 2, 0, -1, 24, 2, 10, -1, 2, 19, -1, 2, 5, -1, 2, 17, -1, 2, 13, -1, 16, 2, 31, 5, 2, -1, 2, 16, -1, 8, 2, 1, -1, 0, 2, 14, -1, 2, 19, -1, 8, 2, 5, -1, 2, 17, -1, 2, 9, -1, 17, 2, 26, 5, 2, -1, 2, 16, -1, 8, 2, 5, -1, 8, 2, 27, -1, 2, 19, -1, 30, 2, 9, -1, 2, 17, 13, 2, 14, -1, 16, 2, 8, 5, 2, -1, 2, 16, -1, 8, 2, 5, -1, 2, -1, 2, 19, -1, 5, 2, -1, 2, 17, 19, 2, 0, -1, 8, 2, 1, 5, 2, -1, 2, 16, -1, 8, 2, 0, -1, 15, 2, 8, -1, 2, 19, -1, 23, 2, 0, -1, 2, 17, 24, 2, 0, 3, -1, 5, 2, 16, 5, 2, -1, 2, 16, -1, 8, 2, 1, -1, 25, 2, 0, -1, 2, 19, -1, 1, 2, 6, -1, 2, 17, 8, 2, 11, -1, 2, 8, 5, 2, -1, 2, 16, -1, 8, 2, 5, -1, 28, 2, -1, 2, 4, 30, -1, 29, 2, -1, 5, 8, 13, 8, 4, -1, 1, 8, 16, 2, 8, 24, -1, 2, 8, -1, 24, 8, -1, 2, 8, 5, -1, 16, 2, 15, -1, 2, 19, -1, 13, 2, -1, 18, 2, 20, -1, 24, 2, 4, -1, 18, 8, 2, 13, -1, 22, 2, 8, 22, -1, 19, 4, 19, -1, 23, -1, 2, 9, -1, 23, -1, 11, 2, 8, 0, 2, 7, -1, 13, 24, 2, 13, -1, 2, 1, 19, 24, 2, 26, 4, 16, -1, 2, 0, 29, -1, 3, 0, 2, 7, -1, 5, 2, 16, -1, 15, 2, 13, -1, 13, 0, 2, 30, -1, 23, 2, 0, 10, -1, 2, 16, 2, 18, -1, 32, 16, 2, 4, -1, 4, 2, 8, 2, 16, 11, 2, 4, -1, 8, 2, 5, 2, 10, -1, 15, 4, 2, 13, -1, 15, -1, 2, 16, -1, 9, -1, 2, 0, 2, 30, -1, 13, 5, 2, 0, 30, -1, 19, 2, 1, -1, 2, 0, 13, -1, 10, 24, 4, 2, 33, -1, 23, -1, 13, 2, 0, -1, 13, -1, 7, 2, 8, 13, -1, 10, 15, 2, 8, 18, -1, 8, 5, 18, 4, 2, -1, 9, 24, -1, 22, 2, 13, -1, 30, 9, 2, 16, 3, -1, 2, 0, 2, 8, 2, 0, -1, 3, 9, 2, 8, 2, 13, -1, 14, 4, 2, 14, -1, 33, 2, 8, -1, 20, 24, 2, 9, 24, 13, -1, 13, 4, 16, 2, 11, -1, 13, 2, 11, -1, 32, 5, 2, 8, 16, 8, 13, -1, 10, 14, 8, 9, 5, 2, 9, -1, 2, 3, -1, 12, 8, 4, 2, 8, 0, 4, 13, 12, -1, 21, 9, 8, 2, 4, 37, -1, 16, 2, 0, -1, 13, 5, 2, 16, 2, 24, 0, 4, 26, 21, 13, -1, 14, 6, 9, 8, 1, 16, 4, 16, 2, 13, -1, 9, 2, -1, 32, 8, 2, 8, 2, 9, 24, 16, 1, 24, 2, 8, 4, 2, 0, 2, 1, 14, -1, 29, 2, -1, 10, 37, 0, 2, 30, -1, 2, 19, -1, 28, 2, 24, 2, 24, 1, 25, -1, 4, 2, 0, -1, 7, 15, 0, 8, 2, 16, 11, -1, 13, 2, 0, -1, 10, 22, 2, 8, 2, 0, 36, -1, 2, 13, -1, 18, 25, 8, 24, 8, 2, 0, 16, 8, 31, -1, 15, 0, 19, -1, 30, 19, 4, 1, 2, 16, 2, 0, 16, 3, -1, 30, 19, 16, 2, 5, 8, 4, 2, 24, 2, 8, 4, 9, 1, 9, 25, 30, -1, 13, 19, 35, 15, 16, 4, 2, 5, 1, 5, 2, 8, 4, 2, 9, 26, 13, 3, -1, 
  };

  static const int imageCounts[] = {
      30416, 1, 15, 1, 1, 4, 1, 12, 1, 1, 10, 1, 1, 8, 1, 1, 1, 10, 1, 1, 7, 1, 1, 1, 5, 13, 1, 1, 5, 1, 15, 1, 1, 355, 18, 1, 3, 1, 15, 1, 6, 1, 15, 1, 6, 1, 13, 1, 3, 1, 15, 1, 1, 3, 19, 1, 353, 19, 1, 2, 1, 16, 1, 4, 1, 17, 1, 4, 1, 16, 2, 1, 17, 1, 2, 19, 1, 1, 352, 20, 1, 1, 1, 16, 1, 3, 1, 18, 1, 4, 17, 1, 1, 1, 17, 1, 2, 20, 1, 352, 20, 1, 1, 1, 17, 1, 2, 20, 1, 2, 1, 17, 1, 1, 1, 18, 1, 1, 21, 352, 1, 13, 1, 5, 1, 1, 1, 7, 1, 3, 1, 5, 1, 1, 1, 6, 1, 4, 1, 1, 1, 6, 1, 2, 1, 5, 1, 5, 1, 5, 1, 2, 12, 1, 5, 1, 1, 1, 14, 6, 366, 1, 5, 1, 14, 1, 4, 1, 1, 1, 5, 1, 8, 1, 6, 1, 1, 5, 1, 7, 1, 5, 1, 14, 1, 4, 1, 15, 1, 6, 364, 1, 1, 6, 1, 14, 1, 4, 1, 1, 6, 10, 1, 5, 1, 1, 1, 4, 1, 7, 1, 5, 1, 14, 6, 13, 1, 1, 7, 361, 1, 1, 1, 7, 1, 1, 13, 1, 5, 1, 1, 5, 1, 11, 6, 1, 5, 1, 1, 1, 4, 1, 6, 1, 13, 1, 5, 1, 10, 1, 1, 9, 1, 359, 1, 11, 1, 9, 1, 10, 1, 1, 5, 1, 11, 1, 5, 1, 1, 18, 9, 1, 10, 1, 8, 1, 1, 10, 1, 357, 1, 2, 11, 1, 10, 1, 10, 1, 1, 5, 12, 1, 5, 1, 1, 17, 1, 9, 1, 10, 7, 1, 12, 1, 356, 1, 1, 12, 1, 1, 11, 1, 9, 1, 1, 1, 5, 12, 6, 1, 1, 1, 16, 1, 9, 1, 9, 1, 4, 1, 1, 13, 1, 356, 1, 1, 10, 1, 1, 1, 13, 1, 9, 1, 1, 1, 5, 12, 1, 5, 1, 1, 17, 1, 9, 1, 10, 3, 1, 11, 1, 1, 1, 357, 1, 1, 9, 1, 1, 16, 1, 10, 1, 1, 5, 1, 11, 1, 5, 1, 1, 17, 1, 9, 1, 10, 1, 2, 10, 1, 1, 1, 359, 1, 9, 1, 18, 1, 4, 1, 5, 1, 1, 6, 11, 1, 5, 1, 6, 1, 4, 1, 1, 6, 9, 1, 3, 1, 1, 5, 1, 1, 1, 7, 1, 1, 1, 362, 6, 1, 1, 27, 1, 4, 1, 1, 5, 1, 10, 1, 5, 1, 1, 1, 4, 1, 7, 1, 5, 1, 14, 6, 1, 1, 5, 1, 1, 1, 364, 5, 1, 29, 1, 4, 1, 1, 6, 1, 8, 1, 6, 1, 1, 5, 1, 7, 1, 5, 1, 14, 5, 1, 1, 5, 1, 367, 5, 1, 13, 1, 2, 1, 6, 1, 4, 6, 1, 1, 7, 1, 1, 1, 4, 1, 6, 1, 1, 1, 1, 5, 1, 6, 5, 1, 2, 12, 7, 1, 1, 19, 353, 20, 2, 1, 17, 1, 1, 1, 1, 19, 1, 2, 1, 17, 1, 1, 1, 18, 1, 1, 20, 353, 1, 19, 2, 1, 17, 3, 1, 18, 1, 1, 3, 17, 1, 1, 1, 18, 2, 20, 353, 1, 1, 18, 2, 1, 16, 1, 4, 1, 17, 1, 4, 1, 15, 1, 2, 1, 17, 1, 2, 1, 19, 354, 1, 1, 17, 2, 1, 14, 1, 1, 6, 1, 1, 14, 1, 6, 1, 1, 12, 1, 1, 2, 1, 16, 1, 4, 1, 18, 355, 1, 1, 1, 3, 1, 10, 1, 2, 1, 12, 1, 1, 1, 8, 1, 1, 1, 1, 2, 1, 5, 1, 1, 9, 1, 1, 9, 1, 1, 4, 1, 13, 1, 1, 1, 6, 1, 1, 1, 14, 1, 9932, 1, 1, 20, 2, 1, 1, 1, 13, 1, 19, 1, 5, 1, 18, 1, 5, 1, 16, 1, 1, 1, 9, 20, 1, 2, 15, 1, 1, 1, 1, 1, 320, 1, 1, 24, 1, 1, 1, 8, 20, 1, 5, 20, 5, 1, 19, 1, 1, 7, 20, 1, 2, 1, 20, 1, 320, 1, 26, 1, 1, 6, 20, 1, 5, 1, 19, 5, 1, 21, 1, 6, 20, 1, 2, 1, 22, 1, 320, 1, 26, 1, 5, 20, 1, 5, 20, 5, 1, 22, 1, 5, 20, 1, 2, 1, 22, 1, 1, 319, 1, 1, 26, 5, 20, 1, 4, 1, 20, 1, 4, 1, 22, 1, 5, 20, 1, 2, 1, 23, 1, 1, 319, 1, 26, 1, 4, 20, 1, 4, 1, 20, 1, 4, 1, 23, 1, 4, 20, 1, 2, 1, 25, 320, 1, 26, 4, 20, 1, 4, 1, 20, 1, 4, 1, 23, 1, 4, 20, 1, 2, 1, 25, 320, 1, 26, 1, 3, 20, 1, 4, 1, 20, 1, 4, 1, 24, 1, 3, 20, 1, 2, 1, 25, 1, 319, 1, 26, 1, 3, 20, 1, 4, 1, 6, 1, 1, 1, 1, 1, 1, 1, 1, 6, 1, 4, 1, 24, 1, 3, 20, 1, 2, 1, 12, 1, 1, 11, 1, 319, 1, 12, 1, 1, 12, 1, 3, 20, 1, 4, 1, 4, 1, 1, 2, 1, 12, 4, 1, 12, 1, 1, 10, 1, 3, 20, 1, 2, 1, 11, 1, 1, 1, 11, 1, 319, 1, 12, 1, 1, 12, 1, 3, 13, 7, 1, 4, 1, 2, 1, 1, 1, 15, 1, 4, 1, 12, 2, 1, 9, 1, 3, 12, 1, 10, 1, 11, 1, 2, 12, 320, 12, 1, 1, 1, 12, 3, 13, 12, 1, 1, 1, 1, 1, 16, 1, 4, 1, 12, 2, 1, 10, 3, 12, 11, 1, 11, 1, 2, 12, 320, 12, 1, 2, 1, 11, 3, 13, 12, 1, 1, 20, 4, 1, 12, 3, 9, 1, 3, 12, 11, 1, 11, 1, 2, 12, 320, 1, 11, 1, 2, 12, 3, 13, 12, 1, 1, 20, 1, 3, 1, 12, 3, 10, 3, 12, 11, 1, 11, 1, 2, 12, 320, 1, 11, 1, 2, 1, 10, 1, 3, 13, 12, 1, 1, 1, 19, 1, 3, 1, 12, 3, 10, 3, 12, 11, 1, 11, 1, 2, 11, 1, 320, 12, 1, 1, 1, 11, 1, 3, 13, 11, 1, 2, 1, 1, 8, 1, 10, 3, 1, 12, 3, 10, 3, 12, 11, 1, 11, 1, 2, 11, 1, 320, 12, 1, 1, 1, 11, 1, 3, 13, 11, 1, 3, 1, 7, 1, 1, 1, 8, 1, 3, 1, 12, 3, 10, 3, 12, 11, 1, 11, 1, 1, 1, 11, 1, 320, 12, 1, 1, 12, 1, 3, 20, 1, 3, 1, 4, 1, 1, 5, 1, 2, 1, 7, 1, 3, 1, 12, 2, 1, 9, 1, 3, 12, 11, 1, 11, 1, 1, 12, 1, 320, 26, 4, 20, 1, 3, 5, 1, 1, 5, 3, 1, 1, 7, 3, 1, 12, 2, 1, 9, 1, 3, 20, 1, 2, 1, 24, 1, 321, 25, 1, 4, 20, 1, 3, 1, 6, 1, 3, 1, 4, 1, 7, 3, 1, 11, 2, 11, 1, 3, 20, 1, 2, 1, 23, 1, 1, 321, 24, 1, 5, 20, 1, 3, 7, 1, 1, 1, 6, 8, 3, 1, 24, 4, 20, 1, 2, 1, 23, 1, 322, 22, 1, 1, 6, 20, 1, 3, 9, 7, 1, 1, 6, 1, 2, 1, 23, 1, 4, 20, 1, 2, 1, 20, 1, 1, 1, 323, 22, 1, 1, 6, 20, 1, 3, 1, 7, 1, 8, 1, 6, 1, 2, 1, 22, 1, 5, 20, 1, 2, 1, 21, 1, 324, 24, 1, 5, 20, 1, 3, 8, 1, 8, 1, 6, 1, 2, 1, 22, 1, 5, 20, 1, 2, 1, 22, 1, 323, 25, 1, 4, 20, 1, 2, 1, 8, 1, 8, 1, 6, 1, 2, 1, 21, 1, 6, 20, 1, 2, 1, 22, 1, 323, 13, 1, 12, 4, 20, 1, 2, 1, 8, 1, 8, 1, 6, 1, 2, 1, 20, 1, 7, 20, 1, 2, 1, 11, 2, 10, 1, 322, 12, 1, 2, 1, 10, 4, 20, 1, 2, 1, 8, 1, 8, 1, 6, 1, 2, 1, 18, 1, 1, 8, 20, 1, 2, 1, 12, 1, 10, 1, 322, 12, 1, 3, 1, 9, 1, 3, 13, 7, 1, 2, 1, 8, 1, 8, 1, 7, 2, 1, 11, 1, 4, 1, 1, 10, 12, 1, 10, 1, 11, 1, 1, 1, 9, 1, 322, 12, 1, 1, 1, 1, 10, 1, 3, 13, 10, 1, 9, 9, 1, 5, 1, 2, 1, 12, 16, 12, 11, 1, 11, 1, 1, 1, 9, 1, 322, 12, 1, 1, 3, 1, 1, 7, 1, 3, 13, 10, 9, 1, 2, 7, 1, 6, 2, 1, 12, 16, 12, 11, 1, 11, 1, 1, 11, 322, 12, 1, 1, 1, 2, 1, 1, 1, 6, 1, 3, 13, 10, 1, 8, 1, 1, 1, 7, 1, 6, 1, 1, 1, 12, 16, 12, 11, 1, 12, 1, 1, 10, 1, 321, 12, 1, 2, 1, 1, 1, 1, 1, 6, 1, 3, 13, 10, 9, 1, 1, 1, 5, 1, 1, 1, 1, 1, 4, 1, 1, 1, 12, 16, 12, 11, 1, 11, 1, 1, 1, 10, 1, 321, 12, 1, 4, 1, 1, 1, 6, 1, 3, 13, 9, 1, 9, 1, 2, 5, 1, 3, 1, 1, 1, 3, 1, 1, 12, 16, 12, 11, 1, 11, 1, 1, 1, 10, 1, 321, 12, 1, 7, 6, 1, 3, 13, 9, 1, 10, 1, 1, 5, 1, 8, 1, 1, 1, 12, 16, 12, 11, 1, 11, 1, 1, 1, 11, 321, 12, 1, 3, 1, 3, 1, 5, 1, 3, 13, 9, 1, 11, 1, 5, 1, 8, 1, 1, 1, 12, 16, 12, 11, 1, 12, 2, 10, 1, 321, 12, 1, 3, 1, 1, 1, 1, 1, 5, 1, 3, 13, 7, 1, 1, 12, 1, 5, 1, 8, 1, 1, 1, 12, 16, 20, 1, 2, 1, 11, 1, 2, 1, 10, 321, 12, 1, 3, 1, 2, 1, 6, 1, 3, 20, 1, 1, 1, 11, 1, 5, 1, 8, 1, 1, 1, 12, 16, 20, 1, 2, 1, 11, 1, 2, 10, 1, 321, 12, 1, 4, 9, 1, 3, 20, 1, 1, 1, 11, 6, 1, 8, 1, 1, 1, 12, 16, 20, 1, 2, 1, 11, 1, 2, 11, 321, 12, 1, 4, 1, 8, 1, 3, 20, 1, 1, 1, 10, 1, 6, 1, 9, 1, 1, 12, 16, 20, 1, 2, 1, 11, 1, 2, 1, 10, 1, 320, 12, 1, 5, 8, 1, 3, 20, 1, 1, 11, 1, 6, 1, 9, 1, 1, 12, 16, 20, 1, 2, 1, 11, 1, 2, 1, 10, 1, 320, 12, 1, 5, 1, 7, 1, 3, 20, 1, 1, 10, 1, 7, 1, 9, 1, 1, 12, 16, 20, 1, 2, 1, 11, 1, 2, 1, 10, 1, 320, 12, 1, 5, 1, 7, 1, 3, 20, 1, 1, 10, 1, 8, 1, 8, 1, 1, 12, 16, 20, 1, 2, 1, 11, 1, 2, 12, 320, 12, 1, 6, 1, 7, 3, 20, 1, 1, 9, 1, 9, 1, 8, 1, 1, 12, 16, 20, 1, 2, 1, 11, 1, 2, 1, 10, 1, 320, 12, 1, 6, 1, 6, 1, 3, 20, 1, 1, 8, 1, 1, 9, 1, 8, 1, 1, 12, 16, 20, 1, 2, 1, 11, 1, 2, 1, 10, 1, 320, 12, 1, 7, 1, 6, 1, 2, 20, 1, 1, 8, 1, 10, 9, 1, 1, 12, 16, 20, 1, 2, 1, 11, 1, 2, 1, 11, 320, 11, 1, 1, 7, 1, 7, 2, 1, 19, 1, 8, 1, 11, 1, 8, 1, 1, 11, 1, 16, 1, 20, 2, 1, 12, 3, 1, 9, 1, 321, 1, 8, 1, 10, 7, 1, 2, 1, 1, 124, 1, 1, 1, 322, 1, 6, 1, 11, 1, 1, 6, 1, 1, 1, 1, 1, 1, 121, 1, 1, 1, 321, 1, 2, 6, 1, 1, 1, 10, 1, 7, 1, 1, 4, 1, 117, 1, 1, 1, 1, 322, 1, 1, 1, 1, 5, 1, 1, 1, 11, 14, 1, 1, 113, 1, 1, 2, 1, 323, 1, 10, 1, 11, 1, 16, 1, 109, 1, 1, 3, 1, 324, 1, 9, 1, 1, 12, 16, 1, 1, 1, 105, 1, 1, 3, 1, 327, 1, 1, 1, 3, 1, 1, 1, 1, 13, 1, 17, 1, 1, 1, 101, 1, 1, 4, 1, 329, 1, 1, 3, 1, 1, 1, 14, 20, 1, 1, 1, 96, 1, 2, 4, 1, 1, 332, 1, 2, 1, 16, 23, 1, 1, 91, 1, 1, 1, 6, 1, 331, 1, 2, 1, 2, 1, 2, 1, 12, 1, 26, 1, 1, 85, 1, 1, 8, 1, 1, 332, 1, 1, 1, 1, 3, 1, 1, 1, 11, 1, 31, 1, 79, 1, 1, 9, 1, 1, 334, 3, 1, 3, 1, 1, 1, 11, 1, 1, 31, 1, 1, 1, 73, 1, 1, 11, 1, 336, 1, 8, 1, 13, 1, 1, 33, 1, 1, 1, 65, 1, 1, 1, 12, 1, 338, 1, 8, 1, 15, 1, 1, 34, 1, 1, 1, 1, 56, 1, 1, 1, 1, 1, 13, 1, 341, 8, 1, 17, 1, 1, 1, 36, 1, 1, 1, 1, 1, 45, 1, 1, 1, 17, 1, 1, 343, 1, 6, 1, 21, 1, 1, 39, 1, 1, 1, 1, 1, 1, 1, 1, 28, 1, 1, 1, 1, 1, 1, 1, 1, 20, 1, 345, 1, 7, 24, 1, 1, 48, 1, 2, 1, 4, 1, 4, 2, 3, 1, 1, 1, 1, 27, 1, 1, 347, 1, 7, 27, 1, 1, 1, 92, 1, 350, 6, 1, 31, 1, 1, 1, 83, 1, 1, 1, 353, 1, 4, 1, 36, 1, 1, 1, 1, 76, 1, 1, 356, 1, 4, 1, 40, 1, 1, 2, 1, 68, 1, 1, 360, 4, 1, 45, 1, 1, 1, 1, 1, 58, 1, 1, 1, 1, 364, 1, 1, 1, 52, 1, 1, 1, 1, 48, 1, 1, 1, 1, 1, 430, 1, 1, 1, 1, 1, 1, 1, 1, 1, 29, 1, 1, 1, 1, 1, 1, 1, 445, 1, 1, 1, 1, 1, 2, 1, 1, 5, 1, 2, 1, 2, 1, 1, 1, 2, 1, 30452, 
  };
  int x = 0, y = 0;
  for(int i = 0; i < sizeof(imageIndices) / sizeof(imageIndices[0]); ++i) {
      int index = imageIndices[i];
      int count = imageCounts[i];
      if(index >= 0) {
          const char* color = imageColors[index];
          Brain.Screen.setPenColor(color);
          for(int j = 0; j < count; ++j) {
              Brain.Screen.drawPixel(x++, y);
              if(x >= 480) { x = 0; y++; }
          }
      } else {
          x += count;
          while(x >= 480) { x -= 480; y++; }
      }
  }
}
static inline void drawCenteredText(const Rect& r, const std::string& s, vex::color pen=vex::white) {
  Brain.Screen.setPenColor(pen);
  int tw = Brain.Screen.getStringWidth(s.c_str());
  int th = Brain.Screen.getStringHeight("A");
  int px = r.x + (r.w - tw) / 2;
  int py = r.y + (r.h + th) / 2; // y 是 baseline
  Brain.Screen.printAt(px, py, false, s.c_str());
}

// Draw rounded rectangle with specified corner radius
static inline void drawRoundedRect(int x, int y, int w, int h, int radius, vex::color fillColor) {
  Brain.Screen.setFillColor(fillColor);
  Brain.Screen.setPenColor(fillColor);
  
  // Draw filled circles at the four corners
  Brain.Screen.drawCircle(x + radius, y + radius, radius);
  Brain.Screen.drawCircle(x + w - radius, y + radius, radius);
  Brain.Screen.drawCircle(x + radius, y + h - radius, radius);
  Brain.Screen.drawCircle(x + w - radius, y + h - radius, radius);
  
  // Draw the main rectangle parts (excluding corners)
  Brain.Screen.drawRectangle(x + radius, y, w - 2*radius, h);
  Brain.Screen.drawRectangle(x, y + radius, radius, h - 2*radius);
  Brain.Screen.drawRectangle(x + w - radius, y + radius, radius, h - 2*radius);
}

// Modern joystick bar design
static inline void drawAxisBarLabeled(const char* name, int val, const Rect& labelBox, const Rect& barBox) {
  // Label (minimal, no background)
  Brain.Screen.setPenColor(vex::color(180, 180, 180));
  Brain.Screen.setFont(vex::fontType::mono12);
  int labelX = labelBox.x + (labelBox.w - Brain.Screen.getStringWidth(name)) / 2;
  Brain.Screen.printAt(labelX, labelBox.y + labelBox.h - 2, false, name);

  // Modern bar background - subtle dark
  fillRect(barBox, vex::color(20, 20, 25));
  strokeRect(barBox, vex::color(50, 50, 55));

  // Center line (subtle)
  int zeroX = barBox.x + barBox.w/2;
  Brain.Screen.setPenColor(vex::color(40, 40, 45));
  Brain.Screen.drawLine(zeroX, barBox.y+1, zeroX, barBox.y + barBox.h - 2);

  // Value bar - modern gradient-like effect with gold accent
  int v = val; if(v>100) v=100; if(v<-100) v=-100;
  int half = (barBox.w-2)/2;
  int pix = (v * half) / 100;
  if (pix != 0) {
    int bx = (pix>0) ? zeroX : (zeroX+pix);
    int bw = (pix>0) ? pix : -pix;
    // Use gold for positive, subtle red-orange for negative
    vex::color barColor = (pix>0) ? vex::color(252, 190, 0) : vex::color(200, 100, 80);
    fillRect({bx, barBox.y+2, bw, barBox.h-4}, barColor);
  }

  // Value text - subtle
  char buf[16];
  snprintf(buf, sizeof(buf), "%4d", val);
  Brain.Screen.setPenColor(vex::color(200, 200, 200));
  Brain.Screen.setFont(vex::fontType::mono12);
  int textX = barBox.x + (barBox.w - Brain.Screen.getStringWidth(buf)) / 2;
  Brain.Screen.printAt(textX, barBox.y + barBox.h - 2, false, buf);
}

// Modern button design - minimal, clean aesthetic
static inline void drawButtonBox(const char* name, bool pressed, const Rect& r) {
  if (pressed) {
    // Pressed: subtle gold fill with gold border
    fillRect(r, vex::color(252, 190, 0));  // Gold fill
    strokeRect(r, vex::color(255, 220, 100));  // Lighter gold border
    drawCenteredText(r, name, vex::color(20, 20, 20));  // Dark text on gold
  } else {
    // Unpressed: transparent with subtle border
    fillRect(r, vex::transparent);
    strokeRect(r, vex::color(60, 60, 70));  // Subtle border
    drawCenteredText(r, name, vex::color(180, 180, 180));  // Gray text
  }
}

// 小型指南針（顯示 heading 0~360）
static inline void drawCompass(const Rect& r, double heading_deg) {
  fillRect(r, vex::color(30,30,30)); strokeRect(r, vex::color(100,100,100));
  // 圓
  int cx = r.x + r.w/2, cy = r.y + r.h/2;
  int rad = (r.w<r.h ? r.w : r.h)/2 - 4;
  Brain.Screen.setPenColor(vex::color(160,160,160));
  Brain.Screen.drawCircle(cx, cy, rad);
  // N 標記
  Brain.Screen.printAt(cx-4, r.y+12, "N");
  // 指針（0 度朝上）
  double radian = (heading_deg - 90.0) * 3.1415926535 / 180.0;
  int tipX = cx + int(rad * std::cos(radian));
  int tipY = cy + int(rad * std::sin(radian));
  Brain.Screen.setPenColor(vex::color(255,80,80));
  Brain.Screen.drawLine(cx, cy, tipX, tipY);
  // 數值
  char buf[32]; snprintf(buf, sizeof(buf), "%3.0f°", heading_deg);
  drawCenteredText({r.x, r.y + r.h - 16, r.w, 16}, buf, vex::white);
}

// 小型 ON/OFF 方塊（氣動）
static inline void drawPneuBox(const char* name, bool on, const Rect& r) {
  vex::color fill = on ? vex::color(0,110,0) : vex::color(40,40,40);
  vex::color pen  = on ? vex::color(0,220,0) : vex::color(90,90,90);
  fillRect(r, fill); strokeRect(r, pen); drawCenteredText(r, name, vex::white);
}

// ========= 分頁列（右側顯示 Auton，不擋內容） =========
enum DashTab { TAB_INPUTS=0, TAB_MOTORS=1 };
static inline DashTab drawTabs(DashTab current, const char* autonText) {
  Rect bar = {0,0,480,22}; // 矮一點，留內容空間
  fillRect(bar, vex::color(35,35,35)); strokeRect(bar, vex::color(90,90,90));

  Rect t1 = {8,  2, 92, 18};
  Rect t2 = {t1.x + t1.w + 4, 2, 92, 18};

  auto tabColor = [&](DashTab t){ return (t==current)?vex::color(60,100,180):vex::color(55,55,55); };
  fillRect(t1, tabColor(TAB_INPUTS));  strokeRect(t1, vex::color(120,120,120));
  fillRect(t2, tabColor(TAB_MOTORS));  strokeRect(t2, vex::color(120,120,120));
  drawCenteredText(t1, "Inputs", vex::white);
  drawCenteredText(t2, "Motors", vex::white);

  // 右側顯示 Auton
  Brain.Screen.setFont(vex::fontType::mono12);
  Brain.Screen.setPenColor(vex::color(220,220,220));
  Brain.Screen.printAt(216, 16, false, "Auton: %s", autonText);

  DashTab out = current;
  if (Brain.Screen.pressing()) {
    int x = Brain.Screen.xPosition(), y = Brain.Screen.yPosition();
    if (y>=t1.y && y<=t1.y+t1.h) {
      if (x>=t1.x && x<=t1.x+t1.w) out = TAB_INPUTS;
      else if (x>=t2.x && x<=t2.x+t2.w) out = TAB_MOTORS;
    }
  }
  return out;
}

extern motor L1,L2,L3,R1,R2,R3,intake1,intake2;
static motor* kMotors[] = { &L1,&R1,&L2,&R2,&L3,&R3,&intake1,&intake2};
static const char* kMotorNames[] = { "L1","R1","L2","R2","L3","R3","INTK","IDWN"};
static const int kMotorCount = sizeof(kMotors)/sizeof(kMotors[0]);

// Get autonomous path waypoints for display
static std::vector<Waypoint> getAutonPath(int autonIndex) {
  std::vector<Waypoint> path;
  
  switch(autonIndex) {
    case 0: // Right_43
      path.push_back(Waypoint(86, 116.3, 0.0, 0.0));  // Start
      path.push_back(Waypoint(100, 105, 0.0, 0.0));   // driveToXY - intake
      path.push_back(Waypoint(93.5, 95, 0.0, 0.0));   // driveToXY - approach
      // driveToXAtHeading(115, 135) - loader
      path.push_back(Waypoint(115, 116.3, 135.0, 0.0)); // Approx loader pos
      break;
    case 1: // Left_43
      path.push_back(Waypoint(54.4, 116.3, 0.0, 0.0)); // Start
      path.push_back(Waypoint(36.4, 104, 0.0, 0.0));   // driveToXY - intake
      path.push_back(Waypoint(47, 92.5, 0.0, 0.0));     // driveToXY - approach goal
      path.push_back(Waypoint(26, 116, 0.0, 0.0));     // driveToXY - loader
      break;
    case 2: // Right_7
      path.push_back(Waypoint(86, 116.3, 0.0, 0.0));    // Start
      path.push_back(Waypoint(93.5, 93.5, 0.0, 0.0));  // driveToXY
      path.push_back(Waypoint(115, 76, 0.0, 0.0));     // driveToXY - intake
      path.push_back(Waypoint(93.5, 93.5, 0.0, 0.0));  // driveToXYBackward
      // driveToXAtHeading(115, 135) - loader
      path.push_back(Waypoint(115, 116.3, 135.0, 0.0)); // Approx loader pos
      break;
    case 3: // Left_7
      path.push_back(Waypoint(54.4, 116.3, 0.0, 0.0)); // Start
      path.push_back(Waypoint(48, 92.5, 0.0, 0.0));    // driveToXY
      path.push_back(Waypoint(26, 75.5, 0.0, 0.0));    // driveToXY - intake
      path.push_back(Waypoint(47, 93.5, 0.0, 0.0));    // driveToXYBackward
      // driveToXAtHeading(24, 230) - loader
      path.push_back(Waypoint(24, 116.3, 230.0, 0.0)); // Approx loader pos
      break;
    case 4: // Solo
      path.push_back(Waypoint(86.4, 117.5, 90.0, 0.0)); // Start
      path.push_back(Waypoint(110, 110, 0.0, 0.0));     // driveToXY
      path.push_back(Waypoint(92, 92, 0.0, 0.0));       // driveToXY
      path.push_back(Waypoint(46, 94.4, 0.0, 0.0));     // driveToXY - upper goal
      // driveToXAtHeading(25, 225) - loader
      path.push_back(Waypoint(25, 116, 225.0, 0.0));   // Approx loader pos
      break;
    case 5: // Skills
      path.push_back(Waypoint(54, 117.5, 270.0, 0.0));  // Start
      path.push_back(Waypoint(24, 116.5, 0.0, 0.0));    // driveToXY - loader 1
      path.push_back(Waypoint(8.1, 90, 0.0, 0.0));     // driveToXY - cross field
      path.push_back(Waypoint(10, 37, 0.0, 0.0));      // driveToXY
      path.push_back(Waypoint(18.4, 24.9, 0.0, 0.0));  // driveToXY - long goal
      path.push_back(Waypoint(116.8, 25, 0.0, 0.0));   // driveToXY - loader 3
      path.push_back(Waypoint(131, 48, 0.0, 0.0));     // driveToXY - cross back
      path.push_back(Waypoint(126.4, 104.4, 0.0, 0.0)); // driveToXY
      path.push_back(Waypoint(119, 115.5, 0.0, 0.0));  // driveToXY - long goal
      break;
    case 6: // Record - no predefined path
      break;
    case 7: // blank1 - no predefined path
      break;
  }
  
  return path;
}

// ========= Award-Winning Status Dashboard =========
// ========== MULTI-PAGE DASHBOARD SYSTEM ==========
enum DashboardPage {
  PAGE_OVERVIEW = 0,
  PAGE_MOTORS = 1,
  PAGE_SENSORS = 2,
  PAGE_SYSTEM = 3
};

static inline void show_status_page(int selectedAuton) {
  while (Brain.Screen.pressing()) wait(10, msec);
  
  static DashboardPage currentPage = PAGE_OVERVIEW;

  const char* labels[8] = {
    "Right_43","Left_43","Right_7","Left_7","Solo",
    "Skills","Record","blank1"
  };

  // Award-winning color scheme
  const vex::color bgDark = vex::color(10, 10, 15);       // Deep dark background
  const vex::color bgPanel = vex::color(20, 20, 28);     // Panel background
  const vex::color accentGold = vex::color(252, 190, 0); // Team gold
  const vex::color accentRed = vex::color(220, 50, 50);   // Alert red
  const vex::color accentGreen = vex::color(50, 220, 100); // Success green
  const vex::color textPrimary = vex::color(255, 255, 255);
  const vex::color textSecondary = vex::color(160, 160, 170);
  const vex::color borderColor = vex::color(50, 50, 60);
  const vex::color tabActive = accentGold;
  const vex::color tabInactive = vex::color(40, 40, 50);

  Brain.Screen.setFont(vex::fontType::mono12);
  
  // Team information
  const char* teamName = "REAPER";
  const char* teamNumber = "23083Z";
  
  while (true) {
    // Clear screen
    fillRect({0,0,480,240}, bgDark);
    
    // ========== TOP HEADER BAR ==========
    Rect headerBar = {0, 0, 480, 35};
    fillRect(headerBar, bgPanel);
    strokeRect(headerBar, borderColor);
    
    // Team branding (left side)
    Brain.Screen.setPenColor(accentGold);
    Brain.Screen.setFont(vex::fontType::mono20);
    Brain.Screen.printAt(10, 25, false, "%s", teamName);
    Brain.Screen.setPenColor(textSecondary);
    Brain.Screen.setFont(vex::fontType::mono12);
    Brain.Screen.printAt(10, 10, false, "Team %s", teamNumber);
    
    // Auton selection (center)
    Brain.Screen.setPenColor(textPrimary);
    Brain.Screen.setFont(vex::fontType::mono12);
    Brain.Screen.printAt(200, 12, false, "Auton: %s", labels[selectedAuton]);
    
    // Robot state (right side)
    const char* robotState = "DISABLED";
    vex::color stateColor = textSecondary;
    if (Competition.isEnabled()) {
      if (Competition.isAutonomous()) {
        robotState = "AUTONOMOUS";
        stateColor = accentGreen;
      } else if (Competition.isDriverControl()) {
        robotState = "DRIVER";
        stateColor = accentGold;
      } else {
        robotState = "ENABLED";
        stateColor = accentGreen;
      }
    }
    Brain.Screen.setPenColor(stateColor);
    Brain.Screen.setFont(vex::fontType::mono12);
    Brain.Screen.printAt(350, 12, false, "STATE:");
    Brain.Screen.printAt(350, 25, false, "%s", robotState);
    
    // ========== TAB NAVIGATION BAR ==========
    Rect tabBar = {0, 35, 480, 28};
    fillRect(tabBar, vex::color(15, 15, 20));
    
    const char* tabNames[] = {"OVERVIEW", "MOTORS", "SENSORS", "SYSTEM"};
    int tabWidth = 120;
    int tabX = 0;
    
    // Draw tabs and handle clicks
    for (int i = 0; i < 4; i++) {
      bool isActive = (currentPage == i);
      Rect tabRect = {tabX, 35, tabWidth, 28};
      
      if (isActive) {
        fillRect(tabRect, tabActive);
        Brain.Screen.setPenColor(vex::color(20, 20, 20));
      } else {
        fillRect(tabRect, tabInactive);
        Brain.Screen.setPenColor(textSecondary);
      }
      strokeRect(tabRect, borderColor);
      
      Brain.Screen.setFont(vex::fontType::mono12);
      int textX = tabX + (tabWidth - Brain.Screen.getStringWidth(tabNames[i])) / 2;
      Brain.Screen.printAt(textX, 53, false, "%s", tabNames[i]);
      
      // Check for tab click
      if (Brain.Screen.pressing()) {
        int tx = Brain.Screen.xPosition();
        int ty = Brain.Screen.yPosition();
        if (tx >= tabX && tx < tabX + tabWidth && ty >= 35 && ty < 63) {
          while (Brain.Screen.pressing()) wait(10, msec);
          currentPage = (DashboardPage)i;
        }
      }
      
      tabX += tabWidth;
    }
    
    // ========== PAGE CONTENT AREA ==========
    Rect contentArea = {5, 68, 470, 167};
    fillRect(contentArea, bgPanel);
    strokeRect(contentArea, borderColor);
    
    // Render current page
    switch (currentPage) {
      case PAGE_OVERVIEW: {
        // ========== OVERVIEW PAGE ==========
        // Left column: Robot Position & Status
        Brain.Screen.setPenColor(accentGold);
        Brain.Screen.setFont(vex::fontType::mono12);
        Brain.Screen.printAt(15, 85, false, "ROBOT POSE");
        
        RobotPose pose = get_robot_pose();
        Brain.Screen.setPenColor(textPrimary);
        Brain.Screen.printAt(15, 105, false, "X: %.1f\"", pose.x);
        Brain.Screen.printAt(15, 125, false, "Y: %.1f\"", pose.y);
        Brain.Screen.printAt(15, 145, false, "H: %.1f", pose.heading);
        Brain.Screen.printAt(15, 165, false, "IMU: %.1f", Inertial.heading(degrees));
        
        // Battery
        double batteryVoltage = Brain.Battery.capacity();
        vex::color batteryColor = (batteryVoltage > 80) ? accentGreen : 
                                  (batteryVoltage > 50) ? accentGold : accentRed;
        Brain.Screen.setPenColor(accentGold);
        Brain.Screen.printAt(15, 190, false, "BATTERY");
        Brain.Screen.setPenColor(batteryColor);
        Brain.Screen.printAt(15, 210, false, "%.0f%%", batteryVoltage);
        
        // Center: Chassis Motor summary (all 6 drive motors)
        Brain.Screen.setPenColor(accentGold);
        Brain.Screen.printAt(200, 85, false, "CHASSIS MOTORS");
        int motorY = 105;
        // Show all 6 drive motors (L1, L2, L3, R1, R2, R3)
        motor* driveMotors[] = {&L1, &L2, &L3, &R1, &R2, &R3};
        const char* driveNames[] = {"L1", "L2", "L3", "R1", "R2", "R3"};
        for (int i = 0; i < 6; i++) {
          if (driveMotors[i]) {
            double pos = deg_to_inches(driveMotors[i]->position(degrees));
            double vel = driveMotors[i]->velocity(vex::rpm);
            double temp = driveMotors[i]->temperature(vex::celsius);
            Brain.Screen.setPenColor(textPrimary);
            Brain.Screen.printAt(200, motorY, false, "%-4s: %.1f\" %.0frpm", driveNames[i], pos, vel);
            Brain.Screen.setPenColor((temp > 50) ? accentRed : textSecondary);
            Brain.Screen.printAt(320, motorY, false, "%.0fC", temp);
            motorY += 18;
          }
        }
        
        // Right: Intake Motors & Sensors
        Brain.Screen.setPenColor(accentGold);
        Brain.Screen.printAt(380, 85, false, "INTAKES");
        Brain.Screen.setPenColor(textPrimary);
        double intakePos = deg_to_inches(intake1.position(degrees));
        double intakeVel = intake1.velocity(vex::rpm);
        double idwnPos = deg_to_inches(intake2.position(degrees));
        double idwnVel = intake2.velocity(vex::rpm);
        Brain.Screen.printAt(380, 105, false, "INTK: %.1f\" %.0frpm", intakePos, intakeVel);
        Brain.Screen.printAt(380, 123, false, "IDWN: %.1f\" %.0frpm", idwnPos, idwnVel);
        
        // Optical sensors brightness
        Brain.Screen.setPenColor(accentGold);
        Brain.Screen.printAt(380, 150, false, "OPTICAL");
        Brain.Screen.setPenColor(textPrimary);
        Brain.Screen.printAt(380, 170, false, "R: %d%%", (int)OpticalFirst.brightness());
        Brain.Screen.printAt(380, 188, false, "L: %d%%", (int)OpticalSecond.brightness());
        break;
      }
      
      case PAGE_MOTORS: {
        // ========== MOTORS PAGE ==========
        Brain.Screen.setPenColor(accentGold);
        Brain.Screen.setFont(vex::fontType::mono12);
        Brain.Screen.printAt(15, 85, false, "MOTOR DETAILED DATA");
        
        // Column headers
        Brain.Screen.setPenColor(textSecondary);
        Brain.Screen.printAt(15, 105, false, "MOTOR");
        Brain.Screen.printAt(80, 105, false, "PORT");
        Brain.Screen.printAt(130, 105, false, "POS");
        Brain.Screen.printAt(200, 105, false, "VEL");
        Brain.Screen.printAt(260, 105, false, "CURR");
        Brain.Screen.printAt(320, 105, false, "VOLT");
        Brain.Screen.printAt(380, 105, false, "TEMP");
        
        int rowY = 125;
        // Display all 8 motors (6 chassis + 2 intakes)
        for (int i = 0; i < kMotorCount; i++) {
          if (kMotors[i] && rowY < 240) {
            motor* m = kMotors[i];
            double pos = deg_to_inches(m->position(degrees));
            double vel = m->velocity(vex::rpm);
            double curr = m->current(vex::amp);
            double volt = m->voltage(vex::volt);
            double temp = m->temperature(vex::celsius);
            int port = m->index() + 1;
            
            // Motor name
            Brain.Screen.setPenColor(textPrimary);
            Brain.Screen.printAt(15, rowY, false, "%-4s", kMotorNames[i]);
            
            // Port
            Brain.Screen.setPenColor(textSecondary);
            Brain.Screen.printAt(80, rowY, false, "%2d", port);
            
            // Position
            Brain.Screen.setPenColor(textPrimary);
            Brain.Screen.printAt(130, rowY, false, "%6.1f", pos);
            
            // Velocity
            Brain.Screen.setPenColor((std::abs(vel) > 100) ? accentGreen : textPrimary);
            Brain.Screen.printAt(200, rowY, false, "%5.0f", vel);
            
            // Current
            vex::color currColor = (curr > 2.0) ? accentRed : textPrimary;
            Brain.Screen.setPenColor(currColor);
            Brain.Screen.printAt(260, rowY, false, "%4.2f", curr);
            
            // Voltage
            Brain.Screen.setPenColor(textPrimary);
            Brain.Screen.printAt(320, rowY, false, "%4.1f", volt);
            
            // Temperature (color coded)
            vex::color tempColor = (temp > 50) ? accentRed : 
                                   (temp > 40) ? accentGold : textPrimary;
            Brain.Screen.setPenColor(tempColor);
            Brain.Screen.printAt(380, rowY, false, "%4.0f", temp);
            
            rowY += 15;  // Spacing to fit all 8 motors
          }
        }
        break;
      }
      
      case PAGE_SENSORS: {
        // ========== SENSORS PAGE ==========
        Brain.Screen.setPenColor(accentGold);
        Brain.Screen.setFont(vex::fontType::mono12);
        Brain.Screen.printAt(15, 85, false, "SENSOR DATA");
        
        // Right Optical (PORT11)
        Rect opt1Panel = {10, 105, 220, 60};
        fillRect(opt1Panel, vex::color(18, 18, 25));
        strokeRect(opt1Panel, borderColor);
        Brain.Screen.setPenColor(accentGold);
        Brain.Screen.printAt(15, 120, false, "RIGHT OPTICAL");
        Brain.Screen.setPenColor(textPrimary);
        Brain.Screen.printAt(15, 140, false, "Hue: %d", (int)OpticalFirst.hue());
        Brain.Screen.printAt(15, 158, false, "Bright: %d", (int)OpticalFirst.brightness());
        vex::color optColor = OpticalFirst.color();
        Brain.Screen.printAt(120, 140, false, "Color: %s", (optColor == vex::color::red) ? "RED" : 
                            (optColor == vex::color::blue) ? "BLUE" : "OTHER");
        Brain.Screen.printAt(120, 158, false, "Object: %s", OpticalFirst.isNearObject() ? "YES" : "NO");
        
        // Left Optical (PORT15)
        Rect opt2Panel = {240, 105, 220, 60};
        fillRect(opt2Panel, vex::color(18, 18, 25));
        strokeRect(opt2Panel, borderColor);
        Brain.Screen.setPenColor(accentGold);
        Brain.Screen.printAt(245, 120, false, "LEFT OPTICAL");
        Brain.Screen.setPenColor(textPrimary);
        Brain.Screen.printAt(245, 140, false, "Hue: %d", (int)OpticalSecond.hue());
        Brain.Screen.printAt(245, 158, false, "Bright: %d", (int)OpticalSecond.brightness());
        vex::color opt2Color = OpticalSecond.color();
        Brain.Screen.printAt(350, 140, false, "Color: %s", (opt2Color == vex::color::red) ? "RED" : 
                            (opt2Color == vex::color::blue) ? "BLUE" : "OTHER");
        Brain.Screen.printAt(350, 158, false, "Object: %s", OpticalSecond.isNearObject() ? "YES" : "NO");
        
        // IMU Sensor with Calibrate Button
        Rect imuPanel = {10, 175, 220, 60};
        fillRect(imuPanel, vex::color(18, 18, 25));
        strokeRect(imuPanel, borderColor);
        Brain.Screen.setPenColor(accentGold);
        Brain.Screen.printAt(15, 190, false, "INERTIAL (IMU)");
        Brain.Screen.setPenColor(textPrimary);
        double imuHeading = Inertial.heading(degrees);
        Brain.Screen.printAt(15, 210, false, "Heading: %.1f", imuHeading);
        
        // Calibrate Button
        Rect calibButton = {150, 190, 70, 25};
        fillRect(calibButton, accentGold);
        strokeRect(calibButton, borderColor);
        Brain.Screen.setPenColor(vex::color(0, 0, 0));
        Brain.Screen.setFont(vex::fontType::mono12);
        Brain.Screen.printAt(165, 205, false, "CALIB");
        
        // Additional sensor info
        Rect extraPanel = {240, 175, 220, 50};
        fillRect(extraPanel, vex::color(18, 18, 25));
        strokeRect(extraPanel, borderColor);
        Brain.Screen.setPenColor(accentGold);
        Brain.Screen.printAt(245, 190, false, "ADDITIONAL INFO");
        Brain.Screen.setPenColor(textPrimary);
        Brain.Screen.printAt(245, 210, false, "Controller: Connected");
        Brain.Screen.printAt(245, 220, false, "Brain: V5");
        break;
      }
      
      case PAGE_SYSTEM: {
        // ========== SYSTEM PAGE ==========
        Brain.Screen.setPenColor(accentGold);
        Brain.Screen.setFont(vex::fontType::mono12);
        Brain.Screen.printAt(15, 85, false, "SYSTEM INFORMATION");
        
        // Team Info Panel
        Rect teamPanel = {10, 105, 220, 50};
        fillRect(teamPanel, vex::color(18, 18, 25));
        strokeRect(teamPanel, borderColor);
        Brain.Screen.setPenColor(accentGold);
        Brain.Screen.printAt(15, 120, false, "TEAM INFO");
        Brain.Screen.setPenColor(textPrimary);
        Brain.Screen.printAt(15, 140, false, "Name: %s", teamName);
        Brain.Screen.printAt(15, 158, false, "Number: %s", teamNumber);
        Brain.Screen.printAt(15, 175, false, "Robot: REAPER");
        
        // Port Assignment Panel (expanded to show all)
        Rect portPanel = {240, 105, 220, 130};
        fillRect(portPanel, vex::color(18, 18, 25));
        strokeRect(portPanel, borderColor);
        Brain.Screen.setPenColor(accentGold);
        Brain.Screen.printAt(245, 120, false, "PORT ASSIGNMENTS");
        Brain.Screen.setPenColor(textSecondary);
        Brain.Screen.setFont(vex::fontType::mono12);
        
        int portY = 138;
        // Drive motors
        Brain.Screen.setPenColor(textPrimary);
        Brain.Screen.printAt(245, portY, false, "L1:%2d L2:%2d L3:%2d", 
                            L1.index()+1, L2.index()+1, L3.index()+1);
        portY += 13;
        Brain.Screen.printAt(245, portY, false, "R1:%2d R2:%2d R3:%2d", 
                            R1.index()+1, R2.index()+1, R3.index()+1);
        portY += 13;
        Brain.Screen.printAt(245, portY, false, "INT:%2d IDN:%2d", 
                            intake1.index()+1, intake2.index()+1);
        portY += 13;
        Brain.Screen.printAt(245, portY, false, "R-OPT:%2d L-OPT:%2d", 
                            OpticalFirst.index()+1, OpticalSecond.index()+1);
        portY += 13;
        Brain.Screen.printAt(245, portY, false, "IMU:%2d", Inertial.index()+1);
        portY += 13;
        // Pneumatics ports
        Brain.Screen.printAt(245, portY, false, "PNEU: D,E,F,G");
        
        // Pneumatics Status
        Rect pneuPanel = {10, 165, 220, 70};
        fillRect(pneuPanel, vex::color(18, 18, 25));
        strokeRect(pneuPanel, borderColor);
        Brain.Screen.setPenColor(accentGold);
        Brain.Screen.printAt(15, 180, false, "PNEUMATICS");
        int pneuY = 200;
        auto drawPneu = [&](const char* name, bool state, int x, int y) {
          Brain.Screen.setPenColor(state ? accentGreen : textSecondary);
          Brain.Screen.printAt(x, y, false, "%s: %s", name, state ? "ON " : "OFF");
        };
        drawPneu("INTK", intakeCylinder.value(), 80, pneuY);
        break;
      }
    }
    
    // ========== CALIBRATE BUTTON HANDLING (Sensors Page) ==========
    if (currentPage == PAGE_SENSORS && Brain.Screen.pressing()) {
      int tx = Brain.Screen.xPosition();
      int ty = Brain.Screen.yPosition();
      Rect calibButton = {150, 190, 70, 25};
      if (tx >= calibButton.x && tx < calibButton.x + calibButton.w &&
          ty >= calibButton.y && ty < calibButton.y + calibButton.h) {
        while (Brain.Screen.pressing()) wait(10, msec);
        // Start calibration
        Inertial.calibrate();
        // Show calibration status
        while (Inertial.isCalibrating()) {
          Brain.Screen.setPenColor(accentGold);
          Brain.Screen.printAt(15, 230, false, "Calibrating IMU...");
          wait(50, msec);
        }
        Brain.Screen.setPenColor(accentGreen);
        Brain.Screen.printAt(15, 230, false, "Calibration Done!");
        wait(1000, msec);
      }
    }
    
    // ========== EXIT HANDLING ==========
    // Touch anywhere in header to return
    if (Brain.Screen.pressing()) {
      int ty = Brain.Screen.yPosition();
      if (ty < 35) {  // Header area
        while (Brain.Screen.pressing()) wait(10, msec);
        break;  // Return to auton selection
      }
    }
    
    // ========== RECORD MODE HANDLING ==========
    if (selectedAuton == 6) {  // Record mode
      // Show START RECORDING button on Overview page
      if (currentPage == PAGE_OVERVIEW) {
        Rect startButton = {160, 180, 160, 35};
        fillRect(startButton, accentGreen);
        strokeRect(startButton, accentGold);
        Brain.Screen.setPenColor(vex::color(20, 20, 20));
        Brain.Screen.setFont(vex::fontType::mono20);
        drawCenteredText(startButton, "START RECORDING", vex::color(20, 20, 20));
        
        if (Brain.Screen.pressing()) {
          int tx = Brain.Screen.xPosition();
          int ty = Brain.Screen.yPosition();
          if (tx >= startButton.x && tx <= startButton.x + startButton.w &&
              ty >= startButton.y && ty <= startButton.y + startButton.h) {
            while (Brain.Screen.pressing()) wait(10, msec);
            recordPath();
            break;
          }
        }
      }
    }
    
    wait(50, msec);  // Update rate
  }
}

// ======================== 第一頁（選擇auto） =========================
void pre_auton(void)
{
  
  vexcodeInit();
  default_constants();

/*// ===== SD Card Test =====
  Brain.Screen.clearScreen();
  Brain.Screen.setFont(vex::fontType::mono12);
  Brain.Screen.setPenColor(vex::white);

  if (Brain.SDcard.isInserted()) {
    Brain.Screen.printAt(10, 20, false, "SD card detected.");

    if (Brain.SDcard.exists("field.bmp")) {
      Brain.Screen.printAt(10, 40, false, "Loading field.bmp ...");
      bool ok = Brain.Screen.drawImageFromFile("field.bmp", 0, 0);
      if (ok) {
        Brain.Screen.printAt(10, 60, false, "Image loaded OK.");
      } else {
        Brain.Screen.printAt(10, 60, false, "Image load failed.");
      }
    } else {
      Brain.Screen.printAt(10, 40, false, "field.bmp not found.");
    }
  } else {
    Brain.Screen.printAt(10, 20, false, "No SD card detected.");
  }
  wait(10,sec);
  */// =======================================================================
  
  // Show cute running animation during calibration
  Brain.Screen.clearScreen();
  
  // Draw "Calibrating..." text
  Brain.Screen.setPenColor(vex::color(255, 255, 255));
  Brain.Screen.printAt(150, 180, "Calibrating IMU...");
  
  wait(50, msec);
  
/*// ===== SD Card Test =====
  Brain.Screen.clearScreen();
  Brain.Screen.setFont(vex::fontType::mono12);
  Brain.Screen.setPenColor(vex::white);

  if (Brain.SDcard.isInserted()) {
    Brain.Screen.printAt(10, 20, false, "SD card detected.");

    if (Brain.SDcard.exists("field.bmp")) {
      Brain.Screen.printAt(10, 40, false, "Loading field.bmp ...");
      bool ok = Brain.Screen.drawImageFromFile("field.bmp", 0, 0);
      if (ok) {
        Brain.Screen.printAt(10, 60, false, "Image loaded OK.");
      } else {
        Brain.Screen.printAt(10, 60, false, "Image load failed.");
      }
    } else {
      Brain.Screen.printAt(10, 40, false, "field.bmp not found.");
    }
  } else {
    Brain.Screen.printAt(10, 20, false, "No SD card detected.");
  }
  wait(10,sec);
  */// =======================================================================
  
  // Show cute running animation during calibration
  Brain.Screen.clearScreen();
  Brain.Screen.setFont(vex::fontType::mono20);
  Brain.Screen.setPenColor(vex::color(252, 190, 0));  // Gold color
  
  int animFrame = 0;
  int animX = 50;
  const int animY = 100;
  const int animSpeed = 3;
  
  Inertial.calibrate();

  while (Inertial.isCalibrating()) {
    whitelight = 0;
    
    // Clear previous frame
    Brain.Screen.setFillColor(vex::color(0, 0, 0));
    Brain.Screen.drawRectangle(animX - 5, animY - 30, 100, 60);
    
    // Draw running character animation (simple stick figure)
    animFrame++;
    int legOffset = (animFrame / 5) % 2 == 0 ? 5 : -5;  // Legs alternate
    
    // Body
    Brain.Screen.setPenColor(vex::color(252, 190, 0));
    Brain.Screen.drawLine(animX, animY - 20, animX, animY);
    
    // Head (circle)
    Brain.Screen.drawCircle(animX, animY - 25, 5);
    
    // Arms
    Brain.Screen.drawLine(animX, animY - 15, animX - 8, animY - 10);
    Brain.Screen.drawLine(animX, animY - 15, animX + 8, animY - 10);
    
    // Legs (alternating for running effect)
    Brain.Screen.drawLine(animX, animY, animX - 5, animY + 10 + legOffset);
    Brain.Screen.drawLine(animX, animY, animX + 5, animY + 10 - legOffset);
    
    // Move character
    animX += animSpeed;
    if (animX > 430) animX = 50;  // Loop back
    
    // Draw "Calibrating..." text
    Brain.Screen.setPenColor(vex::color(255, 255, 255));
    Brain.Screen.printAt(150, 180, "Calibrating IMU...");
    
    wait(50, msec);
  }
  
  // Clear animation when done
  Brain.Screen.clearScreen();
  Controller1.Screen.print("ok!");
  redlight = 1;
  whitelight = 1;
  
  // CRITICAL: Reset motors RIGHT BEFORE dashboard
  // (Must be AFTER IMU calibration to avoid accumulation during calibration)
  L1.resetPosition(); L2.resetPosition(); L3.resetPosition();
  R1.resetPosition(); R2.resetPosition(); R3.resetPosition();
  wait(0.05, sec);  // Small delay for reset to take effect

  // Color #fcbe00 = RGB(252, 190, 0)
  vex::color boxColor = vex::color(252, 190, 0);  // #fcbe00
  vex::color white = vex::color::white;
  vex::color black = vex::color::black;

  int previous_selection = -1;

  const int screen_w  = 480;
  const int screen_h  = 240;
  
  // New layout: 4 boxes on left, 4 boxes on right, logo in center
  const int box_width = 110;  // Width of each box (reduced from 140)
  const int box_height = 55;  // Height of each box
  // No rounded corners - using regular rectangles
  const int left_start_x = 10;  // Left side boxes start here
  const int right_start_x = 360;  // Right side boxes start here (adjusted for narrower boxes)
  const int center_x = 160;  // Center area for logo (between left and right boxes, adjusted)
  const int center_y = 92;   // Vertical center
  const int logo_size = 100;  // Logo size (will be scaled)
  const int box_spacing = 5;  // Spacing between boxes
  
  // Mapping: Left side boxes (autons 1, 3, 5, 7), Rigiht side boxes (autons 0, 2, 4, 6)
  // Left: Left_43(1), Left_7(3), Skills(5), blank1(7)
  // Right: Right_43(0), Right_7(2), Solo(4), Record(6)
  const int left_autons[4] = {1, 3, 5, 7};  // Left_43, Left_7, Skills, blank1
  const int right_autons[4] = {0, 2, 4, 6}; // Right_43, Right_7, Solo, Record

  const char* labels[8] = {
    "Right_43","Left_43","Right_7","Left_7","Solo",
    "Skills","Record","blank1"
  };

  Brain.Screen.setFont(vex::fontType::mono20);

  while (!auto_started)
  {
    if (current_auton_selection != previous_selection)
    {
      previous_selection = current_auton_selection;
      
      // Clear screen
      Brain.Screen.clearScreen();

      // Draw logo in center - draw AFTER boxes so it's on top
      // Logo is drawn programmatically using drawLogo() function

      // Draw left side boxes (4 boxes)
      for (int i = 0; i < 4; i++)
      {
        int auton_index = left_autons[i];
        int x = left_start_x;
        int y = 10 + i * (box_height + box_spacing);

        vex::color fillColor = (current_auton_selection == auton_index) ? white : boxColor;
        vex::color textColor = black;  // All text is black

        // Draw regular rectangle (no rounded corners)
        Brain.Screen.setFillColor(fillColor);
        Brain.Screen.setPenColor(fillColor);  // Set pen color to match fill to avoid border
        Brain.Screen.drawRectangle(x, y, box_width, box_height);

        Rect r { x, y, box_width, box_height };
        drawCenteredText(r, labels[auton_index], textColor);
      }

      // Draw right side boxes (4 boxes)
      for (int i = 0; i < 4; i++)
      {
        int auton_index = right_autons[i];
        int x = right_start_x;
        int y = 10 + i * (box_height + box_spacing);

        vex::color fillColor = (current_auton_selection == auton_index) ? white : boxColor;
        vex::color textColor = black;  // All text is black

        // Draw regular rectangle (no rounded corners)
        Brain.Screen.setFillColor(fillColor);
        Brain.Screen.setPenColor(fillColor);  // Set pen color to match fill to avoid border
        Brain.Screen.drawRectangle(x, y, box_width, box_height);

        Rect r { x, y, box_width, box_height };
        drawCenteredText(r, labels[auton_index], textColor);
      }
      
      // Draw logo AFTER boxes so it appears on top
      // This ensures logo is visible even if boxes overlap
      // Logo data format expects to start from (0,0) - the data includes positioning
      drawLogo();
    }

    if (Brain.Screen.pressing())
    {
      int touchX = Brain.Screen.xPosition();
      int touchY = Brain.Screen.yPosition();

      // Check if touch is in left side boxes
      if (touchX >= left_start_x && touchX <= left_start_x + box_width)
      {
        int box_index = (touchY - 10) / (box_height + box_spacing);
        if (box_index >= 0 && box_index < 4)
        {
          current_auton_selection = left_autons[box_index];
          show_status_page(current_auton_selection);
          wait(0.3, sec);
        }
      }
      // Check if touch is in right side boxes
      else if (touchX >= right_start_x && touchX <= right_start_x + box_width)
      {
        int box_index = (touchY - 10) / (box_height + box_spacing);
        if (box_index >= 0 && box_index < 4)
        {
          current_auton_selection = right_autons[box_index];
          show_status_page(current_auton_selection);
          wait(0.3, sec);
        }
      }
    }

    wait(20, msec);
  }
}

void autonomous(void)
{
  // CRITICAL: Start odometry task to continuously track robot position
  // This runs in the background and updates robot_pose.x, robot_pose.y, robot_pose.heading
  // Works with ANY movement function (driveToXY, chassis.drive_distance, etc.)
  start_odometry_task();
  
  auto_started = true;
  ran_auton = true;
  // 根據選擇的自動任務來決定隊伍顏色
  if (current_auton_selection >= 0 && current_auton_selection <= 4)
  {
    selectedTeamColor = vex::color::red; // 紅隊
  }
  else if (current_auton_selection >= 5 && current_auton_selection <= 7)
  {
    selectedTeamColor = vex::color::blue; // 藍隊
  }
  else
  {
    selectedTeamColor = vex::color::black; // 預設為黑隊
  }

  switch (current_auton_selection)
  {

  case 0:
    Right_43();
    break;
  case 1:
    Left_43();
    break;
  case 2:
    Right_7();
    break;
  case 3:
    Left_7();
    break;
  case 4:
    Solo();
    break;
  case 5:
    Skills();
    break;
  case 6:
    recordPath();  // Path recording mode
    break;
  case 7:
    blank1();
    break;
  }
  
}





int intakeControlTask()
{

  while (true)
  {
    // 依優先權：L1 > L2 > R1 > R2
    if (Controller1.ButtonL1.pressing())
    {
      // reverse all / score middle lower goal
      intake1.spin(reverse, 12, volt);
      intake2.spin(reverse, 12, volt);
      shooterLower.spin(reverse, 12, volt);
    }
    
    else if (Controller1.ButtonL2.pressing())
    {
      // score middle upper goal
      blockCylinder = false;
      intake1.spin(forward, 12, volt);
      intake2.spin(forward, 12, volt);
      shooterLower.spin(reverse, 12, volt);
    }

    else if (Controller1.ButtonR1.pressing() && Controller1.ButtonR2.pressing())
    {
      // score long goal
      blockCylinder = true;
      shooterUpper.spin(forward, 12, volt);
      intake1.spin(forward, 12, volt);
      intake2.spin(forward, 12, volt);
      shooterLower.spin(forward, 12, volt);
    }
      
    else if (Controller1.ButtonR1.pressing() && !Controller1.ButtonR2.pressing())
    {
      // intake and store
      blockCylinder = false;
      intake1.spin(forward, 12, volt);
      intake2.spin(forward, 12, volt);
      shooterLower.spin(forward, 12, volt);
    }
    
    else if (Controller1.ButtonR2.pressing() && !Controller1.ButtonR1.pressing())
    {
      // score long goal
      blockCylinder = true;
      shooterUpper.spin(forward, 12, volt);
    }
    
    else
    {
      // 停止
      intake1.stop(coast);
      intake2.stop(coast);
      shooterUpper.stop(coast);
      shooterLower.stop(coast);
      blockCylinder = false;
    }

    wait(20, msec);
  }
  return 0;
}
void usercontrol(void)
{
  // CRITICAL: Reset motor encoders when entering driver control
  L1.resetPosition(); L2.resetPosition(); L3.resetPosition();
  R1.resetPosition(); R2.resetPosition(); R3.resetPosition();
  
  // Stop odometry task from autonomous (not needed for driver control)
  if(odometry_task_handle != nullptr) {
    odometry_task_handle->stop();
    delete odometry_task_handle;
    odometry_task_handle = nullptr;
  }
  
  // Start background tasks
  task intake(intakeControlTask, 0);
  
  // Set up button callbacks
  Controller1.ButtonDown.pressed(blockswitch);
  Controller1.ButtonUp.pressed(park);
  Controller1.ButtonLeft.pressed(cylinderSwitch);

  
  // Main driver control loop - minimal for maximum responsiveness
  while (1)
  {
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Heading: %.1f", Inertial.heading());
    chassis.control_holonomic(); // Holonomic drive control (throttle, turn, strafe)
  wait(20, msec);
  }
}
int main()
{
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true)
  {
    wait(100, msec);
  }
}
