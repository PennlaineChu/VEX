#include "vex.h"
#include "note.h"
#include <cmath>
#include <algorithm>
extern vex::color selectedTeamColor;
using namespace vex;
competition Competition;


Drive chassis(
    // Specify your drive setup below. There are seven options:
    // ZERO_TRACKER_NO_ODOM, ZERO_TRACKER_ODOM, TANK_ONE_ENCODER, TANK_ONE_ROTATION, TANK_TWO_ENCODER, TANK_TWO_ROTATION, HOLONOMIC_TWO_ENCODER, and HOLONOMIC_TWO_ROTATION
    // For example, if you are not using odometry, put ZERO_TRACKER_NO_ODOM below:
    ZERO_TRACKER_ODOM,
    // Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
    // You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".
    // Left Motors:
    motor_group(L1, L2, L3),
    // Right Motors:
    motor_group(R1, R2, R3),
    // Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
    PORT20,
    // Input your wheel diameter: 3.25 inches
    3.25,
    // External gear ratio: USER MEASURED - 360° motor = 3.25*PI*0.75 inches
    // This means 1 motor rotation = 0.75 wheel rotations, so ratio = 0.75
    0.75,
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
    // If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.
    // FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
    // LF:      //RF:
    PORT12, -PORT14,
    // LB:      //RB:
    PORT13, -PORT20,
    // If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
    // If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
    // If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
    5,
    // Input the Forward Tracker diameter (reverse it to make the direction switch):
    -3.25,
    // Input Forward Tracker center dist_to_target (a positive dist_to_target corresponds to a tracker on the right side of the robot, negative is left.)
    // For a zero tracker tank drive with odom, put the positive dist_to_target from the center of the robot to the right side of the drive.
    // This dist_to_target is in inches:
    5.2,
    // Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
    1,
    // Sideways tracker diameter (reverse to make the direction switch):
    -2.75,
    // Sideways tracker center dist_to_target (positive dist_to_target is behind the center of the robot, negative is in front):
    5.5
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
static const double EXT_GEAR_RATIO = 0.75;  // USER MEASURED: 360° motor = 0.75 wheel circumference
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
    chassis.control_tank(100);  // Enable tank drive control
    
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

void wingSwitch()
{
  wing = !wing;
}

void cylinderSwitch()
{
  intakeCylander = !intakeCylander;
}
void intakecylanderon()
{
  airspace = !airspace;
  intakeCylander = airspace;
}
void intakecylanderoff()
{
  intakeCylander = false;
}
void shooterSwitch()
{
    shooter = !shooter;

}
void shooterOn()  
  {
   shooter = true;  
  }
void shooterOff() {
   shooter = false; 
  }

void pushSwitch()
{
  pushCylinder = !pushCylinder;
}
void pushON()  
  {
   pushCylinder = true;  
  }
void pushOFF() {
   pushCylinder = false; 
  }
bool shooterPushOn = false;

void shooterPushSwitch() {
    shooterPushOn = !shooterPushOn;
    shooter = shooterPushOn; 
    pushCylinder = shooterPushOn;  
}

int opticalLightTask() {
  int lightPower = 100;
  Optical.setLightPower(lightPower, percent);
  Optical_go.setLightPower(lightPower, percent);
  while(true){
    if(Controller1.ButtonUp.pressing()){
      lightPower += 10;
      if(lightPower >= 100) lightPower = 100;
      Optical.setLightPower(lightPower, percent);
      Optical_go.setLightPower(lightPower, percent);
      wait(50, msec);
    }else if(Controller1.ButtonDown.pressing()){
      lightPower -= 10;
      if(lightPower <= 0) lightPower = 0;  
      Optical.setLightPower(lightPower, percent);
      Optical_go.setLightPower(lightPower, percent);
      wait(50, msec);
    }
    
    if(lightPower > 0 && pushCylinder){
      Optical.setLight(ledState::on);
      Optical_go.setLight(ledState::on);
    }else{
      Optical.setLight(ledState::off);
      Optical_go.setLight(ledState::off);
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
static inline void drawCenteredText(const Rect& r, const std::string& s, vex::color pen=vex::white) {
  Brain.Screen.setPenColor(pen);
  int tw = Brain.Screen.getStringWidth(s.c_str());
  int th = Brain.Screen.getStringHeight("A");
  int px = r.x + (r.w - tw) / 2;
  int py = r.y + (r.h + th) / 2; // y 是 baseline
  Brain.Screen.printAt(px, py, false, s.c_str());
}

// 水平條（-100~100）＋內文字置中
static inline void drawAxisBarLabeled(const char* name, int val, const Rect& labelBox, const Rect& barBox) {
  // 左側標籤（置中）
  fillRect(labelBox, vex::color(30,30,30));
  strokeRect(labelBox, vex::color(80,80,80));
  drawCenteredText(labelBox, name, vex::white);

  // 條底
  strokeRect(barBox, vex::color(90,90,90));
  fillRect({barBox.x+1, barBox.y+1, barBox.w-2, barBox.h-2}, vex::color(20,20,20));

  // 0 中線
  int zeroX = barBox.x + barBox.w/2;
  Brain.Screen.setPenColor(vex::color(90,90,90));
  Brain.Screen.drawLine(zeroX, barBox.y+1, zeroX, barBox.y + barBox.h - 2);

  // 值條
  int v = val; if(v>100) v=100; if(v<-100) v=-100;
  int half = (barBox.w-2)/2;
  int pix = (v * half) / 100;
  if (pix != 0) {
    int bx = (pix>0) ? zeroX : (zeroX+pix);
    int bw = (pix>0) ? pix : -pix;
    fillRect({bx, barBox.y+2, bw, barBox.h-4}, (pix>0)?vex::color(0,140,220):vex::color(220,140,0));
  }

  // 內文字置中（顯示數值）
  char buf[16];
  snprintf(buf, sizeof(buf), "%4d", val);
  drawCenteredText(barBox, buf, vex::color(230,230,230));
}

// 按鍵方塊
static inline void drawButtonBox(const char* name, bool pressed, const Rect& r) {
  vex::color fill = pressed ? vex::color(0,110,0) : vex::color(40,40,40);
  vex::color pen  = pressed ? vex::color(0,220,0) : vex::color(90,90,90);
  fillRect(r, fill); strokeRect(r, pen); drawCenteredText(r, name, vex::white);
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

extern motor L1,L2,L3,R1,R2,R3,intake,intakedown;
static motor* kMotors[] = { &L1,&R1,&L2,&R2,&L3,&R3,&intake,&intakedown};
static const char* kMotorNames[] = { "L1","R1","L2","R2","L3","R3","INTK","IDWN"};
static const int kMotorCount = sizeof(kMotors)/sizeof(kMotors[0]);

// ========= Dashboard（雙分頁：Inputs / Motors） =========
// ========= Dashboard（雙分頁：Inputs / Motors） =========
static inline void show_status_page(int selectedAuton) {
  while (Brain.Screen.pressing()) wait(10, msec);

  const char* labels[10] = {
    "R_right","R_left","R_right_F","R_left_F","R_solo",
    "B_right","B_left","B_right_F","B_left_F","B_solo"
  };

  DashTab tab = TAB_INPUTS;
  DashTab prevTab = TAB_INPUTS;
  Brain.Screen.setFont(vex::fontType::mono12);

  // 版面參數
  const int  MARGIN  = 8;
  const int  TAB_H   = 22;
  const Rect content = { MARGIN, TAB_H + MARGIN, 480 - 2*MARGIN, 240 - (TAB_H + 2*MARGIN) };

  // 各分頁初始化旗標（只在剛切入該分頁時做一次）
  bool inputsInit = false;
  bool motorsInit = false;

  // 先鋪一次全局底色
  fillRect({0,0,480,240}, vex::color(18,18,18));

  while (true) {
    // 先畫分頁列（不清整頁，避免把 LOGO 擦掉）
    tab = drawTabs(tab, labels[selectedAuton]);

    // 分頁切換：只清「內容區」並重置 init
    if (tab != prevTab) {
      fillRect(content, vex::color(18,18,18));
      strokeRect(content, vex::color(60,60,60));
      if (tab == TAB_INPUTS)  inputsInit = false;
      if (tab == TAB_MOTORS)  motorsInit = false;
      prevTab = tab;
    }

    if (tab == TAB_INPUTS) {
      // ===== Inputs 頁 =====
      // 第一次進來：畫 LOGO（之後不再從 SD 讀，避免閃爍）
      if (!inputsInit) {
        const int barH = 20;
        const int gapY = 8;
        const int axTop = content.y;

        // A4 條下面顯示 LOGO.bmp（240x240）
        const int imgX = 35;                                        // 你要的 X
        const int imgY = axTop + 3*(barH+gapY) + barH + 10;         // A4 底下 10px

        if (Brain.SDcard.isInserted() && Brain.SDcard.exists("LOGO.bmp")) {
          Brain.Screen.drawImageFromFile("LOGO.bmp", imgX, imgY);
        } else {
          Brain.Screen.setPenColor(vex::color(200,200,200));
          Brain.Screen.printAt(imgX, imgY + 20, false, "LOGO.bmp not found");
        }
        inputsInit = true;
      }

      // —— 每幀只更新動態元件（不清整頁、也不重畫 LOGO） ——
      const int labelW = 36;
      const int barW   = (int)(content.w * 0.55);
      const int barH   = 20;
      const int gapY   = 8;
      const int axLeft = content.x;
      const int axTop  = content.y;

      int a1 = Controller1.Axis1.position();
      int a2 = Controller1.Axis2.position();
      int a3 = Controller1.Axis3.position();
      int a4 = Controller1.Axis4.position();

      drawAxisBarLabeled("A1", a1, {axLeft,             axTop + 0*(barH+gapY), labelW, barH},
                                {axLeft+labelW+6,       axTop + 0*(barH+gapY), barW,   barH});
      drawAxisBarLabeled("A2", a2, {axLeft,             axTop + 1*(barH+gapY), labelW, barH},
                                {axLeft+labelW+6,       axTop + 1*(barH+gapY), barW,   barH});
      drawAxisBarLabeled("A3", a3, {axLeft,             axTop + 2*(barH+gapY), labelW, barH},
                                {axLeft+labelW+6,       axTop + 2*(barH+gapY), barW,   barH});
      drawAxisBarLabeled("A4", a4, {axLeft,             axTop + 3*(barH+gapY), labelW, barH},
                                {axLeft+labelW+6,       axTop + 3*(barH+gapY), barW,   barH});

      // 右上小指南針
      Rect compass = { content.x + content.w - 90, axTop, 80, 80 };
      drawCompass(compass, Inertial.heading(degrees));

      // 右側：按鍵 3×4
      int gridX = content.x + content.w - (3*70 + 2*6);
      int gridY = compass.y + compass.h + 6;
      int bw    = 70, bh = 20, sp = 6;

      drawButtonBox("A",   Controller1.ButtonA.pressing(),   {gridX + 0*(bw+sp), gridY + 0*(bh+sp), bw, bh});
      drawButtonBox("B",   Controller1.ButtonB.pressing(),   {gridX + 1*(bw+sp), gridY + 0*(bh+sp), bw, bh});
      drawButtonBox("X",   Controller1.ButtonX.pressing(),   {gridX + 2*(bw+sp), gridY + 0*(bh+sp), bw, bh});

      drawButtonBox("Y",   Controller1.ButtonY.pressing(),   {gridX + 0*(bw+sp), gridY + 1*(bh+sp), bw, bh});
      drawButtonBox("L1",  Controller1.ButtonL1.pressing(),  {gridX + 1*(bw+sp), gridY + 1*(bh+sp), bw, bh});
      drawButtonBox("L2",  Controller1.ButtonL2.pressing(),  {gridX + 2*(bw+sp), gridY + 1*(bh+sp), bw, bh});

      drawButtonBox("R1",  Controller1.ButtonR1.pressing(),  {gridX + 0*(bw+sp), gridY + 2*(bh+sp), bw, bh});
      drawButtonBox("R2",  Controller1.ButtonR2.pressing(),  {gridX + 1*(bw+sp), gridY + 2*(bh+sp), bw, bh});
      drawButtonBox("Up",  Controller1.ButtonUp.pressing(),  {gridX + 2*(bw+sp), gridY + 2*(bh+sp), bw, bh});

      drawButtonBox("Down", Controller1.ButtonDown.pressing(), {gridX + 0*(bw+sp), gridY + 3*(bh+sp), bw, bh});
      drawButtonBox("Left", Controller1.ButtonLeft.pressing(), {gridX + 1*(bw+sp), gridY + 3*(bh+sp), bw, bh});
      drawButtonBox("Right",Controller1.ButtonRight.pressing(),{gridX + 2*(bw+sp), gridY + 3*(bh+sp), bw, bh});
    
      // 底部：氣動狀態
      int pneuY = content.y + content.h - 20;
      int pneuW = 68, pneuH = 18, pneuSP = 6;
      // Center the 5 boxes: total width = 5*68 + 4*6 = 364, center offset = (content.w - 364) / 2
      int totalPneuWidth = 5 * pneuW + 4 * pneuSP;
      int pneuX = content.x + (content.w - totalPneuWidth) / 2;

      drawPneuBox("no status", redlight.value(),       {pneuX + 0*(pneuW+pneuSP), pneuY, pneuW, pneuH});
      drawPneuBox("no status", whitelight.value(),     {pneuX + 1*(pneuW+pneuSP), pneuY, pneuW, pneuH});
      drawPneuBox("INTK",      intakeCylander.value(), {pneuX + 2*(pneuW+pneuSP), pneuY, pneuW, pneuH});
      drawPneuBox("PUSH",      pushCylinder.value(),   {pneuX + 3*(pneuW+pneuSP), pneuY, pneuW, pneuH});
      drawPneuBox("SHOT",      shooter.value(),        {pneuX + 4*(pneuW+pneuSP), pneuY, pneuW, pneuH});
    }
    else {
      // ===== Motors 頁（放大字體 + 逐行清除避免陰影） =====
      if (!motorsInit) {
        // 清內容區一次、畫框與標題
        fillRect(content, vex::color(18,18,18));
        strokeRect({content.x, content.y, content.w, content.h - 2}, vex::color(90,90,90));
        Brain.Screen.setPenColor(vex::color(200,200,200));
        Brain.Screen.setFont(vex::fontType::mono20);    // 放大字體
        // 標題往下擺，避免貼到 tabs
        Brain.Screen.printAt(content.x, content.y + 4, "Motor position (deg)");
        motorsInit = true;
      }

      // 版面配置（兩欄）
      const int colW = (content.w - 16) / 2;
      const int c1x  = content.x + 8;
      const int c2x  = content.x + 8 + colW;

      // 行高比字高大些，整體下移
      const int rowH = 26;                 // 搭配 mono20
      int rowTop = content.y + 28;         // ★ 往下移，避免貼到標題與 tabs

      // 顏色
      vex::color bg = vex::color(18,18,18);
      vex::color fg = vex::white;

      // 逐行更新：先清該行，再用不透明文字印上，避免陰影
      for (int i = 0; i < kMotorCount; ++i) {
        int colX = (i % 2 == 0) ? c1x : c2x;
        int rowY = rowTop + (i / 2) * rowH;

        // 清這一行的小區塊（不影響其它區域）
        Rect rline = { colX, rowY - 18, colW - 8, rowH };
        fillRect(rline, bg);

        const char* name = (i < (int)(sizeof(kMotorNames)/sizeof(kMotorNames[0]))) ? kMotorNames[i] : "M?";
        double posDeg = 0.0;
        if (kMotors[i]) {
            posDeg = deg_to_inches(kMotors[i]->position(degrees));
        }
        Brain.Screen.setPenColor(fg);
        Brain.Screen.setFillColor(bg);
        Brain.Screen.printAt(colX, rowY, /*bOpaque=*/true, "%-6s %8.1f", name, posDeg);
      }
    }

    // ========== SPECIAL: If "Record" mode selected, show START button ==========
    if (selectedAuton == 6) {  // Case 6 = Record mode
      // Draw a big green START RECORDING button
      Rect startButton = { 140, 170, 200, 50 };
      fillRect(startButton, vex::color(0, 150, 0));
      strokeRect(startButton, vex::color(0, 255, 0));
      drawCenteredText(startButton, "START RECORDING", vex::white);
      
      // Check if START button pressed
      if (Brain.Screen.pressing()) {
        int tx = Brain.Screen.xPosition();
        int ty = Brain.Screen.yPosition();
        
        if (tx >= startButton.x && tx <= startButton.x + startButton.w &&
            ty >= startButton.y && ty <= startButton.y + startButton.h) {
          // Button pressed - start recording!
          while (Brain.Screen.pressing()) wait(10, msec);
          recordPath();  // Start recording immediately
          break;  // Exit dashboard after recording finishes
        }
      }
    }

    // 觸控返回（點 tabs 區域只切換，不返回）
    if (Brain.Screen.pressing()) {
      int ty = Brain.Screen.yPosition();
      if (!(ty >= 0 && ty <= TAB_H)) {
        while (Brain.Screen.pressing()) wait(10, msec);
        break;   // 回到 pre_auton 的選單頁
      }
    }

    wait(100, msec);
  }
}

// ======================== 第一頁（選擇auto） =========================
void pre_auton(void)
{
  
  vexcodeInit();
  default_constants();

/*// ===== SD 卡測試 =====
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
  Inertial.calibrate();

  while (Inertial.isCalibrating()) {
    whitelight = 0;
  }
  Controller1.Screen.print("ok!");
  redlight = 1;
  whitelight = 1;
  
  // CRITICAL: Reset motors RIGHT BEFORE dashboard
  // (Must be AFTER IMU calibration to avoid accumulation during calibration)
  L1.resetPosition(); L2.resetPosition(); L3.resetPosition();
  R1.resetPosition(); R2.resetPosition(); R3.resetPosition();
  wait(0.05, sec);  // Small delay for reset to take effect

  vex::color red   = vex::color::red;
  vex::color blue  = vex::color::blue;
  vex::color white = vex::color::white;

  int previous_selection = -1;

  const int cols      = 5;
  const int screen_w  = 480;
  const int screen_h  = 240;
  const int col_width = screen_w / cols;

  const int red_height  = screen_h / 2 + 6;
  const int blue_height = screen_h - red_height;

  const char* labels[10] = {
    "Right_43","Left_43","Right_7","Left_7","Solo",
    "Skills","Record","blank1","blank2","blank3"
  };

  Brain.Screen.setFont(vex::fontType::mono20);

  while (!auto_started)
  {
    if (current_auton_selection != previous_selection)
    {
      previous_selection = current_auton_selection;

      for (int i = 0; i < 10; i++)
      {
        int col = i % cols;
        bool top = (i < cols);

        int x = col * col_width;
        int y = top ? 0 : red_height;
        int h = top ? red_height : blue_height;

        vex::color fillColor = (current_auton_selection == i) ? white : (top ? red : blue);
        vex::color textColor = (current_auton_selection == i) ? red   : white;

        Brain.Screen.setFillColor(fillColor);
        Brain.Screen.drawRectangle(x, y, col_width, h);

        Rect r { x, y, col_width, h };
        drawCenteredText(r, labels[i], textColor);
        
      }
    }

    if (Brain.Screen.pressing())
    {
      int touchX = Brain.Screen.xPosition();
      int touchY = Brain.Screen.yPosition();

      int col = touchX / col_width;
      int row = (touchY < red_height) ? 0 : (touchY < red_height + blue_height ? 1 : -1);

      if (col >= 0 && col < cols && (row == 0 || row == 1)) {
        current_auton_selection = col + row * cols;
        

        // 進入第二頁（Dashboard）
        
        show_status_page(current_auton_selection);
      }

      wait(0.3, sec);
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
  else if (current_auton_selection >= 5 && current_auton_selection <= 9)
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
  case 8:
    blank2();
    break;
  case 9:
    blank3();
    break;
  }
  
}





int intakeControlTask()
{
  intake.setMaxTorque(100, percent);

  while (true)
  {
    // 依優先權：L1 > L2 > R1 > R2
    if (Controller1.ButtonL1.pressing())
    {
      // L1：只動 intakedown 反轉
      intake.spin(forward, 12, volt);
      intakedown.spin(forward, 12, volt);
    }
    /*
    else if (Controller1.ButtonL2.pressing())
    {
      // L2：只動 intakedown 正轉
      intake.spin(reverse, 12, volt);
      intakedown.spin(reverse, 9, volt);
    }
      */
    else if (Controller1.ButtonR1.pressing())
    {
      // 原本功能保留：R1
      intake.spin(reverse, 12, volt);
      intakedown.spin(reverse, 12, volt);
    }
    /*
    else if (Controller1.ButtonR2.pressing())
    {
      // 原本功能保留：R2
      intake.spin(reverse, 12, volt);
      intakedown.spin(reverse, 12, volt);  
    }
    */
    else
    {
      // 停止
      intake.stop(coast);
      intakedown.stop(coast);
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
  task notetask(autonoteTask, 0);
  task intake(intakeControlTask, 0);
  task opticalLight(opticalLightTask, 0);
  
  // Set up button callbacks
  Controller1.ButtonRight.pressed(shooterSwitch);
  Controller1.ButtonL2.pressed(wingSwitch);
  Controller1.ButtonB.pressed(pushSwitch);

  Controller1.ButtonY.pressed(shooterPushSwitch);
  Controller1.ButtonR2.pressed(cylinderSwitch);
  
  // Main driver control loop - minimal for maximum responsiveness
  while (1)
  {
    chassis.control_tank(100); // 底盤控制
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
