# Autonomous Programming Guide

## Table of Contents
1. [Basic Structure](#basic-structure)
2. [Setting Starting Position](#setting-starting-position)
3. [Movement Functions](#movement-functions)
4. [Motor Control](#motor-control)
5. [Color Sorting](#color-sorting)
6. [Complete Examples](#complete-examples)

---

## Basic Structure

All autonomous functions are defined in `src/autons.cpp`. The main autonomous function calls them based on selection:

```cpp
void Right_43() {
  // Your autonomous code here
}
```

**Important:** The odometry task is automatically started in `autonomous()`, so you don't need to start it manually.

---

## Setting Starting Position

**CRITICAL:** Always set your starting position at the beginning of your autonomous routine!

```cpp
void Right_43() {
  // Set starting position (X, Y, heading)
  // X: 0-140.41 inches (left to right)
  // Y: 0-140.41 inches (top to bottom)
  // Heading: 0-360 degrees (0° = up/forward, 90° = right, 180° = down, 270° = left)
  set_robot_pose(86.0, 116.3, 0.0);  // Example: Right side starting position
  
  // Optional: Set chassis constants for this routine
  default_constants();  // Uses default PID values
  // OR customize:
  chassis.set_drive_constants(12, 0.7, 0.004, 10, 20);
  chassis.set_heading_constants(12, 0.3, 0.007, 5, 40);
  chassis.set_turn_constants(12, 0.35, 0.001, 3, 90);
}
```

---

## Movement Functions

## Built-in Chassis Path Functions

These are the **built-in functions** from the `chassis` object (JAR Template). They use the chassis's internal odometry and PID control.

### 1. Drive Distance (Relative Movement)

**Basic usage:**
```cpp
chassis.drive_distance(24.0);  // Drive forward 24 inches
chassis.drive_distance(-24.0);  // Drive backward 24 inches
```

**With heading correction:**
```cpp
chassis.drive_distance(24.0, 90.0);  // Drive 24 inches while maintaining 90° heading
```

**With custom voltages:**
```cpp
chassis.drive_distance(24.0, 90.0, 6.0, 4.0);
// Parameters: distance, heading, drive_max_voltage, heading_max_voltage
```

**Full control:**
```cpp
chassis.drive_distance(24.0, 90.0, 6.0, 4.0, 0.5, 0, 2000);
// Parameters: distance, heading, drive_voltage, heading_voltage, 
//            settle_error, settle_time, timeout
```

### 2. Turn to Angle

**Basic usage:**
```cpp
chassis.turn_to_angle(90.0);  // Turn to 90 degrees
chassis.turn_to_angle(180.0); // Turn to 180 degrees
```

**With custom voltage:**
```cpp
chassis.turn_to_angle(90.0, 6.0);  // Turn to 90° with max 6V
```

**Full control:**
```cpp
chassis.turn_to_angle(90.0, 6.0, 2.0, 0, 3000);
// Parameters: angle, max_voltage, settle_error, settle_time, timeout
```

### 3. Swing Turns (Pivot on One Side)

**Left swing (pivot on right side):**
```cpp
chassis.left_swing_to_angle(90.0);  // Pivot left to 90°
```

**Right swing (pivot on left side):**
```cpp
chassis.right_swing_to_angle(90.0);  // Pivot right to 90°
```

**Full control:**
```cpp
chassis.left_swing_to_angle(90.0, 6.0, 2.0, 0, 3000, 0.5, 0.001, 2, 15);
// Parameters: angle, max_voltage, settle_error, settle_time, timeout,
//            kp, ki, kd, starti
```

### 4. Drive to Point (Absolute XY)

**Basic usage:**
```cpp
chassis.drive_to_point(100.0, 50.0);  // Drive to field coordinate (100, 50)
```

**With custom voltages:**
```cpp
chassis.drive_to_point(100.0, 50.0, 6.0, 4.0);
// Parameters: X, Y, drive_max_voltage, heading_max_voltage
```

**Full control:**
```cpp
chassis.drive_to_point(100.0, 50.0, 6.0, 4.0, 0.5, 0, 2000);
// Parameters: X, Y, drive_voltage, heading_voltage, 
//            settle_error, settle_time, timeout
```

### 5. Turn to Point

**Basic usage:**
```cpp
chassis.turn_to_point(100.0, 50.0);  // Turn to face point (100, 50)
```

**With extra angle offset:**
```cpp
chassis.turn_to_point(100.0, 50.0, 10.0);  // Face point + 10° offset
```

**Full control:**
```cpp
chassis.turn_to_point(100.0, 50.0, 0.0, 6.0, 2.0, 0, 3000);
// Parameters: X, Y, extra_angle, max_voltage, settle_error, settle_time, timeout
```

### 6. Holonomic Drive to Point (Best for Holonomic Drives)

**Basic usage:**
```cpp
chassis.holonomic_drive_to_point(100.0, 50.0);  // Drive to (100, 50)
```

**With final heading:**
```cpp
chassis.holonomic_drive_to_point(100.0, 50.0, 90.0);  // Drive to (100, 50) facing 90°
```

**With custom voltages:**
```cpp
chassis.holonomic_drive_to_point(100.0, 50.0, 90.0, 6.0, 4.0);
// Parameters: X, Y, final_angle, drive_max_voltage, heading_max_voltage
```

**Full control:**
```cpp
chassis.holonomic_drive_to_point(100.0, 50.0, 90.0, 6.0, 4.0, 0.5, 0, 2000);
// Parameters: X, Y, final_angle, drive_voltage, heading_voltage,
//            settle_error, settle_time, timeout
```

### Setting PID Constants

Before using movement functions, you can customize PID values:

```cpp
// Set drive PID constants
chassis.set_drive_constants(12, 0.7, 0.004, 10, 20);
// Parameters: max_voltage, kp, ki, kd, starti

// Set turn PID constants
chassis.set_turn_constants(12, 0.35, 0.001, 3, 90);

// Set heading PID constants (for maintaining heading while driving)
chassis.set_heading_constants(12, 0.3, 0.007, 5, 40);

// Set swing PID constants
chassis.set_swing_constants(12, 0.5, 0.001, 2, 15);

// Set exit conditions (when to stop)
chassis.set_drive_exit_conditions(0.5, 0, 2000);  // settle_error, settle_time, timeout
chassis.set_turn_exit_conditions(4, 0, 3000);
chassis.set_swing_exit_conditions(10, 0, 3000);
```

### Setting Starting Position (for Built-in Functions)

**IMPORTANT:** Always set starting position before using built-in functions:

```cpp
void Right_43() {
  // Set starting position for chassis odometry
  chassis.set_coordinates(86.0, 116.3, 0.0);
  // Parameters: X, Y, heading
  
  // Now you can use built-in functions
  chassis.drive_to_point(100.0, 105.0);
  chassis.turn_to_angle(90.0);
}
```

---

## Custom Path Functions (Alternative)

### 1. Drive to XY Coordinate (Recommended)

**Best for:** Moving to specific field positions

```cpp
driveToXY(targetX, targetY, maxV, turnMaxV);
```

- `targetX`: Target X coordinate (0-140.41 inches)
- `targetY`: Target Y coordinate (0-140.41 inches)
- `maxV`: Maximum drive voltage (0-12 volts)
- `turnMaxV`: Maximum turn voltage (0-12 volts)

**Example:**
```cpp
driveToXY(100.0, 105.0, 6.0, 4.0);  // Drive to position (100, 105)
```

### 2. Drive to X at Specific Heading

**Best for:** Moving to a specific X coordinate while maintaining heading

```cpp
driveToXAtHeading(targetX, targetHeading, maxV, turnMaxV);
```

**Example:**
```cpp
driveToXAtHeading(115.0, 135.0, 6.0, 4.0);  // Drive to X=115 while facing 135°
```

### 3. Drive Backward to XY

**Best for:** Backing up to a position

```cpp
driveToXYBackward(targetX, targetY, maxV, turnMaxV);
```

### 4. Turn to Face XY Coordinate

**Best for:** Turning to face a specific point before grabbing objects

```cpp
turnToXY(targetX, targetY, turnMaxV);
```

**Example:**
```cpp
turnToXY(50.0, 50.0, 4.0);  // Turn to face point (50, 50)
```

### 5. Chassis Movement Functions (Lower Level)

**Turn to angle:**
```cpp
chassis.turn_to_angle(90.0);  // Turn to 90 degrees
```

**Drive distance:**
```cpp
chassis.drive_distance(24.0);  // Drive forward 24 inches
chassis.drive_distance(24.0, 90.0);  // Drive 24 inches while maintaining 90° heading
chassis.drive_distance(24.0, 90.0, 6.0, 4.0);  // With max voltages
```

**Holonomic drive to point:**
```cpp
chassis.holonomic_drive_to_point(100.0, 50.0, 90.0);  // X, Y, final heading
```

---

## Motor Control

### Intake Motors

```cpp
// Start intake (forward = intake, reverse = outtake)
intake1.spin(forward, 12, volt);
intake2.spin(forward, 12, volt);

// Stop intake
intake1.stop(coast);  // or brake
intake2.stop(coast);

// Wait while intake runs
wait(2, sec);

// Stop intake
intake1.stop();
intake2.stop();
```

### Shooter Motors

```cpp
// Start shooter
shooterUpper.spin(forward, 12, volt);
shooterLower.spin(reverse, 12, volt);  // Usually reverse for scoring

// Wait for shooter to spin up
wait(1, sec);

// Stop shooter
shooterUpper.stop();
shooterLower.stop();
```

### Pneumatics (Digital Outputs)

```cpp
// Activate intake cylinder
intakeCylinder = true;   // Open
intakeCylinder = false;  // Close

// Activate block cylinder (for color sorting)
blockCylinder = true;   // Open
blockCylinder = false;  // Close

// Activate tracking wheel
trackingWheel = true;   // Deploy
trackingWheel = false;  // Retract
```

---

## Color Sorting

### Setup

Start the color sorting task at the beginning of your autonomous:

```cpp
void Right_43() {
  set_robot_pose(86.0, 116.3, 0.0);
  
  // Start color sorting task (runs in background)
  task colorSort(ColorSortTask);
  
  // Enable color sorting with target color and mode
  // Modes: 1 = lid control, 2 = shooter control, 3 = intake control
  enableColorSort(vex::color::red, 2);  // Red team, shooter mode
  
  // Your autonomous code...
}
```

### Modes

**Mode 1 - Lid Control:**
- Opens/closes `blockCylinder` based on color
- Use when you have a lid mechanism

**Mode 2 - Shooter Control:**
- Spins `shooterLower` forward if wrong color detected
- Closes `blockCylinder` to prevent wrong color from falling out
- Use when scoring with shooter

**Mode 3 - Intake Control:**
- Stops `intake1` if wrong color detected
- Use when intaking and need to stop on wrong color

### Disable Color Sorting

```cpp
disableColorSort();  // Turn off color sorting
```

---

## Complete Examples

### Example 1: Using Built-in Chassis Functions

```cpp
void Right_43() {
  // Set starting position for chassis odometry
  chassis.set_coordinates(86.0, 116.3, 0.0);
  
  // Set PID constants (optional, uses defaults if not set)
  default_constants();
  
  // Drive to a point using built-in function
  chassis.drive_to_point(100.0, 105.0);
  
  // Turn to a specific angle
  chassis.turn_to_angle(90.0);
  
  // Drive forward a specific distance
  chassis.drive_distance(24.0, 90.0);  // 24 inches while maintaining 90° heading
  
  // Turn to face another point
  chassis.turn_to_point(50.0, 50.0);
  
  // Drive to that point
  chassis.drive_to_point(50.0, 50.0);
}
```

### Example 2: Holonomic Drive with Built-in Functions

```cpp
void Left_43() {
  // Set starting position
  chassis.set_coordinates(54.4, 116.3, 0.0);
  
  // Use holonomic drive to point (best for holonomic drives)
  chassis.holonomic_drive_to_point(36.4, 104.0, 0.0);
  
  // Drive to another point with specific final heading
  chassis.holonomic_drive_to_point(47.0, 92.5, 90.0);  // End facing 90°
}
```

### Example 3: Simple Right Side Autonomous

```cpp
void Right_43() {
  // Set starting position
  set_robot_pose(86.0, 116.3, 0.0);
  
  // Start color sorting (optional)
  task colorSort(ColorSortTask);
  enableColorSort(vex::color::red, 2);  // Red team, shooter mode
  
  // Drive to intake position
  driveToXY(100.0, 105.0, 6.0, 4.0);
  
  // Start intake
  intake1.spin(forward, 12, volt);
  intake2.spin(forward, 12, volt);
  
  // Drive forward to pick up object
  chassis.drive_distance(10.0, 0.0, 4.0, 3.0);
  wait(1, sec);
  
  // Stop intake
  intake1.stop();
  intake2.stop();
  
  // Drive to scoring position
  driveToXY(93.5, 95.0, 6.0, 4.0);
  
  // Start shooter
  shooterUpper.spin(forward, 12, volt);
  shooterLower.spin(reverse, 12, volt);
  wait(1, sec);  // Wait for shooter to spin up
  
  // Score (color sorting will handle wrong colors automatically)
  wait(2, sec);
  
  // Stop shooter
  shooterUpper.stop();
  shooterLower.stop();
  
  // Disable color sorting
  disableColorSort();
}
```

### Example 2: Using Holonomic Drive

```cpp
void Left_43() {
  set_robot_pose(54.4, 116.3, 0.0);
  
  // Use holonomic drive for precise positioning
  chassis.holonomic_drive_to_point(36.4, 104.0, 0.0);
  
  // Intake
  intake1.spin(forward, 12, volt);
  wait(1, sec);
  intake1.stop();
  
  // Drive to goal
  chassis.holonomic_drive_to_point(47.0, 92.5, 0.0);
  
  // Score
  shooterUpper.spin(forward, 12, volt);
  wait(2, sec);
  shooterUpper.stop();
}
```

### Example 3: Complex Path with Multiple Waypoints

```cpp
void Skills() {
  set_robot_pose(54.0, 117.5, 270.0);
  
  // Start color sorting
  task colorSort(ColorSortTask);
  enableColorSort(vex::color::blue, 2);  // Blue team
  
  // Path 1: Drive to loader
  driveToXY(24.0, 116.5, 6.0, 4.0);
  
  // Path 2: Cross field
  driveToXY(8.1, 90.0, 6.0, 4.0);
  
  // Path 3: Continue to goal
  driveToXY(10.0, 37.0, 6.0, 4.0);
  
  // Path 4: Approach long goal
  driveToXY(18.4, 24.9, 6.0, 4.0);
  
  // Start shooter before scoring
  shooterUpper.spin(forward, 12, volt);
  shooterLower.spin(reverse, 12, volt);
  wait(1, sec);
  
  // Score (color sorting active)
  wait(3, sec);
  
  // Continue path...
  driveToXY(116.8, 25.0, 6.0, 4.0);
  
  // More autonomous code...
  
  // Cleanup
  shooterUpper.stop();
  shooterLower.stop();
  disableColorSort();
}
```

---

## Tips and Best Practices

1. **Always set starting position first** - This is critical for accurate navigation
2. **Use `driveToXY()` for most movements** - It's more accurate than chaining `drive_distance()` calls
3. **Wait after motor commands** - Give motors time to execute before next command
4. **Test incrementally** - Test each movement separately before combining
5. **Use appropriate speeds** - Lower speeds (4-6V) for precision, higher (8-12V) for speed
6. **Coordinate system:**
   - Origin (0,0) is top-left corner
   - X increases right (0 to 140.41)
   - Y increases down (0 to 140.41)
   - Heading: 0° = up, 90° = right, 180° = down, 270° = left

---

## Common Field Coordinates

Based on VEX Over Under field (140.41" x 140.41"):

- **Right side start:** (86.0, 116.3, 0.0)
- **Left side start:** (54.4, 116.3, 0.0)
- **Center:** (70.2, 70.2, 0.0)
- **Right goal area:** ~(93.5, 95.0)
- **Left goal area:** ~(47.0, 92.5)

Adjust these based on your robot's actual starting position and field measurements.

---

## Debugging

### Check Current Position

```cpp
RobotPose pose = get_robot_pose();
Brain.Screen.printAt(10, 20, "X: %.1f Y: %.1f H: %.1f", 
                     pose.x, pose.y, pose.heading);
```

### Print to Controller Screen

```cpp
Controller1.Screen.clearScreen();
Controller1.Screen.setCursor(1, 1);
Controller1.Screen.print("Driving to goal...");
```

---

## Need Help?

- Check `src/main.cpp` for function implementations
- See `src/autons.cpp` for existing autonomous examples
- Review `README.md` for general robot information
- Test movements one at a time before combining

