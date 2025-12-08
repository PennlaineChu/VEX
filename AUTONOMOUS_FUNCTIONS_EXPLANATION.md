# Autonomous Navigation Functions
## VEX Engineering Notebook Entry

---

## Problem Statement

Our robot needs precise autonomous navigation capabilities to move to specific coordinates on the field during autonomous periods. We require two main functions:

1. **`driveToXY()`** - Navigate to a specific (X, Y) coordinate on the field
2. **`driveToXAtHeading()`** - Navigate to a specific X coordinate while maintaining a desired heading angle

These functions must work seamlessly with our odometry system and coordinate system to ensure accurate positioning.

---

## Why Absolute XY Coordinates Are Superior to Relative Movements

### The Old Approach: Chained Relative Movements

**Previous Method**: Using many sequential `drive_distance()` calls

```cpp
// OLD APPROACH - Relative movements
chassis.drive_distance(20, 300, 6, 6);      // Move forward 20 inches
chassis.turn_to_angle(43);                   // Turn to 43°
chassis.drive_distance(7.5, 43, 5, 5);       // Move forward 7.5 inches
chassis.drive_distance(25.5, 43, 6, 6);       // Move forward 25.5 inches
chassis.drive_distance(-15, 43, 4, 4);       // Move backward 15 inches
chassis.turn_to_angle(110);                  // Turn to 110°
chassis.drive_distance(65, 110, 4, 4);       // Move forward 65 inches
// ... and so on
```

**Problems with this approach:**

1. **Cascading Errors**: If the first movement is off by 2 inches, ALL subsequent movements are affected
2. **No Absolute Reference**: You don't know where the robot actually is - only where it *should* be relative to the last command
3. **Hard to Debug**: If something goes wrong at step 5, you can't easily test just that step
4. **Difficult to Modify**: Changing an early movement requires recalculating all later movements
5. **No Recovery**: If the robot drifts, there's no way to "reset" to a known position

### The New Approach: Absolute XY Coordinates

**Current Method**: Using `driveToXY()` with absolute field coordinates

```cpp
// NEW APPROACH - Absolute positions
set_robot_pose(86, 116.3, 0.0);              // Set starting position
driveToXY(46.5, 92.5, 8.0, 6.0);             // Go to goal at (46.5, 92.5)
driveToXAtHeading(25, 225, 8.0, 6.0);        // Go to loader at X=25, heading 225°
driveToXY(105.5, 105.5, 8.0, 6.0);           // Go to long goal at (105.5, 105.5)
```

**Advantages of this approach:**

1. **Independent Movements**: Each movement is independent - if one is wrong, others aren't affected
2. **Absolute Reference**: You always know exactly where the robot is on the field
3. **Easy Debugging**: Can test individual movements in isolation
4. **Easy Modification**: Change one movement without affecting others
5. **Recovery Capability**: Can always "reset" to a known position using `set_robot_pose()`
6. **Flexible Mixing**: Can combine absolute movements with relative movements when needed

### Real-World Example: Modifying Autonomous Routine

**Scenario**: You need to adjust how the robot approaches the first goal.

#### Old Approach (Relative):
```cpp
// Original code
chassis.drive_distance(20, 300, 6, 6);      // Step 1: Move 20 inches
chassis.turn_to_angle(43);                   // Step 2: Turn
chassis.drive_distance(7.5, 43, 5, 5);       // Step 3: Move 7.5 inches
chassis.drive_distance(25.5, 43, 6, 6);     // Step 4: Move 25.5 inches
chassis.drive_distance(-15, 43, 4, 4);       // Step 5: Back up 15 inches
// ... 10 more steps ...
```

**Problem**: If you change Step 1 from 20 to 18 inches, Steps 2-15 are now all starting from the wrong position! You'd need to recalculate everything.

#### New Approach (Absolute):
```cpp
// Original code
set_robot_pose(86, 116.3, 0.0);
driveToXY(46.5, 92.5, 8.0, 6.0);             // Go to goal
driveToXAtHeading(25, 225, 8.0, 6.0);       // Go to loader
driveToXY(105.5, 105.5, 8.0, 6.0);           // Go to long goal
// ... more absolute positions ...
```

**Solution**: Want to approach the goal from a different angle? Just change the target coordinates:
```cpp
driveToXY(48.0, 90.0, 8.0, 6.0);             // Changed approach angle
// All other movements remain unchanged! ✓
```

### Combining Absolute and Relative Movements

The beauty of absolute coordinates is that you can **still use relative movements** when they make sense:

```cpp
// Absolute: Go to precise goal location
driveToXY(46.5, 92.5, 8.0, 6.0);

// Relative: Fine-tune approach (small adjustment)
chassis.drive_distance(2.5, 45, 6, 6);       // Move forward a bit more

// Absolute: Go to next precise location
driveToXY(25, 80, 8.0, 6.0);
```

**Why this works**: The relative movement is small and doesn't accumulate error. The next absolute movement "resets" any small drift.

### Error Accumulation Comparison

#### Relative Movement Chain:
```
Step 1: Target 20", Actual 19.5" → Error: -0.5"
Step 2: Target 7.5", Actual 7.3" → Error: -0.2" (Total: -0.7")
Step 3: Target 25.5", Actual 25.2" → Error: -0.3" (Total: -1.0")
Step 4: Target -15", Actual -14.8" → Error: +0.2" (Total: -0.8")
...
After 10 steps: Total error could be ±3-5 inches!
```

#### Absolute Movement Chain:
```
Step 1: Target (46.5, 92.5), Actual (46.3, 92.7) → Error: ±0.2"
Step 2: Target (25, 80), Actual (25.1, 79.9) → Error: ±0.1" (INDEPENDENT!)
Step 3: Target (105.5, 105.5), Actual (105.4, 105.6) → Error: ±0.1" (INDEPENDENT!)
...
After 10 steps: Each movement has ±0.2" error, but they don't accumulate!
```

### Practical Benefits in Autonomous Development

1. **Iterative Testing**: Test each waypoint independently
   ```cpp
   // Test just the goal approach
   set_robot_pose(86, 116.3, 0.0);
   driveToXY(46.5, 92.5, 8.0, 6.0);
   // Stop here to test - no need to run entire routine
   ```

2. **Easy Waypoint Adjustment**: Change one coordinate without affecting others
   ```cpp
   // Original
   driveToXY(46.5, 92.5, 8.0, 6.0);
   
   // Adjusted for better angle
   driveToXY(48.0, 90.0, 8.0, 6.0);
   // Everything else stays the same!
   ```

3. **Modular Autonomous Routines**: Build reusable waypoint sequences
   ```cpp
   void approachGoal() {
       driveToXY(46.5, 92.5, 8.0, 6.0);
   }
   
   void goToLoader() {
       driveToXAtHeading(25, 225, 8.0, 6.0);
   }
   
   void scoreAtLongGoal() {
       driveToXY(105.5, 105.5, 8.0, 6.0);
   }
   ```

4. **Recovery from Errors**: Reset to known position if something goes wrong
   ```cpp
   if (error_detected) {
       set_robot_pose(86, 116.3, 0.0);  // Reset to start
       driveToXY(46.5, 92.5, 8.0, 6.0); // Try again
   }
   ```

---

## Coordinate System Design

### Field Coordinate System
- **Origin (0, 0)**: Top-left corner of the field
- **Bottom-right corner**: (140.41, 140.41) inches
- **Heading Convention**:
  - **0°** = Up/Forward (negative Y direction)
  - **90°** = Right (positive X direction)
  - **180°** = Down (positive Y direction)
  - **270°** = Left (negative X direction)

### Visual Representation
```
(0, 0) ──────────────────────────────→ +X (140.41, 0)
  │                                          │
  │                                          │
  │               FIELD                      │
  │                                          │
  │                                          │
  ↓                                          ↓
+Y (0, 140.41) ──────────────────────── (140.41, 140.41)
```

### Heading Visualization
```
        0° (Up/Forward)
          ↑
          │
270° ←────┼────→ 90° (Right)
(Left)    │
          ↓
        180° (Down)
```

---

## Function 1: `driveToXY()`

### Purpose
Moves the robot from its current position to a target (X, Y) coordinate on the field.

### Design Approach
We use a **three-step process** to ensure accuracy:

1. **Calculate target heading** - Determine the angle needed to face the target
2. **Turn to target heading** - Rotate the robot to face the correct direction
3. **Drive straight** - Move forward while maintaining the heading

This approach prevents drift and ensures the robot arrives at the target with the correct orientation.

### Mathematical Foundation

#### Step 1: Calculate Target Heading
```
dx = targetX - robot_pose.x
dy = targetY - robot_pose.y
targetHeading = atan2(dx, -dy) × (180° / π)
```

**Why `-dy`?** In our coordinate system:
- 0° points in the **-Y direction** (up/forward)
- `atan2(dx, -dy)` correctly calculates the angle where:
  - When `dx = 0` and `dy < 0` (moving up): `atan2(0, -(-1)) = atan2(0, 1) = 0°` ✓
  - When `dx > 0` and `dy = 0` (moving right): `atan2(1, -0) = atan2(1, 0) = 90°` ✓

#### Step 2: Normalize Heading
The heading is normalized to the range [0°, 360°):
```cpp
while(targetHeading < 0.0) targetHeading += 360.0;
while(targetHeading >= 360.0) targetHeading -= 360.0;
```

#### Step 3: Calculate Distance
```
distance = √(dx² + dy²)
```

### Code Implementation

```cpp
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
```

### Parameters
- **`targetX`**: Target X coordinate (inches)
- **`targetY`**: Target Y coordinate (inches)
- **`maxV`**: Maximum drive voltage (volts)
- **`turnMaxV`**: Maximum turn correction voltage (volts)

### Why This Design Works

1. **Separation of Concerns**: By turning first, then driving straight, we eliminate the need for complex arc-based navigation that can cause drift.

2. **Heading Maintenance**: The `chassis.drive_distance()` function uses PID control to maintain the target heading while driving, ensuring the robot stays on course.

3. **Accuracy**: The three-step process ensures the robot arrives at the exact target coordinate with minimal error.

---

## Function 2: `driveToXAtHeading()`

### Purpose
Moves the robot to a specific X coordinate while maintaining a desired heading angle. This is useful when the robot needs to approach a goal or obstacle at a specific angle.

### Design Approach
This function calculates the Y coordinate that results in the desired heading when moving to the target X, then uses `driveToXY()` to navigate there.

### Mathematical Foundation

#### Deriving the Target Y Coordinate

Given:
- Current position: `(robot_pose.x, robot_pose.y)`
- Target X: `targetX`
- Desired heading: `targetHeading`

We know from our coordinate system:
```
heading = atan2(dx, -dy) × (180° / π)
```

Rearranging:
```
tan(heading × π / 180) = dx / (-dy)
-dy = dx / tan(heading × π / 180)
dy = -dx / tan(heading × π / 180)
```

Therefore:
```
targetY = robot_pose.y + dy
targetY = robot_pose.y - dx / tan(heading × π / 180)
```

#### Special Case: Vertical Headings
When the heading is approximately 90° or 270° (moving purely in Y direction), `tan()` approaches infinity. In this case, we cannot reach a specific X while maintaining the heading, so we handle it as a special case.

### Code Implementation

```cpp
void driveToXAtHeading(double targetX, double targetHeading, double maxV, double turnMaxV) {
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
```

### Parameters
- **`targetX`**: Target X coordinate (inches)
- **`targetHeading`**: Desired heading angle (degrees, 0-360)
- **`maxV`**: Maximum drive voltage (volts)
- **`turnMaxV`**: Maximum turn correction voltage (volts)

### Example Use Case

**Scenario**: Robot needs to approach a goal at X = 25 inches with a heading of 225° (diagonal approach from top-right).

```
Current position: (30, 100)
Target X: 25
Target heading: 225°

dx = 25 - 30 = -5 inches
heading_rad = 225° × π / 180 = 3.927 radians
dy = -(-5) / tan(3.927) = 5 / tan(225°) = 5 / 1 = 5 inches
targetY = 100 + 5 = 105 inches

Result: Robot drives to (25, 105) at 225° heading
```

---

## Integration with Odometry

Both functions rely on the global `robot_pose` structure, which is continuously updated by the odometry task:

```cpp
struct RobotPose {
    double x;      // X position (inches)
    double y;      // Y position (inches)
    double heading; // Heading angle (degrees, 0-360)
};
```

The odometry system updates the robot's position using:
- **IMU (Inertial Sensor)**: Provides heading information
- **Encoders**: Track wheel rotations for distance traveled
- **Sensor Fusion**: Combines both sources for accurate pose estimation

### Position Update Formula
```cpp
robot_pose.x += delta_center * sin(heading_rad);
robot_pose.y -= delta_center * cos(heading_rad);
```

This formula correctly handles our coordinate system where 0° points in the -Y direction.

---

## Testing and Validation

### Test Procedure

1. **Initialization Test**
   - Set robot pose: `set_robot_pose(0, 0, 0)`
   - Verify odometry reads (0, 0, 0°)

2. **`driveToXY()` Test**
   - Test Case 1: Drive to (50, 50) from (0, 0)
     - Expected: Robot turns to 45°, then drives straight
   - Test Case 2: Drive to (100, 0) from (50, 50)
     - Expected: Robot turns to 90°, then drives right
   - Test Case 3: Drive to (0, 100) from (50, 50)
     - Expected: Robot turns to 0°, then drives up

3. **`driveToXAtHeading()` Test**
   - Test Case 1: Drive to X=25 at 225° heading
     - Expected: Robot calculates correct Y, approaches at 225°
   - Test Case 2: Drive to X=100 at 90° heading
     - Expected: Robot handles vertical heading case correctly

### Success Criteria
- Robot arrives within ±1 inch of target coordinates
- Final heading matches target heading within ±2°
- No excessive overshoot or oscillation
- Smooth, controlled motion without jerky movements

---

## Advantages of This Design

1. **Modularity**: `driveToXAtHeading()` reuses `driveToXY()`, reducing code duplication
2. **Accuracy**: Three-step process ensures precise navigation
3. **Flexibility**: Functions work with any coordinate on the field
4. **Maintainability**: Clear separation of concerns makes debugging easier
5. **Robustness**: Special case handling for edge conditions (vertical headings)

---

## Future Improvements

1. **Path Planning**: Add obstacle avoidance by calculating intermediate waypoints
2. **Curved Paths**: Implement arc-based navigation for smoother motion
3. **Dynamic Speed Control**: Adjust speed based on distance to target
4. **Error Recovery**: Add retry logic if robot doesn't reach target

---

## Conclusion

These autonomous navigation functions provide reliable, accurate positioning for our robot during autonomous periods. The mathematical foundation ensures correct behavior across all quadrants of the field, and the modular design allows for easy extension and maintenance.

**Key Takeaways:**

### Technical Implementation
- Coordinate system design is critical for correct navigation
- Separating turning and driving improves accuracy
- Mathematical derivation ensures correct target calculation
- Integration with odometry provides real-time position feedback

### Strategic Advantages
- **Absolute vs Relative**: Absolute XY coordinates eliminate error accumulation
- **Modularity**: Each movement is independent and can be tested/modified separately
- **Flexibility**: Can mix absolute and relative movements as needed
- **Maintainability**: Easy to adjust waypoints without affecting other movements
- **Debugging**: Can test individual waypoints in isolation
- **Recovery**: Can reset to known positions if errors occur

### Real-World Impact

**Before (Relative Movements):**
- Changing one movement affects all subsequent movements
- Errors accumulate throughout the routine
- Difficult to debug and modify
- Must recalculate entire sequence when adjusting

**After (Absolute XY Coordinates):**
- Each movement is independent
- Errors don't accumulate
- Easy to test, debug, and modify individual waypoints
- Can adjust one movement without affecting others
- Can combine with relative movements for fine-tuning

This design philosophy transforms autonomous development from a fragile chain of dependent movements into a robust, modular system of independent waypoints.

