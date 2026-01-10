# Team 23083Z Robotics Project

## Introduction

Welcome to **Team 23083Z's Robotics Project** repository! This repository contains the source code for our VEX Robotics competition robot, featuring advanced odometry, XY navigation, and autonomous path recording capabilities.

Our team is dedicated to developing **efficient**, **innovative**, and **competitive solutions** for both autonomous and driver-controlled operations. We integrate **advanced control systems**, **sensor fusion**, **odometry tracking**, and **feedback loops** to ensure smooth and effective functionality in competitive environments.

## Key Features

### 🎯 Advanced Navigation
- **XY Coordinate Navigation**: Drive to absolute X, Y coordinates using `driveToXY()`
- **Turn to Point**: Turn to face specific coordinates using `turnToXY()`
- **Odometry Tracking**: Real-time position tracking using encoders + IMU sensor fusion
- **Path Recording**: Record manual driving paths and replay them autonomously

### 🤖 Autonomous Capabilities
- **Multiple Autonomous Routines**: Pre-programmed strategies for different match scenarios
- **Sensor Integration**: Optical sensors, vision sensors, and IMU for navigation
- **Precise Movement**: Cosine velocity profiles for smooth acceleration/deceleration

### 🎮 Driver Control
- **Tank Drive**: Standard tank drive control with joystick inputs
- **Intake System**: Dual-motor intake with independent control
- **Pneumatic Control**: Multiple pneumatic cylinders for game piece manipulation
- **Real-time Feedback**: Position and sensor data displayed on controller screen

## Project Structure

```
VEX-1/
├── src/                    # Source code files
│   ├── main.cpp           # Main robot code (odometry, navigation, driver control)
│   ├── autons.cpp         # Autonomous routines
│   ├── robot-config.cpp   # Hardware configuration
│   ├── note.cpp           # Note/autonomous note task
│   └── JAR-Template/      # JAR Template library files
├── include/                # Header files
│   ├── vex.h              # Main header with navigation functions
│   ├── robot-config.h     # Hardware declarations
│   ├── autons.h           # Autonomous routine declarations
│   └── note.h             # Note task declaration
├── PATH_RECORDING_GUIDE.md # Guide for recording paths
└── PID_TUNING_GUIDE.md     # Guide for tuning PID controllers
```

## Hardware Configuration

### Motors

| Subsystem | Type | Name | Port | Notes |
|-----------|------|------|------|-------|
| Chassis | 11W motor | L1, L2, L3 | 18, 17, 19 | Left drive motors |
| Chassis | 11W motor | R1, R2, R3 | 7, 9, 8 | Right drive motors |
| Intake | 5.5W motor | intake | 6 | Main intake motor |
| Intake | 5.5W motor | intakedown | 5 | Intake down motor |

### Sensors

| Subsystem | Port | Notes |
|-----------|------|-------|
| Inertial | 20 | IMU for heading and orientation |
| Optical | 10 | Ring detection |
| Optical_go | 16 | Mobile goal detection |
| Vision | 13, 15 | Vision sensors for object detection |

### Digital Outputs (Pneumatics/Lights)

| Device | Port | Notes |
|--------|------|-------|
| pushCylinder | B | Push mechanism |
| intakeCylander | C | Intake cylinder |
| redlight | H | Red LED indicator |
| whitelight | G | White LED indicator |
| shooter | A | Shooter mechanism |

## Controller Mappings

### Drive Control
- **Left Joystick**: Controls left side motors (L1, L2, L3)
- **Right Joystick**: Controls right side motors (R1, R2, R3)

### Intake Control
- **R1**: Intake forward (both intake and intakedown)
- **R2**: Intake reverse (both intake and intakedown)
- **Right Button**: Intakedown forward only
- **Down Button**: Intakedown reverse only

### Other Controls
- **Y Button**: Toggle shooter
- **B Button**: Toggle push cylinder
- **L1 Button**: Toggle shooter + push cylinder together
- **L2 Button**: Toggle intake cylinder

## Key Functions

### Navigation Functions

#### `driveToXY(targetX, targetY, maxV, turnMaxV)`
Drive to an absolute X, Y coordinate on the field.
- Uses odometry for position tracking
- Automatically corrects heading while driving
- Smooth velocity profiles for acceleration/deceleration
- Coordinate system: 0° = +Y (forward), 90° = +X (right)

#### `turnToXY(targetX, targetY, turnMaxV)`
Turn to face a specific X, Y coordinate.
- Calculates target heading using `atan2`
- PID control for precise turning
- Settle detection to prevent overshoot

#### `set_robot_pose(x, y, heading)`
Initialize the robot's starting position and heading.
- Resets odometry to specified values
- Calibrates IMU offset for accurate heading

#### `get_robot_pose()`
Get the current robot position and heading.
- Returns `RobotPose` structure with X, Y, and heading

### Path Recording

#### `recordPath()`
Record a manual driving path for autonomous replay.
- Drive manually using controller
- Press **Button A** to save waypoints
- Press **Button B** to finish recording
- Path is saved to SD card and printed to console

See [PATH_RECORDING_GUIDE.md](PATH_RECORDING_GUIDE.md) for detailed instructions.

### Odometry

The robot uses **sensor fusion** combining:
- **Motor Encoders**: Track distance traveled by each side
- **IMU (Inertial Sensor)**: Provides absolute heading
- **Fusion Algorithm**: Combines both for accurate position tracking

The odometry task runs continuously in the background, updating robot position every 10ms.

## Autonomous Routines

The robot includes multiple autonomous routines:

- **Right_43()**: Right side autonomous, 4-5 rings
- **Left_43()**: Left side autonomous, 4-5 rings
- **Right_7()**: Right side autonomous, 7 rings
- **Left_7()**: Left side autonomous, 7 rings
- **Solo()**: Solo autonomous routine
- **Skills()**: Skills competition routine
- **blank1()**, **blank2()**, **blank3()**, **blank4()**: Test/blank routines

## Setup Instructions

### Prerequisites
- VEXcode V5 or compatible C++ IDE
- VEX V5 Brain and Controller
- USB cable for programming

### Building and Downloading

1. **Clone the repository** to your local machine
2. **Open the project** in VEXcode V5
3. **Connect the VEX V5 Brain** to your computer via USB
4. **Build the project** (should compile without errors)
5. **Download to Brain** and test

### First Time Setup

1. **Calibrate IMU**: The IMU will auto-calibrate on startup
2. **Set Starting Position**: Use `set_robot_pose(x, y, heading)` in your autonomous
3. **Test Odometry**: Run `odom_test()` to verify position tracking
4. **Tune PID**: See [PID_TUNING_GUIDE.md](PID_TUNING_GUIDE.md) for tuning instructions

## Coordinate System

The robot uses a standard coordinate system:
- **0°** = +Y direction (forward)
- **90°** = +X direction (right)
- **180°** = -Y direction (backward)
- **270°** = -X direction (left)

All angles are measured in degrees (0-360).

## Documentation

- **[PATH_RECORDING_GUIDE.md](PATH_RECORDING_GUIDE.md)**: Complete guide for recording and using paths
- **[PID_TUNING_GUIDE.md](PID_TUNING_GUIDE.md)**: Guide for tuning PID controllers (kH, kHi, kHd)

## Troubleshooting

### Odometry Issues
- **Position drifts**: Check encoder connections and IMU calibration
- **Wrong heading**: Verify IMU is calibrated and `set_robot_pose()` is called correctly
- **X/Y incorrect**: Check coordinate system matches your field setup

### Navigation Issues
- **Robot doesn't stop**: Check stop conditions in `driveToXY()` - may need to adjust thresholds
- **Overshoots target**: Reduce maximum velocity or tune PID gains
- **Turns too much**: See PID_TUNING_GUIDE.md for tuning `turnToXY` PID

### Compilation Issues
- **Missing includes**: Ensure all header files are in the `include/` directory
- **Linker errors**: Check that all source files are included in the build

## Code Organization

### Main Components

- **`main.cpp`**: Core robot code including odometry, navigation, driver control, and dashboard
- **`autons.cpp`**: All autonomous routine implementations
- **`robot-config.cpp`**: Hardware device definitions (motors, sensors, pneumatics)
- **`note.cpp`**: Background task for note/autonomous note functionality

### Key Classes/Structures

- **`RobotPose`**: Structure storing X, Y position and heading
- **`Drive`**: JAR Template drive class for chassis control
- **`Waypoint`**: Structure for path recording waypoints

## Development Notes

- **Odometry Task**: Runs continuously at 10ms intervals
- **Sensor Fusion**: 3% IMU correction applied to encoder-based heading
- **Velocity Profiles**: Cosine profiles for smooth acceleration/deceleration
- **Stop Conditions**: Multiple safety checks prevent infinite loops

## Acknowledgements

- **VEX Robotics** for their hardware and software support
- **JAR Template** for the drive and odometry library
- **HappyrobotTaipei** for their contributions to the development
- **Team 23083Z** for their hard work, dedication, and passion for robotics
- Special thanks to **R.T.1.3**, **Lego Lau mo**, and **Teng Lau** for their support

## License

This project is for Team 23083Z's use in VEX Robotics competitions.

---

**Last Updated**: 2025  
**Team**: 23083Z  
**Platform**: VEX V5

Feel free to explore, modify, and contribute to this repository as we continue to enhance our robot's capabilities for future competitions!
