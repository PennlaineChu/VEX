# VEX Robotics Simulation Framework

A comprehensive simulation environment for testing VEX robotics autonomous functions without hardware. This framework allows you to iterate faster during development by testing your code in a simulated environment.

## Features

- **Mock VEX Hardware**: Simulates motors, sensors, pneumatics, and controllers
- **Physics Simulation**: Basic differential drive kinematics and motor physics
- **Test Framework**: Comprehensive testing with assertions and logging
- **Visualization**: Python-based plotting for robot movement analysis
- **State Tracking**: Record and analyze robot behavior over time
- **Interactive Testing**: Menu-driven test execution

## Quick Start

### 1. Install Dependencies

```bash
# Install Python dependencies for visualization
make install-deps
```

#### Install Dependencies in Virtul Environment
```
python3.11 -m virtualenv -p python3 .venv
source .venv/bin/activate
pip3 install matplotlib pandas numpy
```

### 2. Build and Run Tests

```bash
# Build the simulator
make

# Run tests interactively
make test

# Run tests with visualization
make test-viz
```

### 3. View Results

After running tests with visualization, check the `plots/` directory for:
- Robot path visualization
- Heading over time graphs
- Position component plots
- Distance traveled analysis
- Comprehensive dashboard

## Project Structure

```
simulation/
├── mock_vex.h              # Mock VEX hardware classes
├── mock_vex.cpp            # Implementation of mock hardware
├── mock_robot_config.h     # Mock robot configuration
├── mock_robot_config.cpp   # Robot hardware instances
├── mock_drive.h            # Mock drive system
├── mock_drive.cpp          # Drive system implementation
├── simulation_framework.h  # Test framework and assertions
├── test_b_right.cpp        # Test cases for B_right() function
├── visualizer.py           # Python visualization tool
├── Makefile               # Build system
└── README.md              # This file
```

## Testing Your Functions

### Basic Test Structure

```cpp
#include "simulation_framework.h"
#include "mock_robot_config.h"
#include "mock_drive.h"

// Your autonomous function
void my_auton_function() {
    chassis.set_drive_constants(12, 2.0, 0.005, 2, 10);
    cos_move_distance_smooth(24, 0, 10, 10);
    chassis.turn_to_angle(90);
    // ... more commands
}

// Create a test
TestBuilder("my_auton_test")
    .setup([]() {
        vexcodeInit();
        // Setup code here
    })
    .run([]() {
        my_auton_function();
    })
    .assert([]() {
        auto& sim = SimulationFramework::getInstance();
        ASSERT_EQ(sim.getState().x, 24.0, 1.0); // Within 1 inch
        ASSERT_EQ(sim.getState().heading, 90.0, 5.0); // Within 5 degrees
    })
    .build();
```

### Available Assertions

```cpp
ASSERT_TRUE(condition);                           // Boolean assertion
ASSERT_EQ(expected, actual, tolerance);           // Equality with tolerance
ASSERT_RANGE(value, min, max);                    // Range check
```

### State Access

```cpp
auto& sim = SimulationFramework::getInstance();
auto& state = sim.getState();

// Robot position and orientation
double x = state.x;
double y = state.y;
double heading = state.heading;

// Motor states
double motor_pos = state.motor_positions["Motor_1"];
double motor_vel = state.motor_velocities["Motor_1"];

// Pneumatic states
bool cylinder_state = state.pneumatic_states["intakeCylander"];
```

## Example Test Cases

The framework includes several example test cases for the `B_right()` function:

### 1. Basic Execution Test
Verifies that the function runs without crashes and completes in reasonable time.

### 2. Position Check Test
Validates that the robot moves to expected positions and stays within reasonable bounds.

### 3. Timing Check Test
Ensures the autonomous routine completes within expected time limits.

### 4. Pneumatic States Test
Checks that pneumatic components are activated correctly during the routine.

### 5. Performance Analysis Test
Records detailed state information and exports it for visualization.

## Visualization

The Python visualizer creates several types of plots:

### Robot Path Plot
Shows the 2D path taken by the robot with start/end markers and direction arrows.

### Heading Over Time
Displays how the robot's orientation changes throughout the routine.

### Position Components
Separate X and Y position graphs over time.

### Distance Traveled
Cumulative distance traveled by the robot.

### Dashboard
Comprehensive view combining all plots in a single image.

### Usage Examples

```bash
# Generate individual plots
python3 visualizer.py data.csv --output plots

# Generate dashboard
python3 visualizer.py data.csv --output plots --dashboard

# Print summary statistics
python3 visualizer.py data.csv --summary

# Interactive viewing (no save)
python3 visualizer.py data.csv
```

## Customizing the Simulation

### Adding New Hardware

To add new sensors or actuators:

1. Add the mock class to `mock_vex.h`
2. Implement behavior in `mock_vex.cpp`
3. Add instances to `mock_robot_config.cpp`

### Modifying Physics

The physics simulation is in `SimulationFramework::step()`. You can modify:
- Motor response characteristics
- Wheel diameter and gear ratios
- Wheelbase dimensions
- Friction and acceleration models

### Creating New Tests

```cpp
TestBuilder("your_test_name")
    .setup([]() {
        // Initialize hardware, set starting conditions
    })
    .run([]() {
        // Run your autonomous function
        your_function();
    })
    .assert([]() {
        // Check results
        ASSERT_TRUE(some_condition);
    })
    .teardown([]() {
        // Cleanup if needed
    })
    .build();
```

## Advanced Features

### State Recording

```cpp
StateRecorder recorder;
recorder.startRecording();

// Your code here - recorder will capture state changes

recorder.stopRecording();
recorder.exportToCsv("my_data.csv");
```

### Time Control

```cpp
auto& sim = SimulationFramework::getInstance();
sim.setTimeScale(2.0);  // Run 2x faster
sim.step(100);          // Advance 100ms
```

### Event Logging

```cpp
sim.logEvent("Started intake sequence");
// Events appear in console and can be exported
```

## Troubleshooting

### Common Issues

1. **Compilation Errors**: Ensure you have a C++17 compatible compiler
2. **Python Import Errors**: Run `make install-deps` to install required packages
3. **Missing Plots**: Check that the CSV file was generated during test execution
4. **Assertion Failures**: Adjust tolerance values or check your autonomous logic

### Debug Tips

- Use `std::cout` statements in your autonomous functions for debugging
- Check the simulation log for detailed timing and state information
- Use the visualizer to understand robot behavior
- Start with simple movements and build complexity gradually

## Contributing

To add new features or fix bugs:

1. Follow the existing code style
2. Add tests for new functionality
3. Update documentation
4. Test with multiple autonomous routines

## License

This simulation framework is provided as-is for educational and development purposes.
