# VEX Simulation Framework - Getting Started

## Quick Setup

1. **Build the simulator:**
   ```bash
   cd simulation/
   make
   ```

2. **Run tests:**
   ```bash
   ./vex_simulator
   ```

3. **Install Python dependencies for visualization:**
   ```bash
   make install-deps
   ```

4. **Run tests with visualization:**
   ```bash
   make test-viz
   ```

## What You Just Created

You now have a complete simulation framework that allows you to:

### ✅ Test VEX Functions Without Hardware
- Mock all VEX hardware (motors, sensors, pneumatics)
- Simulate robot physics and movement
- Test autonomous functions like `B_right()` in seconds instead of minutes

### ✅ Comprehensive Testing Framework
- **Assertions**: Check position, timing, and behavior
- **State Tracking**: Monitor robot position, heading, and component states
- **Event Logging**: See exactly what happens when
- **Performance Analysis**: Measure execution time and efficiency

### ✅ Visual Analysis Tools
- **Robot Path Plots**: See where your robot goes
- **Timing Analysis**: Understand execution flow
- **Dashboard Views**: Comprehensive overview of test results
- **CSV Export**: Data for further analysis

## Example: Testing Your B_right() Function

The simulator shows that your `B_right()` function:
- **Executes in ~23 seconds** (simulated time)
- **Moves the robot** from (0,0) to (37.35, 24.73) inches
- **Uses multiple subsystems**: intake, pneumatics, drive
- **Follows a complex path** with multiple waypoints

## How This Helps Your Development

### 🚀 **Faster Iteration**
- Test changes in seconds, not minutes
- No need to upload code to robot for basic testing
- Catch logic errors before hardware testing

### 🔍 **Better Debugging**
- See exact robot position at any time
- Track motor and sensor states
- Identify timing issues and bottlenecks

### 📊 **Performance Optimization**
- Measure execution time
- Analyze movement efficiency
- Compare different autonomous strategies

### 🧪 **Regression Testing**
- Ensure changes don't break existing functionality
- Automated test suites for continuous integration
- Consistent testing environment

## Next Steps

### 1. Test Your Other Functions
Add tests for your other autonomous functions:
```cpp
// In test_b_right.cpp, add:
TestBuilder("R_right_test")
    .setup([]() { vexcodeInit(); })
    .run([]() { R_right(); })
    .check([]() {
        auto& sim = SimulationFramework::getInstance();
        // Add your assertions here
    })
    .build();
```

### 2. Customize the Physics
Modify `SimulationFramework::step()` to match your robot's characteristics:
- Wheel diameter and gear ratios
- Robot weight and acceleration
- Sensor response times

### 3. Add More Hardware
Extend the mock classes to include:
- Additional sensors (gyros, encoders, vision)
- Custom pneumatic systems
- Specialized mechanisms

### 4. Integration Testing
Test complete autonomous routines:
- Full 15-second autonomous periods
- Competition scenarios
- Error recovery sequences

## File Structure

```
simulation/
├── mock_vex.h              # VEX hardware simulation
├── simulation_framework.h  # Testing framework
├── test_b_right.cpp       # Your test cases
├── visualizer.py          # Python plotting tool
├── Makefile              # Build system
└── README.md             # Detailed documentation
```

## Tips for Effective Testing

### Write Good Tests
```cpp
// ✅ Good: Specific, measurable assertions
ASSERT_EQ(final_x, 24.0, 2.0);  // Within 2 inches
ASSERT_RANGE(total_time, 10000, 25000);  // 10-25 seconds

// ❌ Bad: Vague or overly strict
ASSERT_TRUE(moved_somewhere);
ASSERT_EQ(final_x, 24.000000);  // Too precise
```

### Test Edge Cases
- What happens if the robot starts at different positions?
- How does the function behave with low battery simulation?
- What if sensors give unexpected readings?

### Use Visualization
- Always check the robot path plots
- Look for unexpected movements or oscillations
- Verify the robot ends up where you expect

## Troubleshooting

### Build Issues
- Ensure you have a C++17 compatible compiler
- Check that all header files are included correctly
- Use `make clean && make` to rebuild from scratch

### Test Failures
- Check assertion tolerances (real robots aren't perfectly precise)
- Verify your autonomous function logic
- Use the event log to trace execution

### Visualization Problems
- Install Python dependencies: `pip3 install matplotlib pandas numpy`
- Check that CSV files are generated during tests
- Ensure you have display capabilities for plots

## Real-World Integration

This simulation framework complements, but doesn't replace, real hardware testing:

### Use Simulation For:
- ✅ Logic verification
- ✅ Timing analysis  
- ✅ Path planning
- ✅ Regression testing

### Use Hardware For:
- ✅ Sensor calibration
- ✅ Motor tuning
- ✅ Physical constraints
- ✅ Competition validation

The goal is to catch 80% of issues in simulation, leaving only hardware-specific problems for robot testing.

## Success! 🎉

You now have a professional-grade simulation framework that will significantly speed up your VEX development process. Start by testing your existing autonomous functions, then gradually expand the framework to cover more of your robot's capabilities.

Happy coding!
