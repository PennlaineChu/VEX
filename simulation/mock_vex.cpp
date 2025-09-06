#include "mock_vex.h"
#include "simulation_framework.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cstdarg>
#include <cstring>

// Global recorder pointer for automatic snapshot recording
StateRecorder* g_active_recorder = nullptr;

// SimulationFramework implementation
void SimulationFramework::reset() {
    current_time_ms = 0.0;
    robot_state = RobotState();
    event_log.clear();
    std::cout << "=== SIMULATION RESET ===" << std::endl;
}

void SimulationFramework::step(double dt_ms) {
    current_time_ms += dt_ms * time_scale;
    
    // Update robot physics based on motor states
    // This is a simplified differential drive model
    auto& state = robot_state;
    
    // Get left and right motor velocities (simplified)
    double left_vel = 0.0, right_vel = 0.0;
    if (state.motor_velocities.count("Motor_1")) left_vel += state.motor_velocities["Motor_1"];
    if (state.motor_velocities.count("Motor_2")) left_vel += state.motor_velocities["Motor_2"];
    if (state.motor_velocities.count("Motor_3")) left_vel += state.motor_velocities["Motor_3"];
    if (state.motor_velocities.count("Motor_7")) right_vel += state.motor_velocities["Motor_7"];
    if (state.motor_velocities.count("Motor_8")) right_vel += state.motor_velocities["Motor_8"];
    if (state.motor_velocities.count("Motor_9")) right_vel += state.motor_velocities["Motor_9"];
    
    left_vel /= 3.0;  // Average of 3 motors
    right_vel /= 3.0;
    
    // Convert motor velocity to wheel velocity (simplified)
    const double wheel_diameter = 3.25; // inches
    const double gear_ratio = 0.75;
    const double wheel_circumference = M_PI * wheel_diameter;
    
    double left_wheel_speed = left_vel * gear_ratio * wheel_circumference / 360.0; // inches per second
    double right_wheel_speed = right_vel * gear_ratio * wheel_circumference / 360.0;
    
    // Differential drive kinematics
    const double wheelbase = 12.0; // inches between wheels
    double dt_sec = dt_ms / 1000.0;
    
    double linear_velocity = (left_wheel_speed + right_wheel_speed) / 2.0;
    double angular_velocity = (right_wheel_speed - left_wheel_speed) / wheelbase;
    
    // Update robot position (clock-based: 0-359 degrees)
    state.heading += angular_velocity * dt_sec * 180.0 / M_PI; // Convert to degrees
    
    // Normalize to 0-359 degrees (clock system)
    while (state.heading >= 360.0) state.heading -= 360.0;
    while (state.heading < 0.0) state.heading += 360.0;
    
    // Convert clock heading to math coordinates (0° = North = +Y)
    // In clock system: 0° = North (+Y), 90° = East (+X)
    // In math system: 0° = East (+X), 90° = North (+Y)
    // So we need to rotate by 90° and flip Y
    double heading_rad = (90.0 - state.heading) * M_PI / 180.0;
    state.x += linear_velocity * cos(heading_rad) * dt_sec;
    state.y += linear_velocity * sin(heading_rad) * dt_sec;
    
    // Update distance traveled
    state.left_distance += left_wheel_speed * dt_sec;
    state.right_distance += right_wheel_speed * dt_sec;
}

void SimulationFramework::logEvent(const std::string& event) {
    std::ostringstream oss;
    oss << "[" << std::fixed << std::setprecision(1) << current_time_ms << "ms] " << event;
    event_log.push_back(oss.str());
    std::cout << oss.str() << std::endl;
}

// VEX wait function implementation
namespace vex {
    void wait(double time, timeUnits units) {
        double time_ms = (units == sec) ? time * 1000.0 : time;
        SimulationFramework::getInstance().step(time_ms);
        std::cout << "[" << SimulationFramework::getInstance().getTime() << "ms] Waited " << time_ms << "ms" << std::endl;
    }
}
