#include "mock_drive.h"
#include "mock_robot_config.h"
#include "simulation_framework.h"
#include <cmath>

// Global chassis instance
Drive chassis;

// Mock implementation of cos_move_distance_smooth
void cos_move_distance_smooth(double distance_in, double angle_deg, double turn_maxV, double drive_maxV) {
    auto& sim = SimulationFramework::getInstance();
    
    std::cout << "[" << sim.getTime() << "ms] cos_move_distance_smooth: " 
              << distance_in << " inches at " << angle_deg << " degrees" << std::endl;
    
    // Simulate the movement
    double current_heading = sim.getState().heading;
    double heading_error = angle_deg - current_heading;
    
    // Normalize heading error for clock system (shortest path)
    while (heading_error > 180.0) heading_error -= 360.0;
    while (heading_error < -180.0) heading_error += 360.0;
    
    // Simulate turning to correct heading first
    if (std::abs(heading_error) > 5.0) {
        // Calculate differential wheel distances for turning
        const double wheelbase = 12.0; // inches between wheels
        double turn_arc_length = std::abs(heading_error) * M_PI / 180.0 * (wheelbase / 2.0);
        
        if (heading_error > 0) {
            // Turning left: right wheel travels more
            sim.getState().left_distance += turn_arc_length * 0.5;
            sim.getState().right_distance += turn_arc_length * 1.5;
        } else {
            // Turning right: left wheel travels more  
            sim.getState().left_distance += turn_arc_length * 1.5;
            sim.getState().right_distance += turn_arc_length * 0.5;
        }
        
        sim.getState().heading = angle_deg;
        // Normalize to 0-359 degrees
        while (sim.getState().heading >= 360.0) sim.getState().heading -= 360.0;
        while (sim.getState().heading < 0.0) sim.getState().heading += 360.0;
        sim.step(std::abs(heading_error) * 10); // 10ms per degree of turn
    }
    
    // Simulate forward movement incrementally (convert clock heading to math coordinates)
    // Clock system: 0° = North (+Y), 90° = East (+X)
    double angle_rad = (90.0 - angle_deg) * M_PI / 180.0;
    
    // Break movement into small increments for smooth simulation
    const double step_size = 2.0; // inches per step
    const double step_time = 200; // ms per step (10 inches/second = 2 inches/200ms)
    
    double remaining_distance = std::abs(distance_in);
    double direction = (distance_in >= 0) ? 1.0 : -1.0;
    
    while (remaining_distance > 0.1) { // Continue until very close
        double step_distance = std::min(step_size, remaining_distance);
        
        // Move incrementally
        sim.getState().x += direction * step_distance * cos(angle_rad);
        sim.getState().y += direction * step_distance * sin(angle_rad);
        
        // Update distance trackers
        sim.getState().left_distance += step_distance;
        sim.getState().right_distance += step_distance;
        
        // Advance time
        sim.step(step_time);
        
        remaining_distance -= step_distance;
        
        std::cout << "[" << sim.getTime() << "ms] Position: (" 
                  << sim.getState().x << ", " << sim.getState().y 
                  << "), Remaining: " << remaining_distance << " inches" << std::endl;
        
        // Record snapshot for CSV data (if recorder is active)
        // We'll use a global recorder instance
        extern StateRecorder* g_active_recorder;
        if (g_active_recorder) {
            g_active_recorder->recordSnapshot();
        }
    }
    
    std::cout << "[" << sim.getTime() << "ms] Movement complete. Position: (" 
              << sim.getState().x << ", " << sim.getState().y << "), Heading: " 
              << sim.getState().heading << std::endl;
}
