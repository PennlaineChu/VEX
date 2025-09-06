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
    
    // Simulate turning to correct heading incrementally
    if (std::abs(heading_error) > 5.0) {
        const double wheelbase = 12.0; // inches between wheels
        const double turn_step = 5.0; // degrees per step
        const double turn_time_per_step = 50; // ms per 5-degree turn
        
        double remaining_turn = std::abs(heading_error);
        double turn_direction = (heading_error > 0) ? 1.0 : -1.0;
        
        std::cout << "[" << sim.getTime() << "ms] Starting turn from " 
                  << current_heading << "° to " << angle_deg << "° (" 
                  << heading_error << "° turn)" << std::endl;
        
        while (remaining_turn > 1.0) {
            double step_turn = std::min(turn_step, remaining_turn);
            
            // Update heading incrementally
            sim.getState().heading += turn_direction * step_turn;
            
            // Normalize to 0-359 degrees
            while (sim.getState().heading >= 360.0) sim.getState().heading -= 360.0;
            while (sim.getState().heading < 0.0) sim.getState().heading += 360.0;
            
            // Calculate differential wheel distances for this turn step
            double step_arc_length = step_turn * M_PI / 180.0 * (wheelbase / 2.0);
            
            if (turn_direction > 0) {
                // Turning left: right wheel travels more
                sim.getState().left_distance += step_arc_length * 0.5;
                sim.getState().right_distance += step_arc_length * 1.5;
            } else {
                // Turning right: left wheel travels more  
                sim.getState().left_distance += step_arc_length * 1.5;
                sim.getState().right_distance += step_arc_length * 0.5;
            }
            
            // Advance time
            sim.step(turn_time_per_step);
            
            remaining_turn -= step_turn;
            
            std::cout << "[" << sim.getTime() << "ms] Turn step: " 
                      << sim.getState().heading << "°, remaining: " 
                      << remaining_turn << "°" << std::endl;
            
            // Record snapshot during turn (if recorder is active)
            extern StateRecorder* g_active_recorder;
            if (g_active_recorder) {
                g_active_recorder->recordSnapshot();
            }
        }
        
        // Final adjustment to exact target
        sim.getState().heading = angle_deg;
        while (sim.getState().heading >= 360.0) sim.getState().heading -= 360.0;
        while (sim.getState().heading < 0.0) sim.getState().heading += 360.0;
        
        std::cout << "[" << sim.getTime() << "ms] Turn complete: " 
                  << sim.getState().heading << "°" << std::endl;
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
