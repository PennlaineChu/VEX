#pragma once
#include "mock_vex.h"
#include "mock_robot_config.h"

// Mock Drive class - simplified version of your JAR-Template Drive
class Drive {
public:
    vex::motor_group DriveL;
    vex::motor_group DriveR;
    
    Drive() : DriveL({L1, L2, L3}), DriveR({R1, R2, R3}) {}
    
    void drive_with_voltage(float leftVoltage, float rightVoltage) {
        DriveL.spin(vex::fwd, leftVoltage, vex::volt);
        DriveR.spin(vex::fwd, rightVoltage, vex::volt);
    }
    
    void set_drive_constants(float max_voltage, float kp, float ki, float kd, float starti) {
        drive_max_voltage = max_voltage;
        drive_kp = kp;
        drive_ki = ki;
        drive_kd = kd;
        drive_starti = starti;
    }
    
    void set_heading_constants(float max_voltage, float kp, float ki, float kd, float starti) {
        heading_max_voltage = max_voltage;
        heading_kp = kp;
        heading_ki = ki;
        heading_kd = kd;
        heading_starti = starti;
    }
    
    void turn_to_angle(float angle) {
        // Simulate turning to angle (clock system: 0-359 degrees)
        auto& sim = SimulationFramework::getInstance();
        
        // Calculate shortest turn path
        double current = sim.getState().heading;
        double target = angle;
        double turn_amount = target - current;
        
        // Normalize turn to shortest path
        while (turn_amount > 180.0) turn_amount -= 360.0;
        while (turn_amount < -180.0) turn_amount += 360.0;
        
        sim.getState().heading = target;
        // Normalize to 0-359 degrees
        while (sim.getState().heading >= 360.0) sim.getState().heading -= 360.0;
        while (sim.getState().heading < 0.0) sim.getState().heading += 360.0;
        
        sim.step(std::abs(turn_amount) * 10); // 10ms per degree
        std::cout << "[" << sim.getTime() << "ms] Turned to angle: " << angle << " degrees" << std::endl;
    }
    
    float get_absolute_heading() {
        return SimulationFramework::getInstance().getState().heading;
    }
    
    float get_X_position() {
        return SimulationFramework::getInstance().getState().x;
    }
    
    float get_Y_position() {
        return SimulationFramework::getInstance().getState().y;
    }
    
    float get_ForwardTracker_position() {
        return SimulationFramework::getInstance().getState().left_distance;
    }
    
    float get_SidewaysTracker_position() {
        return 0.0; // Simplified
    }
    
private:
    float drive_max_voltage = 12.0;
    float drive_kp = 1.0, drive_ki = 0.0, drive_kd = 0.0, drive_starti = 0.0;
    float heading_max_voltage = 12.0;
    float heading_kp = 1.0, heading_ki = 0.0, heading_kd = 0.0, heading_starti = 0.0;
};

// Global chassis instance
extern Drive chassis;

// Mock cos_move_distance_smooth function
void cos_move_distance_smooth(double distance_in, double angle_deg, double turn_maxV, double drive_maxV);
