/**
 * Example Usage of VEX Simulation Framework
 * 
 * This file demonstrates how to create and run tests for your VEX autonomous functions
 * using the simulation framework. It includes examples of different testing patterns
 * and best practices.
 */

#include "simulation_framework.h"
#include "mock_robot_config.h"
#include "mock_drive.h"
#include <iostream>

// Example autonomous function - simple square path
void drive_square() {
    chassis.set_drive_constants(12, 2.0, 0.005, 2, 10);
    chassis.set_heading_constants(12, 1.5, 0.005, 2, 10);
    
    for (int i = 0; i < 4; i++) {
        cos_move_distance_smooth(24, i * 90, 10, 10);  // Drive 24 inches
        vex::wait(0.5, vex::sec);                      // Pause
        chassis.turn_to_angle((i + 1) * 90);           // Turn 90 degrees
        vex::wait(0.5, vex::sec);                      // Pause
    }
}

// Example autonomous function - intake sequence
void intake_sequence() {
    // Start intake
    intake.spin(vex::fwd, 12, vex::volt);
    intakedown.spin(vex::fwd, 12, vex::volt);
    
    // Drive forward while intaking
    cos_move_distance_smooth(18, 0, 10, 8);
    vex::wait(1.0, vex::sec);
    
    // Stop intake
    intake.stop();
    intakedown.stop();
    
    // Activate pneumatic
    intakeCylander.set(true);
    vex::wait(0.5, vex::sec);
    
    // Back up
    cos_move_distance_smooth(-6, 0, 10, 8);
}

// Example test cases demonstrating different testing patterns
void setupExampleTests() {
    
    // Test 1: Basic movement validation
    TestBuilder("square_path_test")
        .setup([]() {
            std::cout << "Setting up square path test..." << std::endl;
            vexcodeInit();
        })
        .run([]() {
            drive_square();
        })
        .check([]() {
            auto& sim = SimulationFramework::getInstance();
            auto& state = sim.getState();
            
            // Should return close to starting position after square
            ASSERT_EQ(state.x, 0.0, 5.0);  // Within 5 inches of start
            ASSERT_EQ(state.y, 0.0, 5.0);
            ASSERT_EQ(state.heading, 0.0, 10.0);  // Within 10 degrees of start
            
            // Should take reasonable time
            ASSERT_RANGE(sim.getTime(), 8000.0, 20000.0);  // 8-20 seconds
        })
        .build();
    
    // Test 2: Intake sequence test with state tracking
    TestBuilder("intake_sequence_test")
        .setup([]() {
            vexcodeInit();
        })
        .run([]() {
            intake_sequence();
        })
        .check([]() {
            auto& sim = SimulationFramework::getInstance();
            auto& state = sim.getState();
            
            // Should move forward then back
            ASSERT_RANGE(state.x, 10.0, 15.0);  // Net forward movement
            ASSERT_EQ(state.y, 0.0, 2.0);       // Minimal side movement
            
            // Check that motors were activated (simplified check)
            ASSERT_TRUE(state.motor_positions.count("Motor_14") > 0);  // Intake motor
        })
        .build();
    
    // Test 3: Performance benchmark test
    TestBuilder("performance_benchmark")
        .setup([]() {
            vexcodeInit();
        })
        .run([]() {
            StateRecorder recorder;
            recorder.startRecording();
            
            auto& sim = SimulationFramework::getInstance();
            double start_time = sim.getTime();
            
            // Run a complex sequence
            chassis.set_drive_constants(12, 2.0, 0.005, 2, 10);
            
            // Forward
            cos_move_distance_smooth(36, 0, 10, 10);
            
            // Turn and move
            chassis.turn_to_angle(45);
            cos_move_distance_smooth(24, 45, 10, 10);
            
            // Turn and return
            chassis.turn_to_angle(225);
            cos_move_distance_smooth(30, 225, 10, 10);
            
            recorder.stopRecording();
            recorder.exportToCsv("performance_benchmark.csv");
            
            double total_time = sim.getTime() - start_time;
            std::cout << "Benchmark completed in " << total_time << "ms" << std::endl;
        })
        .check([]() {
            auto& sim = SimulationFramework::getInstance();
            
            // Performance should be reasonable
            ASSERT_RANGE(sim.getTime(), 5000.0, 30000.0);
            
            // Should end up in a different location
            auto& state = sim.getState();
            double displacement = sqrt(state.x * state.x + state.y * state.y);
            ASSERT_TRUE(displacement > 10.0);  // Should move at least 10 inches from origin
        })
        .build();
    
    // Test 4: Error condition test
    TestBuilder("boundary_test")
        .setup([]() {
            vexcodeInit();
        })
        .run([]() {
            // Test extreme movements
            cos_move_distance_smooth(100, 0, 10, 10);   // Very long movement
            chassis.turn_to_angle(720);                  // Multiple rotations
            cos_move_distance_smooth(-50, 180, 10, 10); // Reverse movement
        })
        .check([]() {
            auto& sim = SimulationFramework::getInstance();
            auto& state = sim.getState();
            
            // Should handle extreme values gracefully
            ASSERT_RANGE(state.x, -200.0, 200.0);  // Reasonable bounds
            ASSERT_RANGE(state.y, -200.0, 200.0);
            
            // Heading should be normalized
            ASSERT_RANGE(state.heading, -180.0, 180.0);
        })
        .build();
    
    // Test 5: Pneumatic system test
    TestBuilder("pneumatic_system_test")
        .setup([]() {
            vexcodeInit();
        })
        .run([]() {
            // Test all pneumatic systems
            intakeCylander.set(true);
            vex::wait(0.1, vex::sec);
            
            shooter.set(true);
            aligner.set(true);
            vex::wait(0.2, vex::sec);
            
            pushCylinder.set(true);
            vex::wait(0.1, vex::sec);
            
            // Turn everything off
            intakeCylander.set(false);
            shooter.set(false);
            aligner.set(false);
            pushCylinder.set(false);
        })
        .check([]() {
            // This test mainly checks that pneumatics don't crash the system
            auto& sim = SimulationFramework::getInstance();
            ASSERT_TRUE(sim.getTime() > 0);
            
            std::cout << "Pneumatic system test completed successfully" << std::endl;
        })
        .build();
}

// Interactive menu for running specific tests
void runInteractiveExamples() {
    std::cout << "\n=== VEX Simulation Framework Examples ===" << std::endl;
    std::cout << "Available example tests:" << std::endl;
    std::cout << "1. Square path test" << std::endl;
    std::cout << "2. Intake sequence test" << std::endl;
    std::cout << "3. Performance benchmark" << std::endl;
    std::cout << "4. Boundary conditions test" << std::endl;
    std::cout << "5. Pneumatic system test" << std::endl;
    std::cout << "6. Run all example tests" << std::endl;
    std::cout << "0. Exit" << std::endl;
    
    int choice;
    std::cout << "\nEnter your choice: ";
    std::cin >> choice;
    
    auto& runner = TestRunner::getInstance();
    
    switch (choice) {
        case 1:
            runner.runTest("square_path_test");
            break;
        case 2:
            runner.runTest("intake_sequence_test");
            break;
        case 3:
            runner.runTest("performance_benchmark");
            break;
        case 4:
            runner.runTest("boundary_test");
            break;
        case 5:
            runner.runTest("pneumatic_system_test");
            break;
        case 6:
            runner.runAllTests();
            break;
        case 0:
            return;
        default:
            std::cout << "Invalid choice!" << std::endl;
    }
    
    // Ask if user wants to run another test
    char again;
    std::cout << "\nRun another test? (y/n): ";
    std::cin >> again;
    if (again == 'y' || again == 'Y') {
        runInteractiveExamples();
    }
}

// Demonstration of advanced features
void demonstrateAdvancedFeatures() {
    std::cout << "\n=== Advanced Features Demo ===" << std::endl;
    
    // Time scale demonstration
    std::cout << "\n1. Time Scale Control:" << std::endl;
    auto& sim = SimulationFramework::getInstance();
    sim.reset();
    
    std::cout << "Running at normal speed..." << std::endl;
    sim.setTimeScale(1.0);
    cos_move_distance_smooth(12, 0, 10, 10);
    double normal_time = sim.getTime();
    
    sim.reset();
    std::cout << "Running at 5x speed..." << std::endl;
    sim.setTimeScale(5.0);
    cos_move_distance_smooth(12, 0, 10, 10);
    double fast_time = sim.getTime();
    
    std::cout << "Normal time: " << normal_time << "ms" << std::endl;
    std::cout << "Fast time: " << fast_time << "ms" << std::endl;
    
    // State recording demonstration
    std::cout << "\n2. State Recording:" << std::endl;
    sim.reset();
    sim.setTimeScale(1.0);
    
    StateRecorder recorder;
    recorder.startRecording();
    
    // Record state every 100ms during a complex movement
    for (int i = 0; i < 10; i++) {
        cos_move_distance_smooth(6, i * 36, 10, 10);  // Move in a circle
        recorder.recordSnapshot();
    }
    
    recorder.stopRecording();
    recorder.exportToCsv("advanced_demo.csv");
    
    std::cout << "Recorded " << recorder.getSnapshots().size() << " state snapshots" << std::endl;
    std::cout << "Data exported to advanced_demo.csv" << std::endl;
    
    // Event logging demonstration
    std::cout << "\n3. Event Logging:" << std::endl;
    sim.reset();
    
    sim.logEvent("Starting demonstration sequence");
    cos_move_distance_smooth(12, 0, 10, 10);
    sim.logEvent("First movement complete");
    
    chassis.turn_to_angle(90);
    sim.logEvent("Turn complete");
    
    cos_move_distance_smooth(12, 90, 10, 10);
    sim.logEvent("Second movement complete");
    
    std::cout << "\nEvent log:" << std::endl;
    for (const auto& event : sim.getLog()) {
        std::cout << "  " << event << std::endl;
    }
}

int main() {
    std::cout << "VEX Robotics Simulation Framework - Example Usage" << std::endl;
    std::cout << "=================================================" << std::endl;
    
    // Setup example tests
    setupExampleTests();
    
    while (true) {
        std::cout << "\nMain Menu:" << std::endl;
        std::cout << "1. Run example tests" << std::endl;
        std::cout << "2. Demonstrate advanced features" << std::endl;
        std::cout << "3. Exit" << std::endl;
        
        int choice;
        std::cout << "\nEnter your choice: ";
        std::cin >> choice;
        
        switch (choice) {
            case 1:
                runInteractiveExamples();
                break;
            case 2:
                demonstrateAdvancedFeatures();
                break;
            case 3:
                std::cout << "Goodbye!" << std::endl;
                return 0;
            default:
                std::cout << "Invalid choice!" << std::endl;
        }
    }
    
    return 0;
}
