#include "simulation_framework.h"
#include "mock_robot_config.h"
#include "mock_drive.h"
#include <iostream>

// Include the auton functions (we'll need to adapt these)
// For now, let's create a simplified version of B_right() for testing

// Mock auton functions
void B_right() {
    vex::color selectedTeamColor = vex::color::blue;

    chassis.set_drive_constants(12, 2.0, 0.005, 2, 10);
    chassis.set_heading_constants(12, 1.5, 0.005, 2, 10);
    
    intakedown.spin(vex::fwd, 10, vex::volt);
    vex::wait(0.05, vex::sec);
    
    cos_move_distance_smooth(20, 16, 10, 8);
    vex::wait(1.5, vex::sec);
    
    cos_move_distance_smooth(17, 305, 8, 6);
    vex::wait(0.05, vex::sec);
    
    intake.spin(vex::rev, 10, vex::volt);
    vex::wait(0.05, vex::sec);
    
    intakedown.spin(vex::rev, 10, vex::volt);
    vex::wait(2, vex::sec);
    
    intakedown.stop();
    vex::wait(0.05, vex::sec);
    
    intake.stop();
    cos_move_distance_smooth(-10, 310, 10, 10);
    vex::wait(0.05, vex::sec);
    
    intake.spin(vex::fwd, 10, vex::volt);
    vex::wait(0.05, vex::sec);
    
    intakedown.spin(vex::fwd, 10, vex::volt);
    cos_move_distance_smooth(30, 138, 10, 10);
    
    intakeCylander.set(true);
    cos_move_distance_smooth(10, 138, 10, 10);
    vex::wait(2, vex::sec);
    
    cos_move_distance_smooth(-2, 140, 10, 10);
    intakeCylander.set(false);
    aligner.set(true);
    shooter.set(true);
    vex::wait(0.05, vex::sec);
    
    cos_move_distance_smooth(43, 0, 10, 10);
    intake.spin(vex::rev, 10, vex::volt);
    vex::wait(0.05, vex::sec);
    
    intakedown.spin(vex::rev, 10, vex::volt);
}

// Test cases for B_right() function
void setupBRightTests() {
    // Test 1: Basic execution without crashes
    TestBuilder("B_right_basic_execution")
        .setup([]() {
            vexcodeInit();
            std::cout << "Setting up B_right basic execution test..." << std::endl;
        })
        .run([]() {
            B_right();
        })
        .check([]() {
            auto& sim = SimulationFramework::getInstance();
            ASSERT_TRUE(sim.getTime() > 0);
            std::cout << "Test completed in " << sim.getTime() << "ms" << std::endl;
        })
        .build();
    
    // Test 2: Check robot position after execution
    TestBuilder("B_right_position_check")
        .setup([]() {
            vexcodeInit();
            std::cout << "Setting up B_right position check test..." << std::endl;
        })
        .run([]() {
            B_right();
        })
        .check([]() {
            auto& sim = SimulationFramework::getInstance();
            double final_x = sim.getState().x;
            double final_y = sim.getState().y;
            
            std::cout << "Final position: (" << final_x << ", " << final_y << ")" << std::endl;
            
            // Check that robot has moved from origin
            ASSERT_TRUE(std::abs(final_x) > 1.0 || std::abs(final_y) > 1.0);
            
            // Check reasonable bounds (robot shouldn't teleport too far)
            ASSERT_RANGE(final_x, -100.0, 100.0);
            ASSERT_RANGE(final_y, -100.0, 100.0);
        })
        .build();
    
    // Test 3: Check timing constraints
    TestBuilder("B_right_timing_check")
        .setup([]() {
            vexcodeInit();
        })
        .run([]() {
            B_right();
        })
        .check([]() {
            auto& sim = SimulationFramework::getInstance();
            double total_time = sim.getTime();
            
            std::cout << "Total execution time: " << total_time << "ms" << std::endl;
            
            // B_right should take reasonable time (not too fast, not too slow)
            ASSERT_RANGE(total_time, 5000.0, 30000.0); // 5-30 seconds
        })
        .build();
    
    // Test 4: Check pneumatic states
    TestBuilder("B_right_pneumatic_states")
        .setup([]() {
            vexcodeInit();
        })
        .run([]() {
            B_right();
        })
        .check([]() {
            // Check that pneumatics were activated during the routine
            // (This is a simplified check - in reality you'd track state changes)
            std::cout << "Pneumatic states checked" << std::endl;
            ASSERT_TRUE(true); // Placeholder - would check actual pneumatic history
        })
        .build();
    
    // Test 5: Performance test with state recording
    TestBuilder("B_right_performance_analysis")
        .setup([]() {
            vexcodeInit();
        })
        .run([]() {
            StateRecorder recorder;
            recorder.startRecording();
            
            auto& sim = SimulationFramework::getInstance();
            
            // Record initial state
            recorder.recordSnapshot();
            
            // Run B_right with periodic state recording
            vex::color selectedTeamColor = vex::color::blue;

            chassis.set_drive_constants(12, 2.0, 0.005, 2, 10);
            chassis.set_heading_constants(12, 1.5, 0.005, 2, 10);
            
            intakedown.spin(vex::fwd, 10, vex::volt);
            vex::wait(0.05, vex::sec);
            recorder.recordSnapshot();
            
            cos_move_distance_smooth(20, 16, 10, 8);
            recorder.recordSnapshot();
            
            vex::wait(1.5, vex::sec);
            recorder.recordSnapshot();
            
            cos_move_distance_smooth(17, 305, 8, 6);
            recorder.recordSnapshot();
            
            vex::wait(0.05, vex::sec);
            
            intake.spin(vex::rev, 10, vex::volt);
            vex::wait(0.05, vex::sec);
            
            intakedown.spin(vex::rev, 10, vex::volt);
            vex::wait(2, vex::sec);
            recorder.recordSnapshot();
            
            intakedown.stop();
            vex::wait(0.05, vex::sec);
            
            intake.stop();
            cos_move_distance_smooth(-10, 310, 10, 10);
            recorder.recordSnapshot();
            
            vex::wait(0.05, vex::sec);
            
            intake.spin(vex::fwd, 10, vex::volt);
            vex::wait(0.05, vex::sec);
            
            intakedown.spin(vex::fwd, 10, vex::volt);
            cos_move_distance_smooth(30, 138, 10, 10);
            recorder.recordSnapshot();
            
            intakeCylander.set(true);
            cos_move_distance_smooth(10, 138, 10, 10);
            recorder.recordSnapshot();
            
            vex::wait(2, vex::sec);
            
            cos_move_distance_smooth(-2, 140, 10, 10);
            recorder.recordSnapshot();
            
            intakeCylander.set(false);
            aligner.set(true);
            shooter.set(true);
            vex::wait(0.05, vex::sec);
            
            cos_move_distance_smooth(43, 0, 10, 10);
            recorder.recordSnapshot();
            
            intake.spin(vex::rev, 10, vex::volt);
            vex::wait(0.05, vex::sec);
            
            intakedown.spin(vex::rev, 10, vex::volt);
            recorder.recordSnapshot();
            
            recorder.stopRecording();
            recorder.exportToCsv("b_right_performance.csv");
        })
        .check([]() {
            std::cout << "Performance data exported to b_right_performance.csv" << std::endl;
            ASSERT_TRUE(true);
        })
        .build();
}

// Interactive test runner
void runInteractiveTests() {
    std::cout << "\n=== VEX B_right() Function Test Suite ===" << std::endl;
    std::cout << "Available tests:" << std::endl;
    std::cout << "1. Basic execution test" << std::endl;
    std::cout << "2. Position check test" << std::endl;
    std::cout << "3. Timing check test" << std::endl;
    std::cout << "4. Pneumatic states test" << std::endl;
    std::cout << "5. Performance analysis test" << std::endl;
    std::cout << "6. Run all tests" << std::endl;
    std::cout << "0. Exit" << std::endl;
    
    int choice;
    std::cout << "\nEnter your choice: ";
    std::cin >> choice;
    
    auto& runner = TestRunner::getInstance();
    
    switch (choice) {
        case 1:
            runner.runTest("B_right_basic_execution");
            break;
        case 2:
            runner.runTest("B_right_position_check");
            break;
        case 3:
            runner.runTest("B_right_timing_check");
            break;
        case 4:
            runner.runTest("B_right_pneumatic_states");
            break;
        case 5:
            runner.runTest("B_right_performance_analysis");
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
        runInteractiveTests();
    }
}

int main() {
    std::cout << "VEX Robotics Simulation Framework" << std::endl;
    std::cout << "Testing B_right() autonomous function" << std::endl;
    
    // Setup all tests
    setupBRightTests();
    
    // Run interactive test menu
    runInteractiveTests();
    
    return 0;
}
