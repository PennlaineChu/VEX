#pragma once
#include "mock_vex.h"
#include <vector>
#include <functional>
#include <string>
#include <map>
#include <fstream>

// Test assertion framework
class TestAssert {
public:
    static void assertTrue(bool condition, const std::string& message) {
        if (!condition) {
            throw std::runtime_error("Assertion failed: " + message);
        }
    }
    
    static void assertEquals(double expected, double actual, double tolerance, const std::string& message) {
        if (std::abs(expected - actual) > tolerance) {
            throw std::runtime_error("Assertion failed: " + message + 
                " (expected: " + std::to_string(expected) + 
                ", actual: " + std::to_string(actual) + ")");
        }
    }
    
    static void assertRange(double value, double min, double max, const std::string& message) {
        if (value < min || value > max) {
            throw std::runtime_error("Assertion failed: " + message + 
                " (value: " + std::to_string(value) + 
                ", range: [" + std::to_string(min) + ", " + std::to_string(max) + "])");
        }
    }
};

// Test case structure
struct TestCase {
    std::string name;
    std::function<void()> setup;
    std::function<void()> test_function;
    std::function<void()> teardown;
    std::vector<std::function<void()>> assertions;
    
    TestCase(const std::string& test_name) : name(test_name) {}
};

// Robot state recorder for analysis
class StateRecorder {
public:
    struct StateSnapshot {
        double timestamp;
        double x, y, heading;
        std::map<std::string, double> motor_positions;
        std::map<std::string, double> motor_velocities;
        std::map<std::string, bool> pneumatic_states;
    };
    
    void startRecording() {
        recording = true;
        snapshots.clear();
    }
    
    void stopRecording() {
        recording = false;
    }
    
    void recordSnapshot() {
        if (!recording) return;
        
        auto& sim = SimulationFramework::getInstance();
        StateSnapshot snapshot;
        snapshot.timestamp = sim.getTime();
        snapshot.x = sim.getState().x;
        snapshot.y = sim.getState().y;
        snapshot.heading = sim.getState().heading;
        snapshot.motor_positions = sim.getState().motor_positions;
        snapshot.motor_velocities = sim.getState().motor_velocities;
        snapshot.pneumatic_states = sim.getState().pneumatic_states;
        
        snapshots.push_back(snapshot);
    }
    
    const std::vector<StateSnapshot>& getSnapshots() const { return snapshots; }
    
    void exportToCsv(const std::string& filename) const {
        std::ofstream file(filename);
        file << "timestamp,x,y,heading,left_distance,right_distance\n";
        
        for (const auto& snapshot : snapshots) {
            file << snapshot.timestamp << ","
                 << snapshot.x << ","
                 << snapshot.y << ","
                 << snapshot.heading << ","
                 << "0,0\n"; // Placeholder for distances
        }
    }
    
private:
    bool recording = false;
    std::vector<StateSnapshot> snapshots;
};

// Test runner
class TestRunner {
public:
    static TestRunner& getInstance() {
        static TestRunner instance;
        return instance;
    }
    
    void addTest(const TestCase& test) {
        tests.push_back(test);
    }
    
    void runAllTests() {
        int passed = 0, failed = 0;
        
        std::cout << "\n=== RUNNING TESTS ===" << std::endl;
        
        for (auto& test : tests) {
            std::cout << "\nRunning test: " << test.name << std::endl;
            
            try {
                // Setup
                SimulationFramework::getInstance().reset();
                if (test.setup) test.setup();
                
                // Run test
                if (test.test_function) test.test_function();
                
                // Run assertions
                for (auto& assertion : test.assertions) {
                    assertion();
                }
                
                // Teardown
                if (test.teardown) test.teardown();
                
                std::cout << "✓ PASSED: " << test.name << std::endl;
                passed++;
                
            } catch (const std::exception& e) {
                std::cout << "✗ FAILED: " << test.name << " - " << e.what() << std::endl;
                failed++;
            }
        }
        
        std::cout << "\n=== TEST RESULTS ===" << std::endl;
        std::cout << "Passed: " << passed << std::endl;
        std::cout << "Failed: " << failed << std::endl;
        std::cout << "Total:  " << (passed + failed) << std::endl;
    }
    
    void runTest(const std::string& test_name) {
        for (auto& test : tests) {
            if (test.name == test_name) {
                std::cout << "\nRunning test: " << test.name << std::endl;
                
                try {
                    SimulationFramework::getInstance().reset();
                    if (test.setup) test.setup();
                    if (test.test_function) test.test_function();
                    for (auto& assertion : test.assertions) {
                        assertion();
                    }
                    if (test.teardown) test.teardown();
                    
                    std::cout << "✓ PASSED: " << test.name << std::endl;
                } catch (const std::exception& e) {
                    std::cout << "✗ FAILED: " << test.name << " - " << e.what() << std::endl;
                }
                return;
            }
        }
        std::cout << "Test not found: " << test_name << std::endl;
    }
    
private:
    std::vector<TestCase> tests;
};

// Convenience macros for testing
#define ASSERT_TRUE(condition) TestAssert::assertTrue(condition, #condition)
#define ASSERT_EQ(expected, actual, tolerance) TestAssert::assertEquals(expected, actual, tolerance, #expected " == " #actual)
#define ASSERT_RANGE(value, min, max) TestAssert::assertRange(value, min, max, #min " <= " #value " <= " #max)

// Test builder helper
class TestBuilder {
public:
    TestBuilder(const std::string& name) : test_case(name) {}
    
    TestBuilder& setup(std::function<void()> setup_func) {
        test_case.setup = setup_func;
        return *this;
    }
    
    TestBuilder& run(std::function<void()> test_func) {
        test_case.test_function = test_func;
        return *this;
    }
    
    TestBuilder& check(std::function<void()> assertion_func) {
        test_case.assertions.push_back(assertion_func);
        return *this;
    }
    
    TestBuilder& teardown(std::function<void()> teardown_func) {
        test_case.teardown = teardown_func;
        return *this;
    }
    
    void build() {
        TestRunner::getInstance().addTest(test_case);
    }
    
private:
    TestCase test_case;
};
