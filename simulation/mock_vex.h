#pragma once
#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <chrono>
#include <cmath>
#include <functional>

// Mock VEX namespace and types
namespace vex {
    enum class directionType { fwd, rev };
    enum class velocityUnits { pct, rpm };
    enum class voltageUnits { volt };
    enum class rotationUnits { deg, turn };
    enum class brakeType { coast, brake, hold };
    enum class timeUnits { sec, msec };
    enum class fontType { mono12, mono20 };
    enum class color { red, blue, white, black, transparent };
    
    const directionType fwd = directionType::fwd;
    const directionType rev = directionType::rev;
    const velocityUnits pct = velocityUnits::pct;
    const velocityUnits rpm = velocityUnits::rpm;
    const voltageUnits volt = voltageUnits::volt;
    const rotationUnits deg = rotationUnits::deg;
    const rotationUnits turn = rotationUnits::turn;
    const brakeType coast = brakeType::coast;
    const brakeType brake = brakeType::brake;
    const brakeType hold = brakeType::hold;
    const timeUnits sec = timeUnits::sec;
    const timeUnits msec = timeUnits::msec;
    const fontType mono12 = fontType::mono12;
    const fontType mono20 = fontType::mono20;
    const color red = color::red;
    const color blue = color::blue;
    const color white = color::white;
    const color black = color::black;
    const color transparent = color::transparent;
    
    // Forward declarations
    class motor;
    class motor_group;
    class inertial;
    class controller;
    class optical;
    class digital_out;
    class vision;
    class brain;
    class competition;
    class triport;
    class rotation;
    class encoder;
    class task;
}

// Simulation framework
class SimulationFramework {
public:
    static SimulationFramework& getInstance() {
        static SimulationFramework instance;
        return instance;
    }
    
    void reset();
    void step(double dt_ms = 10.0);
    double getTime() const { return current_time_ms; }
    void setTimeScale(double scale) { time_scale = scale; }
    
    // Robot state tracking
    struct RobotState {
        double x = 0.0, y = 0.0, heading = 0.0;
        double left_distance = 0.0, right_distance = 0.0;
        std::map<std::string, double> motor_positions;
        std::map<std::string, double> motor_velocities;
        std::map<std::string, bool> pneumatic_states;
    };
    
    RobotState& getState() { return robot_state; }
    const RobotState& getState() const { return robot_state; }
    
    void logEvent(const std::string& event);
    const std::vector<std::string>& getLog() const { return event_log; }
    
private:
    double current_time_ms = 0.0;
    double time_scale = 1.0;
    RobotState robot_state;
    std::vector<std::string> event_log;
};

// Mock VEX classes
namespace vex {
    
class motor {
public:
    motor(int port, bool reversed = false) : port_num(port), is_reversed(reversed) {
        name = "Motor_" + std::to_string(port);
    }
    
    void spin(directionType dir, double value, voltageUnits units) {
        double voltage = (dir == fwd) ? value : -value;
        if (is_reversed) voltage = -voltage;
        current_voltage = voltage;
        
        // Simple physics simulation
        double dt = 0.01; // 10ms
        velocity += (voltage * 10.0 - velocity * 0.1) * dt; // Simple motor model
        motor_position += velocity * dt;
        
        auto& sim = SimulationFramework::getInstance();
        sim.getState().motor_positions[name] = motor_position;
        sim.getState().motor_velocities[name] = velocity;
        
        std::cout << "[" << sim.getTime() << "ms] " << name << " spin: " << voltage << "V" << std::endl;
    }
    
    void stop(brakeType mode = coast) {
        current_voltage = 0.0;
        if (mode == brake || mode == hold) {
            velocity = 0.0;
        }
        std::cout << "[" << SimulationFramework::getInstance().getTime() << "ms] " << name << " stopped" << std::endl;
    }
    
    double position(rotationUnits units = deg) const {
        return (units == deg) ? motor_position : motor_position / 360.0;
    }
    
    void resetPosition() {
        motor_position = 0.0;
        std::cout << "[" << SimulationFramework::getInstance().getTime() << "ms] " << name << " position reset" << std::endl;
    }
    
    void setStopping(brakeType mode) {
        brake_mode = mode;
    }
    
    void setMaxTorque(double torque, velocityUnits units) {
        max_torque = torque;
    }
    
    void spinToPosition(double pos, rotationUnits units, bool wait_for_completion = false) {
        target_position = (units == deg) ? pos : pos * 360.0;
        // Simulate movement to position
        motor_position = target_position;
        std::cout << "[" << SimulationFramework::getInstance().getTime() << "ms] " << name << " moved to position: " << target_position << std::endl;
    }
    
private:
    int port_num;
    bool is_reversed;
    std::string name;
    double motor_position = 0.0;
    double velocity = 0.0;
    double current_voltage = 0.0;
    double target_position = 0.0;
    double max_torque = 100.0;
    brakeType brake_mode = coast;
};

class motor_group {
public:
    motor_group(std::initializer_list<motor> motors) : motors_(motors) {}
    
    void spin(directionType dir, double value, voltageUnits units) {
        for (auto& m : motors_) {
            m.spin(dir, value, units);
        }
    }
    
    void stop(brakeType mode = coast) {
        for (auto& m : motors_) {
            m.stop(mode);
        }
    }
    
private:
    std::vector<motor> motors_;
};

class inertial {
public:
    inertial(int port) : port_num(port) {}
    
    void calibrate() {
        is_calibrating_ = true;
        calibration_start_time = SimulationFramework::getInstance().getTime();
        std::cout << "[" << SimulationFramework::getInstance().getTime() << "ms] Inertial calibrating..." << std::endl;
    }
    
    bool isCalibrating() {
        if (is_calibrating_) {
            double elapsed = SimulationFramework::getInstance().getTime() - calibration_start_time;
            if (elapsed > 2000.0) { // 2 seconds calibration
                is_calibrating_ = false;
                std::cout << "[" << SimulationFramework::getInstance().getTime() << "ms] Inertial calibration complete" << std::endl;
            }
        }
        return is_calibrating_;
    }
    
    double heading(rotationUnits units = deg) const {
        return SimulationFramework::getInstance().getState().heading;
    }
    
private:
    int port_num;
    bool is_calibrating_ = false;
    double calibration_start_time = 0.0;
};

class digital_out {
public:
    digital_out() {}
    
    void set(bool state) {
        current_state = state;
        std::cout << "[" << SimulationFramework::getInstance().getTime() << "ms] Digital out set to: " << (state ? "true" : "false") << std::endl;
    }
    
    bool value() const { return current_state; }
    
    digital_out& operator=(bool state) {
        set(state);
        return *this;
    }
    
    operator bool() const { return current_state; }
    
private:
    bool current_state = false;
};

class optical {
public:
    optical(int port) : port_num(port) {}
    
    void setLightPower(double power, velocityUnits units) {
        light_power = power;
    }
    
    bool isNearObject() const {
        // Simulate object detection based on time or other conditions
        return false; // Default: no object
    }
    
private:
    int port_num;
    double light_power = 0.0;
};

class vision {
public:
    vision(int port, int signature_id) : port_num(port), sig_id(signature_id) {}
    
    void setLedColor(int r, int g, int b) {
        led_r = r; led_g = g; led_b = b;
        std::cout << "[" << SimulationFramework::getInstance().getTime() << "ms] Vision LED set to RGB(" << r << "," << g << "," << b << ")" << std::endl;
    }
    
private:
    int port_num;
    int sig_id;
    int led_r = 0, led_g = 0, led_b = 0;
};

class controller {
public:
    struct Button {
        bool pressing() const { return false; } // Default: not pressed
        void pressed(std::function<void()> callback) { press_callback = callback; }
        void released(std::function<void()> callback) { release_callback = callback; }
        
    private:
        std::function<void()> press_callback;
        std::function<void()> release_callback;
    };
    
    struct Axis {
        int position() const { return 0; } // Default: centered
    };
    
    struct Screen {
        void setCursor(int row, int col) {}
        void print(const char* format, ...) {}
    };
    
    Button ButtonA, ButtonB, ButtonX, ButtonY;
    Button ButtonL1, ButtonL2, ButtonR1, ButtonR2;
    Button ButtonUp, ButtonDown, ButtonLeft, ButtonRight;
    Axis Axis1, Axis2, Axis3, Axis4;
    Screen Screen;
};

class brain {
public:
    struct Screen {
        void clearScreen() {}
        void printAt(int x, int y, const char* format, ...) {}
        void printAt(int x, int y, bool opaque, const char* format, ...) {}
        void setFillColor(color c) {}
        void setPenColor(color c) {}
        void setFont(fontType f) {}
        void drawRectangle(int x, int y, int w, int h, color fill = transparent) {}
        void drawCircle(int x, int y, int radius) {}
        void drawLine(int x1, int y1, int x2, int y2) {}
        bool drawImageFromFile(const char* filename, int x, int y) { return false; }
        bool pressing() const { return false; }
        int xPosition() const { return 0; }
        int yPosition() const { return 0; }
        int getStringWidth(const char* str) const { return strlen(str) * 8; }
        int getStringHeight(const char* str) const { return 16; }
    };
    
    struct SDCard {
        bool isInserted() const { return false; }
        bool exists(const char* filename) const { return false; }
    };
    
    Screen Screen;
    SDCard SDcard;
};

class competition {
public:
    void autonomous(std::function<void()> callback) { auto_callback = callback; }
    void drivercontrol(std::function<void()> callback) { driver_callback = callback; }
    
    void runAutonomous() { if (auto_callback) auto_callback(); }
    void runDriverControl() { if (driver_callback) driver_callback(); }
    
private:
    std::function<void()> auto_callback;
    std::function<void()> driver_callback;
};

class triport {
public:
    triport(int port) {}
    struct TriPort {
        TriPort operator[](int index) { return TriPort(); }
    };
    TriPort Port;
};

class rotation {
public:
    rotation(int port) : port_num(port) {}
    double position(rotationUnits units = deg) const { return 0.0; }
    
private:
    int port_num;
};

class encoder {
public:
    encoder(triport::TriPort port) {}
    double position(rotationUnits units = deg) const { return 0.0; }
};

class task {
public:
    task(std::function<int()> func, int priority) : task_func(func) {
        std::cout << "[" << SimulationFramework::getInstance().getTime() << "ms] Task created" << std::endl;
    }
    
    task(int (*func)(), int priority) : task(std::function<int()>(func), priority) {}
    
    static void sleep(int time_ms) {
        SimulationFramework::getInstance().step(time_ms);
    }
    
private:
    std::function<int()> task_func;
};

// Global wait function
void wait(double time, timeUnits units);

} // namespace vex

// Port definitions
#define PORT1 1
#define PORT2 2
#define PORT3 3
#define PORT7 7
#define PORT8 8
#define PORT9 9
#define PORT11 11
#define PORT12 12
#define PORT13 13
#define PORT14 14
#define PORT15 15
#define PORT16 16
#define PORT19 19
#define PORT20 20
#define PORT22 22

// Ratio definitions
enum ratio { ratio6_1, ratio36_1 };

// Controller type
enum controllerType { primary };
