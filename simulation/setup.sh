#!/bin/bash

# VEX Simulation Framework Setup Script

echo "Setting up VEX Simulation Framework..."

# Check for required tools
echo "Checking dependencies..."

# Check for C++ compiler
if ! command -v g++ &> /dev/null; then
    echo "Error: g++ compiler not found. Please install a C++17 compatible compiler."
    exit 1
fi

# Check for Python3
if ! command -v python3 &> /dev/null; then
    echo "Error: python3 not found. Please install Python 3.6 or later."
    exit 1
fi

# Check for pip3
if ! command -v pip3 &> /dev/null; then
    echo "Error: pip3 not found. Please install pip for Python 3."
    exit 1
fi

echo "✓ All required tools found"

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install matplotlib pandas numpy

if [ $? -eq 0 ]; then
    echo "✓ Python dependencies installed successfully"
else
    echo "Warning: Some Python dependencies may not have installed correctly"
fi

# Create necessary directories
echo "Creating directories..."
mkdir -p build
mkdir -p plots

echo "✓ Directories created"

# Build the simulator
echo "Building simulator..."
make clean
make

if [ $? -eq 0 ]; then
    echo "✓ Simulator built successfully"
else
    echo "Error: Failed to build simulator"
    exit 1
fi

# Make visualizer executable
chmod +x visualizer.py

echo ""
echo "Setup complete! 🎉"
echo ""
echo "Quick start:"
echo "  make test          - Run interactive tests"
echo "  make test-viz      - Run tests with visualization"
echo "  ./vex_simulator    - Run the main test suite"
echo ""
echo "For more information, see README.md"
