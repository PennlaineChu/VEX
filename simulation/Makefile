# VEX Simulation Makefile
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -g
INCLUDES = -I.
SRCDIR = .
OBJDIR = build
TARGET = vex_simulator

# Source files
SOURCES = mock_vex.cpp \
          mock_robot_config.cpp \
          mock_drive.cpp \
          test_b_right.cpp

# Object files
OBJECTS = $(SOURCES:%.cpp=$(OBJDIR)/%.o)

# Default target
all: $(TARGET)

# Create build directory
$(OBJDIR):
	mkdir -p $(OBJDIR)

# Build target
$(TARGET): $(OBJDIR) $(OBJECTS)
	$(CXX) $(OBJECTS) -o $(TARGET)
	@echo "Build complete: $(TARGET)"

# Compile source files
$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Run tests
test: $(TARGET)
	./$(TARGET)

# Run with visualization
test-viz: $(TARGET)
	./$(TARGET)
	@if [ -f b_right_performance.csv ]; then \
		echo "Generating visualizations..."; \
		if python3 -c "import pandas, matplotlib" 2>/dev/null; then \
			python3 visualizer.py b_right_performance.csv --output plots --dashboard --summary; \
		else \
			echo "Using simple visualizer (install pandas/matplotlib for advanced plots):"; \
			python3 simple_visualizer.py b_right_performance.csv; \
		fi \
	else \
		echo "No performance data found. Run test first."; \
	fi

# Simple analysis without external dependencies
test-simple: $(TARGET)
	./$(TARGET)
	@if [ -f b_right_performance.csv ]; then \
		python3 simple_visualizer.py b_right_performance.csv; \
	else \
		echo "No performance data found. Run test first."; \
	fi

# Clean build files
clean:
	rm -rf $(OBJDIR) $(TARGET) *.csv plots/

# Install Python dependencies for visualization
install-deps:
	pip3 install matplotlib pandas numpy

# Help
help:
	@echo "Available targets:"
	@echo "  all        - Build the simulator"
	@echo "  test       - Build and run tests"
	@echo "  test-viz   - Run tests and generate visualizations"
	@echo "  test-simple - Run tests with simple analysis (no external deps)"
	@echo "  clean      - Clean build files"
	@echo "  install-deps - Install Python dependencies"
	@echo "  help       - Show this help"

.PHONY: all test test-viz clean install-deps help
