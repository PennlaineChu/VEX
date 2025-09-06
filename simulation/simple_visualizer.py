#!/usr/bin/env python3
"""
Simple VEX Robot Simulation Visualizer
Uses only Python standard library - no external dependencies required
"""

import csv
import math
import sys

def load_csv_data(filename):
    """Load simulation data from CSV file"""
    data = []
    try:
        with open(filename, 'r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                if row['timestamp']:  # Skip empty rows
                    data.append({
                        'timestamp': float(row['timestamp']),
                        'x': float(row['x']),
                        'y': float(row['y']),
                        'heading': float(row['heading'])
                    })
        return data
    except FileNotFoundError:
        print(f"Error: Could not find file {filename}")
        return None
    except Exception as e:
        print(f"Error loading data: {e}")
        return None

def print_summary(data):
    """Print summary statistics"""
    if not data:
        print("No data to analyze")
        return
    
    print("\n=== SIMULATION SUMMARY ===")
    print(f"Total time: {data[-1]['timestamp']:.1f} ms ({data[-1]['timestamp']/1000:.1f} seconds)")
    print(f"Data points: {len(data)}")
    print(f"Start position: ({data[0]['x']:.2f}, {data[0]['y']:.2f})")
    print(f"End position: ({data[-1]['x']:.2f}, {data[-1]['y']:.2f})")
    print(f"Final heading: {data[-1]['heading']:.1f} degrees (clock system: 0°=North, 90°=East, 180°=South, 270°=West)")
    
    # Calculate total distance traveled
    total_distance = 0
    for i in range(1, len(data)):
        dx = data[i]['x'] - data[i-1]['x']
        dy = data[i]['y'] - data[i-1]['y']
        total_distance += math.sqrt(dx*dx + dy*dy)
    
    print(f"Total distance traveled: {total_distance:.2f} inches")
    
    # Calculate net displacement
    dx_total = data[-1]['x'] - data[0]['x']
    dy_total = data[-1]['y'] - data[0]['y']
    displacement = math.sqrt(dx_total*dx_total + dy_total*dy_total)
    print(f"Net displacement: {displacement:.2f} inches")
    
    # Calculate average speed
    if data[-1]['timestamp'] > 0:
        avg_speed = total_distance / (data[-1]['timestamp'] / 1000.0)
        print(f"Average speed: {avg_speed:.2f} inches/second")

def create_ascii_plot(data):
    """Create a simple ASCII plot of the robot path"""
    if not data:
        return
    
    print("\n=== ROBOT PATH (ASCII Plot) ===")
    
    # Find bounds
    min_x = min(d['x'] for d in data)
    max_x = max(d['x'] for d in data)
    min_y = min(d['y'] for d in data)
    max_y = max(d['y'] for d in data)
    
    # Add padding
    padding = 2
    min_x -= padding
    max_x += padding
    min_y -= padding
    max_y += padding
    
    # Plot dimensions (increased for better resolution)
    width = 80
    height = 30
    
    # Create grid
    grid = [[' ' for _ in range(width)] for _ in range(height)]
    
    # Plot points
    for i, point in enumerate(data):
        # Convert to grid coordinates
        if max_x != min_x:
            grid_x = int((point['x'] - min_x) / (max_x - min_x) * (width - 1))
        else:
            grid_x = width // 2
            
        if max_y != min_y:
            grid_y = int((max_y - point['y']) / (max_y - min_y) * (height - 1))
        else:
            grid_y = height // 2
        
        # Clamp to grid bounds
        grid_x = max(0, min(width - 1, grid_x))
        grid_y = max(0, min(height - 1, grid_y))
        
        # Mark points
        if i == 0:
            grid[grid_y][grid_x] = 'S'  # Start
        elif i == len(data) - 1:
            grid[grid_y][grid_x] = 'E'  # End
        else:
            grid[grid_y][grid_x] = '*'  # Path point
    
    # Print grid with coordinates
    print(f"Y-axis: {max_y:.1f} to {min_y:.1f} inches")
    for row in grid:
        print(''.join(row))
    print(f"X-axis: {min_x:.1f} to {max_x:.1f} inches")
    print("Legend: S=Start, E=End, *=Path points")

def print_waypoints(data):
    """Print detailed waypoint information"""
    if not data:
        return
    
    print("\n=== WAYPOINTS ===")
    print("Time (s)    X (in)    Y (in)    Heading (deg)")
    print("-" * 45)
    
    for point in data:
        time_s = point['timestamp'] / 1000.0
        print(f"{time_s:7.1f}   {point['x']:7.2f}   {point['y']:7.2f}   {point['heading']:10.1f}")

def analyze_movement_phases(data):
    """Analyze different phases of movement"""
    if len(data) < 2:
        return
    
    print("\n=== MOVEMENT ANALYSIS ===")
    
    phases = []
    for i in range(1, len(data)):
        dt = (data[i]['timestamp'] - data[i-1]['timestamp']) / 1000.0  # seconds
        dx = data[i]['x'] - data[i-1]['x']
        dy = data[i]['y'] - data[i-1]['y']
        distance = math.sqrt(dx*dx + dy*dy)
        
        if dt > 0:
            speed = distance / dt
        else:
            speed = 0
        
        # Calculate heading change using clock system (0-359 degrees)
        h1 = data[i-1]['heading']
        h2 = data[i]['heading']
        heading_change = abs(h2 - h1)
        if heading_change > 180:
            heading_change = 360 - heading_change
        
        phases.append({
            'start_time': data[i-1]['timestamp'] / 1000.0,
            'end_time': data[i]['timestamp'] / 1000.0,
            'duration': dt,
            'distance': distance,
            'speed': speed,
            'heading_change': heading_change
        })
    
    print("Phase  Start(s)  End(s)  Duration(s)  Distance(in)  Speed(in/s)  Turn(deg)")
    print("-" * 75)
    
    for i, phase in enumerate(phases):
        print(f"{i+1:5d}  {phase['start_time']:7.1f}  {phase['end_time']:6.1f}  "
              f"{phase['duration']:10.1f}  {phase['distance']:11.2f}  "
              f"{phase['speed']:10.2f}  {phase['heading_change']:8.1f}")

def create_detailed_path_trace(data):
    """Create a detailed trace showing all incremental movements"""
    print("\n=== DETAILED PATH TRACE ===")
    print("Showing every movement step (incremental data points):")
    print()
    
    movement_segments = []
    current_segment = []
    
    # Group consecutive points with same heading (same movement)
    for i, point in enumerate(data):
        if i == 0:
            current_segment = [point]
            continue
            
        # Check if this is a new movement (heading change or significant time gap)
        prev_point = data[i-1]
        time_gap = point['timestamp'] - prev_point['timestamp']
        heading_change = abs(point['heading'] - prev_point['heading'])
        
        if heading_change > 1.0 or time_gap > 500:  # New movement
            if len(current_segment) > 1:
                movement_segments.append(current_segment)
            current_segment = [point]
        else:
            current_segment.append(point)
    
    # Add the last segment
    if len(current_segment) > 1:
        movement_segments.append(current_segment)
    
    # Display each movement segment
    for seg_idx, segment in enumerate(movement_segments, 1):
        if len(segment) < 2:
            continue
            
        start = segment[0]
        end = segment[-1]
        
        print(f"Movement {seg_idx}: {start['timestamp']:.0f}ms - {end['timestamp']:.0f}ms")
        print(f"  Heading: {start['heading']:.0f}° | Steps: {len(segment)} | Duration: {(end['timestamp']-start['timestamp'])/1000:.1f}s")
        
        # Show incremental steps for this movement
        for i, point in enumerate(segment):
            if i == 0:
                print(f"    Start: ({point['x']:.2f}, {point['y']:.2f})")
            elif i == len(segment) - 1:
                print(f"    End:   ({point['x']:.2f}, {point['y']:.2f})")
            else:
                # Show every few intermediate steps to avoid clutter
                if len(segment) <= 5 or i % max(1, len(segment)//3) == 0:
                    print(f"    Step:  ({point['x']:.2f}, {point['y']:.2f}) at {point['timestamp']:.0f}ms")
        print()

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 simple_visualizer.py <csv_file>")
        print("Example: python3 simple_visualizer.py b_right_performance.csv")
        return 1
    
    filename = sys.argv[1]
    print(f"VEX Robot Simulation Analysis - {filename}")
    print("=" * 50)
    
    data = load_csv_data(filename)
    if not data:
        return 1
    
    print_summary(data)
    create_ascii_plot(data)
    create_detailed_path_trace(data)
    print_waypoints(data)
    analyze_movement_phases(data)
    
    print(f"\n=== ANALYSIS COMPLETE ===")
    print(f"For advanced visualization with plots, install: pip install matplotlib pandas numpy")
    print(f"Then run: python3 visualizer.py {filename} --dashboard")
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
