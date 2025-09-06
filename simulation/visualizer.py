#!/usr/bin/env python3
"""
VEX Robot Simulation Visualizer
Reads CSV data from simulation and creates plots for analysis
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse
import os

class VEXVisualizer:
    def __init__(self, csv_file):
        """Initialize visualizer with CSV data file"""
        self.csv_file = csv_file
        self.data = None
        self.load_data()
    
    def load_data(self):
        """Load simulation data from CSV file"""
        try:
            self.data = pd.read_csv(self.csv_file)
            print(f"Loaded {len(self.data)} data points from {self.csv_file}")
        except FileNotFoundError:
            print(f"Error: Could not find file {self.csv_file}")
            return False
        except Exception as e:
            print(f"Error loading data: {e}")
            return False
        return True
    
    def plot_robot_path(self, save_path=None):
        """Plot the robot's path in 2D space"""
        if self.data is None:
            print("No data loaded")
            return
        
        fig, ax = plt.subplots(figsize=(12, 8))
        
        # Plot the path
        ax.plot(self.data['x'], self.data['y'], 'b-', linewidth=2, label='Robot Path')
        
        # Mark start and end points
        if len(self.data) > 0:
            ax.plot(self.data['x'].iloc[0], self.data['y'].iloc[0], 'go', 
                   markersize=10, label='Start')
            ax.plot(self.data['x'].iloc[-1], self.data['y'].iloc[-1], 'ro', 
                   markersize=10, label='End')
        
        # Add arrows to show direction
        if len(self.data) > 1:
            for i in range(0, len(self.data)-1, max(1, len(self.data)//20)):
                dx = self.data['x'].iloc[i+1] - self.data['x'].iloc[i]
                dy = self.data['y'].iloc[i+1] - self.data['y'].iloc[i]
                if dx != 0 or dy != 0:
                    ax.arrow(self.data['x'].iloc[i], self.data['y'].iloc[i], 
                            dx*0.5, dy*0.5, head_width=1, head_length=1, 
                            fc='red', ec='red', alpha=0.6)
        
        ax.set_xlabel('X Position (inches)')
        ax.set_ylabel('Y Position (inches)')
        ax.set_title('Robot Path - B_right() Autonomous Function')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.axis('equal')
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Path plot saved to {save_path}")
        else:
            plt.show()
    
    def plot_heading_over_time(self, save_path=None):
        """Plot robot heading over time"""
        if self.data is None:
            print("No data loaded")
            return
        
        fig, ax = plt.subplots(figsize=(12, 6))
        
        ax.plot(self.data['timestamp'], self.data['heading'], 'b-', linewidth=2)
        ax.set_xlabel('Time (ms)')
        ax.set_ylabel('Heading (degrees)')
        ax.set_title('Robot Heading Over Time')
        ax.grid(True, alpha=0.3)
        
        # Add horizontal lines for common angles
        for angle in [0, 90, 180, 270, -90, -180]:
            ax.axhline(y=angle, color='gray', linestyle='--', alpha=0.5)
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Heading plot saved to {save_path}")
        else:
            plt.show()
    
    def plot_position_components(self, save_path=None):
        """Plot X and Y positions over time"""
        if self.data is None:
            print("No data loaded")
            return
        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
        
        ax1.plot(self.data['timestamp'], self.data['x'], 'r-', linewidth=2, label='X Position')
        ax1.set_ylabel('X Position (inches)')
        ax1.set_title('Robot Position Components Over Time')
        ax1.grid(True, alpha=0.3)
        ax1.legend()
        
        ax2.plot(self.data['timestamp'], self.data['y'], 'g-', linewidth=2, label='Y Position')
        ax2.set_xlabel('Time (ms)')
        ax2.set_ylabel('Y Position (inches)')
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Position components plot saved to {save_path}")
        else:
            plt.show()
    
    def plot_distance_traveled(self, save_path=None):
        """Plot cumulative distance traveled"""
        if self.data is None or len(self.data) < 2:
            print("Insufficient data for distance calculation")
            return
        
        # Calculate distance between consecutive points
        distances = []
        cumulative_distance = 0
        
        for i in range(len(self.data)):
            if i == 0:
                distances.append(0)
            else:
                dx = self.data['x'].iloc[i] - self.data['x'].iloc[i-1]
                dy = self.data['y'].iloc[i] - self.data['y'].iloc[i-1]
                distance = np.sqrt(dx*dx + dy*dy)
                cumulative_distance += distance
                distances.append(cumulative_distance)
        
        fig, ax = plt.subplots(figsize=(12, 6))
        
        ax.plot(self.data['timestamp'], distances, 'purple', linewidth=2)
        ax.set_xlabel('Time (ms)')
        ax.set_ylabel('Cumulative Distance (inches)')
        ax.set_title('Distance Traveled Over Time')
        ax.grid(True, alpha=0.3)
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Distance plot saved to {save_path}")
        else:
            plt.show()
    
    def create_dashboard(self, save_path=None):
        """Create a comprehensive dashboard with all plots"""
        if self.data is None:
            print("No data loaded")
            return
        
        fig = plt.figure(figsize=(16, 12))
        
        # Robot path (top left)
        ax1 = plt.subplot(2, 2, 1)
        ax1.plot(self.data['x'], self.data['y'], 'b-', linewidth=2)
        if len(self.data) > 0:
            ax1.plot(self.data['x'].iloc[0], self.data['y'].iloc[0], 'go', markersize=8)
            ax1.plot(self.data['x'].iloc[-1], self.data['y'].iloc[-1], 'ro', markersize=8)
        ax1.set_xlabel('X Position (inches)')
        ax1.set_ylabel('Y Position (inches)')
        ax1.set_title('Robot Path')
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # Heading over time (top right)
        ax2 = plt.subplot(2, 2, 2)
        ax2.plot(self.data['timestamp'], self.data['heading'], 'b-', linewidth=2)
        ax2.set_xlabel('Time (ms)')
        ax2.set_ylabel('Heading (degrees)')
        ax2.set_title('Heading Over Time')
        ax2.grid(True, alpha=0.3)
        
        # X and Y positions (bottom left)
        ax3 = plt.subplot(2, 2, 3)
        ax3.plot(self.data['timestamp'], self.data['x'], 'r-', linewidth=2, label='X')
        ax3.plot(self.data['timestamp'], self.data['y'], 'g-', linewidth=2, label='Y')
        ax3.set_xlabel('Time (ms)')
        ax3.set_ylabel('Position (inches)')
        ax3.set_title('Position Components')
        ax3.grid(True, alpha=0.3)
        ax3.legend()
        
        # Distance traveled (bottom right)
        ax4 = plt.subplot(2, 2, 4)
        distances = [0]
        cumulative = 0
        for i in range(1, len(self.data)):
            dx = self.data['x'].iloc[i] - self.data['x'].iloc[i-1]
            dy = self.data['y'].iloc[i] - self.data['y'].iloc[i-1]
            cumulative += np.sqrt(dx*dx + dy*dy)
            distances.append(cumulative)
        
        ax4.plot(self.data['timestamp'], distances, 'purple', linewidth=2)
        ax4.set_xlabel('Time (ms)')
        ax4.set_ylabel('Distance (inches)')
        ax4.set_title('Cumulative Distance')
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Dashboard saved to {save_path}")
        else:
            plt.show()
    
    def print_summary(self):
        """Print summary statistics"""
        if self.data is None:
            print("No data loaded")
            return
        
        print("\n=== SIMULATION SUMMARY ===")
        print(f"Total time: {self.data['timestamp'].iloc[-1]:.1f} ms")
        print(f"Start position: ({self.data['x'].iloc[0]:.2f}, {self.data['y'].iloc[0]:.2f})")
        print(f"End position: ({self.data['x'].iloc[-1]:.2f}, {self.data['y'].iloc[-1]:.2f})")
        print(f"Final heading: {self.data['heading'].iloc[-1]:.1f} degrees")
        
        # Calculate total distance
        total_distance = 0
        for i in range(1, len(self.data)):
            dx = self.data['x'].iloc[i] - self.data['x'].iloc[i-1]
            dy = self.data['y'].iloc[i] - self.data['y'].iloc[i-1]
            total_distance += np.sqrt(dx*dx + dy*dy)
        
        print(f"Total distance traveled: {total_distance:.2f} inches")
        
        # Calculate displacement
        dx_total = self.data['x'].iloc[-1] - self.data['x'].iloc[0]
        dy_total = self.data['y'].iloc[-1] - self.data['y'].iloc[0]
        displacement = np.sqrt(dx_total*dx_total + dy_total*dy_total)
        print(f"Net displacement: {displacement:.2f} inches")

def main():
    parser = argparse.ArgumentParser(description='VEX Robot Simulation Visualizer')
    parser.add_argument('csv_file', help='Path to CSV file with simulation data')
    parser.add_argument('--output', '-o', help='Output directory for plots')
    parser.add_argument('--dashboard', '-d', action='store_true', 
                       help='Create comprehensive dashboard')
    parser.add_argument('--summary', '-s', action='store_true',
                       help='Print summary statistics')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.csv_file):
        print(f"Error: File {args.csv_file} not found")
        return 1
    
    visualizer = VEXVisualizer(args.csv_file)
    
    if args.summary:
        visualizer.print_summary()
    
    if args.output:
        os.makedirs(args.output, exist_ok=True)
        
        if args.dashboard:
            dashboard_path = os.path.join(args.output, 'dashboard.png')
            visualizer.create_dashboard(dashboard_path)
        else:
            # Create individual plots
            visualizer.plot_robot_path(os.path.join(args.output, 'robot_path.png'))
            visualizer.plot_heading_over_time(os.path.join(args.output, 'heading.png'))
            visualizer.plot_position_components(os.path.join(args.output, 'positions.png'))
            visualizer.plot_distance_traveled(os.path.join(args.output, 'distance.png'))
    else:
        if args.dashboard:
            visualizer.create_dashboard()
        else:
            # Show plots interactively
            visualizer.plot_robot_path()
            visualizer.plot_heading_over_time()
            visualizer.plot_position_components()
            visualizer.plot_distance_traveled()

if __name__ == '__main__':
    main()
