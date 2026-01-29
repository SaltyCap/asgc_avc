#!/usr/bin/env python3
"""
Interactive Log Viewer for ASGC Motor Control Logs

Features:
- File selection dialog for CSV logs
- Multi-panel interactive plots
- Auto-scaling for all data
- Zoom, pan, and legend controls
- Color-coded navigation states
"""

import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import tkinter as tk
from tkinter import filedialog
import sys
import os

class LogViewer:
    def __init__(self):
        self.fig = None
        self.axes = None
        self.data = None
        self.filename = None
        
    def select_file(self):
        """Open file dialog to select a log file"""
        root = tk.Tk()
        root.withdraw()
        
        # Default to logs directory
        initial_dir = os.path.dirname(os.path.abspath(__file__))
        
        filename = filedialog.askopenfilename(
            title="Select Motor Control Log File",
            initialdir=initial_dir,
            filetypes=[
                ("CSV files", "*.csv"),
                ("All files", "*.*")
            ]
        )
        
        root.destroy()
        return filename
    
    def load_data(self, filename):
        """Load and parse CSV log file"""
        try:
            self.data = pd.read_csv(filename)
            self.filename = os.path.basename(filename)
            print(f"Loaded {len(self.data)} data points from {self.filename}")
            print(f"Columns: {list(self.data.columns)}")
            return True
        except Exception as e:
            print(f"Error loading file: {e}")
            return False
    
    def create_plots(self):
        """Create interactive multi-panel plots"""
        if self.data is None:
            print("No data loaded!")
            return
        
        # Create figure with subplots
        self.fig, self.axes = plt.subplots(4, 2, figsize=(16, 12))
        self.fig.suptitle(f'Motor Control Log: {self.filename}', fontsize=14, fontweight='bold')
        
        # Color map for navigation states
        state_colors = {
            'IDLE': 'lightgray',
            'TURNING': 'yellow',
            'DRIVING': 'lightgreen',
            'GOTO': 'lightblue'
        }
        
        # Add background shading for navigation states
        for ax_row in self.axes:
            for ax in ax_row:
                self._add_state_background(ax, state_colors)
        
        # Plot 1: PWM Commands
        ax = self.axes[0, 0]
        ax.plot(self.data['time'], self.data['pwm_l'], 'b-', label='Left PWM', linewidth=1)
        ax.plot(self.data['time'], self.data['pwm_r'], 'r-', label='Right PWM', linewidth=1)
        ax.set_ylabel('PWM (ns)')
        ax.set_title('Motor PWM Commands')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        ax.autoscale(enable=True, axis='both', tight=True)
        
        # Plot 2: Encoder Raw Angles
        ax = self.axes[0, 1]
        ax.plot(self.data['time'], self.data['i2c_l'], 'b-', label='Left Raw', linewidth=1)
        ax.plot(self.data['time'], self.data['i2c_r'], 'r-', label='Right Raw', linewidth=1)
        ax.set_ylabel('Raw Angle (0-4095)')
        ax.set_title('Encoder Raw Angles (I2C)')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        ax.autoscale(enable=True, axis='both', tight=True)
        
        # Plot 3: Encoder Targets vs Actuals (Left)
        ax = self.axes[1, 0]
        ax.plot(self.data['time'], self.data['target_l'], 'b--', label='Target', linewidth=1.5)
        ax.plot(self.data['time'], self.data['actual_l'], 'b-', label='Actual', linewidth=1)
        ax.set_ylabel('Counts')
        ax.set_title('Left Motor: Target vs Actual')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        ax.autoscale(enable=True, axis='both', tight=True)
        
        # Plot 4: Encoder Targets vs Actuals (Right)
        ax = self.axes[1, 1]
        ax.plot(self.data['time'], self.data['target_r'], 'r--', label='Target', linewidth=1.5)
        ax.plot(self.data['time'], self.data['actual_r'], 'r-', label='Actual', linewidth=1)
        ax.set_ylabel('Counts')
        ax.set_title('Right Motor: Target vs Actual')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        ax.autoscale(enable=True, axis='both', tight=True)
        
        # Plot 5: Gyro Z-axis (MPU6050)
        ax = self.axes[2, 0]
        ax.plot(self.data['time'], self.data['gyro_z'], 'g-', label='Gyro Z', linewidth=1)
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax.set_ylabel('Angular Rate (deg/s)')
        ax.set_title('MPU6050 Gyro Z-axis')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        ax.autoscale(enable=True, axis='both', tight=True)
        
        # Plot 6: Odometry Heading
        ax = self.axes[2, 1]
        ax.plot(self.data['time'], self.data['odom_heading'], 'm-', label='Heading (Fused)', linewidth=1.5)
        ax.set_ylabel('Heading (degrees)')
        ax.set_title('Robot Heading (Kalman Filtered)')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        ax.autoscale(enable=True, axis='both', tight=True)
        
        # Plot 7: Odometry Position (X, Y)
        ax = self.axes[3, 0]
        ax.plot(self.data['time'], self.data['odom_x'], 'c-', label='X Position', linewidth=1.5)
        ax.plot(self.data['time'], self.data['odom_y'], 'orange', label='Y Position', linewidth=1.5)
        ax.set_ylabel('Position (feet)')
        ax.set_xlabel('Time (seconds)')
        ax.set_title('Robot Position (Odometry)')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        ax.autoscale(enable=True, axis='both', tight=True)
        
        # Plot 8: 2D Path Visualization
        ax = self.axes[3, 1]
        # Color code path by navigation state
        for state, color in state_colors.items():
            mask = self.data['nav_state'] == state
            if mask.any():
                ax.plot(self.data.loc[mask, 'odom_x'], 
                       self.data.loc[mask, 'odom_y'], 
                       'o', color=color, markersize=2, label=state, alpha=0.6)
        
        # Add start and end markers
        ax.plot(self.data['odom_x'].iloc[0], self.data['odom_y'].iloc[0], 
               'go', markersize=10, label='Start', markeredgecolor='black', markeredgewidth=1.5)
        ax.plot(self.data['odom_x'].iloc[-1], self.data['odom_y'].iloc[-1], 
               'rs', markersize=10, label='End', markeredgecolor='black', markeredgewidth=1.5)
        
        ax.set_xlabel('X Position (feet)')
        ax.set_ylabel('Y Position (feet)')
        ax.set_title('Robot Path (Top-Down View)')
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        ax.autoscale(enable=True, axis='both', tight=True)
        
        # Adjust layout
        plt.tight_layout(rect=[0, 0.03, 1, 0.97])
        
        # Add interactive controls
        self._add_controls()
        
    def _add_state_background(self, ax, state_colors):
        """Add background shading for navigation states"""
        if self.data is None or 'nav_state' not in self.data.columns:
            return
        
        # Get state transitions
        states = self.data['nav_state'].values
        times = self.data['time'].values
        
        current_state = states[0]
        start_time = times[0]
        
        for i in range(1, len(states)):
            if states[i] != current_state:
                # State changed, draw background
                color = state_colors.get(current_state, 'white')
                ax.axvspan(start_time, times[i-1], alpha=0.15, color=color, zorder=0)
                
                current_state = states[i]
                start_time = times[i]
        
        # Draw final state
        color = state_colors.get(current_state, 'white')
        ax.axvspan(start_time, times[-1], alpha=0.15, color=color, zorder=0)
    
    def _add_controls(self):
        """Add interactive control buttons"""
        # Add button for reloading
        ax_reload = plt.axes([0.81, 0.01, 0.08, 0.025])
        btn_reload = Button(ax_reload, 'Reload')
        btn_reload.on_clicked(lambda event: self.reload())
        
        # Add button for new file
        ax_new = plt.axes([0.90, 0.01, 0.08, 0.025])
        btn_new = Button(ax_new, 'New File')
        btn_new.on_clicked(lambda event: self.load_new_file())
        
        # Store buttons to prevent garbage collection
        self.buttons = [btn_reload, btn_new]
    
    def reload(self):
        """Reload the current file"""
        if self.filename:
            plt.close(self.fig)
            # Reconstruct full path
            log_dir = os.path.dirname(os.path.abspath(__file__))
            full_path = os.path.join(log_dir, self.filename)
            if os.path.exists(full_path):
                self.load_data(full_path)
                self.create_plots()
                plt.show()
    
    def load_new_file(self):
        """Load a new log file"""
        plt.close(self.fig)
        self.run()
    
    def run(self):
        """Main execution flow"""
        # Select file
        filename = self.select_file()
        if not filename:
            print("No file selected. Exiting.")
            return
        
        # Load data
        if not self.load_data(filename):
            return
        
        # Create plots
        self.create_plots()
        
        # Show interactive plot
        plt.show()

def main():
    """Entry point"""
    print("=" * 60)
    print("ASGC Motor Control Log Viewer")
    print("=" * 60)
    print("\nFeatures:")
    print("  • Interactive file selection")
    print("  • Auto-scaling plots")
    print("  • Zoom and pan controls")
    print("  • Navigation state visualization")
    print("  • MPU6050 gyro and odometry data")
    print("\nControls:")
    print("  • Left click + drag: Pan")
    print("  • Right click + drag: Zoom")
    print("  • Home button: Reset view")
    print("  • Reload button: Refresh current file")
    print("  • New File button: Load different log")
    print("=" * 60)
    print()
    
    viewer = LogViewer()
    viewer.run()

if __name__ == "__main__":
    main()
