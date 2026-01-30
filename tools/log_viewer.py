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
    def __init__(self, initial_file=None):
        self.fig = None
        self.axes = None
        self.data = None
        self.filename = None
        self.initial_file = initial_file
        
    def select_file(self):
        """Open file dialog to select a log file"""
        import tkinter as tk
        from tkinter import ttk
        
        # Default to logs directory (../logs from tools/log_viewer.py)
        tools_dir = os.path.dirname(os.path.abspath(__file__))
        initial_dir = os.path.abspath(os.path.join(tools_dir, "../logs"))
        
        # Get all CSV files sorted by modification time (newest first)
        try:
            csv_files = [f for f in os.listdir(initial_dir) if f.endswith('.csv')]
            csv_files.sort(key=lambda x: os.path.getmtime(os.path.join(initial_dir, x)), reverse=True)
        except Exception as e:
            print(f"Error reading log directory: {e}")
            csv_files = []
        
        if not csv_files:
            print("No log files found!")
            return None
        
        # Create custom selection dialog
        root = tk.Tk()
        root.title("Select Log File")
        root.geometry("600x400")
        
        selected_file = [None]  # Use list to allow modification in nested function
        
        def on_select(event=None):
            selection = listbox.curselection()
            if selection:
                idx = selection[0]
                selected_file[0] = os.path.join(initial_dir, csv_files[idx])
                root.destroy()
        
        def on_cancel():
            root.destroy()
        
        # Create listbox with scrollbar
        frame = ttk.Frame(root, padding="10")
        frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        label = ttk.Label(frame, text="Select a log file (most recent first):", font=('Arial', 10, 'bold'))
        label.grid(row=0, column=0, columnspan=2, sticky=tk.W, pady=(0, 10))
        
        scrollbar = ttk.Scrollbar(frame, orient=tk.VERTICAL)
        listbox = tk.Listbox(frame, yscrollcommand=scrollbar.set, width=80, height=15, font=('Courier', 9))
        scrollbar.config(command=listbox.yview)
        
        listbox.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        scrollbar.grid(row=1, column=1, sticky=(tk.N, tk.S))
        
        # Add files with shortened names
        for f in csv_files:
            # Shorten filename: motor_log_voice_20260130_130252.csv -> voice_01/30_13:02
            parts = f.replace('motor_log_', '').replace('.csv', '').split('_')
            if len(parts) >= 3:
                mode = parts[0]  # voice or joystick
                date = parts[1]  # YYYYMMDD
                time = parts[2]  # HHMMSS
                
                # Format: mode_MM/DD_HH:MM
                month = date[4:6]
                day = date[6:8]
                hour = time[0:2]
                minute = time[2:4]
                
                display_name = f"{mode:8s} {month}/{day} {hour}:{minute}"
            else:
                display_name = f  # Fallback to full name
            
            listbox.insert(tk.END, display_name)
        
        # Select first item by default
        listbox.selection_set(0)
        listbox.focus_set()
        
        # Bind double-click and Enter key
        listbox.bind('<Double-Button-1>', on_select)
        listbox.bind('<Return>', on_select)
        
        # Buttons
        button_frame = ttk.Frame(frame)
        button_frame.grid(row=2, column=0, columnspan=2, pady=(10, 0))
        
        select_btn = ttk.Button(button_frame, text="Select", command=on_select)
        select_btn.pack(side=tk.LEFT, padx=5)
        
        cancel_btn = ttk.Button(button_frame, text="Cancel", command=on_cancel)
        cancel_btn.pack(side=tk.LEFT, padx=5)
        
        # Configure grid weights
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(1, weight=1)
        
        root.mainloop()
        
        return selected_file[0]
    
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
        ax.margins(x=0.05, y=0.1) # Add padding
        ax.autoscale(enable=True, axis='both', tight=False)
        
        # Plot 8: 2D Path Visualization
        ax = self.axes[3, 1]
        
        # Draw arena boundaries (30x30 feet)
        arena_width = 30
        arena_height = 30
        ax.plot([0, arena_width, arena_width, 0, 0], 
                [0, 0, arena_height, arena_height, 0], 
                'k-', linewidth=2, label='Arena', zorder=1)
        
        # Mark bucket locations
        buckets = {
            'Red': (0, 0),
            'Yellow': (0, 30),
            'Blue': (30, 30),
            'Green': (30, 0)
        }
        bucket_colors = {
            'Red': 'red',
            'Yellow': 'gold',
            'Blue': 'blue',
            'Green': 'green'
        }
        
        for name, (x, y) in buckets.items():
            ax.plot(x, y, 'o', color=bucket_colors[name], markersize=12, 
                   markeredgecolor='black', markeredgewidth=1.5, 
                   label=name, zorder=3)
        
        # Mark center
        ax.plot(15, 15, 'x', color='purple', markersize=10, 
               markeredgewidth=2, label='Center', zorder=3)
        
        # Color code path by navigation state
        for state, color in state_colors.items():
            mask = self.data['nav_state'] == state
            if mask.any():
                ax.plot(self.data.loc[mask, 'odom_x'], 
                       self.data.loc[mask, 'odom_y'], 
                       'o', color=color, markersize=2, label=state, alpha=0.6, zorder=2)
        
        # Add start and end markers
        ax.plot(self.data['odom_x'].iloc[0], self.data['odom_y'].iloc[0], 
               'go', markersize=10, label='Start', markeredgecolor='black', 
               markeredgewidth=1.5, zorder=4)
        ax.plot(self.data['odom_x'].iloc[-1], self.data['odom_y'].iloc[-1], 
               'rs', markersize=10, label='End', markeredgecolor='black', 
               markeredgewidth=1.5, zorder=4)
        
        ax.set_xlabel('X Position (feet)')
        ax.set_ylabel('Y Position (feet)')
        ax.set_title('Robot Path (Top-Down View - 30x30 Arena)')
        ax.legend(loc='center left', bbox_to_anchor=(1, 0.5), fontsize=8, ncol=1)
        ax.grid(True, alpha=0.3)
        
        # Set fixed arena limits
        ax.set_xlim(-2, 32)
        ax.set_ylim(-2, 32)
        ax.set_aspect('equal')  # Keep square aspect ratio

        
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
        """Reload the current file by restarting the application"""
        plt.close('all')
        if self.filename:
            # Restart with the current filename as argument
            # We use sys.executable to run the same python interpreter
            # sys.argv[0] is the script name
            # os.path.join needed to handle full path correctly
            log_dir = os.path.dirname(os.path.abspath(__file__))
            full_path = os.path.join(log_dir, self.filename)
            
            print(f"Reloading {full_path}...")
            os.execv(sys.executable, [sys.executable, sys.argv[0], full_path])
        else:
            # Fallback to just restart
             os.execv(sys.executable, [sys.executable, sys.argv[0]])
    
    def load_new_file(self):
        """Load a new log file by restarting the application without arguments"""
        plt.close('all')
        print("Restarting to select new file...")
        os.execv(sys.executable, [sys.executable, sys.argv[0]])
    
    def run(self):
        """Main execution flow"""
        # Determine filename
        if self.initial_file:
            filename = self.initial_file
        else:
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
    
    # Check for command line argument
    initial_file = None
    if len(sys.argv) > 1:
        initial_file = sys.argv[1]
        
    viewer = LogViewer(initial_file)
    viewer.run()

if __name__ == "__main__":
    main()
