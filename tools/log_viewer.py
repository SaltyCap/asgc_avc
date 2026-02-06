#!/usr/bin/env python3
"""
Interactive Log Viewer for ASGC Motor Control Logs

Features:
- Tabbed interface for multiple logs
- File selection dialog for CSV logs
- Multi-panel interactive plots
- Auto-scaling for all data
- Zoom, pan, and legend controls
- Color-coded navigation states
"""

import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import tkinter as tk
from tkinter import filedialog, ttk, messagebox
import sys
import os

class LogTab:
    """Represents a single log file's plots"""
    def __init__(self, filename, parent_frame):
        self.filename = filename
        self.data = None
        self.fig = None
        self.canvas = None
        self.toolbar = None
        self.parent_frame = parent_frame
        
        # Load and create plots
        if self.load_data():
            self.create_plots()
    
    def load_data(self):
        """Load and parse CSV log file"""
        try:
            self.data = pd.read_csv(self.filename)
            print(f"Loaded {len(self.data)} data points from {os.path.basename(self.filename)}")
            return True
        except Exception as e:
            print(f"Error loading file: {e}")
            return False
    
    def create_plots(self):
        """Create interactive multi-panel plots"""
        if self.data is None:
            return
        
        # Create figure with subplots
        self.fig, axes = plt.subplots(4, 2, figsize=(14, 10))
        self.fig.suptitle(f'{os.path.basename(self.filename)}', fontsize=12, fontweight='bold')
        
        # Color map for navigation states
        state_colors = {
            'IDLE': 'lightgray',
            'TURNING': 'yellow',
            'DRIVING': 'lightgreen',
            'GOTO': 'lightblue',
            'BUCKET_APPROACH': 'lightyellow',
            'BUCKET_ROTATE': 'lightcoral',
            'BUCKET_BACKUP': 'lightpink'
        }
        
        # Add background shading for navigation states
        for ax_row in axes:
            for ax in ax_row:
                self._add_state_background(ax, state_colors)
        
        # Plot 1: PWM Commands
        ax = axes[0, 0]
        ax.plot(self.data['time'], self.data['pwm_l'], 'b-', label='Left PWM', linewidth=1)
        ax.plot(self.data['time'], self.data['pwm_r'], 'r-', label='Right PWM', linewidth=1)
        ax.set_ylabel('PWM (ns)', fontsize=9)
        ax.set_title('Motor PWM Commands', fontsize=10)
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)
        
        # Plot 2: Encoder Raw Angles
        ax = axes[0, 1]
        ax.plot(self.data['time'], self.data['i2c_l'], 'b-', label='Left Raw', linewidth=1)
        ax.plot(self.data['time'], self.data['i2c_r'], 'r-', label='Right Raw', linewidth=1)
        ax.set_ylabel('Raw Angle (0-4095)', fontsize=9)
        ax.set_title('Encoder Raw Angles (I2C)', fontsize=10)
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)
        
        # Plot 3: Encoder Targets vs Actuals (Left)
        ax = axes[1, 0]
        ax.plot(self.data['time'], self.data['target_l'], 'b--', label='Target', linewidth=1.5)
        ax.plot(self.data['time'], self.data['actual_l'], 'b-', label='Actual', linewidth=1)
        ax.set_ylabel('Counts', fontsize=9)
        ax.set_title('Left Motor: Target vs Actual', fontsize=10)
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)
        
        # Plot 4: Encoder Targets vs Actuals (Right)
        ax = axes[1, 1]
        ax.plot(self.data['time'], self.data['target_r'], 'r--', label='Target', linewidth=1.5)
        ax.plot(self.data['time'], self.data['actual_r'], 'r-', label='Actual', linewidth=1)
        ax.set_ylabel('Counts', fontsize=9)
        ax.set_title('Right Motor: Target vs Actual', fontsize=10)
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)
        
        # Plot 5: Gyro Z-axis (MPU6050)
        ax = axes[2, 0]
        ax.plot(self.data['time'], self.data['gyro_z'], 'g-', label='Gyro Z', linewidth=1)
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax.set_ylabel('Angular Rate (deg/s)', fontsize=9)
        ax.set_title('MPU6050 Gyro Z-axis', fontsize=10)
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)
        
        # Plot 6: Odometry Heading
        ax = axes[2, 1]
        ax.plot(self.data['time'], self.data['odom_heading'], 'm-', label='Heading (Fused)', linewidth=1.5)
        ax.set_ylabel('Heading (degrees)', fontsize=9)
        ax.set_title('Robot Heading (Kalman Filtered)', fontsize=10)
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)
        
        # Plot 7: Odometry Position (X, Y)
        ax = axes[3, 0]
        ax.plot(self.data['time'], self.data['odom_x'], 'c-', label='X Position', linewidth=1.5)
        ax.plot(self.data['time'], self.data['odom_y'], 'orange', label='Y Position', linewidth=1.5)
        ax.set_ylabel('Position (feet)', fontsize=9)
        ax.set_xlabel('Time (seconds)', fontsize=9)
        ax.set_title('Robot Position (Odometry)', fontsize=10)
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)
        
        # Plot 8: 2D Path Visualization
        ax = axes[3, 1]
        
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
            ax.plot(x, y, 'o', color=bucket_colors[name], markersize=10, 
                   markeredgecolor='black', markeredgewidth=1.5, 
                   label=name, zorder=3)
        
        # Mark center
        ax.plot(15, 15, 'x', color='purple', markersize=8, 
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
               'go', markersize=8, label='Start', markeredgecolor='black', 
               markeredgewidth=1.5, zorder=4)
        ax.plot(self.data['odom_x'].iloc[-1], self.data['odom_y'].iloc[-1], 
               'rs', markersize=8, label='End', markeredgecolor='black', 
               markeredgewidth=1.5, zorder=4)
        
        ax.set_xlabel('X Position (feet)', fontsize=9)
        ax.set_ylabel('Y Position (feet)', fontsize=9)
        ax.set_title('Robot Path (Top-Down View)', fontsize=10)
        ax.legend(loc='center left', bbox_to_anchor=(1, 0.5), fontsize=7, ncol=1)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)
        
        # Set fixed arena limits
        ax.set_xlim(-2, 32)
        ax.set_ylim(-2, 32)
        ax.set_aspect('equal')
        
        # Adjust layout
        self.fig.tight_layout(rect=[0, 0, 1, 0.96])
        
        # Create canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.parent_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Add toolbar
        toolbar_frame = ttk.Frame(self.parent_frame)
        toolbar_frame.pack(fill=tk.X)
        self.toolbar = NavigationToolbar2Tk(self.canvas, toolbar_frame)
        self.toolbar.update()
    
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
    
    def destroy(self):
        """Clean up resources"""
        if self.fig:
            plt.close(self.fig)


class TabbedLogViewer:
    """Main application with tabbed interface"""
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("ASGC Log Viewer")
        self.root.geometry("1400x900")
        
        # Create menu bar
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)
        
        # File menu
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Open Log(s)...", command=self.add_tab, accelerator="Ctrl+O")
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.root.quit, accelerator="Ctrl+Q")
        
        # Tab menu
        tab_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Tab", menu=tab_menu)
        tab_menu.add_command(label="Close Tab", command=self.close_current_tab, accelerator="Ctrl+W")
        tab_menu.add_command(label="Delete Tab (and file)", command=self.delete_current_tab, accelerator="Ctrl+D")
        tab_menu.add_command(label="Refresh Tab", command=self.refresh_current_tab, accelerator="Ctrl+R")
        
        # Bind keyboard shortcuts
        self.root.bind('<Control-o>', lambda e: self.add_tab())
        self.root.bind('<Control-w>', lambda e: self.close_current_tab())
        self.root.bind('<Control-d>', lambda e: self.delete_current_tab())
        self.root.bind('<Control-r>', lambda e: self.refresh_current_tab())
        self.root.bind('<Control-q>', lambda e: self.root.quit())
        
        # Create notebook (tabbed interface)
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Store tabs
        self.tabs = {}
        
        # Create button frame
        button_frame = ttk.Frame(self.root)
        button_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Add buttons
        ttk.Button(button_frame, text="➕ Open Log", command=self.add_tab).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="✖ Close Tab", command=self.close_current_tab).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="�️ Delete Tab", command=self.delete_current_tab).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="�🔄 Refresh", command=self.refresh_current_tab).pack(side=tk.LEFT, padx=5)
        
        # Status label
        self.status_label = ttk.Label(button_frame, text="No logs open", foreground="gray")
        self.status_label.pack(side=tk.RIGHT, padx=10)
        
        # Default log directory
        tools_dir = os.path.dirname(os.path.abspath(__file__))
        self.log_dir = os.path.abspath(os.path.join(tools_dir, "../logs"))
    
    def select_log_file(self):
        """Open file dialog to select one or more log files"""
        # Get all CSV files sorted by modification time (newest first)
        try:
            csv_files = [f for f in os.listdir(self.log_dir) if f.endswith('.csv')]
            csv_files.sort(key=lambda x: os.path.getmtime(os.path.join(self.log_dir, x)), reverse=True)
        except Exception as e:
            print(f"Error reading log directory: {e}")
            return []
        
        if not csv_files:
            print("No log files found!")
            return []
        
        # Create custom selection dialog
        dialog = tk.Toplevel(self.root)
        dialog.title("Select Log Files")
        dialog.geometry("650x450")
        dialog.transient(self.root)
        dialog.grab_set()
        
        selected_files = []
        check_vars = []
        checkbuttons = []
        
        def refresh_file_list():
            """Refresh the file list after deletion"""
            nonlocal csv_files, check_vars, checkbuttons
            
            # Get updated file list
            try:
                csv_files = [f for f in os.listdir(self.log_dir) if f.endswith('.csv')]
                csv_files.sort(key=lambda x: os.path.getmtime(os.path.join(self.log_dir, x)), reverse=True)
            except Exception as e:
                print(f"Error reading log directory: {e}")
                return
            
            # Clear existing checkboxes
            for cb in checkbuttons:
                cb.destroy()
            check_vars.clear()
            checkbuttons.clear()
            
            # Recreate checkboxes
            for i, f in enumerate(csv_files):
                parts = f.replace('motor_log_', '').replace('.csv', '').split('_')
                if len(parts) >= 3:
                    mode = parts[0]
                    date = parts[1]
                    time = parts[2]
                    
                    month = date[4:6]
                    day = date[6:8]
                    hour = time[0:2]
                    minute = time[2:4]
                    
                    display_name = f"{mode:8s} {month}/{day} {hour}:{minute}"
                else:
                    display_name = f
                
                var = tk.BooleanVar()
                check_vars.append(var)
                
                cb = ttk.Checkbutton(scrollable_frame, text=display_name, variable=var)
                cb.pack(anchor=tk.W, pady=2)
                checkbuttons.append(cb)
        
        def on_select():
            # Get all checked items
            for i, var in enumerate(check_vars):
                if var.get():
                    selected_files.append(os.path.join(self.log_dir, csv_files[i]))
            dialog.destroy()
        
        def on_cancel():
            dialog.destroy()
        
        def select_all():
            for var in check_vars:
                var.set(True)
        
        def deselect_all():
            for var in check_vars:
                var.set(False)
        
        def delete_selected():
            """Delete selected log files with confirmation"""
            # Get selected files
            files_to_delete = []
            for i, var in enumerate(check_vars):
                if var.get():
                    files_to_delete.append(csv_files[i])
            
            if not files_to_delete:
                tk.messagebox.showwarning("No Selection", "Please select files to delete.")
                return
            
            # Confirm deletion
            count = len(files_to_delete)
            message = f"Are you sure you want to permanently delete {count} file(s)?\n\nThis cannot be undone!"
            if not tk.messagebox.askyesno("Confirm Delete", message):
                return
            
            # Delete files
            deleted_count = 0
            for filename in files_to_delete:
                try:
                    filepath = os.path.join(self.log_dir, filename)
                    os.remove(filepath)
                    deleted_count += 1
                    print(f"Deleted: {filename}")
                except Exception as e:
                    print(f"Error deleting {filename}: {e}")
                    tk.messagebox.showerror("Delete Error", f"Failed to delete {filename}:\n{e}")
            
            # Show result
            if deleted_count > 0:
                tk.messagebox.showinfo("Delete Complete", f"Successfully deleted {deleted_count} file(s).")
                refresh_file_list()
        
        # Create main frame
        frame = ttk.Frame(dialog, padding="10")
        frame.pack(fill=tk.BOTH, expand=True)
        
        label = ttk.Label(frame, text="Select log files to open (most recent first):", font=('Arial', 10, 'bold'))
        label.pack(pady=(0, 10))
        
        # Create scrollable frame for checkboxes
        canvas = tk.Canvas(frame, highlightthickness=0)
        scrollbar = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Add checkboxes for each file
        for i, f in enumerate(csv_files):
            parts = f.replace('motor_log_', '').replace('.csv', '').split('_')
            if len(parts) >= 3:
                mode = parts[0]
                date = parts[1]
                time = parts[2]
                
                month = date[4:6]
                day = date[6:8]
                hour = time[0:2]
                minute = time[2:4]
                
                display_name = f"{mode:8s} {month}/{day} {hour}:{minute}"
            else:
                display_name = f
            
            var = tk.BooleanVar()
            check_vars.append(var)
            
            cb = ttk.Checkbutton(scrollable_frame, text=display_name, variable=var)
            cb.pack(anchor=tk.W, pady=2)
            checkbuttons.append(cb)
        
        # Selection buttons
        select_frame = ttk.Frame(frame)
        select_frame.pack(pady=(10, 0))
        
        ttk.Button(select_frame, text="Select All", command=select_all).pack(side=tk.LEFT, padx=5)
        ttk.Button(select_frame, text="Deselect All", command=deselect_all).pack(side=tk.LEFT, padx=5)
        ttk.Button(select_frame, text="🗑️ Delete Selected", command=delete_selected).pack(side=tk.LEFT, padx=5)
        
        # Action buttons
        button_frame = ttk.Frame(frame)
        button_frame.pack(pady=(10, 0))
        
        ttk.Button(button_frame, text="Open Selected", command=on_select).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Cancel", command=on_cancel).pack(side=tk.LEFT, padx=5)
        
        dialog.wait_window()
        
        return selected_files
    
    def add_tab(self, filename=None):
        """Add a new tab with a log file"""
        if filename is None:
            filenames = self.select_log_file()
            if not filenames:
                return
            
            # Open all selected files
            for fname in filenames:
                self._add_single_tab(fname)
        else:
            self._add_single_tab(filename)
    
    def _add_single_tab(self, filename):
        """Add a single tab for a log file"""
        
        # Check if already open
        basename = os.path.basename(filename)
        if basename in self.tabs:
            # Switch to existing tab
            for i in range(self.notebook.index("end")):
                if self.notebook.tab(i, "text") == self._format_tab_name(basename):
                    self.notebook.select(i)
                    return
        
        # Create new tab
        tab_frame = ttk.Frame(self.notebook)
        self.notebook.add(tab_frame, text=self._format_tab_name(basename))
        
        # Create log tab
        log_tab = LogTab(filename, tab_frame)
        self.tabs[basename] = log_tab
        
        # Switch to new tab
        self.notebook.select(tab_frame)
        
        # Update status
        self.update_status()
    
    def _format_tab_name(self, filename):
        """Format filename for tab display"""
        parts = filename.replace('motor_log_', '').replace('.csv', '').split('_')
        if len(parts) >= 3:
            mode = parts[0]
            date = parts[1]
            time = parts[2]
            
            month = date[4:6]
            day = date[6:8]
            hour = time[0:2]
            minute = time[2:4]
            
            return f"{mode} {month}/{day} {hour}:{minute}"
        return filename
    
    def close_current_tab(self):
        """Close the currently selected tab"""
        current = self.notebook.select()
        if not current:
            return
        
        tab_index = self.notebook.index(current)
        tab_text = self.notebook.tab(tab_index, "text")
        
        # Find and destroy the log tab
        for basename, log_tab in list(self.tabs.items()):
            if self._format_tab_name(basename) == tab_text:
                log_tab.destroy()
                del self.tabs[basename]
                break
        
        # Remove tab
        self.notebook.forget(current)
        
        # Update status
        self.update_status()
    
    def delete_current_tab(self):
        """Delete the log file of the currently selected tab"""
        current = self.notebook.select()
        if not current:
            messagebox.showwarning("No Tab", "No tab is currently open.")
            return
        
        tab_index = self.notebook.index(current)
        tab_text = self.notebook.tab(tab_index, "text")
        
        # Find the log tab and filename
        filename = None
        basename = None
        for bn, log_tab in self.tabs.items():
            if self._format_tab_name(bn) == tab_text:
                filename = log_tab.filename
                basename = bn
                break
        
        if not filename:
            messagebox.showerror("Error", "Could not find file for current tab.")
            return
        
        # Confirm deletion
        message = f"Are you sure you want to permanently delete this log file?\n\n{os.path.basename(filename)}\n\nThis cannot be undone!"
        if not messagebox.askyesno("Confirm Delete", message):
            return
        
        # Delete the file
        try:
            os.remove(filename)
            print(f"Deleted: {filename}")
            
            # Close the tab
            log_tab = self.tabs[basename]
            log_tab.destroy()
            del self.tabs[basename]
            self.notebook.forget(current)
            
            # Update status
            self.update_status()
            
            messagebox.showinfo("Delete Complete", f"Successfully deleted:\n{os.path.basename(filename)}")
        except Exception as e:
            print(f"Error deleting file: {e}")
            messagebox.showerror("Delete Error", f"Failed to delete file:\n{e}")
    
    def refresh_current_tab(self):
        """Refresh the currently selected tab"""
        current = self.notebook.select()
        if not current:
            return
        
        tab_index = self.notebook.index(current)
        tab_text = self.notebook.tab(tab_index, "text")
        
        # Find the filename
        for basename, log_tab in self.tabs.items():
            if self._format_tab_name(basename) == tab_text:
                filename = log_tab.filename
                
                # Destroy old tab
                log_tab.destroy()
                
                # Create new frame
                tab_frame = ttk.Frame(self.notebook)
                self.notebook.insert(tab_index, tab_frame, text=tab_text)
                self.notebook.forget(tab_index + 1)
                
                # Reload
                new_log_tab = LogTab(filename, tab_frame)
                self.tabs[basename] = new_log_tab
                
                # Select refreshed tab
                self.notebook.select(tab_frame)
                break
    
    def update_status(self):
        """Update status label"""
        count = len(self.tabs)
        if count == 0:
            self.status_label.config(text="No logs open")
        elif count == 1:
            self.status_label.config(text="1 log open")
        else:
            self.status_label.config(text=f"{count} logs open")
    
    def run(self):
        """Start the application"""
        # Print welcome message
        print("=" * 60)
        print("ASGC Motor Control Log Viewer - Tabbed Interface")
        print("=" * 60)
        print("\nFeatures:")
        print("  • Multiple logs open in tabs")
        print("  • Interactive plots with zoom/pan")
        print("  • Navigation state visualization")
        print("  • MPU6050 gyro and odometry data")
        print("\nControls:")
        print("  • Open Log: Add new tab")
        print("  • Close Tab: Remove current tab")
        print("  • Delete Tab: Delete current log file")
        print("  • Refresh: Reload current log")
        print("=" * 60)
        print()
        
        # Open file selection dialog after window is ready
        self.root.after(100, self.add_tab)
        
        self.root.mainloop()


def main():
    """Entry point"""
    viewer = TabbedLogViewer()
    viewer.run()


if __name__ == "__main__":
    main()
