#!/usr/bin/env python3
"""
Interactive Log Viewer for ASGC Motor Control Logs

Features:
- Tabbed interface for multiple logs
- File selection dialog for CSV logs
- Multi-panel interactive plots
- Downsampled rendering for large logs
- Timing diagnostics (dt trend + histogram)
- Zoom, pan, and legend controls
- Safe move-to-trash file management
"""

import argparse
import os
import shutil
import sys
import tkinter as tk
from tkinter import ttk, messagebox

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

class LogTab:
    """Represents a single log file's plots"""
    REQUIRED_COLUMNS = {
        'time',
        'mode',
        'pwm_l',
        'i2c_l',
        'pwm_r',
        'i2c_r',
        'target_l',
        'actual_l',
        'target_r',
        'actual_r',
        'gyro_z',
        'odom_x',
        'odom_y',
        'odom_heading',
        'nav_state',
    }

    NUMERIC_COLUMNS = [
        'time',
        'pwm_l',
        'i2c_l',
        'pwm_r',
        'i2c_r',
        'target_l',
        'actual_l',
        'target_r',
        'actual_r',
        'gyro_z',
        'odom_x',
        'odom_y',
        'odom_heading',
    ]

    def __init__(self, filename, parent_frame):
        self.filename = filename
        self.data = None
        self.plot_data = None
        self.state_segments = []
        self.state_durations = {}
        self.loaded = False
        self.fig = None
        self.canvas = None
        self.toolbar = None
        self.parent_frame = parent_frame
        
        # Load and create plots
        if self.load_data():
            self.loaded = self.create_plots()
    
    def load_data(self):
        """Load and parse CSV log file"""
        try:
            data = pd.read_csv(self.filename)
        except Exception as e:
            messagebox.showerror("Load Error", f"Failed to load {os.path.basename(self.filename)}:\n{e}")
            return False
        if data.empty:
            messagebox.showwarning("Empty Log", f"{os.path.basename(self.filename)} contains no rows.")
            return False

        missing = sorted(self.REQUIRED_COLUMNS - set(data.columns))
        if missing:
            messagebox.showerror(
                "Schema Error",
                f"{os.path.basename(self.filename)} is missing required columns:\n"
                + ", ".join(missing),
            )
            return False

        for col in self.NUMERIC_COLUMNS:
            data[col] = pd.to_numeric(data[col], errors='coerce')

        before_drop = len(data)
        data = data.dropna(subset=self.NUMERIC_COLUMNS).reset_index(drop=True)
        dropped = before_drop - len(data)
        if dropped > 0:
            print(f"Dropped {dropped} invalid numeric rows from {os.path.basename(self.filename)}")
        if data.empty:
            messagebox.showerror("Schema Error", f"{os.path.basename(self.filename)} has no valid numeric rows.")
            return False

        data['mode'] = data['mode'].astype(str)
        data['nav_state'] = data['nav_state'].astype(str)
        data = data.sort_values('time').reset_index(drop=True)

        self.data = data
        self.plot_data = self._downsample_data(self.data)
        self.state_segments = self._compute_state_segments(self.data)
        self.state_durations = self._compute_state_durations(self.state_segments)

        print(
            f"Loaded {len(self.data)} points from {os.path.basename(self.filename)} "
            f"(plotting {len(self.plot_data)} points)"
        )
        return True

    def _downsample_data(self, data, max_points=20000):
        if data is None or len(data) <= max_points:
            return data
        step = max(1, len(data) // max_points)
        sampled = data.iloc[::step].copy()
        if sampled.index[-1] != data.index[-1]:
            sampled = pd.concat([sampled, data.iloc[[-1]]])
        return sampled.reset_index(drop=True)

    def _compute_state_segments(self, data):
        if data is None or data.empty or 'nav_state' not in data.columns:
            return []
        states = data['nav_state'].to_numpy()
        times = data['time'].to_numpy()
        segments = []
        start_idx = 0
        current_state = states[0]
        for i in range(1, len(states)):
            if states[i] != current_state:
                segments.append((times[start_idx], times[i - 1], current_state))
                start_idx = i
                current_state = states[i]
        segments.append((times[start_idx], times[-1], current_state))
        return segments

    def _compute_state_durations(self, segments):
        durations = {}
        for start_t, end_t, state in segments:
            duration = max(0.0, end_t - start_t)
            durations[state] = durations.get(state, 0.0) + duration
        return durations

    def _compute_heading_rate(self, data):
        if data is None or len(data) < 3:
            return None
        times = data['time'].to_numpy()
        headings_deg = data['odom_heading'].to_numpy()
        if np.any(np.diff(times) <= 0):
            return None
        headings_rad = np.deg2rad(headings_deg)
        unwrapped_deg = np.rad2deg(np.unwrap(headings_rad))
        return np.gradient(unwrapped_deg, times)

    def _compute_sample_intervals(self, data):
        if data is None or len(data) < 2:
            return np.array([]), np.array([])
        times = data['time'].to_numpy()
        dt = np.diff(times)
        return times[1:], dt

    def create_plots(self):
        """Create interactive multi-panel plots"""
        if self.data is None or self.plot_data is None:
            return False

        data = self.plot_data
        self.fig, axes = plt.subplots(5, 2, figsize=(15, 12))
        self.fig.suptitle(os.path.basename(self.filename), fontsize=12, fontweight='bold')

        state_colors = {
            'IDLE': 'lightgray',
            'TURNING': 'yellow',
            'DRIVING': 'lightgreen',
            'GOTO': 'lightblue',
            'BUCKET_APPROACH': 'khaki',
            'BUCKET_ROTATE': 'salmon',
            'BUCKET_BACKUP': 'lightpink',
            'UNKNOWN': 'whitesmoke',
        }

        time_axes = [
            axes[0, 0], axes[0, 1],
            axes[1, 0], axes[1, 1],
            axes[2, 0], axes[2, 1],
            axes[3, 0], axes[4, 0],
        ]
        for ax in time_axes:
            self._add_state_background(ax, state_colors, self.state_segments)

        # Plot 1: PWM Commands
        ax = axes[0, 0]
        ax.plot(data['time'], data['pwm_l'], 'b-', label='Left PWM', linewidth=1)
        ax.plot(data['time'], data['pwm_r'], 'r-', label='Right PWM', linewidth=1)
        ax.set_ylabel('PWM (ns)', fontsize=9)
        ax.set_title('Motor PWM Commands', fontsize=10)
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)

        # Plot 2: Encoder Raw Angles
        ax = axes[0, 1]
        ax.plot(data['time'], data['i2c_l'], 'b-', label='Left Raw', linewidth=1)
        ax.plot(data['time'], data['i2c_r'], 'r-', label='Right Raw', linewidth=1)
        ax.set_ylabel('Raw Angle (0-4095)', fontsize=9)
        ax.set_title('Encoder Raw Angles', fontsize=10)
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)

        # Plot 3: Left target/actual/error
        ax = axes[1, 0]
        left_error = data['target_l'] - data['actual_l']
        ax.plot(data['time'], data['target_l'], 'b--', label='Target', linewidth=1.3)
        ax.plot(data['time'], data['actual_l'], 'b-', label='Actual', linewidth=1)
        ax.plot(data['time'], left_error, color='navy', linestyle=':', linewidth=1, alpha=0.45, label='Error')
        ax.set_ylabel('Counts', fontsize=9)
        ax.set_title('Left Encoder: Target vs Actual', fontsize=10)
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)

        # Plot 4: Right target/actual/error
        ax = axes[1, 1]
        right_error = data['target_r'] - data['actual_r']
        ax.plot(data['time'], data['target_r'], 'r--', label='Target', linewidth=1.3)
        ax.plot(data['time'], data['actual_r'], 'r-', label='Actual', linewidth=1)
        ax.plot(data['time'], right_error, color='darkred', linestyle=':', linewidth=1, alpha=0.45, label='Error')
        ax.set_ylabel('Counts', fontsize=9)
        ax.set_title('Right Encoder: Target vs Actual', fontsize=10)
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)

        # Plot 5: Gyro Z with derived heading rate overlay
        ax = axes[2, 0]
        heading_rate = self._compute_heading_rate(data)
        ax.plot(data['time'], data['gyro_z'], 'g-', label='Gyro Z', linewidth=1)
        if heading_rate is not None:
            ax.plot(data['time'], heading_rate, color='purple', linestyle='--', linewidth=1, alpha=0.8, label='dHeading/dt')
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax.set_ylabel('Rate (deg/s)', fontsize=9)
        ax.set_title('Gyro vs Derived Heading Rate', fontsize=10)
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)

        # Plot 6: Heading (wrapped + unwrapped)
        ax = axes[2, 1]
        wrapped = data['odom_heading'].to_numpy()
        unwrapped = np.rad2deg(np.unwrap(np.deg2rad(wrapped)))
        ax.plot(data['time'], wrapped, 'm-', label='Heading (0-360)', linewidth=1.2)
        ax.plot(data['time'], unwrapped, color='gray', linestyle='--', linewidth=1, alpha=0.8, label='Heading Unwrapped')
        ax.set_ylabel('Heading (deg)', fontsize=9)
        ax.set_title('Robot Heading', fontsize=10)
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)

        # Plot 7: Position over time
        ax = axes[3, 0]
        ax.plot(data['time'], data['odom_x'], 'c-', label='X Position', linewidth=1.2)
        ax.plot(data['time'], data['odom_y'], color='orange', label='Y Position', linewidth=1.2)
        ax.set_ylabel('Position (ft)', fontsize=9)
        ax.set_xlabel('Time (s)', fontsize=9)
        ax.set_title('Odometry Position', fontsize=10)
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)

        # Plot 8: 2D path
        self._plot_path_by_state(axes[3, 1], data, state_colors)

        # Plot 9 + 10: timing diagnostics
        self._plot_timing_diagnostics(axes[4, 0], axes[4, 1])

        self.fig.tight_layout(rect=[0, 0, 1, 0.96])

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.parent_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        toolbar_frame = ttk.Frame(self.parent_frame)
        toolbar_frame.pack(fill=tk.X)
        self.toolbar = NavigationToolbar2Tk(self.canvas, toolbar_frame)
        self.toolbar.update()
        return True

    def _plot_path_by_state(self, ax, data, state_colors):
        arena_width = 30
        arena_height = 30
        ax.plot(
            [0, arena_width, arena_width, 0, 0],
            [0, 0, arena_height, arena_height, 0],
            'k-',
            linewidth=1.8,
            label='Arena',
            zorder=1,
        )

        buckets = {
            'Red': (0, 0),
            'Yellow': (0, 30),
            'Blue': (30, 30),
            'Green': (30, 0),
        }
        bucket_colors = {
            'Red': 'red',
            'Yellow': 'gold',
            'Blue': 'blue',
            'Green': 'green',
        }
        for name, (x, y) in buckets.items():
            ax.plot(
                x,
                y,
                'o',
                color=bucket_colors[name],
                markersize=8,
                markeredgecolor='black',
                markeredgewidth=1.2,
                label=name,
                zorder=3,
            )

        ax.plot(15, 15, 'x', color='purple', markersize=8, markeredgewidth=2, label='Center', zorder=3)

        states = data['nav_state'].to_numpy()
        xs = data['odom_x'].to_numpy()
        ys = data['odom_y'].to_numpy()

        label_used = set()
        start_idx = 0
        for i in range(1, len(states) + 1):
            state_changed = (i == len(states)) or (states[i] != states[start_idx])
            if not state_changed:
                continue
            state = states[start_idx]
            color = state_colors.get(state, 'gray')
            label = None
            if state not in label_used:
                duration = self.state_durations.get(state, 0.0)
                label = f"{state} ({duration:.1f}s)"
                label_used.add(state)
            ax.plot(xs[start_idx:i], ys[start_idx:i], color=color, linewidth=1.3, alpha=0.9, label=label, zorder=2)
            start_idx = i

        ax.plot(xs[0], ys[0], 'go', markersize=8, label='Start', markeredgecolor='black', markeredgewidth=1.2, zorder=4)
        ax.plot(xs[-1], ys[-1], 'rs', markersize=8, label='End', markeredgecolor='black', markeredgewidth=1.2, zorder=4)

        ax.set_xlabel('X Position (ft)', fontsize=9)
        ax.set_ylabel('Y Position (ft)', fontsize=9)
        ax.set_title('Robot Path (State-colored)', fontsize=10)
        ax.legend(loc='center left', bbox_to_anchor=(1, 0.5), fontsize=7, ncol=1)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=8)
        ax.set_xlim(-2, 32)
        ax.set_ylim(-2, 32)
        ax.set_aspect('equal')

    def _plot_timing_diagnostics(self, ax_dt, ax_hist):
        times, dt = self._compute_sample_intervals(self.data)
        if len(dt) == 0:
            ax_dt.set_title('Sample Interval Over Time', fontsize=10)
            ax_dt.text(0.5, 0.5, 'Insufficient samples', ha='center', va='center', transform=ax_dt.transAxes)
            ax_hist.set_title('Sample Interval Histogram', fontsize=10)
            ax_hist.text(0.5, 0.5, 'Insufficient samples', ha='center', va='center', transform=ax_hist.transAxes)
            return

        dt_ms = dt * 1000.0
        stride = max(1, len(dt_ms) // 25000)
        dt_ms_ds = dt_ms[::stride]
        time_ds = times[::stride]

        median_dt = float(np.median(dt))
        mean_dt = float(np.mean(dt))
        std_dt_ms = float(np.std(dt_ms))
        effective_hz = (1.0 / median_dt) if median_dt > 0 else 0.0
        non_positive = int(np.sum(dt <= 0))
        gap_count = int(np.sum(dt > (3.0 * median_dt))) if median_dt > 0 else 0

        ax_dt.plot(time_ds, dt_ms_ds, color='teal', linewidth=1, label='dt')
        ax_dt.axhline(median_dt * 1000.0, color='black', linestyle='--', linewidth=1, alpha=0.6, label='Median dt')
        ax_dt.set_ylabel('dt (ms)', fontsize=9)
        ax_dt.set_xlabel('Time (s)', fontsize=9)
        ax_dt.set_title('Sample Interval Over Time', fontsize=10)
        ax_dt.legend(loc='upper right', fontsize=8)
        ax_dt.grid(True, alpha=0.3)
        ax_dt.tick_params(labelsize=8)

        bins = min(120, max(20, int(np.sqrt(len(dt_ms)))))
        ax_hist.hist(dt_ms, bins=bins, color='slateblue', alpha=0.8)
        ax_hist.set_xlabel('dt (ms)', fontsize=9)
        ax_hist.set_ylabel('Count', fontsize=9)
        ax_hist.set_title('Sample Interval Histogram', fontsize=10)
        ax_hist.grid(True, alpha=0.2)
        ax_hist.tick_params(labelsize=8)

        diagnostics = (
            f"Rows: {len(self.data):,}\n"
            f"Median dt: {median_dt * 1000.0:.3f} ms\n"
            f"Mean dt: {mean_dt * 1000.0:.3f} ms\n"
            f"Effective rate: {effective_hz:.2f} Hz\n"
            f"Jitter (std): {std_dt_ms:.3f} ms\n"
            f"Non-monotonic dt<=0: {non_positive}\n"
            f"Gaps (>3x median): {gap_count}"
        )
        ax_hist.text(
            0.98,
            0.98,
            diagnostics,
            transform=ax_hist.transAxes,
            fontsize=8,
            ha='right',
            va='top',
            bbox={'facecolor': 'white', 'alpha': 0.85, 'edgecolor': 'gray'},
        )

    def _add_state_background(self, ax, state_colors, state_segments):
        """Add background shading for navigation states"""
        if not state_segments:
            return
        for start_time, end_time, state in state_segments:
            color = state_colors.get(state, 'white')
            ax.axvspan(start_time, end_time, alpha=0.12, color=color, zorder=0)
    
    def destroy(self):
        """Clean up resources"""
        if self.fig:
            plt.close(self.fig)


class TabbedLogViewer:
    """Main application with tabbed interface"""
    def __init__(self, log_dir=None):
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
        tab_menu.add_command(label="Move Tab File to Trash", command=self.delete_current_tab, accelerator="Ctrl+D")
        tab_menu.add_command(label="Refresh Tab", command=self.refresh_current_tab, accelerator="Ctrl+R")

        # Tools menu
        tools_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Tools", menu=tools_menu)
        tools_menu.add_command(label="Clean Empty Logs...", command=self.clean_logs, accelerator="Ctrl+Shift+C")

        # Bind keyboard shortcuts
        self.root.bind('<Control-o>', lambda e: self.add_tab())
        self.root.bind('<Control-w>', lambda e: self.close_current_tab())
        self.root.bind('<Control-d>', lambda e: self.delete_current_tab())
        self.root.bind('<Control-r>', lambda e: self.refresh_current_tab())
        self.root.bind('<Control-q>', lambda e: self.root.quit())
        self.root.bind('<Control-C>', lambda e: self.clean_logs())  # Ctrl+Shift+C

        # Create notebook (tabbed interface)
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Tab registry keyed by notebook tab id.
        self.tabs = {}
        self.path_to_tab = {}

        # Create button frame
        button_frame = ttk.Frame(self.root)
        button_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Button(button_frame, text="Open Log", command=self.add_tab).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Close Tab", command=self.close_current_tab).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Move to Trash", command=self.delete_current_tab).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Refresh", command=self.refresh_current_tab).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="🧹 Clean Logs", command=self.clean_logs).pack(side=tk.LEFT, padx=5)

        self.status_label = ttk.Label(button_frame, text="No logs open", foreground="gray")
        self.status_label.pack(side=tk.RIGHT, padx=10)

        # Default log directory (persistent)
        tools_dir = os.path.dirname(os.path.abspath(__file__))
        default_log_dir = os.path.abspath(os.path.join(tools_dir, "../logs"))
        self.log_dir = self._normalize_path(log_dir or default_log_dir)
        self.trash_dir = os.path.join(self.log_dir, ".trash")
        # RAM disk directory where C code saves live copies
        self.shm_dir = "/dev/shm"

    def _normalize_path(self, path):
        return os.path.abspath(os.path.expanduser(path))

    def _list_csv_files(self):
        """Return (basename, full_path) pairs from shm (live) + persistent log dir.
        RAM-disk files appear first; persistent files fill in the rest."""
        entries = {}  # basename -> full_path (first-seen wins, so shm takes priority)

        # 1. Live RAM-disk files (motor_log_*.csv in /dev/shm)
        if os.path.isdir(self.shm_dir):
            shm_files = [
                f for f in os.listdir(self.shm_dir)
                if f.lower().startswith("motor_log_") and f.lower().endswith(".csv")
            ]
            shm_files.sort(
                key=lambda x: os.path.getmtime(os.path.join(self.shm_dir, x)),
                reverse=True,
            )
            for f in shm_files:
                entries[f] = os.path.join(self.shm_dir, f)

        # 2. Persistent log directory
        if os.path.isdir(self.log_dir):
            log_files = [
                f for f in os.listdir(self.log_dir) if f.lower().endswith(".csv")
            ]
            log_files.sort(
                key=lambda x: os.path.getmtime(os.path.join(self.log_dir, x)),
                reverse=True,
            )
            for f in log_files:
                if f not in entries:  # don't shadow shm copy
                    entries[f] = os.path.join(self.log_dir, f)

        # Return basenames in modification-time order (shm first, then persistent)
        return list(entries.keys())

    def _full_path_for(self, basename):
        """Resolve a basename to its full path (shm takes priority over log_dir)."""
        shm_path = os.path.join(self.shm_dir, basename)
        if os.path.exists(shm_path) and basename.startswith("motor_log_"):
            return shm_path
        return os.path.join(self.log_dir, basename)

    def get_latest_files(self, count=1, mode="any"):
        count = max(0, int(count))
        if count == 0:
            return []

        files = self._list_csv_files()
        if mode and mode != "any":
            prefix = f"motor_log_{mode.lower()}_"
            files = [name for name in files if name.startswith(prefix)]

        return [self._full_path_for(name) for name in files[:count]]

    def _move_to_trash(self, filepath):
        path = self._normalize_path(filepath)
        if not os.path.exists(path):
            raise FileNotFoundError(f"File does not exist: {path}")
        os.makedirs(self.trash_dir, exist_ok=True)

        basename = os.path.basename(path)
        stem, ext = os.path.splitext(basename)
        destination = os.path.join(self.trash_dir, basename)
        counter = 1
        while os.path.exists(destination):
            destination = os.path.join(self.trash_dir, f"{stem}_{counter}{ext}")
            counter += 1

        shutil.move(path, destination)
        return destination

    def _close_tab(self, tab_id):
        log_tab = self.tabs.pop(tab_id, None)
        if not log_tab:
            return
        normalized = self._normalize_path(log_tab.filename)
        self.path_to_tab.pop(normalized, None)
        log_tab.destroy()
        self.notebook.forget(tab_id)

    def select_log_file(self):
        """Open file dialog to select one or more log files"""
        try:
            csv_files = self._list_csv_files()
        except Exception as e:
            messagebox.showerror("Log Directory Error", f"Unable to read log directory:\n{e}")
            return []

        if not csv_files:
            messagebox.showinfo(
                "No Logs Found",
                f"No CSV logs found in:\n  {self.shm_dir}  (live RAM)\n  {self.log_dir}  (persistent)",
            )
            return []

        dialog = tk.Toplevel(self.root)
        dialog.title("Select Log Files")
        dialog.geometry("650x450")
        dialog.transient(self.root)
        dialog.grab_set()

        selected_files = []
        check_vars = []
        checkbuttons = []

        def refresh_file_list():
            nonlocal csv_files, check_vars, checkbuttons
            csv_files = self._list_csv_files()
            for cb in checkbuttons:
                cb.destroy()
            check_vars.clear()
            checkbuttons.clear()

            for f in csv_files:
                var = tk.BooleanVar()
                check_vars.append(var)
                cb = ttk.Checkbutton(scrollable_frame, text=self._format_tab_name(f), variable=var)
                cb.pack(anchor=tk.W, pady=2)
                checkbuttons.append(cb)

        def on_select():
            for i, var in enumerate(check_vars):
                if var.get():
                    selected_files.append(self._full_path_for(csv_files[i]))
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
            files_to_delete = []
            for i, var in enumerate(check_vars):
                if var.get():
                    files_to_delete.append(csv_files[i])

            if not files_to_delete:
                messagebox.showwarning("No Selection", "Please select files to move to trash.")
                return

            count = len(files_to_delete)
            message = (
                f"Move {count} file(s) to the log trash folder?\n\n"
                f"Destination: {self.trash_dir}"
            )
            if not messagebox.askyesno("Confirm Move to Trash", message):
                return

            moved_count = 0
            for filename in files_to_delete:
                try:
                    filepath = self._full_path_for(filename)
                    normalized = self._normalize_path(filepath)
                    destination = self._move_to_trash(filepath)
                    moved_count += 1
                    print(f"Moved to trash: {filename} -> {destination}")
                    open_tab = self.path_to_tab.get(normalized)
                    if open_tab:
                        self._close_tab(open_tab)
                except Exception as e:
                    messagebox.showerror("Trash Move Error", f"Failed to move {filename}:\n{e}")

            if moved_count > 0:
                self.update_status()
                messagebox.showinfo("Move Complete", f"Moved {moved_count} file(s) to trash.")
                refresh_file_list()

        frame = ttk.Frame(dialog, padding="10")
        frame.pack(fill=tk.BOTH, expand=True)

        label = ttk.Label(frame, text="Select log files to open (most recent first):", font=('Arial', 10, 'bold'))
        label.pack(pady=(0, 10))

        canvas = tk.Canvas(frame, highlightthickness=0)
        scrollbar = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        select_frame = ttk.Frame(frame)
        select_frame.pack(pady=(10, 0))

        ttk.Button(select_frame, text="Select All", command=select_all).pack(side=tk.LEFT, padx=5)
        ttk.Button(select_frame, text="Deselect All", command=deselect_all).pack(side=tk.LEFT, padx=5)
        ttk.Button(select_frame, text="Move Selected to Trash", command=delete_selected).pack(side=tk.LEFT, padx=5)
        ttk.Button(select_frame, text="🧹 Clean Logs", command=lambda: [self.clean_logs(), refresh_file_list()]).pack(side=tk.LEFT, padx=5)

        button_frame = ttk.Frame(frame)
        button_frame.pack(pady=(10, 0))

        ttk.Button(button_frame, text="Open Selected", command=on_select).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Cancel", command=on_cancel).pack(side=tk.LEFT, padx=5)

        refresh_file_list()
        dialog.wait_window()

        return selected_files

    def add_tab(self, filename=None):
        """Add a new tab with a log file"""
        if filename is None:
            filenames = self.select_log_file()
            if not filenames:
                return
            for fname in filenames:
                self._add_single_tab(fname)
        else:
            self._add_single_tab(filename)

    def _add_single_tab(self, filename):
        """Add a single tab for a log file"""
        normalized = self._normalize_path(filename)
        if not os.path.exists(normalized):
            messagebox.showerror("File Not Found", f"Cannot open missing file:\n{normalized}")
            return

        existing_tab = self.path_to_tab.get(normalized)
        if existing_tab and existing_tab in self.tabs:
            self.notebook.select(existing_tab)
            return
        if existing_tab and existing_tab not in self.tabs:
            self.path_to_tab.pop(normalized, None)

        tab_frame = ttk.Frame(self.notebook)
        self.notebook.add(tab_frame, text=self._format_tab_name(os.path.basename(normalized)))
        tab_id = str(tab_frame)

        log_tab = LogTab(normalized, tab_frame)
        if not log_tab.loaded:
            log_tab.destroy()
            self.notebook.forget(tab_frame)
            return

        self.tabs[tab_id] = log_tab
        self.path_to_tab[normalized] = tab_id
        self.notebook.select(tab_frame)
        self.update_status()

    def _format_tab_name(self, filename):
        """Format filename for tab display"""
        parts = filename.replace('motor_log_', '').replace('.csv', '').split('_')
        if len(parts) >= 3 and len(parts[1]) == 8 and len(parts[2]) >= 6:
            mode = parts[0]
            date = parts[1]
            time = parts[2]

            month = date[4:6]
            day = date[6:8]
            hour = time[0:2]
            minute = time[2:4]
            second = time[4:6]
            suffix = ""
            if len(parts) > 3:
                suffix = f" #{parts[3]}"
            return f"{mode} {month}/{day} {hour}:{minute}:{second}{suffix}"
        return filename

    def close_current_tab(self):
        """Close the currently selected tab"""
        current = self.notebook.select()
        if not current:
            return
        self._close_tab(current)
        self.update_status()

    def delete_current_tab(self):
        """Move the current tab's file to trash and close the tab."""
        current = self.notebook.select()
        if not current:
            messagebox.showwarning("No Tab", "No tab is currently open.")
            return

        log_tab = self.tabs.get(current)
        if not log_tab:
            messagebox.showerror("Tab Error", "Could not resolve selected tab.")
            return

        filename = log_tab.filename
        prompt = (
            "Move this log file to trash?\n\n"
            f"{os.path.basename(filename)}\n\n"
            f"Destination: {self.trash_dir}"
        )
        if not messagebox.askyesno("Confirm Move to Trash", prompt):
            return

        try:
            destination = self._move_to_trash(filename)
            print(f"Moved to trash: {filename} -> {destination}")
            self._close_tab(current)
            self.update_status()
            messagebox.showinfo("Moved to Trash", f"Moved:\n{os.path.basename(filename)}")
        except Exception as e:
            messagebox.showerror("Trash Move Error", f"Failed to move file:\n{e}")

    def refresh_current_tab(self):
        """Refresh the currently selected tab"""
        current = self.notebook.select()
        if not current:
            return

        current_log_tab = self.tabs.get(current)
        if not current_log_tab:
            return

        filename = current_log_tab.filename
        current_log_tab.destroy()

        frame = self.root.nametowidget(current)
        for child in frame.winfo_children():
            child.destroy()

        refreshed = LogTab(filename, frame)
        if not refreshed.loaded:
            self._close_tab(current)
            self.update_status()
            return

        self.tabs[current] = refreshed
        self.notebook.select(current)

    def clean_logs(self):
        """Scan log dirs for CSV files with <=5 data rows and move them to trash."""
        MIN_ROWS = 5
        candidates = []  # list of (filepath, row_count)

        dirs_to_scan = []
        if os.path.isdir(self.log_dir):
            dirs_to_scan.append((self.log_dir, lambda f: f.lower().endswith(".csv")))
        if os.path.isdir(self.shm_dir):
            dirs_to_scan.append((
                self.shm_dir,
                lambda f: f.lower().startswith("motor_log_") and f.lower().endswith(".csv"),
            ))

        for scan_dir, predicate in dirs_to_scan:
            try:
                entries = os.listdir(scan_dir)
            except OSError:
                continue
            for fname in entries:
                if not predicate(fname):
                    continue
                fpath = os.path.join(scan_dir, fname)
                if not os.path.isfile(fpath):
                    continue
                try:
                    with open(fpath, "r", errors="replace") as fh:
                        # Count non-empty lines; first is header
                        non_empty = sum(1 for line in fh if line.strip())
                    data_rows = max(0, non_empty - 1)  # subtract header
                    if data_rows <= MIN_ROWS:
                        candidates.append((fpath, data_rows))
                except OSError:
                    pass

        if not candidates:
            messagebox.showinfo(
                "Clean Logs",
                f"No logs with ≤{MIN_ROWS} data rows found — nothing to clean!",
            )
            return

        summary_lines = [f"{os.path.basename(p)}  ({r} rows)" for p, r in candidates]
        summary = "\n".join(summary_lines)
        if not messagebox.askyesno(
            "Clean Logs",
            f"Move {len(candidates)} empty/near-empty log(s) to trash?\n\n"
            f"{summary}\n\nDestination: {self.trash_dir}",
        ):
            return

        moved, errors = 0, []
        for fpath, _ in candidates:
            try:
                normalized = self._normalize_path(fpath)
                dest = self._move_to_trash(fpath)
                print(f"Clean: moved {fpath} -> {dest}")
                moved += 1
                # Close any open tab for this file
                open_tab = self.path_to_tab.get(normalized)
                if open_tab:
                    self._close_tab(open_tab)
            except Exception as exc:
                errors.append(f"{os.path.basename(fpath)}: {exc}")

        self.update_status()
        msg = f"Moved {moved} file(s) to trash."
        if errors:
            msg += "\n\nErrors:\n" + "\n".join(errors)
            messagebox.showwarning("Clean Logs — Done", msg)
        else:
            messagebox.showinfo("Clean Logs — Done", msg)

    def update_status(self):
        """Update status label"""
        count = len(self.tabs)
        if count == 0:
            self.status_label.config(text="No logs open")
        elif count == 1:
            self.status_label.config(text="1 log open")
        else:
            self.status_label.config(text=f"{count} logs open")
    
    def run(self, initial_files=None):
        """Start the application"""
        # Print welcome message
        print("=" * 60)
        print("ASGC Motor Control Log Viewer - Tabbed Interface")
        print("=" * 60)
        print("\nFeatures:")
        print("  • Multiple logs open in tabs")
        print("  • Interactive plots with zoom/pan")
        print("  • Navigation state visualization")
        print("  • Timing diagnostics and effective sample rate stats")
        print("  • MPU6050 gyro and odometry data")
        print("\nControls:")
        print("  • Open Log: Add new tab")
        print("  • Close Tab: Remove current tab")
        print("  • Move to Trash: Move current log file to logs/.trash")
        print("  • Refresh: Reload current log")
        print("=" * 60)
        print()
        
        initial_files = initial_files or []
        if initial_files:
            for path in initial_files:
                self.add_tab(filename=path)
        else:
            # Open file selection dialog after window is ready.
            self.root.after(100, self.add_tab)

        self.root.mainloop()


def main():
    """Entry point"""
    parser = argparse.ArgumentParser(description="ASGC motor control log viewer")
    parser.add_argument(
        "files",
        nargs="*",
        help="optional CSV log files to open immediately",
    )
    parser.add_argument(
        "--latest",
        type=int,
        default=0,
        help="open latest N files from the logs directory",
    )
    parser.add_argument(
        "--mode",
        choices=["any", "voice", "joystick", "ml"],
        default="any",
        help="mode filter when using --latest",
    )
    parser.add_argument(
        "--log-dir",
        default=None,
        help="custom log directory (default: ../logs)",
    )
    args = parser.parse_args()

    viewer = TabbedLogViewer(log_dir=args.log_dir)

    preload_files = []
    if args.latest > 0:
        preload_files.extend(viewer.get_latest_files(args.latest, mode=args.mode))

    for path in args.files:
        normalized = os.path.abspath(os.path.expanduser(path))
        preload_files.append(normalized)

    # Preserve order and remove duplicates.
    deduped = []
    seen = set()
    for path in preload_files:
        if path in seen:
            continue
        seen.add(path)
        deduped.append(path)

    if args.latest > 0 and not deduped:
        print(
            f"No matching log files found in {viewer.shm_dir} (live) or {viewer.log_dir} (persistent)",
            file=sys.stderr,
        )

    viewer.run(initial_files=deduped)


if __name__ == "__main__":
    main()
