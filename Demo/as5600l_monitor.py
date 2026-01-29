#!/usr/bin/env python3
"""
AS5600L Sensor Monitor
Reads two AS5600L magnetic rotary position sensors and tracks rotation counts
"""

import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QGroupBox, QPushButton)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont
import smbus2
import time

# AS5600L Register Addresses
ANGLE_HIGH_REGISTER = 0x0C
ANGLE_LOW_REGISTER = 0x0D

# Sensor I2C Addresses
SENSOR_1_ADDRESS = 0x40
SENSOR_2_ADDRESS = 0x1B

class AS5600LSensor:
    """Class to handle AS5600L sensor reading and rotation tracking"""

    def __init__(self, bus, address, name):
        self.bus = bus
        self.address = address
        self.name = name
        self.current_angle = 0
        self.previous_angle = 0
        self.rotation_count = 0
        self.total_rotations = 0
        self.initialized = False

    def read_angle(self):
        """Read the current angle from the sensor (0-4095 for 0-360 degrees)"""
        try:
            # Read high byte and low byte
            high_byte = self.bus.read_byte_data(self.address, ANGLE_HIGH_REGISTER)
            low_byte = self.bus.read_byte_data(self.address, ANGLE_LOW_REGISTER)

            # Combine into 12-bit value
            angle_raw = (high_byte << 8) | low_byte
            angle_raw = angle_raw & 0x0FFF  # Mask to 12 bits

            return angle_raw
        except Exception as e:
            print(f"Error reading from sensor at 0x{self.address:02X}: {e}")
            return None

    def update(self):
        """Update sensor reading and track rotations"""
        angle = self.read_angle()

        if angle is None:
            return False

        self.current_angle = angle

        if not self.initialized:
            self.previous_angle = angle
            self.initialized = True
            return True

        # Detect rotation wrap-around
        # If we go from high value to low value, we've rotated forward
        if self.previous_angle > 3000 and self.current_angle < 1000:
            self.rotation_count += 1
            self.total_rotations += 1
        # If we go from low value to high value, we've rotated backward
        elif self.previous_angle < 1000 and self.current_angle > 3000:
            self.rotation_count -= 1
            self.total_rotations += 1

        self.previous_angle = self.current_angle
        return True

    def get_angle_degrees(self):
        """Convert raw angle to degrees"""
        return (self.current_angle * 360.0) / 4096.0

    def reset_count(self):
        """Reset the rotation counter"""
        self.rotation_count = 0
        self.total_rotations = 0


class SensorMonitorGUI(QMainWindow):
    """Main GUI window for sensor monitoring"""

    def __init__(self):
        super().__init__()
        self.i2c_bus = None
        self.sensor1 = None
        self.sensor2 = None
        self.init_ui()
        self.init_sensors()
        self.start_monitoring()

    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle('AS5600L Sensor Monitor')
        self.setGeometry(100, 100, 600, 400)

        # Central widget and main layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # Title
        title = QLabel('AS5600L Rotation Monitor')
        title.setFont(QFont('Arial', 18, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)

        # Sensor displays
        sensors_layout = QHBoxLayout()

        # Sensor 1 Group
        self.sensor1_group = self.create_sensor_group('Sensor 1 (0x40)')
        sensors_layout.addWidget(self.sensor1_group)

        # Sensor 2 Group
        self.sensor2_group = self.create_sensor_group('Sensor 2 (0x1B)')
        sensors_layout.addWidget(self.sensor2_group)

        main_layout.addLayout(sensors_layout)

        # Control buttons
        button_layout = QHBoxLayout()

        reset_btn = QPushButton('Reset Counters')
        reset_btn.clicked.connect(self.reset_counters)
        button_layout.addWidget(reset_btn)

        quit_btn = QPushButton('Quit')
        quit_btn.clicked.connect(self.close)
        button_layout.addWidget(quit_btn)

        main_layout.addLayout(button_layout)

        # Status bar
        self.statusBar().showMessage('Initializing...')

    def create_sensor_group(self, title):
        """Create a group box for sensor display"""
        group = QGroupBox(title)
        layout = QVBoxLayout()

        # Angle display
        angle_label = QLabel('Angle:')
        angle_label.setFont(QFont('Arial', 10, QFont.Bold))
        layout.addWidget(angle_label)

        angle_value = QLabel('0.0°')
        angle_value.setFont(QFont('Arial', 24))
        angle_value.setAlignment(Qt.AlignCenter)
        angle_value.setStyleSheet('QLabel { color: #2196F3; }')
        layout.addWidget(angle_value)

        # Rotation count display
        count_label = QLabel('Rotations:')
        count_label.setFont(QFont('Arial', 10, QFont.Bold))
        layout.addWidget(count_label)

        count_value = QLabel('0')
        count_value.setFont(QFont('Arial', 32, QFont.Bold))
        count_value.setAlignment(Qt.AlignCenter)
        count_value.setStyleSheet('QLabel { color: #4CAF50; }')
        layout.addWidget(count_value)

        # Total rotations
        total_label = QLabel('Total: 0')
        total_label.setFont(QFont('Arial', 9))
        total_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(total_label)

        # Store references to value labels
        group.angle_value = angle_value
        group.count_value = count_value
        group.total_label = total_label

        group.setLayout(layout)
        return group

    def init_sensors(self):
        """Initialize I2C bus and sensors"""
        try:
            # Initialize I2C bus (usually bus 1 on Raspberry Pi)
            self.i2c_bus = smbus2.SMBus(1)

            # Initialize sensors
            self.sensor1 = AS5600LSensor(self.i2c_bus, SENSOR_1_ADDRESS, "Sensor 1")
            self.sensor2 = AS5600LSensor(self.i2c_bus, SENSOR_2_ADDRESS, "Sensor 2")

            self.statusBar().showMessage('Sensors initialized successfully')
        except Exception as e:
            self.statusBar().showMessage(f'Error initializing sensors: {e}')
            print(f"Error initializing I2C: {e}")

    def start_monitoring(self):
        """Start the periodic sensor update timer"""
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_sensors)
        self.timer.start(50)  # Update every 50ms (20Hz)

    def update_sensors(self):
        """Update sensor readings and display"""
        if self.sensor1 and self.sensor2:
            # Update sensor 1
            if self.sensor1.update():
                angle1 = self.sensor1.get_angle_degrees()
                self.sensor1_group.angle_value.setText(f'{angle1:.1f}°')
                self.sensor1_group.count_value.setText(f'{self.sensor1.rotation_count}')
                self.sensor1_group.total_label.setText(f'Total: {self.sensor1.total_rotations}')

            # Update sensor 2
            if self.sensor2.update():
                angle2 = self.sensor2.get_angle_degrees()
                self.sensor2_group.angle_value.setText(f'{angle2:.1f}°')
                self.sensor2_group.count_value.setText(f'{self.sensor2.rotation_count}')
                self.sensor2_group.total_label.setText(f'Total: {self.sensor2.total_rotations}')

    def reset_counters(self):
        """Reset all rotation counters"""
        if self.sensor1:
            self.sensor1.reset_count()
        if self.sensor2:
            self.sensor2.reset_count()
        self.statusBar().showMessage('Counters reset', 2000)

    def closeEvent(self, event):
        """Clean up when closing the application"""
        if self.i2c_bus:
            self.i2c_bus.close()
        event.accept()


def main():
    """Main application entry point"""
    app = QApplication(sys.argv)
    window = SensorMonitorGUI()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
