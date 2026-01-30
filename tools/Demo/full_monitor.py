#!/usr/bin/env python3
"""
Full Sensor Monitor
Monitors AS5600L encoders and MPU6050 IMU across three I2C buses.
Verifies all sensors are working and providing data.
"""

import sys
import smbus2
import time
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QGroupBox, QPushButton)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont

# --- Constants ---

# AS5600L (Encoders)
ANGLE_HIGH_REGISTER = 0x0C
ANGLE_LOW_REGISTER = 0x0D

# MPU6050 (IMU)
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
GYRO_ZOUT_H = 0x47

# I2C Addresses
LEFT_ENCODER_ADDR = 0x40
RIGHT_ENCODER_ADDR = 0x1B
IMU_ADDR = 0x68

# I2C Buses
BUS_LEFT = 3   # Left Encoder (0x40 on Bus 3)
BUS_RIGHT = 1  # Right Encoder (0x1B on Bus 1)
BUS_IMU = 2    # IMU (0x68 on Bus 2)

class I2CDevice:
    """Base class for I2C devices"""
    def __init__(self, bus_num, address, name):
        self.bus_num = bus_num
        self.address = address
        self.name = name
        self.bus = None
        self.connected = False
        self.error_count = 0
        
        try:
            self.bus = smbus2.SMBus(bus_num)
            self.connected = True
        except Exception as e:
            print(f"Error opening bus {bus_num} for {name}: {e}")

    def read_byte_data(self, register):
        try:
            if not self.bus:
                self.bus = smbus2.SMBus(self.bus_num)
            return self.bus.read_byte_data(self.address, register)
        except Exception as e:
            self.connected = False
            self.error_count += 1
            return None

    def write_byte_data(self, register, value):
        try:
            if not self.bus:
                self.bus = smbus2.SMBus(self.bus_num)
            self.bus.write_byte_data(self.address, register, value)
            return True
        except Exception as e:
            self.connected = False
            self.error_count += 1
            return False

class AS5600LSensor(I2CDevice):
    """Class to handle AS5600L sensor reading and rotation tracking"""
    def __init__(self, bus_num, address, name):
        super().__init__(bus_num, address, name)
        self.current_angle = 0
        self.previous_angle = 0
        self.rotation_count = 0
        self.total_rotations = 0
        self.initialized = False

    def read_angle(self):
        """Read the current angle from the sensor (0-4095 for 0-360 degrees)"""
        try:
            if not self.bus:
                 self.bus = smbus2.SMBus(self.bus_num)

            # Read high byte and low byte
            high_byte = self.bus.read_byte_data(self.address, ANGLE_HIGH_REGISTER)
            low_byte = self.bus.read_byte_data(self.address, ANGLE_LOW_REGISTER)

            # Combine into 12-bit value
            angle_raw = (high_byte << 8) | low_byte
            angle_raw = angle_raw & 0x0FFF  # Mask to 12 bits
            
            self.connected = True
            return angle_raw
        except Exception as e:
            self.connected = False
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
        if self.previous_angle > 3000 and self.current_angle < 1000:
            self.rotation_count += 1
            self.total_rotations += 1
        elif self.previous_angle < 1000 and self.current_angle > 3000:
            self.rotation_count -= 1
            self.total_rotations += 1

        self.previous_angle = self.current_angle
        return True

    def get_angle_degrees(self):
        return (self.current_angle * 360.0) / 4096.0

    def reset_count(self):
        self.rotation_count = 0
        self.total_rotations = 0

class MPU6050Sensor(I2CDevice):
    """Class to handle MPU6050 IMU reading"""
    def __init__(self, bus_num, address, name):
        super().__init__(bus_num, address, name)
        self.gyro_z = 0.0
        self.z_gyro_offset = 0.0
        self.init_imu()

    def init_imu(self):
        """Initialize MPU6050"""
        if self.write_byte_data(PWR_MGMT_1, 0x00): # Wake up
            time.sleep(0.1)
            self.connected = True
            print(f"IMU {self.name} initialized")
        else:
            print(f"Failed to initialize IMU {self.name}")

    def read_gyro_z(self):
        """Read Z-axis gyro data"""
        try:
            if not self.bus:
                 self.bus = smbus2.SMBus(self.bus_num)

            # Read high and low bytes
            high_byte = self.bus.read_byte_data(self.address, GYRO_ZOUT_H)
            low_byte = self.bus.read_byte_data(self.address, GYRO_ZOUT_H + 1)
            
            # Combine
            raw_z = (high_byte << 8) | low_byte
            if raw_z > 32767:
                raw_z -= 65536
                
            # Convert to degrees per second
            rate = raw_z / 131.0
            
            # Apply offset
            self.gyro_z = -(rate - self.z_gyro_offset)
            self.connected = True
            return self.gyro_z
        except Exception:
            self.connected = False
            return None

    def calibrate(self, samples=100):
        """Calibrate the gyro by averaging samples"""
        print("Calibrating Gyro...")
        sum_z = 0
        count = 0
        try:
            if not self.bus:
                 self.bus = smbus2.SMBus(self.bus_num)
                 
            for _ in range(samples):
                high = self.bus.read_byte_data(self.address, GYRO_ZOUT_H)
                low = self.bus.read_byte_data(self.address, GYRO_ZOUT_H + 1)
                raw = (high << 8) | low
                if raw > 32767:
                    raw -= 65536
                sum_z += (raw / 131.0)
                count += 1
                time.sleep(0.005) # 200Hz
                
            if count > 0:
                self.z_gyro_offset = sum_z / count
                print(f"Gyro Calibration Complete. Offset: {self.z_gyro_offset:.4f} dps")
                return True
        except Exception as e:
            print(f"Calibration failed: {e}")
            return False
        return False

class SensorMonitorGUI(QMainWindow):
    """Main GUI window for sensor monitoring"""

    def __init__(self):
        super().__init__()
        self.left_encoder = None
        self.right_encoder = None
        self.imu = None
        self.init_ui()
        self.init_sensors()
        self.start_monitoring()

    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle('Full Sensor Monitor')
        self.setGeometry(100, 100, 900, 500)

        # Central widget and main layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # Title
        title = QLabel('ASGC Robot Sensor Status')
        title.setFont(QFont('Arial', 18, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)

        # Sensor displays
        sensors_layout = QHBoxLayout()

        # Left Encoder Group (I2C3)
        self.left_group = self.create_encoder_group('Left Encoder (I2C3 - 0x40)')
        sensors_layout.addWidget(self.left_group)

        # IMU Group (I2C2)
        self.imu_group = self.create_imu_group('IMU (I2C2 - 0x68)')
        sensors_layout.addWidget(self.imu_group)

        # Right Encoder Group (I2C1)
        self.right_group = self.create_encoder_group('Right Encoder (I2C1 - 0x1B)')
        sensors_layout.addWidget(self.right_group)

        main_layout.addLayout(sensors_layout)

        # Control buttons
        button_layout = QHBoxLayout()

        reset_btn = QPushButton('Reset Counters')
        reset_btn.clicked.connect(self.reset_counters)
        button_layout.addWidget(reset_btn)

        cal_btn = QPushButton('Calibrate Gyro')
        cal_btn.clicked.connect(self.calibrate_gyro)
        button_layout.addWidget(cal_btn)

        quit_btn = QPushButton('Quit')
        quit_btn.clicked.connect(self.close)
        button_layout.addWidget(quit_btn)

        main_layout.addLayout(button_layout)

        # Status bar
        self.statusBar().showMessage('Initializing...')

    def create_encoder_group(self, title):
        group = QGroupBox(title)
        layout = QVBoxLayout()

        # Status
        status_label = QLabel('Status: Checking...')
        status_label.setStyleSheet('color: orange')
        layout.addWidget(status_label)
        
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

        group.status_label = status_label
        group.angle_value = angle_value
        group.count_value = count_value
        group.setLayout(layout)
        return group

    def create_imu_group(self, title):
        group = QGroupBox(title)
        layout = QVBoxLayout()

        # Status
        status_label = QLabel('Status: Checking...')
        status_label.setStyleSheet('color: orange')
        layout.addWidget(status_label)

        # Gyro Z display
        gyro_label = QLabel('Gyro Z Rate:')
        gyro_label.setFont(QFont('Arial', 10, QFont.Bold))
        layout.addWidget(gyro_label)

        gyro_value = QLabel('0.0 dps')
        gyro_value.setFont(QFont('Arial', 24))
        gyro_value.setAlignment(Qt.AlignCenter)
        gyro_value.setStyleSheet('QLabel { color: #9C27B0; }')
        layout.addWidget(gyro_value)
        
        # Heading (Integrated)
        heading_label = QLabel('Est. Heading change:')
        layout.addWidget(heading_label)
        
        heading_value = QLabel('0.0°')
        heading_value.setFont(QFont('Arial', 18))
        heading_value.setAlignment(Qt.AlignCenter)
        layout.addWidget(heading_value)

        group.status_label = status_label
        group.gyro_value = gyro_value
        group.heading_value = heading_value
        group.heading = 0.0
        group.last_time = time.time()
        group.setLayout(layout)
        return group

    def init_sensors(self):
        """Initialize sensors on their respective buses"""
        try:
            self.left_encoder = AS5600LSensor(BUS_LEFT, LEFT_ENCODER_ADDR, "Left Encoder")
            self.right_encoder = AS5600LSensor(BUS_RIGHT, RIGHT_ENCODER_ADDR, "Right Encoder")
            self.imu = MPU6050Sensor(BUS_IMU, IMU_ADDR, "IMU")
            self.statusBar().showMessage('Sensors initialized')
        except Exception as e:
            self.statusBar().showMessage(f'Error initializing: {e}')

    def start_monitoring(self):
        """Start the periodic sensor update timer"""
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_sensors)
        self.timer.start(50)  # 20Hz update (GUI refresh rate)

    def update_sensors(self):
        """Update sensor readings and display"""
        # Update Left Encoder
        if self.left_encoder:
            if self.left_encoder.update():
                self.left_group.status_label.setText("CONNECTED")
                self.left_group.status_label.setStyleSheet('color: green; font-weight: bold')
                self.left_group.angle_value.setText(f'{self.left_encoder.get_angle_degrees():.1f}°')
                self.left_group.count_value.setText(f'{self.left_encoder.rotation_count}')
            else:
                 self.left_group.status_label.setText("DISCONNECTED")
                 self.left_group.status_label.setStyleSheet('color: red; font-weight: bold')

        # Update Low Encoder (Right)
        if self.right_encoder:
            if self.right_encoder.update():
                self.right_group.status_label.setText("CONNECTED")
                self.right_group.status_label.setStyleSheet('color: green; font-weight: bold')
                self.right_group.angle_value.setText(f'{self.right_encoder.get_angle_degrees():.1f}°')
                self.right_group.count_value.setText(f'{self.right_encoder.rotation_count}')
            else:
                 self.right_group.status_label.setText("DISCONNECTED")
                 self.right_group.status_label.setStyleSheet('color: red; font-weight: bold')

        # Update IMU
        if self.imu:
            gyro = self.imu.read_gyro_z()
            if gyro is not None:
                self.imu_group.status_label.setText("CONNECTED")
                self.imu_group.status_label.setStyleSheet('color: green; font-weight: bold')
                self.imu_group.gyro_value.setText(f'{gyro:.1f} dps')
                
                # Simple integration for display
                now = time.time()
                dt = now - self.imu_group.last_time
                self.imu_group.last_time = now
                if abs(gyro) > 1.0: # Deadband
                    self.imu_group.heading += gyro * dt
                self.imu_group.heading_value.setText(f'{self.imu_group.heading:.1f}°')
            else:
                self.imu_group.status_label.setText("DISCONNECTED")
                self.imu_group.status_label.setStyleSheet('color: red; font-weight: bold')
                self.imu_group.last_time = time.time()

    def reset_counters(self):
        """Reset all rotation counters"""
        if self.left_encoder: self.left_encoder.reset_count()
        if self.right_encoder: self.right_encoder.reset_count()
        if self.imu_group: self.imu_group.heading = 0.0
        self.statusBar().showMessage('Counters reset', 2000)

    def calibrate_gyro(self):
        """Trigger gyro calibration"""
        if self.imu:
            self.statusBar().showMessage('Calibrating Gyro... Do not move robot!')
            self.timer.stop() # Pause updates during cal
            QApplication.processEvents() # Ensure UI updates
            
            if self.imu.calibrate():
                self.statusBar().showMessage(f'Calibration Done. Offset: {self.imu.z_gyro_offset:.2f}', 3000)
                self.imu_group.heading = 0.0 # Reset heading
            else:
                self.statusBar().showMessage('Calibration Failed!', 3000)
                
            self.timer.start(50) # Resume updates

    def closeEvent(self, event):
        event.accept()

def main():
    """Main application entry point"""
    app = QApplication(sys.argv)
    window = SensorMonitorGUI()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
