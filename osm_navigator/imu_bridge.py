#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import re
import math
from sensor_msgs.msg import Imu

class IMUBridge(Node):
    def __init__(self):
        super().__init__('imu_bridge')
        
        # --- CONFIGURATION ---
        self.serial_port = '/dev/ttyUSB0'  # Change to /dev/ttyACM0 if needed
        self.baud_rate = 115200            # Ensure this matches your microcontroller's baud rate
        
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        
        # State variables to hold data until a full block is received
        self.current_msg = Imu()
        self.current_msg.header.frame_id = "base_link"
        
        # Fake covariance (Trusting the sensor decently well)
        cov = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        self.current_msg.orientation_covariance = cov
        self.current_msg.angular_velocity_covariance = cov
        self.current_msg.linear_acceleration_covariance = cov

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to IMU on {self.serial_port}")
            # Use a fast timer to constantly read the serial buffer
            self.create_timer(0.01, self.read_serial)
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Serial Port: {e}")

    def read_serial(self):
        if not self.ser.is_open:
            return
            
        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                
                # Extract all numbers from the line (handles typos like -0-02 or attached commas)
                nums = [float(x) for x in re.findall(r'[-+]?\d*\.\d+|[-+]?\d+', line.replace('-0-02', '-0.02'))]

                if "Angular velocity" in line and len(nums) >= 3:
                    # Convert DPS to Rad/s
                    self.current_msg.angular_velocity.x = nums[0] * (math.pi / 180.0)
                    self.current_msg.angular_velocity.y = nums[1] * (math.pi / 180.0)
                    self.current_msg.angular_velocity.z = nums[2] * (math.pi / 180.0)
                    
                elif "Linear Acc" in line and len(nums) >= 3:
                    # Convert G's to m/s^2
                    self.current_msg.linear_acceleration.x = nums[0] * 9.80665
                    self.current_msg.linear_acceleration.y = nums[1] * 9.80665
                    self.current_msg.linear_acceleration.z = nums[2] * 9.80665
                    
                elif "Quaternion" in line and len(nums) >= 4:
                    # Quaternions need no math conversion, just mapping
                    self.current_msg.orientation.x = nums[0]
                    self.current_msg.orientation.y = nums[1]
                    self.current_msg.orientation.z = nums[2]
                    self.current_msg.orientation.w = nums[3]
                    
                    # Quaternion is usually the last line in the block, so we publish now!
                    self.current_msg.header.stamp = self.get_clock().now().to_msg()
                    self.imu_pub.publish(self.current_msg)
                    
            except Exception as e:
                pass # Ignore garbled serial lines during startup

def main(args=None):
    rclpy.init(args=args)
    node = IMUBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()