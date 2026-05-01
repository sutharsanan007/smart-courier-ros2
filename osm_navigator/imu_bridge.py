#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import json
from sensor_msgs.msg import Imu

class IMUBridge(Node):
    def __init__(self):
        super().__init__('imu_bridge')
        
        # --- CONFIGURATION ---
        self.serial_port = '/dev/ttyUSB0'  # Adjust if your Pi uses ttyUSB1 or ttyACM0
        self.baud_rate = 115200            
        
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        
        # Pre-allocate message to save processing power
        self.current_msg = Imu()
        self.current_msg.header.frame_id = "base_link"
        
        # Covariance matrix (Adjust these later if the EKF acts jittery)
        cov = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        self.current_msg.orientation_covariance = cov
        self.current_msg.angular_velocity_covariance = cov
        self.current_msg.linear_acceleration_covariance = cov

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to IMU on {self.serial_port}")
            # Run very fast to keep up with the serial stream
            self.create_timer(0.01, self.read_serial)
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Serial Port: {e}")

    def read_serial(self):
        # Ensure the serial connection actually exists before reading
        if getattr(self, 'ser', None) is None or not self.ser.is_open:
            return
            
        while self.ser.in_waiting > 0:
            try:
                # Read the line and clean up whitespace
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                
                # Check if it contains JSON payload (ignoring the "10:28:46 -> " timestamp)
                if "{" in line and "}" in line:
                    # Extract just the JSON string
                    json_str = line[line.find("{") : line.rfind("}") + 1]
                    data = json.loads(json_str)
                    
                    # --- MAP JSON DIRECTLY TO ROS 2 MESSAGE ---
                    self.current_msg.angular_velocity.x = float(data.get('gx', 0.0))
                    self.current_msg.angular_velocity.y = float(data.get('gy', 0.0))
                    self.current_msg.angular_velocity.z = float(data.get('gz', 0.0))
                    
                    self.current_msg.linear_acceleration.x = float(data.get('ax', 0.0))
                    self.current_msg.linear_acceleration.y = float(data.get('ay', 0.0))
                    self.current_msg.linear_acceleration.z = float(data.get('az', 0.0))
                    
                    self.current_msg.orientation.x = float(data.get('qx', 0.0))
                    self.current_msg.orientation.y = float(data.get('qy', 0.0))
                    self.current_msg.orientation.z = float(data.get('qz', 0.0))
                    self.current_msg.orientation.w = float(data.get('qw', 1.0))
                    
                    # Stamp it with the Raspberry Pi's exact system time and publish!
                    self.current_msg.header.stamp = self.get_clock().now().to_msg()
                    self.imu_pub.publish(self.current_msg)
                    
            except json.JSONDecodeError:
                pass # Ignore lines that are half-written or corrupted during serial transmission
            except Exception as e:
                pass # Ignore other random serial noise

def main(args=None):
    rclpy.init(args=args)
    node = IMUBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()