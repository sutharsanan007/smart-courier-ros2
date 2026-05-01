#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import paho.mqtt.client as mqtt
from sensor_msgs.msg import Imu, NavSatFix

class CloudTelemetryBridge(Node):
    def __init__(self):
        super().__init__('telemetry_bridge')
        
        # --- MQTT CONFIGURATION ---
        self.mqtt_broker = "127.0.0.1"  # IP of your MQTT Broker
        self.mqtt_port = 1883           # <--- THIS IS THE ONLY PORT WE NEED NOW!
        self.mqtt_user = "my_robot_user"
        self.mqtt_pass = "my_secure_password"
        self.telemetry_topic = "autonomous/001/telemetry"
        
        # --- ROS 2 PUBLISHERS ---
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        
        # Pre-allocate IMU message structure
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = "base_link"
        cov = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        self.imu_msg.orientation_covariance = cov
        self.imu_msg.angular_velocity_covariance = cov
        self.imu_msg.linear_acceleration_covariance = cov

        # Pre-allocate GPS message structure
        self.gps_msg = NavSatFix()
        self.gps_msg.header.frame_id = "gps_link"
        self.gps_msg.position_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0]
        self.gps_msg.position_covariance_type = 2 # COVARIANCE_TYPE_DIAGONAL_KNOWN

        # --- START MQTT CLIENT ---
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set(self.mqtt_user, self.mqtt_pass)
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info("--- CLOUD TELEMETRY BRIDGE ACTIVE ---")
        except Exception as e:
            self.get_logger().error(f"MQTT Connection Failed: {e}")

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info(f"Connected to Broker. Listening to: {self.telemetry_topic}")
            client.subscribe(self.telemetry_topic)

    def on_mqtt_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode('utf-8')
            data = json.loads(payload)
            
            if data.get("type") == "telemetry":
                current_time = self.get_clock().now().to_msg()
                
                # 1. Parse and Publish IMU Data
                if "imu" in data:
                    imu_data = data["imu"]
                    self.imu_msg.header.stamp = current_time
                    
                    self.imu_msg.linear_acceleration.x = float(imu_data.get('ax', 0.0))
                    self.imu_msg.linear_acceleration.y = float(imu_data.get('ay', 0.0))
                    self.imu_msg.linear_acceleration.z = float(imu_data.get('az', 0.0))
                    
                    self.imu_msg.angular_velocity.x = float(imu_data.get('gx', 0.0))
                    self.imu_msg.angular_velocity.y = float(imu_data.get('gy', 0.0))
                    self.imu_msg.angular_velocity.z = float(imu_data.get('gz', 0.0))
                    
                    self.imu_msg.orientation.x = float(imu_data.get('qx', 0.0))
                    self.imu_msg.orientation.y = float(imu_data.get('qy', 0.0))
                    self.imu_msg.orientation.z = float(imu_data.get('qz', 0.0))
                    self.imu_msg.orientation.w = float(imu_data.get('qw', 1.0))
                    
                    self.imu_pub.publish(self.imu_msg)

                # 2. Parse and Publish GPS Data
                if "gps" in data:
                    gps_data = data["gps"]
                    self.gps_msg.header.stamp = current_time
                    self.gps_msg.latitude = float(gps_data.get('lat', 0.0))
                    self.gps_msg.longitude = float(gps_data.get('lon', 0.0))
                    
                    self.gps_pub.publish(self.gps_msg)

        except json.JSONDecodeError:
            pass # Ignore broken network packets

def main(args=None):
    rclpy.init(args=args)
    node = CloudTelemetryBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()