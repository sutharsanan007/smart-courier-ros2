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
        self.mqtt_broker = "172.16.21.221"
        self.mqtt_port = 1883
        self.mqtt_user = "raghulrajg"
        self.mqtt_pass = "Gr2_nemam"
        
        # SYNCED WITH RASPBERRY PI:
        self.telemetry_topic = "autonomous/robot/2/telemetry"
        
        # --- ROS 2 PUBLISHERS ---
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = "base_link"
        cov = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        self.imu_msg.orientation_covariance = cov
        self.imu_msg.angular_velocity_covariance = cov
        self.imu_msg.linear_acceleration_covariance = cov

        self.gps_msg = NavSatFix()
        self.gps_msg.header.frame_id = "gps_link"
        self.gps_msg.position_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0]
        self.gps_msg.position_covariance_type = 2

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
            current_time = self.get_clock().now().to_msg()
            
            # --- 1. IMU PARSING ---
            if "ax" in data and "gx" in data:
                self.imu_msg.header.stamp = current_time
                self.imu_msg.linear_acceleration.x = float(data.get('ax', 0.0))
                self.imu_msg.linear_acceleration.y = float(data.get('ay', 0.0))
                self.imu_msg.linear_acceleration.z = float(data.get('az', 0.0))
                
                self.imu_msg.angular_velocity.x = float(data.get('gx', 0.0))
                self.imu_msg.angular_velocity.y = float(data.get('gy', 0.0))
                self.imu_msg.angular_velocity.z = float(data.get('gz', 0.0))
                
                self.imu_msg.orientation.x = float(data.get('qx', 0.0))
                self.imu_msg.orientation.y = float(data.get('qy', 0.0))
                self.imu_msg.orientation.z = float(data.get('qz', 0.0))
                self.imu_msg.orientation.w = float(data.get('qw', 1.0))
                
                self.imu_pub.publish(self.imu_msg)

            # --- 2. GPS PARSING ---
            if "lat" in data and "lon" in data:
                self.gps_msg.header.stamp = current_time
                self.gps_msg.latitude = float(data.get('lat', 0.0))
                self.gps_msg.longitude = float(data.get('lon', 0.0))
                self.gps_pub.publish(self.gps_msg)

        except json.JSONDecodeError:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = CloudTelemetryBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()