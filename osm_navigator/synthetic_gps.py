#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import utm

class SyntheticGPS(Node):
    def __init__(self):
        super().__init__('synthetic_gps')

        # --- CONFIGURATION: SET YOUR STARTING LOCATION HERE ---
        self.start_lat = 48.8584
        self.start_lon = 2.2945
        
        self.base_easting, self.base_northing, self.zone_number, self.zone_letter = utm.from_latlon(self.start_lat, self.start_lon)

        # STATE VARIABLES (Defaults to start location)
        self.current_lat = self.start_lat
        self.current_lon = self.start_lon

        # SUBSCRIBER
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)

        # PUBLISHER & TIMER (The Fix: Publish continuously at 5Hz)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.create_timer(0.2, self.timer_callback)
        
        self.get_logger().info(f"Synthetic GPS Ready! Broadcasting: {self.start_lat}, {self.start_lon}")

    def odom_callback(self, msg):
        # Update current coordinates based on EKF movement
        x_meters = msg.pose.pose.position.x
        y_meters = msg.pose.pose.position.y

        current_easting = self.base_easting + x_meters
        current_northing = self.base_northing + y_meters

        try:
            self.current_lat, self.current_lon = utm.to_latlon(current_easting, current_northing, self.zone_number, self.zone_letter)
        except Exception:
            pass # Ignore edge-case math errors

    def timer_callback(self):
        # Constantly blast the current GPS location to keep the system alive
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "base_link"
        gps_msg.latitude = self.current_lat
        gps_msg.longitude = self.current_lon
        
        # Fake covariance to keep the EKF happy
        gps_msg.position_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        
        self.gps_pub.publish(gps_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SyntheticGPS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()