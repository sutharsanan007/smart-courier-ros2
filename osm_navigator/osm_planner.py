#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import requests
import utm
import math
import json
import paho.mqtt.client as mqtt

# Message Types
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

class OSMNavigator(Node):
    def __init__(self):
        super().__init__('osm_navigator')
        
        # --- CONFIGURATION ---
        self.base_url = "http://router.project-osrm.org/route/v1/foot/"
        
        # --- MQTT CONFIGURATION ---
        self.mqtt_broker = "172.16.21.221"  # <--- CHANGE THIS TO YOUR MQTT BROKER IP
        self.mqtt_port = 1883
        self.mqtt_topic = "autonomous/001/cmd/maplocation"
        self.mqtt_username = "raghulrajg"
        self.mqtt_password = "Gr2_nemam"
        
        # --- STATE ---
        self.current_speed = 0.0
        self.current_pose = None      # From EKF (Best)
        self.last_known_gps = None    # From Raw GPS (Backup)
        
        self.path_points = []
        self.current_idx = 0
        self.raw_gps = None
        self.base_utm = None          # (Easting, Northing) of the start point

        # --- SUBSCRIBERS ---
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        
        # (Optional) Kept the ROS goal subscriber as a local backup
        self.create_subscription(NavSatFix, '/gps/goal', self.ros_goal_callback, 10)

        self.path_pub = self.create_publisher(Path, '/osm_path', 10)
        self.create_timer(0.2, self.navigation_loop)

        # --- INITIALIZE MQTT ---
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        # ADD THIS LINE HERE (Must be before connect!):
        self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)
        
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start() 
            self.get_logger().info("--- NAVIGATOR READY (FAILSAFE + SECURE MQTT MODE) ---")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT Broker: {e}")

    # --- MQTT CALLBACKS ---
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info(f"Connected to MQTT! Subscribing to: {self.mqtt_topic}")
            client.subscribe(self.mqtt_topic)
        else:
            self.get_logger().error(f"MQTT connection failed with code {rc}")

    def on_mqtt_message(self, client, userdata, msg):
        try:
            # Decode the payload and parse the JSON
            payload = msg.payload.decode('utf-8')
            data = json.loads(payload)
            print(f"Received MQTT message: {data}")
            target_lat = data.get('lat')
            target_lng = data.get('lng')
            
            if target_lat is not None and target_lng is not None:
                self.get_logger().info(f"Goal received via MQTT: Lat={target_lat}, Lng={target_lng}")
                self.handle_new_goal(target_lat, target_lng)
            else:
                self.get_logger().warn(f"MQTT JSON missing 'lat' or 'lng': {payload}")
                
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON received on MQTT: {msg.payload}")
        except Exception as e:
            self.get_logger().error(f"Error processing MQTT message: {e}")

    # --- SENSOR CALLBACKS ---
    def odom_callback(self, msg):
        # EKF is working! Use its data.
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        self.current_speed = math.sqrt(vx**2 + vy**2 + vz**2)
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def gps_callback(self, msg):
        self.raw_gps = (msg.latitude, msg.longitude)
        
        # If we have a path datum, we can calculate local position from Raw GPS
        if self.base_utm:
            e, n, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
            self.last_known_gps = (e - self.base_utm[0], n - self.base_utm[1])

    def ros_goal_callback(self, msg):
        # Fallback to allow sending goals via terminal if needed
        self.get_logger().info("Goal received via ROS Topic.")
        self.handle_new_goal(msg.latitude, msg.longitude)

    # --- NAVIGATION LOGIC ---
    def handle_new_goal(self, target_lat, target_lng):
        """ Shared logic for both MQTT and ROS goals """
        if self.raw_gps is None:
            self.get_logger().warn("Waiting for initial GPS fix before routing...")
            return
        
        target_gps = (target_lat, target_lng)
        self.get_logger().info("Fetching new route...")
        self.fetch_route(self.raw_gps, target_gps)

    def fetch_route(self, start, end):
        url = f"{self.base_url}{start[1]},{start[0]};{end[1]},{end[0]}?overview=full&geometries=geojson"
        try:
            resp = requests.get(url)
            if resp.status_code == 200:
                coords = resp.json()['routes'][0]['geometry']['coordinates']
                self.process_path(coords)
            else:
                self.get_logger().error(f"OSRM Error: {resp.status_code}")
        except Exception as e:
            self.get_logger().error(f"Network Error: {e}")

    def process_path(self, coords_list):
        if not coords_list: return

        self.path_points = []
        self.current_idx = 0
        path_msg = Path()
        path_msg.header.frame_id = "map"

        # Set the Datum (0,0 point) based on the path start
        self.base_utm = utm.from_latlon(coords_list[0][1], coords_list[0][0])[0:2]

        for lon, lat in coords_list:
            e, n, _, _ = utm.from_latlon(lat, lon)
            x, y = e - self.base_utm[0], n - self.base_utm[1]
            self.path_points.append((x,y))
            
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            pose.header.frame_id = "map"
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        
    def navigation_loop(self):
        if not self.path_points:
            return

        # --- FAILSAFE LOGIC ---
        active_pose = self.current_pose
        source_name = "EKF"
        
        if active_pose is None:
            if self.last_known_gps is not None:
                active_pose = self.last_known_gps
                source_name = "GPS (Backup)"
            else:
                print(">> WAITING FOR SENSORS...")
                return

        # 1. Navigation Logic
        target_x, target_y = self.path_points[self.current_idx]
        curr_x, curr_y = active_pose
        
        dist = math.sqrt((target_x - curr_x)**2 + (target_y - curr_y)**2)

        if dist < 3.0: # Waypoint reached radius
            if self.current_idx < len(self.path_points) - 1:
                self.current_idx += 1
            else:
                print(f">> DESTINATION REACHED | Source: {source_name}")
                return

        # 2. Heading Calculation
        dx = target_x - curr_x
        dy = target_y - curr_y
        desired_heading = math.degrees(math.atan2(dy, dx))
        
        instruction = "FORWARD"
        angle_display = 0.0
        
        if self.current_idx > 0:
            prev_x, prev_y = self.path_points[self.current_idx-1]
            prev_angle = math.degrees(math.atan2(target_y - prev_y, target_x - prev_x))
            
            turn = desired_heading - prev_angle
            while turn <= -180: turn += 360
            while turn > 180: turn -= 360
            angle_display = turn

            if turn > 25: instruction = "TURN RIGHT"
            elif turn < -25: instruction = "TURN LEFT"

        # 3. Publish Downstream Command via MQTT
        sign = "+" if angle_display >= 0 else ""
        
        # Terminal visual output for debugging
        print(f">> {instruction:<12} {sign}{angle_display:.1f}°  |  Speed: {self.current_speed:.2f} m/s  [{source_name}]")
        
        # Build the structured command for the robot
        command_payload = {
            "type": "command",
            "action": instruction,
            "angle": float(round(angle_display, 2)),
            "speed": float(round(self.current_speed, 2)),
            "source": source_name
        }
        
        # Publish it back down to the edge device (Raspberry Pi)
        command_topic = "autonomous/001/commands"
        self.mqtt_client.publish(command_topic, json.dumps(command_payload))

    def destroy_node(self):
        # Gracefully shut down the MQTT background thread
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OSMNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()