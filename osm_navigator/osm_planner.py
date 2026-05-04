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
        self.mqtt_broker = "172.16.21.221"
        self.mqtt_port = 1883
        self.mqtt_username = "raghulrajg"
        self.mqtt_password = "Gr2_nemam"
        
        # SYNCED WITH MOSQUITTO COMMANDS & RASPBERRY PI
        self.mqtt_topic = "autonomous/robot/2/goal"
        self.command_topic = "autonomous/robot/2/commands"
        
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
        
        # Authentication must be set before connecting
        self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)
        
        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start() 
            self.get_logger().info("--- NAVIGATOR READY (DISTRIBUTED EDGE ARCHITECTURE) ---")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT Broker: {e}")

    # --- MQTT CALLBACKS ---
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info(f"Connected to MQTT Broker. Subscribed to: {self.mqtt_topic}")
            client.subscribe(self.mqtt_topic)
        else:
            self.get_logger().error(f"MQTT connection failed with code {rc}")

    def on_mqtt_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode('utf-8')
            data = json.loads(payload)
            target_lat = data.get('lat')
            target_lng = data.get('lng')
            
            if target_lat is not None and target_lng is not None:
                self.get_logger().info(f"Target Acquired via MQTT: Lat={target_lat}, Lng={target_lng}")
                self.handle_new_goal(target_lat, target_lng)
            else:
                self.get_logger().warn(f"Malformed payload received (missing coordinates): {payload}")
                
        except json.JSONDecodeError:
            self.get_logger().error(f"Data Decode Error: {msg.payload}")
        except Exception as e:
            self.get_logger().error(f"Processing Error: {e}")

    # --- SENSOR CALLBACKS ---
    def odom_callback(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        self.current_speed = math.sqrt(vx**2 + vy**2 + vz**2)
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def gps_callback(self, msg):
        self.raw_gps = (msg.latitude, msg.longitude)
        if self.base_utm:
            e, n, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
            self.last_known_gps = (e - self.base_utm[0], n - self.base_utm[1])

    def ros_goal_callback(self, msg):
        self.get_logger().info("Target Acquired via Local ROS Topic.")
        self.handle_new_goal(msg.latitude, msg.longitude)

    # --- NAVIGATION LOGIC ---
    def handle_new_goal(self, target_lat, target_lng):
        if self.raw_gps is None:
            self.get_logger().warn("Awaiting initial telemetry fix before calculating trajectory.")
            return
        
        target_gps = (target_lat, target_lng)
        self.get_logger().info("Calculating OSRM trajectory...")
        self.fetch_route(self.raw_gps, target_gps)

    def fetch_route(self, start, end):
        url = f"{self.base_url}{start[1]},{start[0]};{end[1]},{end[0]}?overview=full&geometries=geojson"
        try:
            resp = requests.get(url)
            if resp.status_code == 200:
                coords = resp.json()['routes'][0]['geometry']['coordinates']
                self.process_path(coords)
            else:
                self.get_logger().error(f"Routing Engine Error: {resp.status_code}")
        except Exception as e:
            self.get_logger().error(f"Network Fault: {e}")

    def process_path(self, coords_list):
        if not coords_list: return

        self.path_points = []
        self.current_idx = 0
        path_msg = Path()
        path_msg.header.frame_id = "map"

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
        self.get_logger().info("Trajectory calculated and published successfully.")
        
    def navigation_loop(self):
        if not self.path_points:
            return

        # --- TELEMETRY EVALUATION ---
        active_pose = self.current_pose
        source_name = "EKF"
        
        if active_pose is None:
            if self.last_known_gps is not None:
                active_pose = self.last_known_gps
                source_name = "GPS_BACKUP"
            else:
                # Silently return to keep terminal clean during initialization
                return

        # 1. Waypoint Evaluation
        target_x, target_y = self.path_points[self.current_idx]
        curr_x, curr_y = active_pose
        
        dist = math.sqrt((target_x - curr_x)**2 + (target_y - curr_y)**2)

        if dist < 3.0: 
            if self.current_idx < len(self.path_points) - 1:
                self.current_idx += 1
            else:
                # Stop the robot when destination is reached
                stop_payload = {"type": "command", "action": "STOP", "angle": 0.0, "speed": 0.0, "source": source_name}
                self.mqtt_client.publish(self.command_topic, json.dumps(stop_payload))
                print(f"[NAV] Destination achieved via {source_name}. Halting execution.")
                self.path_points = [] # Clear path
                return

        # 2. Kinematic Calculation
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

            if turn > 25: instruction = "TURN_RIGHT"
            elif turn < -25: instruction = "TURN_LEFT"

        # 3. Network Dispatch
        sign = "+" if angle_display >= 0 else ""
        
        print(f"[EXEC] Action: {instruction:<12} | Vector: {sign}{angle_display:.1f}° | Velocity: {self.current_speed:.2f} m/s | Lock: {source_name}")
        
        command_payload = {
            "type": "command",
            "action": instruction,
            "angle": float(round(angle_display, 2)),
            "speed": float(round(self.current_speed, 2)),
            "source": source_name
        }
        
        self.mqtt_client.publish(self.command_topic, json.dumps(command_payload))

    def destroy_node(self):
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