import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'osm_navigator'
    config_file = os.path.join(get_package_share_directory(pkg_name), 'config', 'ekf.yaml')

    return LaunchDescription([
        # 1. GPS to Metric Transform
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn'],
            parameters=[{'use_odometry_yaw': False, 'frequency': 10.0}],
            remappings=[('/gps/fix', '/gps/fix'), 
                        ('/imu', '/imu/data'),
                        ('/odometry/filtered', '/odometry/filtered')]
        ),
        
        # 2. EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[config_file],
            remappings=[('/odometry/filtered', '/odometry/filtered')]
        ),
        
        # 3. OSM Navigator
        Node(
            package='osm_navigator',
            executable='osm_planner', 
            name='navigator',
            output='screen'
        )
    ])