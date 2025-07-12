#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import math
import json
import argparse
import time
import numpy as np

from amrl_msgs.msg import Localization2DMsg

class WaypointNavigator(Node):
    WAYPOINT_THRESHOLD = 0.75
    
    def __init__(self, map_name, waypoints):
        super().__init__('waypoint_navigation')
        
        self.map_name = map_name
        self.waypoints = waypoints
        self.current_waypoint = 0
        
        # Create QoS profile for reliable delivery
        qos_profile = QoSProfile(depth=10)
        
        # Create subscriber for localization
        self.localization_sub = self.create_subscription(
            Localization2DMsg,
            'localization',
            self.loc_callback,
            qos_profile
        )
        
        # Create publisher for navigation goals
        self.nav_pub = self.create_publisher(
            Localization2DMsg,
            '/move_base_simple/goal_amrl',
            qos_profile
        )
        
        # Initialize goal message
        self.goal_msg = Localization2DMsg()
        self.goal_msg.map = map_name
        
        self.get_logger().info(f'Waypoint navigator initialized with {len(waypoints)} waypoints')
        
        # Give some time for publishers/subscribers to be established
        time.sleep(1.0)
        self.send_nav_command()

    def get_target_waypoint(self):
        if self.current_waypoint >= len(self.waypoints):
            if args.loop:
                self.get_logger().info("Circuit Complete, restarting...")
                self.current_waypoint = 0
            else:
                self.get_logger().info("Completed waypoint navigation, exiting...")
                rclpy.shutdown()
                return None

        return self.waypoints[self.current_waypoint]

    def loc_callback(self, loc):
        target_waypoint = self.get_target_waypoint()
        if target_waypoint is None:
            return

        if self.is_close(target_waypoint, loc.pose):
            self.current_waypoint += 1
            self.send_nav_command()

    def send_nav_command(self):
        target_waypoint = self.get_target_waypoint()
        if target_waypoint is None:
            return
            
        self.get_logger().info(
            f"Navigating to waypoint {self.current_waypoint + 1}/{len(self.waypoints)}: "
            f"({target_waypoint['x']}, {target_waypoint['y']}, {target_waypoint['theta']})"
        )

        self.goal_msg.pose.x = target_waypoint["x"]
        self.goal_msg.pose.y = target_waypoint["y"]
        self.goal_msg.pose.theta = target_waypoint["theta"]

        self.nav_pub.publish(self.goal_msg)

    @classmethod
    def is_close(cls, target, pose):
        diff = np.linalg.norm(
            np.array([pose.x, pose.y]) - np.array([target["x"], target["y"]])
        )
        return diff < cls.WAYPOINT_THRESHOLD


def main(argv=None):
    global args
    
    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--loop', action='store_true', 
                       help='Loop through waypoints continuously')
    parser.add_argument('--map', type=str, required=True,
                       help='Map name for navigation')
    parser.add_argument('--waypoints', type=str, required=True, 
                       help='JSON file containing an array of waypoints')
    
    args = parser.parse_args()
    
    # Load waypoints from JSON file
    try:
        with open(args.waypoints, 'r') as f:
            waypoints = json.load(f)
    except FileNotFoundError:
        print(f"Error: Could not find waypoints file: {args.waypoints}")
        return 1
    except json.JSONDecodeError:
        print(f"Error: Invalid JSON in waypoints file: {args.waypoints}")
        return 1
    
    if not waypoints:
        print("Error: No waypoints found in file")
        return 1
    
    # Validate waypoint format
    for i, waypoint in enumerate(waypoints):
        if not all(key in waypoint for key in ['x', 'y', 'theta']):
            print(f"Error: Waypoint {i} missing required keys (x, y, theta)")
            return 1
    
    # Initialize ROS2
    rclpy.init(args=argv)
    
    try:
        # Create and run the waypoint navigator
        waypoint_nav = WaypointNavigator(args.map, waypoints)
        
        # Spin the node
        rclpy.spin(waypoint_nav)
        
    except KeyboardInterrupt:
        print("\nWaypoint navigation interrupted by user")
    except Exception as e:
        print(f"Error during waypoint navigation: {e}")
        return 1
    finally:
        # Clean shutdown
        if rclpy.ok():
            waypoint_nav.destroy_node()
            rclpy.shutdown()
    
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())



