"""
======================================================================
File: frontier_exploration.py
Author: Toh Leong Chuan
Date: 16/03/25

This file is based on original code and ideas by:
    Kai Nakamura
    https://kainakamura.com/project/slam-robot

Description:
    ROS2 Node that explores frontiers and publishes best path for use in 
    pure_pursuit.

Modifications:
    - ROS2 Implementation was used instead of ROS1 Features
======================================================================
"""

import numpy as np
import rclpy
import tf2_ros
import random
import math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from typing import Union
from .path_planner import PathPlanner
from .frontier_search import FrontierSearch
from nav_msgs.msg import OccupancyGrid, Path, GridCells, Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from cde2310_interfaces.msg import FrontierList
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.duration import Duration 
import time ## TEST
from std_msgs.msg import Header


# TODO: Determine if it is worth the effort to write debugging code


####################################################################
'''
TODO
'''
TIMER_PERIOD = 0.05


'''
TODO
'''
NUM_EXPLORE_FAILS_BEFORE_FINISH = 50

'''
TODO
'''
A_STAR_COST_WEIGHT = 10.0

'''
TODO
'''
FRONTIER_SIZE_COST_WEIGHT = 1.0

MAX_NUM_FRONTIERS_TO_CHECK = 8


'''
TODO
'''
MIN_RANDOM_GOAL_DISTANCE = 10  # minimum distance (in grid cells) from current position

####################################################################

# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

class FrontierExploration(Node):
    def __init__(self):
        super().__init__("frontier_exploration")

        # Publishers
        self.pure_pursuit_pub = self.create_publisher(
            Path, 'pure_pursuit_path', 10
        )

        # For inflated map
        self.cspace_pub = self.create_publisher(OccupancyGrid, 'inflated_map', 10)


        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        ) 
        self.occ_sub = self.create_subscription(
            OccupancyGrid, "map", self.occ_callback,
            qos_profile_sensor_data
        )

        # tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timed callback to run primary functions
        self.timer = self.create_timer(TIMER_PERIOD, self.run_loop_callback)

        ''' State Variables
        pose stores the tuple (Point, Quaternion) for information on Robot's Position.
        map stores the occupancy grid
        '''
        self.pose = None 
        self.map = None
        self.no_path_found_counter = 0
        self.no_frontiers_found_counter = 0
        self.is_finished_exploring = False
        self.random_exploration_mode = False  

        return


    # Subscriber Callbacks
    def odom_callback(self, msg):
        """
        Updates the current pose of the robot.
        """
        try:
            # Lookup the transform from "map" to "base_footprint"
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                'map', 
                'base_footprint', 
                now, 
                timeout=Duration(seconds=1.0)
            )

            # Update self.pose using the transform data
            self.pose = Pose()
            self.pose.position = Point(
                x=transform.transform.translation.x,
                y=transform.transform.translation.y,
                z=transform.transform.translation.z
            )
            self.pose.orientation = Quaternion(
                x=transform.transform.rotation.x,
                y=transform.transform.rotation.y,
                z=transform.transform.rotation.z,
                w=transform.transform.rotation.w
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().error(f'Could not transform: {ex}')
            return
    
    def occ_callback(self, msg: OccupancyGrid):
        self.map = msg
        return
    
    @staticmethod
    def get_top_frontiers(frontiers, n):
        # Sort the frontiers by sie in descending order
        sorted_frontiers = sorted(
            frontiers, key=lambda frontier: frontier.size, reverse=True
        )

        # Return the top n frontiers
        return sorted_frontiers[:n]

    def check_if_finished_exploring(self):
        # Publish empty path to stop the robot momentarily
        self.pure_pursuit_pub.publish(Path())
        
        # If failures exceed the threshold, switch to random exploration mode instead of finishing
        if (self.no_frontiers_found_counter >= NUM_EXPLORE_FAILS_BEFORE_FINISH or
            self.no_path_found_counter >= NUM_EXPLORE_FAILS_BEFORE_FINISH):
            if not self.random_exploration_mode:
                self.get_logger().info("Frontier exploration failing. Switching to random exploration mode.")
                self.random_exploration_mode = True
            return

    
    def explore_frontier(self, frontier_list: FrontierList):
        # If Finished Exploring, No Pose, No map or no frontier list, return
        if self.is_finished_exploring or self.pose is None or self.map is None:
            return

        ## Every element in frontiers consists of a size (of frontier) and a point
        frontiers = frontier_list.frontiers

        # If no frontiers are found, check if finished exploring
        if not frontiers:
            self.get_logger().warning("No Frontiers Found")
            self.no_frontiers_found_counter += 1
            self.get_logger().warning(f"Count Before Termination: {NUM_EXPLORE_FAILS_BEFORE_FINISH}")
            self.get_logger().warning(f"Current Count: {self.no_frontiers_found_counter}")
            self.check_if_finished_exploring()
            return
        else:
            self.no_frontiers_found_counter = 0

        # Calculate the C-Space.
        # C-Space: Configuration Space. TLDR; Makes it easier to plan paths for Robot
        # TODO: Implement method in PathPlanner (modified)

        c_space, cspace_cells = PathPlanner.calc_cspace(self.map, False)

        # Publish inflated map for visualization
        inflated_msg = OccupancyGrid()
        inflated_msg.header = Header()
        inflated_msg.header.stamp = self.get_clock().now().to_msg()
        inflated_msg.header.frame_id = 'map'
        inflated_msg.info = self.map.info
        inflated_msg = c_space  # c_space is already an OccupancyGrid
        inflated_msg.header.stamp = self.get_clock().now().to_msg()
        c_space.header.stamp = self.get_clock().now().to_msg()
        self.cspace_pub.publish(c_space)

        # Calculate the cost map
        cost_map = PathPlanner.calc_cost_map(self.map)

        # Get the start
        start = PathPlanner.world_to_grid(self.map, self.pose.position)

        # Execute A* for every frontier
        lowest_cost = float("inf")
        best_path = None

        # Check only the top frontiers in terms of size
        top_frontiers = FrontierExploration.get_top_frontiers(
            frontiers, MAX_NUM_FRONTIERS_TO_CHECK
        )

        starts = []
        goals = []

        # Log how many frontiers are being explored
        self.get_logger().info(f"Exploring {len(top_frontiers)} frontiers")

        for frontier in top_frontiers:
            # Get Goal (In this case, the goal is the centroid of the current frontier)
            goal = PathPlanner.world_to_grid(self.map, frontier.centroid)

            # Execute A*
            path, a_star_cost, start, goal = PathPlanner.a_star(
                c_space, cost_map, start, goal
            )

            if path is None or a_star_cost is None:
                continue

             # Calculate cost
            cost = (A_STAR_COST_WEIGHT * a_star_cost) + (
                FRONTIER_SIZE_COST_WEIGHT / frontier.size
            )

            # Update best path
            if cost < lowest_cost:
                lowest_cost = cost
                best_path = path
        
        # If a path was found, publish it
        if best_path:
            self.get_logger().info(f"Found best path with cost {lowest_cost}")
            start = best_path[0]
            path = PathPlanner.path_to_message(self.map, best_path)
            self.pure_pursuit_pub.publish(path)
            self.no_path_found_counter = 0
            return
        # If no path was found, check if finished exploring
        else:
            self.get_logger().info("No paths found")
            self.no_path_found_counter += 1
            self.check_if_finished_exploring()
            return

    def explore_randomly(self):
        # Get map dimensions from the occupancy grid
        width = self.map.info.width
        height = self.map.info.height
        self.get_logger().info(f"Width: {width}, Height: {height}")
        max_attempts = 100
        random_goal_cell = None

        # Get the current start cell (robot's position in grid coordinates)
        start = PathPlanner.world_to_grid(self.map, self.pose.position)

        # Try to find a random free cell that is at least MIN_RANDOM_GOAL_DISTANCE away
        for _ in range(max_attempts):
            x = random.randint(0, width - 1)
            y = random.randint(0, height - 1)
            self.get_logger().info(f"Random candidate: ({x}, {y})")
            cell = (x, y)
            if not PathPlanner.is_cell_in_bounds(self.map, cell):
                continue
            if not PathPlanner.is_cell_walkable(self.map, cell):
                continue
            if PathPlanner.euclidean_distance(start, cell) < MIN_RANDOM_GOAL_DISTANCE:
                continue
            random_goal_cell = cell
            self.get_logger().info(f"Random goal selected: {cell}")
            break

        if random_goal_cell is None:
            self.get_logger().error("Random exploration: Failed to find a free cell for goal.")
            return

        # Calculate C-space and cost map as in frontier exploration.
        c_space, _ = PathPlanner.calc_cspace(self.map, False)
        
        # Publish inflated map for visualization
        inflated_msg = OccupancyGrid()
        inflated_msg.header = Header()
        inflated_msg.header.stamp = self.get_clock().now().to_msg()
        inflated_msg.header.frame_id = 'map'
        inflated_msg.info = self.map.info
        inflated_msg = c_space  # c_space is already an OccupancyGrid
        inflated_msg.header.stamp = self.get_clock().now().to_msg()
        c_space.header.stamp = self.get_clock().now().to_msg()
        self.cspace_pub.publish(c_space)


        cost_map = PathPlanner.calc_cost_map(self.map)

        self.get_logger().info(f"Start cell: {start}")
        path, a_star_cost, start, goal = PathPlanner.a_star(c_space, cost_map, start, random_goal_cell)
        self.get_logger().info("Call to A* finished")
        
        if path is not None and a_star_cost is not None:
            self.get_logger().info(f"Random exploration path found with cost {a_star_cost}")
            path_msg = PathPlanner.path_to_message(self.map, path)
            self.pure_pursuit_pub.publish(path_msg)
            self.no_path_found_counter = 0
            self.no_frontiers_found_counter = 0

            self.get_logger().info("Waiting for robot to reach random goal...")
            time.sleep(10)  # Wait (adjust as needed) for the robot to reach the random goal

            # Once at the goal, publish a rotation path 30° left...
            self.publish_rotation_at_goal(random_goal_cell, 30)
            time.sleep(2)  # Short pause between rotations
            # ... and 30° right.
            self.publish_rotation_at_goal(random_goal_cell, -30)
        else:
            self.get_logger().warning("Random exploration: No path found with A*. Retrying...")
            self.no_path_found_counter += 1
            self.check_if_finished_exploring()
            return
  
    def publish_rotation_at_goal(self, goal_cell, angle_deg):
        """
        Publishes a simple two-pose path at the given goal cell that rotates the robot in place by angle_deg.
        """
        # Convert the goal grid cell to a world coordinate.
        goal_point = PathPlanner.grid_to_world(self.map, goal_cell)
        
        # Create two PoseStamped messages: one with default orientation, and one rotated.
        from geometry_msgs.msg import PoseStamped
        pose_initial = PoseStamped()
        pose_initial.header.frame_id = "map"
        pose_initial.pose.position = goal_point
        # Assume an initial orientation of 0 (identity quaternion).
        pose_initial.pose.orientation.x = 0.0
        pose_initial.pose.orientation.y = 0.0
        pose_initial.pose.orientation.z = 0.0
        pose_initial.pose.orientation.w = 1.0

        # Compute the rotated orientation (about Z axis).
        rotated_angle = math.radians(angle_deg)
        from geometry_msgs.msg import Quaternion
        rotated_quat = Quaternion()
        rotated_quat.x = 0.0
        rotated_quat.y = 0.0
        rotated_quat.z = math.sin(rotated_angle/2)
        rotated_quat.w = math.cos(rotated_angle/2)

        pose_rotated = PoseStamped()
        pose_rotated.header.frame_id = "map"
        pose_rotated.pose.position = goal_point
        pose_rotated.pose.orientation = rotated_quat

        # Create a Path message with these two poses.
        from nav_msgs.msg import Path
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.poses = [pose_initial, pose_rotated]
        
        self.pure_pursuit_pub.publish(path_msg)
        self.get_logger().info(f"Published rotation path at goal with a {angle_deg}° offset.")

    
    # TODO
    def run_loop_callback(self):
        if self.pose is None or self.map is None:
            return

        # If in random exploration mode, call the new random exploration method.
        if self.random_exploration_mode:
            self.explore_randomly()
            return

        # Get the start position of the robot in grid coordinates.
        start = PathPlanner.world_to_grid(self.map, self.pose.position)

        # Get frontiers using the FrontierSearch (debug mode not enabled).
        frontier_list, frontier_cells = FrontierSearch.search(
            self.map, start, False
        )
        if frontier_list is None:
            return
        
        self.explore_frontier(frontier_list)
        return


# Original ROS1 Implementation has a rate of 20hz = 0.05 seconds.
def main():
    rclpy.init()

    frontierExploration = FrontierExploration()
    rclpy.spin(frontierExploration)

    frontierExploration.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()