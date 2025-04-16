"""
======================================================================
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
        # Publish empty path to stop the robot
        # TODO: Implement publish() method in pure_pursuit.py
        self.pure_pursuit_pub.publish(Path())

        # if no frontiers or paths are found for a certain number of times,
        # then likely exploration has finished
        if (
            self.no_frontiers_found_counter >= NUM_EXPLORE_FAILS_BEFORE_FINISH
            or self.no_path_found_counter >= NUM_EXPLORE_FAILS_BEFORE_FINISH
        ):
            self.get_logger().info("Done Exploring!")
            self.is_finished_exploring = True
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
    
    # TODO
    def run_loop_callback(self):

        if self.pose is None or self.map is None:
            return
        
        # Get the start position of the robot
        start = PathPlanner.world_to_grid(self.map, self.pose.position)

        # Get frontiers
        # False is there since debug feature is not implemented
        frontier_list, frontier_cells = FrontierSearch.search(
            self.map, start, False
        )

        if frontier_list is None:
            return
        
        # TODO: Debug mode code not implemented here

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