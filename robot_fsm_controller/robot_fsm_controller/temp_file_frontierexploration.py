import os
import math  
import threading
import subprocess
import numpy as np
import rclpy
from rclpy.node import Node
from typing import Union
from robot_fsm_controller.path_planner import PathPlanner
from robot_fsm_controller.frontier_search import FrontierSearch
from nav_msgs.msg import OccupancyGrid, Path, GridCells, Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from cde2310_interfaces.msg import FrontierList

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

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

        # CHANGED: Added timer callback (instead of the blocking while-loop)
        self.timer = self.create_timer(0.2, self.control_loop_callback)


        # Publishers
        self.pure_pursuit_pub = self.create_publisher(
            "pure_pursuit_path", Path, 10
        )

        # Subscribers
        self.odom_subscription = self.create_subscription(Odometry, "odom", self.odom_callback)
        self.occ_subscription = self.create_subscription(OccupancyGrid, "map", self.occ_callback)

        ## tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.lock = threading.Lock()
        self.pose = None
        self.map = None

        self.NUM_EXPLORE_FAILS_BEFORE_FINISH = 30
        self.no_path_found_counter = 0
        self.no_frontiers_found_counter = 0
        self.is_finished_exploring = False

    def odom_callback(self, msg: Odometry):
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
            return

        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().error(f'Could not transform: {ex}')
            return

    def occ_callback(self, msg):
        self.map = msg
        return

    @staticmethod
    def get_top_frontiers(frontiers, n):
        # Sort the frontiers by size in descending order
        sorted_frontiers = sorted(
            frontiers, key=lambda frontier: frontier.size, reverse=True
        )

        # Return the top n frontiers
        return sorted_frontiers[:n]

    def publish_cost_map(self, mapdata: OccupancyGrid, cost_map: np.ndarray):
        # Create an OccupancyGrid message
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()  # CHANGED: Use get_clock() now
        grid.header.frame_id = "map"
        grid.info.resolution = mapdata.info.resolution
        grid.info.width = cost_map.shape[1]
        grid.info.height = cost_map.shape[0]
        grid.info.origin = mapdata.info.origin

        # Normalize the cost map to the range [0, 100] and convert it to integers
        cost_map_normalized = (cost_map / np.max(cost_map) * 100).astype(np.int8)

        # Flatten the cost map and convert it to a list
        grid.data = cost_map_normalized.flatten().tolist()

        # Publish the OccupancyGrid message
        #self.cost_map_pub.publish(grid)

    def check_if_finished_exploring(self):
        # Publish empty path to stop the robot
        self.pure_pursuit_pub.publish(Path())

        # If no frontiers or paths are found for a certain number of times, finish exploring
        if (
            self.no_frontiers_found_counter >= self.NUM_EXPLORE_FAILS_BEFORE_FINISH
            or self.no_path_found_counter >= self.NUM_EXPLORE_FAILS_BEFORE_FINISH
        ):
            self.get_logger().info("Done exploring!")
            #self.save_map()
            self.get_logger().info("Saved map")
            self.is_finished_exploring = True

    def explore_frontier(self, frontier_list: FrontierList):
        # If finished exploring, no pose, no map, or no frontier list, return
        if self.is_finished_exploring or self.pose is None or self.map is None:
            return

        frontiers = frontier_list.frontiers

        # If no frontiers are found, check if finished exploring
        if not frontiers:
            self.get_logger().info.loginfo("No frontiers")
            self.no_frontiers_found_counter += 1
            self.check_if_finished_exploring()
            return
        else:
            self.no_frontiers_found_counter = 0

        A_STAR_COST_WEIGHT = 10.0
        FRONTIER_SIZE_COST_WEIGHT = 1.0

        # Calculate the C-space
        cspace, cspace_cells = PathPlanner.calc_cspace(self.map, self.is_in_debug_mode)
        # if cspace_cells is not None:
        #     self.cspace_pub.publish(cspace_cells)

        # Calculate the cost map
        cost_map = PathPlanner.calc_cost_map(self.map)
        if self.is_in_debug_mode:
            self.publish_cost_map(self.map, cost_map)

        # Get the start
        start = PathPlanner.world_to_grid(self.map, self.pose.position)

        # Execute A* for every frontier
        lowest_cost = float("inf")
        best_path = None

        # Check only the top frontiers in terms of size
        MAX_NUM_FRONTIERS_TO_CHECK = 8
        top_frontiers = FrontierExploration.get_top_frontiers(
            frontiers, MAX_NUM_FRONTIERS_TO_CHECK
        )

        starts = []
        goals = []

        # Log how many frontiers are being explored
        self.get_logger().info(f"Exploring {len(top_frontiers)} frontiers")

        for frontier in top_frontiers:
            # Get goal
            goal = PathPlanner.world_to_grid(self.map, frontier.centroid)

            # Execute A*
            path, a_star_cost, start, goal = PathPlanner.a_star(
                cspace, cost_map, start, goal
            )

            # If in debug mode, append start and goal
            if self.is_in_debug_mode:
                starts.append(start)
                goals.append(goal)

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

        # If in debug mode, publish the start and goal
        if self.is_in_debug_mode:
            self.start_pub.publish(PathPlanner.get_grid_cells(self.map, starts))
            self.goal_pub.publish(PathPlanner.get_grid_cells(self.map, goals))

        # If a path was found, publish it
        if best_path:
            self.get_logger().info(f"Found best path with cost {lowest_cost}")
            start = best_path[0]
            path = PathPlanner.path_to_message(self.map, best_path)
            self.pure_pursuit_pub.publish(path)
            self.no_path_found_counter = 0
        # If no path was found, check if finished exploring
        else:
            self.get_logger().info("No paths found")
            self.no_path_found_counter += 1
            self.check_if_finished_exploring()

 # CHANGED: Replaced the blocking while-loop in main() with a timer callback.
    def control_loop_callback(self):
        if self.pose is None or self.map is None:
            return
        # Get start cell
        start = PathPlanner.world_to_grid(self.map, self.pose.position)
        # Get frontiers using FrontierSearch
        frontier_list, frontier_cells = FrontierSearch.search(self.map, start, self.is_in_debug_mode)
        if frontier_list is None:
            return
        if self.is_in_debug_mode:
            self.frontier_cells_pub.publish(PathPlanner.get_grid_cells(self.map, frontier_cells))
        self.explore_frontier(frontier_list)

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExploration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
