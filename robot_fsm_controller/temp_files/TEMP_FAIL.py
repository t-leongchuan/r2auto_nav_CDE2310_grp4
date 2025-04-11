"""
======================================================================
File: frontier_exploration_semi.py
Author: Toh Leong Chuan
Date: 16/03/25

Modified to call a Cartographer service to reset the mapping state when no 
further frontiers are detected.
======================================================================
"""

import numpy as np
import rclpy
import tf2_ros
import math
import time
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


from cartographer_ros_msgs.srv import FinishTrajectory, StartTrajectory  

# Global constants
TIMER_PERIOD = 0.05
NUM_EXPLORE_FAILS_BEFORE_FINISH = 50

A_STAR_COST_WEIGHT = 10.0
FRONTIER_SIZE_COST_WEIGHT = 1.0
MAX_NUM_FRONTIERS_TO_CHECK = 8

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
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
    return roll_x, pitch_y, yaw_z  # in radians


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
            OccupancyGrid, "map", self.occ_callback, qos_profile_sensor_data
        )

        self.cartographer_finish_client = self.create_client(FinishTrajectory, "finish_trajectory")  # NEW:
        self.cartographer_start_client = self.create_client(StartTrajectory, "start_trajectory")        # NEW:

        # tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timed callback
        self.timer = self.create_timer(TIMER_PERIOD, self.run_loop_callback)

        # State Variables
        self.pose = None
        self.map = None
        self.no_path_found_counter = 0
        self.no_frontiers_found_counter = 0
        self.is_finished_exploring = False

        # NEW: Create a client for the Cartographer reset service.
        self.cartographer_reset_client = self.create_client(FinishTrajectory, "finish_trajectory")

    def odom_callback(self, msg):
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                now,
                timeout=Duration(seconds=1.0)
            )
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

    @staticmethod
    def get_top_frontiers(frontiers, n):
        sorted_frontiers = sorted(
            frontiers, key=lambda frontier: frontier.size, reverse=True
        )
        return sorted_frontiers[:n]

    # NEW: Asynchronous reset_cartographer without blocking the main loop.
    def reset_cartographer(self):
        if not self.cartographer_reset_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Cartographer reset service not available.")
            return
        req = FinishTrajectory.Request()
        req.trajectory_id = 0  # Adjust this if your setup uses a different trajectory id.
        future = self.cartographer_reset_client.call_async(req)
        future.add_done_callback(self.reset_response_callback)

    # NEW: Callback to process the service response.
    def reset_response_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        else:
            # Here we assume the StatusResponse has a 'code' field (0 for success)
            # and a 'message' field. If your response doesn't have these fields,
            # simply log the response.status.
            if hasattr(response.status, 'code'):
                if response.status.code == 0:
                    self.get_logger().info("Cartographer map reset successful.")
                else:
                    self.get_logger().error(f"Cartographer map reset failed: {response.status.message}")
            else:
                self.get_logger().info(f"Cartographer map reset response: {response.status}")

# --- Add the following new functions to perform the finish/start sequence:
    def reset_and_restart_cartographer(self):  # NEW:
        # First, call finish_trajectory to finalize the current trajectory.
        if not self.cartographer_finish_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Cartographer finish_trajectory service not available.")
            return
        finish_req = FinishTrajectory.Request()
        finish_req.trajectory_id = 0  # Adjust trajectory id as needed.
        finish_future = self.cartographer_finish_client.call_async(finish_req)
        finish_future.add_done_callback(self.finish_response_callback)

    def finish_response_callback(self, future):  # NEW:
        try:
            finish_response = future.result()
        except Exception as e:
            self.get_logger().error(f"Finish trajectory service call failed: {e}")
            return
        else:
            # Log the response status. Adjust based on your StatusResponse fields.
            self.get_logger().info("Cartographer trajectory finished. Now starting new trajectory.")
            # Once finished, immediately call start_trajectory.
            self.call_start_trajectory()

    def call_start_trajectory(self):  # NEW:
        if not self.cartographer_start_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Cartographer start_trajectory service not available.")
            return
        start_req = StartTrajectory.Request()
        # Optionally, set start_req.initial_pose if required.
        start_future = self.cartographer_start_client.call_async(start_req)
        start_future.add_done_callback(self.start_response_callback)

    def start_response_callback(self, future):  # NEW:
        try:
            start_response = future.result()
        except Exception as e:
            self.get_logger().error(f"Start trajectory service call failed: {e}")
            return
        else:
            # Check the returned status. Depending on your Cartographer version,
            # the response might have a status field with a code/message.
            # Here we simply log that a new trajectory has started.
            self.get_logger().info("New trajectory started. Mapping has restarted.")



    def check_if_finished_exploring(self):  
        # Publish empty path to stop the robot momentarily
        self.pure_pursuit_pub.publish(Path())
        if (self.no_frontiers_found_counter >= NUM_EXPLORE_FAILS_BEFORE_FINISH or
            self.no_path_found_counter >= NUM_EXPLORE_FAILS_BEFORE_FINISH):
            self.get_logger().info("No further frontiers detected. Resetting Cartographer mapping state.")
            # Instead of manually resetting the map, call the reset-and-restart procedure.
            self.reset_and_restart_cartographer()  # NEW:
            # Reset the failure counters.
            self.no_frontiers_found_counter = 0
            self.no_path_found_counter = 0

    def explore_frontier(self, frontier_list: FrontierList):
        if self.is_finished_exploring or self.pose is None or self.map is None:
            return

        frontiers = frontier_list.frontiers
        if not frontiers:
            self.get_logger().warning("No Frontiers Found")
            self.no_frontiers_found_counter += 1
            self.get_logger().warning(f"Count Before Termination: {NUM_EXPLORE_FAILS_BEFORE_FINISH}")
            self.get_logger().warning(f"Current Count: {self.no_frontiers_found_counter}")
            self.check_if_finished_exploring()
            return
        else:
            self.no_frontiers_found_counter = 0

        c_space, cspace_cells = PathPlanner.calc_cspace(self.map, False)
        cost_map = PathPlanner.calc_cost_map(self.map)
        start = PathPlanner.world_to_grid(self.map, self.pose.position)
        lowest_cost = float("inf")
        best_path = None
        top_frontiers = FrontierExploration.get_top_frontiers(frontiers, MAX_NUM_FRONTIERS_TO_CHECK)
        for frontier in top_frontiers:
            goal = PathPlanner.world_to_grid(self.map, frontier.centroid)
            path, a_star_cost, start, goal = PathPlanner.a_star(c_space, cost_map, start, goal)
            if path is None or a_star_cost is None:
                continue
            cost = (A_STAR_COST_WEIGHT * a_star_cost) + (FRONTIER_SIZE_COST_WEIGHT / frontier.size)
            if cost < lowest_cost:
                lowest_cost = cost
                best_path = path
        if best_path:
            self.get_logger().info(f"Found best path with cost {lowest_cost}")
            path_msg = PathPlanner.path_to_message(self.map, best_path)
            self.pure_pursuit_pub.publish(path_msg)
            self.no_path_found_counter = 0
            return
        else:
            self.get_logger().info("No paths found")
            self.no_path_found_counter += 1
            self.check_if_finished_exploring()
            return

    def run_loop_callback(self):
        if self.pose is None or self.map is None:
            return

        start = PathPlanner.world_to_grid(self.map, self.pose.position)
        frontier_list, frontier_cells = FrontierSearch.search(self.map, start, False)
        if frontier_list is None:
            return
        self.explore_frontier(frontier_list)
        return


def main():
    rclpy.init()
    frontierExploration = FrontierExploration()
    rclpy.spin(frontierExploration)
    frontierExploration.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
