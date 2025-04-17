'''
Node that processes information from thermal sensor and determines if the target should be pursued.
'''
import numpy as np
from cde2310_interfaces.srv import ActivateNode
from cde2310_interfaces.srv import NodeFinish
from std_msgs.msg import Float32MultiArray
from .path_planner import PathPlanner
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.qos import qos_profile_sensor_data

import rclpy
from rclpy.node import Node
from enum import Enum
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.duration import Duration

SEARCH_RADIUS = 2.0  # Arbitrary value; consider the resolution of the occupancy grid.
TEMPERATURE_THRESHOLD = 27  # Arbitrary value

class ThermalTarget(Node):

    def __init__(self):
        super().__init__('thermal_target')

        # Thermal Sensor Data Subscription
        self.thermal_sub = self.create_subscription(
            Float32MultiArray, '/heat_source_detected', self.thermal_sub_callback, 10
        )

        # Odom Subscription
        self.odom_sub = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )

        # Occ Grid Subscription (use a separate variable name)
        self.map_sub = self.create_subscription(
            OccupancyGrid, "map", self.occ_grid_callback, qos_profile_sensor_data
        )

        # Thermal Node State Variables
        self.FIRED_COORDINATES = []  # List of grid cells where a target has already been fired
        self.thermal_array = None
        self.map = None
        self.pixel_array = None
        self.pose = None

        # tf2 buffer and listener for transform lookup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Thermal Sensor Client Creation: forwards to supervisor node to activate alignment
        self.ThermalTargetClient = self.create_client(ActivateNode, 'target_detected')
        while not self.ThermalTargetClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Thermal Target Service not available, waiting again...')
        self.TargetDetected = ActivateNode.Request()

    def target_detected(self, activate, map_msg):
        self.TargetDetected.activate = activate
        # Get the robot's current grid cell and record it as a fired coordinate.
        robot_cell = PathPlanner.world_to_grid(self.map, self.pose.position)
        self.FIRED_COORDINATES.append(robot_cell)
        return self.ThermalTargetClient.call_async(self.TargetDetected)

    def is_new_target(self):
        """
        Determines if the current target (based on the robot's current grid cell)
        is new—that is, not within SEARCH_RADIUS of any previously fired target.
        """
        if self.map is None or self.pose is None:
            return False  # Without a map or pose, we cannot decide.
        if not self.FIRED_COORDINATES:
            return True
        current_cell = PathPlanner.world_to_grid(self.map, self.pose.position)
        for fired in self.FIRED_COORDINATES:
            if self.within_range(current_cell, fired):
                return False
        return True

    def within_range(self, cell1, cell2):
        # Use the euclidean distance (in grid cell units) to check if cell1 is within SEARCH_RADIUS of cell2.
        return PathPlanner.euclidean_distance(cell1, cell2) <= SEARCH_RADIUS

    def thermal_sub_callback(self, msg):
        # Convert the incoming data to an 8x8 array.
        self.pixel_array = self.format_as_8x8(msg.data)
        # Check the region of interest (rows 4-7 and columns 4-7) for a high temperature.
        if np.any(self.pixel_array[4:8, 4:8] >= TEMPERATURE_THRESHOLD):
            # Only trigger a target if it's new.
            if self.is_new_target():
                self.target_detected(True, self.map)
        return

    def odom_callback(self, msg):
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                'map', 
                'base_footprint', 
                now, 
                timeout=Duration(seconds=1.0)
            )
            # Update self.pose using the transform data.
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

    def occ_grid_callback(self, msg):
        self.map = msg

    def format_as_8x8(self, pixels):
        return np.array(pixels).reshape(8, 8)


def main():
    rclpy.init()
    thermal_target = ThermalTarget()
    rclpy.spin(thermal_target)
    thermal_target.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
