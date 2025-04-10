"""
======================================================================
File: pure_pursuit.py
Author: Toh Leong Chuan
Date: 16/03/25

This file is based on original code and ideas by:
    Kai Nakamura
    https://kainakamura.com/project/slam-robot

Description:
    ROS2 Node that implements pure pursuit technique to achieve navigation,
    using paths generated from frontier_exploration.py

Modifications:
    - ROS2 Implementation was used instead of ROS1 Features
======================================================================
"""

import math
import rclpy
import numpy as np
import tf2_ros 
from rclpy.qos import qos_profile_sensor_data
from .path_planner import PathPlanner
from std_msgs.msg import Header, Bool
from nav_msgs.msg import Path, Odometry, GridCells, OccupancyGrid
from geometry_msgs.msg import Point, PointStamped, Twist, Vector3, Pose, Quaternion
from rclpy.node import Node
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.duration import Duration 

from cde2310_interfaces.srv import NodeFinish
from cde2310_interfaces.srv import ActivateNode

##################################################################
# ==================================================================
# Constants for path following and obstacle avoidance
# ==================================================================

# Timer callback interval (in seconds)
TIMER_PERIOD = 0.05

# Lookahead distance to next waypoint (in meters)
# Effect: Shorter = more responsive, safer in tight spaces
LOOKAHEAD_DISTANCE = 0.15

# Distance between front and rear wheels (not currently used)
WHEEL_BASE = 0.16

# Max linear drive speed (m/s)
# Effect: Lower speed = more control, less momentum during turns
MAX_DRIVE_SPEED = 0.08

# Max angular speed (rad/s)
MAX_TURN_SPEED = 1.25

# Proportional gain for turn rate
TURN_SPEED_KP = 1.25

# Goal tolerance in meters
# Effect: Stops robot if this close to final waypoint
DISTANCE_TOLERANCE = 0.05

# How aggressively to avoid walls
# Effect: Higher = sharper turning away from obstacles
OBSTACLE_AVOIDANCE_GAIN = 0.5

# Distance to begin slowing down when near obstacles (m)
# Effect: Starts soft braking earlier to prevent bumps
OBSTACLE_AVOIDANCE_MAX_SLOW_DOWN_DISTANCE = 0.16

# Minimum distance to trigger max slow down (m)
OBSTACLE_AVOIDANCE_MIN_SLOW_DOWN_DISTANCE = 0.15

# Minimum factor to scale drive speed during slow-down (0.0 to 1.0)
OBSTACLE_AVOIDANCE_MIN_SLOW_DOWN_FACTOR = 0.25

# Field of View for obstacle checking (degrees)
FOV = 200

# Forward distance to check for obstacles (in grid cells)
FOV_DISTANCE = 25

# Region in front of robot to ignore for obstacle avoidance (degrees)
FOV_DEADZONE = 80

# Secondary wide FOV when close to objects (degrees)
SMALL_FOV = 300

# Range for secondary FOV (in grid cells)
SMALL_FOV_DISTANCE = 10
##################################################################


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

class PurePursuit(Node):
    
    # TODO: Decide if there's a new for additional debugging.
    def __init__(self):
        super().__init__("pure_pursuit")

        # Server: Handle Start/Stop Pure Pursuit
        self.PurePursuitServer = self.create_service(ActivateNode, 'activate_pure_pursuit', self.activate_pure_pursuit_callback)

        # # Client: Inform Exploration Node If Pure Pursuit Complete
        # self.PurePursuitStatusClient = self.create_client(NodeFinish, 'pure_pursuit_finish')
        # while not self.PurePursuitStatusClient.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info("Service for pure_pursuit_finish not available, waiting again...")
        # self.isPurePursuitComplete = NodeFinish.Request()

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, "cmd_vel", 10
        )

        self.lookahead_pub = self.create_publisher(
            PointStamped, "pure_pursuit_lookahead", 10
        )

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )

        self.occ_grid_sub = self.create_subscription(
            OccupancyGrid, "map", self.occ_grid_callback, qos_profile_sensor_data
        )

        self.pure_pursuit_path_sub = self.create_subscription(
            Path, "pure_pursuit_path", self.pure_pursuit_path_callback, 10
        )

        # tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timed callback to run primary functions
        self.timer = self.create_timer(TIMER_PERIOD, self.run_loop_callback)

        # State Variables
        self.pose = None
        self.map = None
        self.path = Path()
        self.alpha = 0
        self.enabled = True
        self.reversed = False
        self.closest_distance = float("inf")
    
    
    # ############# Server Response: Handle Start/Stop Requests from Exploration Node ############# #
    def activate_pure_pursuit_callback(self, request, response):
        activate = request.activate

        if not activate:
            self.get_logger().info('Exploration Node has requested to **deactivate exploration**')
            self.enabled = False
            self.get_logger().info('Disabled Pure Pursuit.')
            response.message = "Disabled Pure Pursuit"
            return response
        
        if activate:
            self.get_logger().info('Exploration Node has requested to **activate exploration**')
            self.enabled = True
            self.get_logger().info('Enabled Pure Pursuit.')
            response.message = "Enabled Pure Pursuit"
            return response
    
    # ############# Client Call: Notify Exploration Node About Pure Pursuit Completion ############# # [service: pure_pursuit_finish]
    # def pure_pursuit_complete(self, finish):
    #     """ Sends a message to Exploration Node if pure pursuit is complete"""
    #     self.isPurePursuitComplete.finish = finish
    #     return self.PurePursuitStatusClient.call_async(self.isPurePursuitComplete)

    # Main Function
    def run_loop_callback(self):
        
        if self.pose is None:
            return

        # If not enabled, do nothing
        if not self.enabled:
            self.stop()
            return

        # If no path, stop
        if self.path is None or not self.path.poses:
            self.stop()
            #self.pure_pursuit_complete(True)
            return

        goal = self.get_goal()

        nearest_waypoint_index = self.find_nearest_waypoint_index()
        lookahead = self.find_lookahead(
            nearest_waypoint_index, LOOKAHEAD_DISTANCE
        )

        self.lookahead_pub.publish(
            PointStamped(header=Header(frame_id="map"), point=lookahead)
        )

        # Calculate alpha (angle between target and current position)
        position = self.pose.position
        orientation = self.pose.orientation
        roll, pitch, yaw = euler_from_quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        x = position.x
        y = position.y
        dx = lookahead.x - x
        dy = lookahead.y - y
        self.alpha = float(np.arctan2(dy, dx) - yaw)
        if self.alpha > np.pi:
            self.alpha -= 2 * np.pi
        elif self.alpha < -np.pi:
            self.alpha += 2 * np.pi

        # If the lookahead is behind the robot, follow the path backwards
        # Disable backwards navigation altogether
        #self.reversed = abs(self.alpha) > np.pi / 2

        # Calculate the lookahead distance and center of curvature
        lookahead_distance = PurePursuit.distance(x, y, lookahead.x, lookahead.y)
        radius_of_curvature = float(lookahead_distance / (2 * np.sin(self.alpha)))

        # Calculate drive speed
        drive_speed = (-1 if self.reversed else 1) * MAX_DRIVE_SPEED

        # Stop if at goal
        distance_to_goal = PurePursuit.distance(x, y, goal.x, goal.y)
        if distance_to_goal < DISTANCE_TOLERANCE:
            self.stop()
            return

        # Calculate turn speed
        turn_speed = TURN_SPEED_KP * drive_speed / radius_of_curvature

        # Obstacle avoicance
        turn_speed += self.calculate_steering_adjustment()

        # Clamp turn speed
        turn_speed = max(-MAX_TURN_SPEED, min(MAX_TURN_SPEED, turn_speed))

        # Slow down if close to obstacle
        if self.closest_distance < OBSTACLE_AVOIDANCE_MAX_SLOW_DOWN_DISTANCE:
            drive_speed *= float(
                np.interp(
                    self.closest_distance,
                    [
                        OBSTACLE_AVOIDANCE_MIN_SLOW_DOWN_DISTANCE,
                        OBSTACLE_AVOIDANCE_MAX_SLOW_DOWN_DISTANCE,
                    ],
                    [OBSTACLE_AVOIDANCE_MIN_SLOW_DOWN_FACTOR, 1],
                )
            )

        # Send speed
        self.send_speed(drive_speed, turn_speed)

    # Callback Functions
    """
    Updates the current map.
    """
    def odom_callback(self, msg: Odometry):
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


    def occ_grid_callback(self, msg: OccupancyGrid):
        self.map = msg
        return
    
    def pure_pursuit_path_callback(self, msg: Path):
        self.path = msg
        return

    def calculate_steering_adjustment(self) -> float:
        if self.pose is None or self.map is None:
            return 0

        orientation = self.pose.orientation
        roll, pitch, yaw = euler_from_quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w
        )

        yaw = float(np.rad2deg(yaw))

        # Get the grid cell of the robot
        robot_cell = PathPlanner.world_to_grid(self.map, self.pose.position)

        weighted_sum_of_angles = 0
        total_weight = 0
        self.closest_distance = float("inf")

        # Get all wall cells near the robot within the distance
        fov_cells = []
        wall_cells = []
        wall_cell_count = 0
        for dx in range(-FOV_DISTANCE, FOV_DISTANCE + 1):
            for dy in range(-FOV_DISTANCE, FOV_DISTANCE + 1):
                cell = (robot_cell[0] + dx, robot_cell[1] + dy)
                distance = PathPlanner.euclidean_distance(robot_cell, cell)

                # If the cell is out of bounds, ignore it
                if not PathPlanner.is_cell_in_bounds(self.map, cell):
                    continue

                is_wall = not PathPlanner.is_cell_walkable(self.map, cell)
                if is_wall and distance < self.closest_distance:
                    self.closest_distance = distance

                # Calculate the angle of the cell relative to the robot
                angle = float(np.rad2deg(np.arctan2(dy, dx))) - yaw

                # If reversed, add 180 to the angle
                if self.reversed:
                    angle += 180

                # Keep angle in the range of -180 to 180
                if angle < -180:
                    angle += 360
                elif angle > 180:
                    angle -= 360

                # Ignore scans that are outside the field of view
                is_in_fov = (
                    distance <= FOV_DISTANCE
                    and angle >= -FOV / 2
                    and angle <= FOV / 2
                    and not abs(angle) < FOV_DEADZONE / 2
                )
                is_in_small_fov = (
                    distance <= SMALL_FOV_DISTANCE
                    and angle >= -SMALL_FOV / 2
                    and angle <= SMALL_FOV / 2
                )
                if not is_in_fov and not is_in_small_fov:
                    continue
                
                # If cell is not a wall, ignore it
                if not is_wall:
                    continue

                weight = 1 / (distance**2) if distance != 0 else 0

                weighted_sum_of_angles += weight * angle
                total_weight += weight

                wall_cell_count += 1

        if total_weight == 0:
            return 0

        # Calculate the average angle (weighted sum of angles divided by total weight)
        average_angle = weighted_sum_of_angles / total_weight

        # Calculate the steering adjustment based on the average angle
        steering_adjustment = (
            -OBSTACLE_AVOIDANCE_GAIN * average_angle / wall_cell_count
        )
        return steering_adjustment

    @staticmethod
    def distance(x0, y0, x1, y1) -> float:
        return math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)

    def get_distance_to_waypoint_index(self, i: int) -> float:
        if self.pose is None or self.path.poses is None:
            return -1

        position = self.pose.position
        waypoint = self.path.poses[i].pose.position
        return PurePursuit.distance(position.x, position.y, waypoint.x, waypoint.y)

    def find_nearest_waypoint_index(self) -> int:
        nearest_waypoint_index = -1
        if self.path.poses is None:
            return nearest_waypoint_index

        closest_distance = float("inf")
        for i in range(len(self.path.poses) - 1):
            distance = self.get_distance_to_waypoint_index(i)
            if distance and distance < closest_distance:
                closest_distance = distance
                nearest_waypoint_index = i
        return nearest_waypoint_index

    def find_lookahead(self, nearest_waypoint_index, lookahead_distance) -> Point:
        # If there is no path or no poses, return a default Point.
        if self.path is None or not self.path.poses:
            return Point()

        # If nearest_waypoint_index is negative, default to the first pose.
        if nearest_waypoint_index < 0:
            nearest_waypoint_index = 0

        i = nearest_waypoint_index
        while i < len(self.path.poses) and self.get_distance_to_waypoint_index(i) < lookahead_distance:
            i += 1
        # If i is zero, return the first pose; else return the previous pose.
        if i == 0:
            return self.path.poses[0].pose.position
        return self.path.poses[i - 1].pose.position

    def get_goal(self) -> Point:
        if self.path.poses is None:
            return Point()

        poses = self.path.poses
        return poses[len(poses) - 1].pose.position

    def send_speed(self, linear_speed: float, angular_speed: float):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        twist = Twist(linear=Vector3(x=linear_speed), angular=Vector3(z=angular_speed))
        self.cmd_vel_pub.publish(twist)

    def stop(self):
        self.send_speed(0.0, 0.0)

def main():
    rclpy.init()

    purePursuit = PurePursuit()
    rclpy.spin(purePursuit)
    PurePursuit.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
                    


        

        


    
    