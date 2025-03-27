import math
import rclpy
from rclpy.node import Node
import numpy as np
from path_planner import PathPlanner
from std_msgs.msg import Header, Bool
from nav_msgs.msg import Path, Odometry, GridCells, OccupancyGrid
from geometry_msgs.msg import Point, PointStamped, Twist, Vector3, Pose, Quaternion
from rclpy.qos import qos_profile_sensor_data

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.duration import Duration  # CHANGED: Import Duration from rclpy

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
    def __init__(self):
        super().__init__("pure_pursuit")

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.lookahead_pub = self.create_publisher(PointStamped, "pure_pursuit_lookahead", 10)

        # Subscribers
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(OccupancyGrid, 'map', self.occ_callback, qos_profile_sensor_data)
        self.create_subscription(Path, 'pure_pursuit_path', self.pursuit_path_callback, 10)
        # Optional: Uncomment if you plan to use the enabled topic
        # self.create_subscription(Bool, 'pure_pursuit_enabled', self.pursuit_enable_callback, 10)

        # CHANGED: Create a tf2 Buffer and Listener instead of using old tf imports.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Pure pursuit parameters
        self.LOOKAHEAD_DISTANCE = 0.18  # m
        self.WHEEL_BASE = 0.16          # m
        self.MAX_DRIVE_SPEED = 0.1      # m/s
        self.MAX_TURN_SPEED = 1.25      # rad/s
        self.TURN_SPEED_KP = 1.25
        self.DISTANCE_TOLERANCE = 0.1   # m

        # Obstacle avoidance parameters
        self.OBSTACLE_AVOIDANCE_GAIN = 0.3
        self.OBSTACLE_AVOIDANCE_MAX_SLOW_DOWN_DISTANCE = 0.16  # m
        self.OBSTACLE_AVOIDANCE_MIN_SLOW_DOWN_DISTANCE = 0.12  # m
        self.OBSTACLE_AVOIDANCE_MIN_SLOW_DOWN_FACTOR = 0.25
        self.FOV = 200              # degrees
        self.FOV_DISTANCE = 25      # number of grid cells
        self.FOV_DEADZONE = 80      # degrees
        self.SMALL_FOV = 300        # degrees
        self.SMALL_FOV_DISTANCE = 10  # number of grid cells

        self.pose = None
        self.map = None
        self.path = Path()
        self.alpha = 0.0
        self.enabled = True
        self.reversed = False
        self.closest_distance = float("inf")

        # CHANGED: Added timer callback to drive the control loop at 20Hz.
        self.timer = self.create_timer(0.05, self.control_loop_callback)

    def odom_callback(self, msg: Odometry):
        """
        Updates the current pose of the robot using tf2.
        """
        try:
            now = self.get_clock().now()
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

    def occ_callback(self, msg: OccupancyGrid):
        self.map = msg

    def pursuit_path_callback(self, msg: Path):
        self.path = msg

    # Optional: Uncomment if using an enabled topic
    # def pursuit_enable_callback(self, msg: Bool):
    #     self.enabled = msg.data

    def calculate_steering_adjustment(self) -> float:
        if self.pose is None or self.map is None:
            return 0

        orientation = self.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        yaw = float(np.rad2deg(yaw))
        robot_cell = PathPlanner.world_to_grid(self.map, self.pose.position)

        weighted_sum_of_angles = 0.0
        total_weight = 0.0
        self.closest_distance = float("inf")
        wall_cell_count = 0

        for dx in range(-self.FOV_DISTANCE, self.FOV_DISTANCE + 1):
            for dy in range(-self.FOV_DISTANCE, self.FOV_DISTANCE + 1):
                cell = (robot_cell[0] + dx, robot_cell[1] + dy)
                distance = PathPlanner.euclidean_distance(robot_cell, cell)
                if not PathPlanner.is_cell_in_bounds(self.map, cell):
                    continue
                is_wall = not PathPlanner.is_cell_walkable(self.map, cell)
                if is_wall and distance < self.closest_distance:
                    self.closest_distance = distance
                angle = float(np.rad2deg(np.arctan2(dy, dx))) - yaw
                if self.reversed:
                    angle += 180
                if angle < -180:
                    angle += 360
                elif angle > 180:
                    angle -= 360
                is_in_fov = (distance <= self.FOV_DISTANCE and -self.FOV/2 <= angle <= self.FOV/2 and abs(angle) >= self.FOV_DEADZONE/2)
                is_in_small_fov = (distance <= self.SMALL_FOV_DISTANCE and -self.SMALL_FOV/2 <= angle <= self.SMALL_FOV/2)
                if not (is_in_fov or is_in_small_fov):
                    continue
                if not is_wall:
                    continue
                weight = 1.0 / (distance**2) if distance != 0 else 0.0
                weighted_sum_of_angles += weight * angle
                total_weight += weight
                wall_cell_count += 1

        if total_weight == 0:
            return 0

        average_angle = weighted_sum_of_angles / total_weight
        steering_adjustment = -self.OBSTACLE_AVOIDANCE_GAIN * average_angle / wall_cell_count
        return steering_adjustment

    @staticmethod
    def distance(x0, y0, x1, y1) -> float:
        return math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)

    def get_distance_to_waypoint_index(self, i: int) -> float:
        if self.pose is None or not self.path.poses:
            return -1
        position = self.pose.position
        waypoint = self.path.poses[i].pose.position
        return PurePursuit.distance(position.x, position.y, waypoint.x, waypoint.y)

    def find_nearest_waypoint_index(self) -> int:
        nearest_index = -1
        if not self.path.poses:
            return nearest_index
        closest = float('inf')
        for i in range(len(self.path.poses) - 1):
            d = self.get_distance_to_waypoint_index(i)
            if d and d < closest:
                closest = d
                nearest_index = i
        return nearest_index

    def find_lookahead(self, nearest_index, lookahead_distance) -> Point:
        if not self.path.poses:
            return Point()
        i = nearest_index
        while i < len(self.path.poses) and self.get_distance_to_waypoint_index(i) < lookahead_distance:
            i += 1
        return self.path.poses[i - 1].pose.position

    def get_goal(self) -> Point:
        if not self.path.poses:
            return Point()
        return self.path.poses[-1].pose.position

    def send_speed(self, linear_speed: float, angular_speed: float):
        twist = Twist()
        twist.linear = Vector3(x=linear_speed)
        twist.angular = Vector3(z=angular_speed)
        self.cmd_vel_pub.publish(twist)

    def stop(self):
        self.send_speed(0.0, 0.0)

    # CHANGED: Added timer callback to replace the blocking while-loop in main()
    def control_loop_callback(self):
        if self.pose is None or self.map is None or not self.path.poses:
            return

        goal = self.get_goal()
        nearest_index = self.find_nearest_waypoint_index()
        lookahead = self.find_lookahead(nearest_index, self.LOOKAHEAD_DISTANCE)
        ps = PointStamped()
        hdr = Header()
        hdr.frame_id = "map"
        ps.header = hdr
        ps.point = lookahead
        self.lookahead_pub.publish(ps)

        orientation = self.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        yaw_deg = np.rad2deg(yaw)
        x = self.pose.position.x
        y = self.pose.position.y
        dx = lookahead.x - x
        dy = lookahead.y - y
        self.alpha = np.arctan2(dy, dx) - yaw
        if self.alpha > np.pi:
            self.alpha -= 2 * math.pi
        elif self.alpha < -np.pi:
            self.alpha += 2 * math.pi
        self.reversed = abs(self.alpha) > (np.pi / 2)
        lookahead_distance = PurePursuit.distance(x, y, lookahead.x, lookahead.y)
        try:
            radius_of_curvature = lookahead_distance / (2 * math.sin(self.alpha))
        except ZeroDivisionError:
            radius_of_curvature = float('inf')
        drive_speed = (-1 if self.reversed else 1) * self.MAX_DRIVE_SPEED
        distance_to_goal = PurePursuit.distance(x, y, goal.x, goal.y)
        if distance_to_goal < self.DISTANCE_TOLERANCE:
            self.stop()
            return
        turn_speed = self.TURN_SPEED_KP * drive_speed / radius_of_curvature
        turn_speed += self.calculate_steering_adjustment()
        turn_speed = max(-self.MAX_TURN_SPEED, min(self.MAX_TURN_SPEED, turn_speed))
        if self.closest_distance < self.OBSTACLE_AVOIDANCE_MAX_SLOW_DOWN_DISTANCE:
            drive_speed *= np.interp(
                self.closest_distance,
                [self.OBSTACLE_AVOIDANCE_MIN_SLOW_DOWN_DISTANCE, self.OBSTACLE_AVOIDANCE_MAX_SLOW_DOWN_DISTANCE],
                [self.OBSTACLE_AVOIDANCE_MIN_SLOW_DOWN_FACTOR, 1]
            )
        self.send_speed(drive_speed, turn_speed)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
