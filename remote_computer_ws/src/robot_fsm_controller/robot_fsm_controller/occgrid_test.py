import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PointStamped, Twist, Vector3, Pose, Quaternion
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
from rclpy.duration import Duration

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class Test(Node):

    def __init__(self):
        super().__init__('occupancy_test')

        ## Internal Variables ##
        self.map = None
        self.pose = None

        ## tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Create Subscriber to keep track of occupancy
        self.occupancy_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data
        )

        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
    
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
        
        if self.pose is None or self.map is None:
            self.get_logger().error('map info not available yet')
            return
        
        robot_cell = Test.world_to_grid(self.map, self.pose.position)
        
        self.get_logger().error(f'testing location: {robot_cell}')
        return
    
    def grid_to_index(mapdata: OccupancyGrid, p: "tuple[int, int]") -> int:
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param p [(int, int)] The cell coordinate.
        :return  [int] The index.
        """
        return p[1] * mapdata.info.width + p[0]

    @staticmethod
    def get_cell_value(mapdata: OccupancyGrid, p: "tuple[int, int]") -> int:
        """
        Returns the cell corresponding to the given (x,y) coordinates in the occupancy grid.
        :param p [(int, int)] The cell coordinate.
        :return  [int] The cell.
        """
        return mapdata.data[PathPlanner.grid_to_index(mapdata, p)]

    @staticmethod
    def euclidean_distance(
        p1: "tuple[float, float]", p2: "tuple[float, float]"
    ) -> float:
        """
        Calculates the Euclidean distance between two points.
        :param p1 [(float, float)] first point.
        :param p2 [(float, float)] second point.
        :return   [float]          distance.
        """
        return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

    @staticmethod
    def grid_to_world(mapdata: OccupancyGrid, p: "tuple[int, int]") -> Point:
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param p [(int, int)] The cell coordinate.
        :return        [Point]         The position in the world.
        """
        x = (p[0] + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.x
        y = (p[1] + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.y
        return Point(x, y, 0)

    @staticmethod
    def world_to_grid(mapdata: OccupancyGrid, wp: Point) -> "tuple[int, int]":
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        x = int((wp.x - mapdata.info.origin.position.x) / mapdata.info.resolution)
        y = int((wp.y - mapdata.info.origin.position.y) / mapdata.info.resolution)
        return (x, y)

def main(args=None):
    rclpy.init(args=args)

    test = Test()
    rclpy.spin(test)

    test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
