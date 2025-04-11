import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import numpy as np

class ThermalProcessor(Node):
    def __init__(self):
        super().__init__('thermal_processor')
        self.subscription = self.create_subscription(Float32MultiArray,'/heat_source_detected',self.listener_callback,10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.difference_list = []

    def format_as_8x8(self, pixels):
        return np.array(pixels).reshape(8, 8)

    def alignment(self, pixels_array):
        left = [[pixels_array[j][i] for i in range(4)] for j in range(8)]
        right = [[pixels_array[j][i] for i in range(4, 8)] for j in range(8)]
        left_sum = sum(sum(row) for row in left)
        right_sum = sum(sum(row) for row in right)
        return abs(left_sum - right_sum), left_sum > right_sum

    def listener_callback(self, msg):
        pixel_array = self.format_as_8x8(msg.data)
        difference, left_greater = self.alignment(pixel_array)
        self.difference_list.append(difference)

        twist = Twist()
        if min(self.difference_list) == difference:
            twist.linear.x = 0.2  # Forward speed (adjust as needed)
            twist.angular.z = 0.0
        elif left_greater:
            twist.linear.x = 0.0
            twist.angular.z = 0.3  # Angular speed (adjust as needed)
        else:
            twist.linear.x = 0.0
            twist.angular.z = -0.3  # Angular speed (adjust as needed)

        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"Difference: {difference}, Published cmd_vel: linear.x={twist.linear.x}, angular.z={twist.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    thermal_processor = ThermalProcessor()
    try:
        rclpy.spin(thermal_processor)
    except KeyboardInterrupt:
        pass
    finally:
        thermal_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
