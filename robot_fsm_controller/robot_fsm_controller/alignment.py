from cde2310_interfaces.srv import ActivateNode
from cde2310_interfaces.srv import NodeFinish

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

class Alignment(Node):
    
    def __init__(self):
        
        # ############# Alignment Node Initialization ############# #
        super().__init__('alignment')

        # ############# Subscription to Thermal Sensor Readings ############# #
        self.thermal_sub = self.create_subscription(Float32MultiArray,'/heat_source_detected', self.thermal_sub_callback, 10)

        # ############# Publisher for robot movement Readings ############# #
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # ############# Server: Handle Activate Alignment Request ############# #
        self.AlignmentServer = self.create_service(ActivateNode, 'activate_alignment', self.activate_alignment_callback)
        
        # ############# Client: Notify Supervisor When Alignment Completes ############# #
        self.AlignmentStatusClient = self.create_client(NodeFinish, 'alignment_finish')
        while not self.AlignmentStatusClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.IsAlignmentComplete = NodeFinish.Request()

        # State variables
        self.difference_list = []

    # ############# Thermal Sensor Callback ############# #
    def thermal_sub_callback(self, msg):
        pixel_array = self.format_as_8x8(msg.data)
        difference, direction = self.alignment(pixel_array, threshold=5)
        self.difference_list.append(difference)

        twist = Twist()

        # Define a threshold below which alignment is considered complete.
        ALIGNMENT_COMPLETE_THRESHOLD = 5

        if difference < ALIGNMENT_COMPLETE_THRESHOLD:
            # Alignment is achieved; stop movement and notify supervisor.
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.difference_list = []
            self.is_alignment_complete(True)  # Call your service to indicate completion.
        else:
            # If the current difference is the smallest observed, drive forward.
            if min(self.difference_list) == difference:
                twist.linear.x = 0.2
                twist.angular.z = 0.0
            else:
                # Adjust turning based on the direction.
                if direction == 1:  # left is hotter; turn right.
                    twist.linear.x = 0.0
                    twist.angular.z = 0.3
                elif direction == -1:  # right is hotter; turn left.
                    twist.linear.x = 0.0
                    twist.angular.z = -0.3

        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(
            f"Difference: {difference}, Direction: {direction}, cmd_vel: linear.x={twist.linear.x}, angular.z={twist.angular.z}"
        )


        
    # ############# Client Call: Notify Supervisor About alignment Completion ############# #
    def is_alignment_complete(self, activate):
        """ Sends a message to Supervisor if alignment is complete or still in progress. """
        self.IsAlignmentComplete.activate = activate
        return self.AlignmentStatusClient.call_async(self.IsAlignmentComplete) 
    
    # ############# Server Response: Handle Start/Stop Requests from Supervisor ############# #
    def activate_alignment_callback(self, request, response):
        activate = request.activate
        
        if not activate:
            self.get_logger().info('Supervisor has requested to **deactivate alignment**.')
            self.get_logger().info('Simulating Alignment Deactivation...')
            response.message = "Alignment Node Acknowledges: Alignment is deactivated."
            return response
            
        if activate:
            self.get_logger().info('Supervisor has requested to **activate alignment**.')
            self.get_logger().info('Simulating alignment...')
            response.message = "Alignment Node Acknowledges: Alignment is activated."
            return response

    # ############# Helper Function ############# #
    def alignment(self, pixels_array, threshold=5):
        """
        Computes the difference in thermal values between the left and right halves of an 8x8 pixel array.
        
        Args:
            pixels_array: An 8x8 array (or list) of thermal values.
            threshold: A value below which the difference is considered negligible.
            
        Returns:
            A tuple (abs_difference, direction) where:
                - abs_difference is the absolute difference between left and right sums.
                - direction is 1 if left is significantly hotter, -1 if right is significantly hotter,
                and 0 if the difference is within the threshold.
        """
        # Ensure pixels_array is a NumPy array.
        pixels = np.array(pixels_array)
        # Sum over columns: left half (columns 0 to 3) and right half (columns 4 to 7)
        left_sum = np.sum(pixels[:, :4])
        right_sum = np.sum(pixels[:, 4:])
        
        diff = left_sum - right_sum
        abs_diff = abs(diff)
        
        if abs_diff < threshold:
            direction = 0  # Considered aligned.
        elif diff > 0:
            direction = 1  # Left is hotter.
        else:
            direction = -1  # Right is hotter.
        
        return abs_diff, direction


    def format_as_8x8(self, pixels):
        return np.array(pixels).reshape(8, 8)


def main():
    rclpy.init()
    alignment = Alignment()

    rclpy.spin(alignment)

    alignment.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()