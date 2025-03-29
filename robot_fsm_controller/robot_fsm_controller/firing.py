'''
TODO: 
The Code given below is a quick prototype as to how the firing node should look like
and should be used more as a reference/template rather than used.

The Code may have errors as it is written speculatively.
'''
import rclpy
from rclpy.node import Node
from cde2310_interfaces.srv import ActivateNode, NodeFinish

class Firing(Node):
    def __init__(self):
        super().__init__('firing')

        # Service server for receiving firing activation command from the supervisor.
        self.firing_activation_server = self.create_service(
            ActivateNode, 'activate_firing', self.activate_firing_callback
        )

        # Client to forward the firing command to the TurtleBot3's firing node.
        self.robot_fire_client = self.create_client(ActivateNode, 'robot_fire')
        while not self.robot_fire_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for robot_fire service...')

        # Service server for receiving the firing completion signal from the robot.
        self.robot_firing_finish_server = self.create_service(
            NodeFinish, 'firing_finish_robot', self.robot_firing_finish_callback
        )

        # Client to notify the supervisor that firing is complete.
        self.firing_finish_client = self.create_client(NodeFinish, 'firing_finish')
        while not self.firing_finish_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for firing_finish service from supervisor...')

    def activate_firing_callback(self, request, response):
        """
        Callback for the supervisor's firing activation command.
        Forwards the command to the robot's firing node.
        """
        if request.activate:
            self.get_logger().info('Received firing activation command from supervisor.')
            # Prepare the request for the robot's firing node.
            robot_request = ActivateNode.Request()
            robot_request.activate = True
            future = self.robot_fire_client.call_async(robot_request)
            # (Optional) Add a callback or wait for future.result() if needed.
            response.message = "Firing activation command forwarded to robot."
        else:
            self.get_logger().info('Received firing deactivation command from supervisor.')
            robot_request = ActivateNode.Request()
            robot_request.activate = False
            future = self.robot_fire_client.call_async(robot_request)
            response.message = "Firing deactivation command forwarded to robot."
        return response

    def robot_firing_finish_callback(self, request, response):
        """
        Callback for receiving the firing completion signal from the robot.
        Forwards the completion signal to the supervisor.
        """
        if request.finish:
            self.get_logger().info('Received firing completion signal from robot.')
            supervisor_request = NodeFinish.Request()
            supervisor_request.finish = True
            future = self.firing_finish_client.call_async(supervisor_request)
            # (Optional) Add a callback or wait for future.result() if needed.
            response.message = "Firing completion signal forwarded to supervisor."
        else:
            self.get_logger().info('Received firing not complete signal from robot.')
            supervisor_request = NodeFinish.Request()
            supervisor_request.finish = False
            future = self.firing_finish_client.call_async(supervisor_request)
            response.message = "Firing incomplete signal forwarded to supervisor."
        return response

def main(args=None):
    rclpy.init(args=args)
    firing_node = Firing()
    rclpy.spin(firing_node)
    firing_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
