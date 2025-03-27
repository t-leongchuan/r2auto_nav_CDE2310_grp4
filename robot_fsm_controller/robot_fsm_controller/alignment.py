from cde2310_interfaces.srv import ActivateNode
from cde2310_interfaces.srv import NodeFinish

import rclpy
from rclpy.node import Node

class Alignment(Node):
    
    def __init__(self):
        
        # ############# Alignment Node Initialization ############# #
        super().__init__('alignment')
        
        # ############# Server: Handle Activate Alignment Request ############# #
        self.AlignmentServer = self.create_service(ActivateNode, 'activate_alignment', self.activate_alignment)
        
        # ############# Client: Notify Supervisor When Alignment Completes ############# #
        self.AlignmentStatusClient = self.create_client(NodeFinish, 'alignment_finish')
        while not self.AlignmentStatusClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.IsAlignmentComplete = NodeFinish.Request()
    
    # ############# Client Call: Notify Supervisor About Exploration Completion ############# #
    def is_alignment_complete(self, activate):
        """ Sends a message to Supervisor if exploration is complete or still in progress. """
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

def main():
    rclpy.init()
    alignment = Alignment()

    rclpy.spin(alignment)

    alignment.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()