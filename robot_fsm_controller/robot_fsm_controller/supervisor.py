from cde2310_interfaces.srv import ActivateNode
from cde2310_interfaces.srv import NodeFinish

import rclpy
from rclpy.node import Node
from enum import Enum


class State(Enum):
    EXPLORATION = 1
    ALIGNMENT = 2
    FIRING = 3
    POST_FIRING_RECOVERY = 4

class Supervisor(Node):
    
    def __init__(self):
        
        # ############# Supervisor Node Initialization ############# #
        super().__init__('supervisor')
        self.state = State.EXPLORATION  # Initial FSM state
    

        # ############# EXPLORATION Client Creation: Activate Exploration ############# #
        self.ExplorationClient = self.create_client(ActivateNode, 'activate_exploration')
        while not self.ExplorationClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('exploration service not available, waiting again...')
        self.ActivateExploration = ActivateNode.Request()
        
        # ############# EXPLORATION Server Creation: Receive Exploration Completion Status ############# #
        self.ExplorationStatusServer = self.create_service(NodeFinish, 'exploration_finish', self.exploration_status_callback)
        
        # ############# ALIGNMENT Client Creation: Activate Alignment ############# #
        self.AlignmentClient = self.create_client(ActivateNode, 'activate_alignment')
        # while not self.AlignmentClient.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('alignment service not available, waiting again...')
        self.ActivateAlignment = ActivateNode.Request()
        
        # ############# ALIGNMENT Server Creation: Receive Alignment Completion Status ############# #
        self.AlignmentStatusServer = self.create_service(NodeFinish, 'alignment_finish', self.alignment_status_callback)
        
        return
    
    
    
    # ############# EXPLORATION Client Call: Request to Start/Stop Exploration ############# #
    def toggle_exploration(self, activate):
        self.ActivateExploration.activate = activate
        return self.ExplorationClient.call_async(self.ActivateExploration)
    
    # ############# EXPLORATION Server Response: Handle Exploration Completion Feedback ############# #
    def exploration_status_callback(self, request, response):
        finish_exploration = request.finish
        
        if not finish_exploration:
            self.get_logger().info('Exploration Node Has Declared Exploration Not Complete.')
            self.get_logger().info('Simulating Checks to see If mission has ended...')
            self.get_logger().info('If all targets achieved, proceed to prepare shutdown')
            self.get_logger().info('Else Continue Exploration.')
            response.message = "Supervisor Acknowledges Exploration Is not Finished."
            return response
            
        if finish_exploration:
            self.get_logger().info('Exploration Node Has Declare Exploration Complete.')
            self.get_logger().info('Simulating Checks to see if mission has ended....')
            self.get_logger().info('If all targets achieved, proceed to prepare shutdown')
            self.get_logger().info('Else transit to rudimentary wall-following to detect remaining targets as fallback')
            response.message = "Supervisor Acknowledges Exploration Is Finished."
            return response
        
    # ############# ALIGNMENT Client Call: Request to Start/Stop Alignment ############# #
    def toggle_alignment(self, activate):
        self.ActivateAlignment.activate = activate
        return self.AlignmentClient.call_async(self.ActivateExploration)
    
    # ############# ALIGNMENT Server Response: Handle Alignment Completion Feedback ############# #
    def alignment_status_callback(self, request, response):
        activate = request.activate
        
        if not activate:
            self.get_logger().info('Alignment Node Has Declared Alignment Not Complete.')
            response.message = "Supervisor Acknowledges Alignment Is not Finished."
            return response
            
        if activate:
            self.get_logger().info('Alignment Node Has Declared Alignment Complete.')
            self.get_logger().info('Simulating A Transition to Firing State....')
            response.message = "Supervisor Acknowledges Alignment Is Finished."
            return response

def main():
    rclpy.init()
    supervisor = Supervisor()
   
    rclpy.spin(supervisor)
    
    supervisor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
