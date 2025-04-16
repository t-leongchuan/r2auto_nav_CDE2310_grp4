"""
======================================================================
File: exploration.py
Author: Toh Leong Chuan
Date: 16/03/25

Description:
    This python file is an abstraction from the code/nodes used by the
    turtlebot to perform its exploration phase.

    The code/nodes used by the turtlebot to perform its exploration phase
    is based on pure_pursuit and relies on other nodes.

    Note that the comments here are more detailed than the relevant
    exploration code.

Modifications:
    - ROS2 Implementation was used instead of ROS1 Features
======================================================================
"""
from cde2310_interfaces.srv import ActivateNode
from cde2310_interfaces.srv import NodeFinish

import rclpy
from rclpy.node import Node

class Exploration(Node):
    
    def __init__(self):
        
        # ############# Exploration Node Initialization ############# #
        super().__init__('exploration')
        
        # ############# Server: Handle Activate Exploration Request ############# #
        self.ExplorationServer = self.create_service(ActivateNode, 'activate_exploration', self.activate_exploration_callback)

        # ############# Server: Handle Pure Pursuit's Complete Exploration ############# #
        self.PurePursuitStatusServer = self.create_service(NodeFinish, 'pure_pursuit_finish', self.pure_pursuit_complete_callback)

        # ############# Client: Notify pure_pursuit Node When to start Exploring ############# # 
        self.PurePursuitClient = self.create_client(ActivateNode, 'activate_pure_pursuit')
        while not self.PurePursuitClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service for activate_pure_pursuit not available, waiting again...')
        self.ActivatePurePursuit = ActivateNode.Request()
        
        # ############# Client: Notify Supervisor When Exploration Completes ############# #
        self.ExplorationStatusClient = self.create_client(NodeFinish, 'exploration_finish')
        while not self.ExplorationStatusClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service for exploration_finish not available, waiting again...')
        self.IsExplorationComplete = NodeFinish.Request()
    
    
    # ############# Client Call: Notify Supervisor About Exploration Completion ############# # [service: exploration_finish]
    def is_exploration_complete(self, finish):
        """ Sends a message to Supervisor if exploration is complete or still in progress. """
        self.IsExplorationComplete.finish = finish
        return self.ExplorationStatusClient.call_async(self.IsExplorationComplete) 
    
    # ############# Client Call: Notify pure_pursuit to start/stop exploring ############# # [service: activate_pure_pursuit]
    def activate_exploration(self, activate):
        self.ActivatePurePursuit.activate = activate
        return self.PurePursuitClient.call_async(self.ActivatePurePursuit)

    # ############# Server Response: Handle Pure Pursuit Completion from pure_pursuit ############# #
    def pure_pursuit_complete_callback(self, request, response):
        is_pure_pursuit_complete = request.finish

        if not is_pure_pursuit_complete:
            # Nothing is done here but relay information to supervisor (no logging here)
            response.message = "Exploration Node Acknowledges: Pure Pursuit is not Complete."
            # Relay information back to supervisor
            self.is_exploration_complete(is_pure_pursuit_complete)
            return response

        if is_pure_pursuit_complete:
            response.message = "Exploration Node Acknowledges: Pure Pursuit is Complete."
            self.is_exploration_complete(is_pure_pursuit_complete)
            return response

    
    # ############# Server Response: Handle Start/Stop Requests from Supervisor ############# #
    def activate_exploration_callback(self, request, response):
        activate = request.activate
        
        if not activate:
            self.get_logger().info('Supervisor has requested to **deactivate exploration**.')
            self.activate_exploration(activate)
            self.get_logger().info('Requested pure_pursuit to stop exploring.')
            response.message = "Exploration Node Acknowledges: Exploration is deactivated."
            return response
            
        if activate:
            self.get_logger().info('Supervisor has requested to **activate exploration**.')
            self.activate_exploration(activate)
            self.get_logger().info('Requested pure_pursuit to start exploring.')
            response.message = "Exploration Node Acknowledges: Exploration is activated."
            return response

def main():
    rclpy.init()
    exploration = Exploration()

    rclpy.spin(exploration)

    exploration.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
