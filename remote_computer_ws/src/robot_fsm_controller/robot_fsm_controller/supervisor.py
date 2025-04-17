from cde2310_interfaces.srv import ActivateNode
from cde2310_interfaces.srv import NodeFinish

import rclpy
from rclpy.node import Node
from enum import Enum

NUM_TARGETS = 3
TIMER_PERIOD = 0.05 # interval between running main logic

class State(Enum):
    EXPLORATION = 1
    ALIGNMENT = 2
    FIRING = 3
    POST_FIRING_RECOVERY = 4 # Not Used For Now

class Supervisor(Node):
    
    def __init__(self):

        # ############# Supervisor State Variables ############# #
        # Capitalized to make it easier for me to read
        self.COMPLETED_TARGETS = 0
        self.IS_MISSION_COMPLETE = False
        
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

        # ############# THERMAL SENSOR Server Creation: Notify New Target Found ############# #
        self.ThermalTargetServer = self.create_service(ActivateNode, 'target_detected', self.thermal_target_callback)
        
        
        # ############# ALIGNMENT Client Creation: Activate Alignment ############# #
        self.AlignmentClient = self.create_client(ActivateNode, 'activate_alignment')
        while not self.AlignmentClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('alignment service not available, waiting again...')
        self.ActivateAlignment = ActivateNode.Request()
        
        # ############# ALIGNMENT Server Creation: Receive Alignment Completion Status ############# #
        self.AlignmentStatusServer = self.create_service(NodeFinish, 'alignment_finish', self.alignment_status_callback)
        

        ############# FIRING Client Creation: Activate Firing ############# #
        self.FiringClient = self.create_client(ActivateNode, 'activate_firing')
        while not self.FiringClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('firing service not available, waiting again...')
        self.ActivateFiring = ActivateNode.Request()

        # # ############# FIRING Server Creation: Receive Exploration Completion Status ############# #
        self.FiringStatusServer = self.create_service(NodeFinish, 'firing_finish', self.firing_status_callback)

        # ############# Timed callback to run primary functions ############# #
        self.timer = self.create_timer(TIMER_PERIOD, self.run_main_loop_callback)
        
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
    
    # Alignment Not Used
    # ############# ALIGNMENT Client Call: Request to Start/Stop Alignment ############# #
    def toggle_alignment(self, activate):
        self.ActivateAlignment.activate = activate
        self.state = State.Alignment
        return self.AlignmentClient.call_async(self.ActivateAlignment)

    
    # ############# ALIGNMENT Server Response: Handle Alignment Completion Feedback ############# #
    def alignment_status_callback(self, request, response):
        activate = request.activate
        
        if not activate:
            self.get_logger().info('Alignment Node Has Declared Alignment Not Complete.')
            response.message = "Supervisor Acknowledges Alignment Is not Finished. Reactivating Exploration."
            self.state = State.Exploration
            self.toggle_exploration(True)
            return response
            
        if activate:
            self.get_logger().info('Alignment Node Has Declared Alignment Complete.')
            self.toggle_firing(True)
            response.message = "Supervisor Acknowledges Alignment Is Finished."
            return response



    # ############# THERMAL TARGET Server Response: Handle Transition to Firing State ############# #
    def thermal_target_callback(self, request, response):
        thermal_target_detected = request.activate

        if (thermal_target_detected):
            self.get_logger().info('Supervisor Acknowledges Thermal Target Detected.')
            self.toggle_firing(thermal_target_detected)
            response.message = 'Supervisor Transitioned to Firing State.'
            return response.message
        else:
            self.get_logger().info('Supervisor Acknowledges Thermal Target Not Detected.')
            response.message = 'Supervisor Transitioned to Firing State.'
            return response.message

    
    # ############# FIRING Client Call: Request to Start/Stop Firing ############# #
    def toggle_firing(self, activate):
        self.ActivateFiring.activate = activate
        if activate:
            self.state = State.Firing
            return self.FiringClient.call_async(self.ActivateFiring)
    
    # ############# FIRING Server Response: Handle Firing Completion Feedback ############# #
    def firing_status_callback(self, request, response):
        finish_firing = request.finish

        if finish_firing:
            self.COMPLETED_TARGETS += 1
            self.get_logger().info(f"Completed targets: {self.COMPLETED_TARGETS}/{NUM_TARGETS}")
            response.message = "Supervisor Acknowledges Firing Is Finished."

            if self.COMPLETED_TARGETS >= NUM_TARGETS:
                self.get_logger().info("All targets completed. Mission complete.")
                self.IS_MISSION_COMPLETE = True
            else:
                self.state = State.EXPLORATION
                self.toggle_exploration(True)

            return response
        else:
            response.message = "Supervisor Acknowledges Firing Is Not Finished."
            return response


    # TODO
    def clean_up(self):
        self.get_logger().info("Shutting down. Mission complete.")
        self.toggle_exploration(False)
        self.toggle_alignment(False)
        self.toggle_firing(False)
        return

    def run_main_loop_callback(self):
        if not self.IS_MISSION_COMPLETE:
            return
        else:
            self.clean_up()
            return





def main():
    rclpy.init()
    supervisor = Supervisor()
   
    rclpy.spin(supervisor)
    
    supervisor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
