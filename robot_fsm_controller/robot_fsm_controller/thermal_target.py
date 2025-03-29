'''
TODO:
The code below is a rough template of what i think the thermal target should look like.

This should be written alongside the code that will be used on the turtlebot, and this code
is written speculatively.
'''
from cde2310_interfaces.srv import ActivateNode
from cde2310_interfaces.srv import NodeFinish

import rclpy
from rclpy.node import Node
from enum import Enum

'''
TODO:
Importing other necessary packages required (such as using occupancy grid to track fired coordinates, etc
'''

class ThermalTarget(Node):

    SEARCH_RADIUS = 2.0 # Arbitrary Value. Should consider the resolution of the occupancy grid as well.

    def __init__(self):


        
        # ############# Thermal Node State Variables ############# #
        self.FIRED_COORDINATES = []
        self.thermal_array = None
        
        # ############# THERMAL SENSOR Client Creation: Activate Firing Sequence ############# #
        self.ThermalClient = self.create_client(ActivateNode, 'target_detected')
        while not self.ThermalClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Thermal Target Service not available, waiting again...')
        self.TargetDetected = ActivateNode.Request()


        '''
        TODO: Thermal Sensor Subscriber Creation: Subscribe To Raspberry Pi's Thermal Sensor Node
        self.thermal_sub = self.create_subscription(
            ???, "???", self.thermal_callback, 10
        )

        '''

        def target_detected(self, activate):
            self.TargetDetected.activate = activate
            return self.ThermalTargetClient.call_async(self.TargetDetected)
        
        '''
        TODO
        def is_previous_target(self, coordinate):
            for prev_target in FIRED_COORDINATES:
                checking of coordinates within our search radius
                if within search radius:
                    return True (It is Previous Target, So Ignore)
                else:
                    update Fired Coordinate (We are assuming we are
                    going to transition to firing, no alignment)
                    self.FIRED_COORDINATES.append(coordinate)
                    return False (It is a new Target)
        '''
        
        '''
        TODO: Callback Function
        def thermal_callback(self, msg):
            thermal_array = msg

            # Process Thermal_array

            # If center of the camera detects sufficient temperature, 
            # first check current turtlebot's coordinates
            
            #

            return
        '''

def main():
    rclpy.init()
    thermal_target = ThermalTarget()

    rclpy.spin(thermal_target)

    thermal_target.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        
