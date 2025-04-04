import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray
import adafruit_amg88xx
import busio
import board

class AMGSensorNode(Node):
    def __init__(self):
        super().__init__('amg_sensor_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/heat_source_detected', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        
        # Initialize I2C and AMG8833 sensor
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.sensor = adafruit_amg88xx.AMG88XX(i2c)
            self.get_logger().info("AMG8833 sensor initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize AMG8833: {str(e)}")
            raise

    def timer_callback(self):
        # Read the 64 pixels from the sensor
        pixels = self.sensor.readPixels()
        # Create and populate the ROS2 message
        msg = Float32MultiArray()
        msg.data = pixels
        # Publish the raw data
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published thermal data {pixels}")

    # Function to format the 1D pixel list into an 8x8 2D array
    def format_as_8x8(pixels):
        return np.array(pixels).reshape(8, 8) 
        

def main(args=None):
    rclpy.init(args=args)
    node = AMGSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()