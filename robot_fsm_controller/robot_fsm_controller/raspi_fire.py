import RPi.GPIO as GPIO
import time
import rclpy
from rclpy.node import Node
from cde2310_interfaces.srv import ActivateNode

class FiringNode(Node):
    def __init__(self):
        super().__init__('firing_node')

        # Setup GPIO in BCM mode.
        GPIO.setmode(GPIO.BCM)

        # Flywheel GPIO pins.
        self.IN1 = 27
        self.IN2 = 17
        self.ENA = 22

        self.IN3 = 4
        self.IN4 = 26
        self.ENB = 19

        # Setup flywheel pins as outputs.
        GPIO.setup([self.IN1, self.IN2, self.IN3, self.IN4], GPIO.OUT)
        GPIO.setup(self.ENA, GPIO.OUT)
        GPIO.setup(self.ENB, GPIO.OUT)

        # Create PWM objects for flywheels at 1 kHz.
        self.flywheel_pwmA = GPIO.PWM(self.ENA, 1000)
        self.flywheel_pwmB = GPIO.PWM(self.ENB, 1000)
        self.flywheel_pwmA.start(0)
        self.flywheel_pwmB.start(0)

        # Servo Motor GPIO pin.
        self.servo_pin = 18
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.servo_pwm = GPIO.PWM(self.servo_pin, 50)  # 50 Hz for servo control.
        self.servo_pwm.start(0)

        # Create a service to trigger firing.
        self.firing_service = self.create_service(ActivateNode, 'activate_firing', self.fire_ball_callback)
        self.get_logger().info("Firing node is ready.")

    def fire_3_ball(self, flywheel_speed=50):

        ##### Set-up Code for Flywheel ####

        # GPIO
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)

        # change pwm
        flywheel_pwmA.ChangeDutyCycle(flywheel_speed)
        flywheel_pwmB.ChangeDutyCycle(flywheel_speed)

        #### Set-up Code for Servo Motor ####
        servo_pwm.ChangeDutyCycle(2)

        #### delay to get electrical components time to continue running
        time.sleep(2)
        servo_pwm.ChangeDutyCycle(12.5)

        ### Give time for ball to launch
        time.sleep(0.5)

        ## Reset Servo Position
        servo_pwm.ChangeDutyCycle(2)

        ### 4 Second delay
        time.sleep(4)

        ## launch 2nd ball
        servo_pwm.ChangeDutyCycle(12.5)

        ### Time for ball to launch
        time.sleep(0.5)

        ## Reset Servo Position
        servo_pwm.ChangeDutyCycle(2)

        ## 2 second Delay
        time.sleep(2)

        ### Launch 3rd ball
        servo_pwm.ChangeDutyCycle(12.5)

        ## Time for ball to launch
        time.sleep(0.5)

        #### Tear-down Code for Flywheel ####
        GPIO.output([IN1,IN2,IN3,IN4], GPIO.LOW)
        flywheel_pwmA.ChangeDutyCycle(0)
        flywheel_pwmB.ChangeDutyCycle(0)

        #### Tear-down code for motor
        servo_pwm.ChangeDutyCycle(2)
        # None for now

        return

    def fire_ball_callback(self, request, response):
        try:
            # Use a flywheel speed of 60 (or extract from request if defined).
            self.get_logger().info("Firing ball!")
            self.fire_3_ball(flywheel_speed=60)
            response.message = "Ball fired successfully."
        except Exception as e:
            self.get_logger().error(f"Firing failed: {e}")
            response.message = f"Firing failed: {e}"
        return response

    def destroy_node(self):
        # Stop PWM and clean up GPIO.
        self.flywheel_pwmA.stop()
        self.flywheel_pwmB.stop()
        self.servo_pwm.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FiringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
