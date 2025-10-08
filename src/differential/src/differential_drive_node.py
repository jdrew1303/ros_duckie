#!/usr/bin/env python
from __future__ import division
import atexit
import rospy
from Adafruit_MotorHAT import Adafruit_MotorHAT
from ackermann_msgs.msg import AckermannDrive

class DifferentialDriveNode:
    def __init__(self):
        rospy.init_node('drivetrain', anonymous=False)

        # Get parameters
        self.motor_addr = rospy.get_param('~motor_addr', 0x60)
        self.pulse_min = rospy.get_param('~pulse_min', 60)
        self.pulse_max = rospy.get_param('~pulse_max', 255)

        # Initialize MotorHAT
        self.motorhat = Adafruit_MotorHAT(addr=self.motor_addr)
        self.left_motor = self.motorhat.getMotor(1)
        self.right_motor = self.motorhat.getMotor(2)

        # Setup shutdown hooks
        rospy.on_shutdown(self.release_motors)
        atexit.register(self.release_motors)

        # Initial state
        self.release_motors()

        # Subscriber
        rospy.Subscriber('~ackermann', AckermannDrive, self.command_callback)

    def command_callback(self, message):
        speed = message.speed
        steering_angle = message.steering_angle
        left_speed, right_speed = self.throttle_angle_to_thrust(speed, steering_angle)
        self.move(left_speed, right_speed)

    def throttle_angle_to_thrust(self, speed, steering_angle):
        # Converts steering angle (+90 is straight) and speed to left/right motor thrust.
        # steering_angle: 0-180, speed: 0-1.0
        steering = (steering_angle - 90) / 90.0  # Convert to -1.0 (left) to 1.0 (right)

        # Clamp steering to [-1, 1]
        steering = max(-1.0, min(1.0, steering))

        left_speed = speed
        right_speed = speed

        if steering > 0:  # Turn right
            right_speed = speed * (1 - steering)
        else:  # Turn left or straight
            left_speed = speed * (1 + steering)

        return left_speed, right_speed

    def move(self, left_speed, right_speed):
        self.set_motor_speed(self.left_motor, left_speed)
        self.set_motor_speed(self.right_motor, right_speed)

    def set_motor_speed(self, motor, speed):
        speed = max(0, min(1, speed))

        if speed == 0:
            motor.run(Adafruit_MotorHAT.RELEASE)
            return

        pulse = self.pulse_min + (self.pulse_max - self.pulse_min) * speed

        motor.run(Adafruit_MotorHAT.FORWARD)
        motor.setSpeed(int(pulse))

    def release_motors(self):
        self.left_motor.run(Adafruit_MotorHAT.RELEASE)
        self.right_motor.run(Adafruit_MotorHAT.RELEASE)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = DifferentialDriveNode()
    node.run()