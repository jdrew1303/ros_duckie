import unittest
from unittest.mock import MagicMock, patch

# Mock the ROS and hardware libraries
import sys
sys.modules['rospy'] = MagicMock()
sys.modules['Adafruit_MotorHAT'] = MagicMock()
sys.modules['ackermann_msgs.msg'] = MagicMock()

from src.differential.src.differential_drive_node import DifferentialDriveNode

class TestDifferentialDrive(unittest.TestCase):
    def setUp(self):
        # Patch the rospy.get_param to return default values
        with patch('rospy.get_param') as mock_get_param:
            mock_get_param.side_effect = lambda key, default: default
            self.node = DifferentialDriveNode()

    def test_straight_forward(self):
        left, right = self.node.throttle_angle_to_thrust(0.5, 90)
        self.assertAlmostEqual(left, 0.5)
        self.assertAlmostEqual(right, 0.5)

    def test_hard_right(self):
        left, right = self.node.throttle_angle_to_thrust(0.5, 180)
        self.assertAlmostEqual(left, 0.5)
        self.assertAlmostEqual(right, 0.0)

    def test_hard_left(self):
        left, right = self.node.throttle_angle_to_thrust(0.5, 0)
        self.assertAlmostEqual(left, 0.0)
        self.assertAlmostEqual(right, 0.5)

    def test_soft_right(self):
        left, right = self.node.throttle_angle_to_thrust(0.5, 135)
        self.assertAlmostEqual(left, 0.5)
        self.assertAlmostEqual(right, 0.25)

    def test_soft_left(self):
        left, right = self.node.throttle_angle_to_thrust(0.5, 45)
        self.assertAlmostEqual(left, 0.25)
        self.assertAlmostEqual(right, 0.5)

    def test_zero_speed(self):
        left, right = self.node.throttle_angle_to_thrust(0, 90)
        self.assertAlmostEqual(left, 0)
        self.assertAlmostEqual(right, 0)

if __name__ == '__main__':
    unittest.main()