import unittest
import numpy as np
from unittest.mock import MagicMock, patch

# Mock the ROS library
import sys
sys.modules['rospy'] = MagicMock()
sys.modules['sensor_msgs.msg'] = MagicMock()
sys.modules['ackermann_msgs.msg'] = MagicMock()

from src.cv_lane_follower.src.lane_follower_node import LaneFollowerNode

class TestLaneFollower(unittest.TestCase):
    def setUp(self):
        # Patch the rospy.get_param to return default values
        with patch('rospy.get_param') as mock_get_param:
            mock_get_param.side_effect = lambda key, default: default
            self.node = LaneFollowerNode()
        self.image = np.zeros((100, 160, 3), dtype=np.uint8)

    def test_no_lines(self):
        lines = self.node.average_slope_intercept(self.image, None)
        self.assertEqual(len(lines), 0)
        angle = self.node.compute_steering_angle(self.image, lines)
        self.assertEqual(angle, 90)

    def test_one_left_line(self):
        line_segments = [[(10, 100, 60, 50)]] # A single left-sloping line
        lines = self.node.average_slope_intercept(self.image, line_segments)
        self.assertEqual(len(lines), 1)
        angle = self.node.compute_steering_angle(self.image, lines)
        self.assertNotEqual(angle, 90)

    def test_one_right_line(self):
        line_segments = [[(150, 100, 100, 50)]] # A single right-sloping line
        lines = self.node.average_slope_intercept(self.image, line_segments)
        self.assertEqual(len(lines), 1)
        angle = self.node.compute_steering_angle(self.image, lines)
        self.assertNotEqual(angle, 90)

    def test_two_lines(self):
        line_segments = [
            [(10, 100, 60, 50)],   # Left line
            [(150, 100, 100, 50)]  # Right line
        ]
        lines = self.node.average_slope_intercept(self.image, line_segments)
        self.assertEqual(len(lines), 2)
        angle = self.node.compute_steering_angle(self.image, lines)
        # With two parallel lines, the steering angle should be straight
        self.assertAlmostEqual(angle, 90, delta=5)

    def test_steering_angle_calculation_left(self):
        # Creates a scenario where the car should steer left
        lane_lines = [[[-10, 100, 40, 50]]] # Single left line
        angle = self.node.compute_steering_angle(self.image, lane_lines)
        self.assertLess(angle, 90)

    def test_steering_angle_calculation_right(self):
        # Creates a scenario where the car should steer right
        lane_lines = [[[200, 100, 120, 50]]] # Single right line
        angle = self.node.compute_steering_angle(self.image, lane_lines)
        self.assertGreater(angle, 90)

if __name__ == '__main__':
    unittest.main()