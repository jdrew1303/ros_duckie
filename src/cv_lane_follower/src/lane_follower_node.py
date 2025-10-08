#!/usr/bin/env python
from __future__ import division
import cv2
import math
import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage
from ackermann_msgs.msg import AckermannDrive

class LaneFollowerNode:
    def __init__(self):
        rospy.init_node('cv_lane_follower', anonymous=False)

        # Get parameters from the ROS parameter server
        self.save_images = rospy.get_param('~save_images', False)
        self.usb_mount = rospy.get_param('~usb_mount', '/opt/usb/')

        # HSV color mask parameters
        self.lower_yellow = np.array(rospy.get_param('~lower_yellow', [25, 50, 50]))
        self.upper_yellow = np.array(rospy.get_param('~upper_yellow', [45, 255, 255]))
        self.lower_white = np.array(rospy.get_param('~lower_white', [0, 0, 150]))
        self.upper_white = np.array(rospy.get_param('~upper_white', [180, 100, 255]))

        # Canny edge detection parameters
        self.canny_sigma = rospy.get_param('~canny_sigma', 0.33)

        # Hough line transform parameters
        self.hough_rho = rospy.get_param('~hough_rho', 1)
        self.hough_angle = rospy.get_param('~hough_angle', np.pi / 180)
        self.hough_threshold = rospy.get_param('~hough_threshold', 15)
        self.hough_min_line_length = rospy.get_param('~hough_min_line_length', 3)
        self.hough_max_line_gap = rospy.get_param('~hough_max_line_gap', 4)

        # Create a publisher for the drive commands
        self.publisher = rospy.Publisher("drivetrain/ackermann", AckermannDrive, queue_size=1)

        # Create a subscriber for the camera images
        rospy.Subscriber('camera/image', CompressedImage, self.image_received, queue_size=2)

        self.message_count = 1

    def image_received(self, message):
        # Decompress the image
        bgr_image = self.decompress_image(message)

        # Process the image to find the steering angle
        steering_angle = self.process_image(bgr_image)

        # Publish the drive command
        self.publish_drive_command(steering_angle)

        # Save images for debugging if the parameter is set
        if self.save_images:
            self.save_debug_images(bgr_image, steering_angle)

    def decompress_image(self, message):
        compressed_image = message.data
        image_string = np.fromstring(compressed_image, np.uint8)
        return cv2.imdecode(image_string, cv2.IMREAD_COLOR)

    def process_image(self, bgr_image):
        # Convert to HSV color space
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

        # Apply color masks
        masked_image = self.apply_color_masks(hsv_image)

        # Apply Canny edge detection
        canny_edged_image = self.apply_canny_edge_detection(masked_image)

        # Crop the region of interest
        cropped_region = self.crop_region_of_interest(canny_edged_image)

        # Detect line segments
        line_segments = self.detect_line_segments(cropped_region)

        # Average the line segments to get the lane lines
        lane_lines = self.average_slope_intercept(bgr_image, line_segments)

        # Compute the steering angle
        return self.compute_steering_angle(bgr_image, lane_lines)

    def apply_color_masks(self, hsv_image):
        yellow_mask = cv2.inRange(hsv_image, self.lower_yellow, self.upper_yellow)
        white_mask = cv2.inRange(hsv_image, self.lower_white, self.upper_white)
        return cv2.bitwise_and(hsv_image, hsv_image, mask=cv2.bitwise_or(yellow_mask, white_mask))

    def apply_canny_edge_detection(self, image):
        v = np.median(image)
        lower = int(max(0, (1.0 - self.canny_sigma) * v))
        upper = int(min(255, (1.0 + self.canny_sigma) * v))
        return cv2.Canny(image, lower, upper)

    def crop_region_of_interest(self, image):
        height, width = image.shape
        mask = np.zeros_like(image)
        polygon = np.array([[
            (0, height * 1 / 2),
            (width, height * 1 / 2),
            (width, height),
            (0, height),
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        return cv2.bitwise_and(image, mask)

    def detect_line_segments(self, image):
        return cv2.HoughLinesP(
            image,
            self.hough_rho,
            self.hough_angle,
            self.hough_threshold,
            np.array([]),
            minLineLength=self.hough_min_line_length,
            maxLineGap=self.hough_max_line_gap
        )

    def make_points(self, frame, line):
        height, width, _ = frame.shape
        slope, intercept = line
        y1 = height
        y2 = int(y1 * 1 / 2)
        x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
        x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
        return [[x1, y1, x2, y2]]

    def average_slope_intercept(self, image, line_segments):
        lane_lines = []
        if line_segments is None:
            return lane_lines

        height, width, _ = image.shape
        left_fit = []
        right_fit = []

        boundary = 1/3
        left_region_boundary = width * (1 - boundary)
        right_region_boundary = width * boundary

        for line_segment in line_segments:
            for x1, y1, x2, y2 in line_segment:
                if x1 == x2:
                    continue
                fit = np.polyfit((x1, x2), (y1, y2), 1)
                slope = fit[0]
                intercept = fit[1]
                if slope < 0:
                    if x1 < left_region_boundary and x2 < left_region_boundary:
                        left_fit.append((slope, intercept))
                else:
                    if x1 > right_region_boundary and x2 > right_region_boundary:
                        right_fit.append((slope, intercept))

        if len(left_fit) > 0:
            left_fit_average = np.average(left_fit, axis=0)
            lane_lines.append(self.make_points(image, left_fit_average))

        if len(right_fit) > 0:
            right_fit_average = np.average(right_fit, axis=0)
            lane_lines.append(self.make_points(image, right_fit_average))

        return lane_lines

    def compute_steering_angle(self, image, lane_lines):
        if len(lane_lines) == 0:
            return 90

        height, width, _ = image.shape

        if len(lane_lines) == 1:
            x1, _, x2, _ = lane_lines[0][0]
            x_offset = x1 - x2
        else:
            _, _, left_x2, _ = lane_lines[0][0]
            _, _, right_x2, _ = lane_lines[1][0]
            camera_mid_offset_percent = 0.0
            mid = int(width / 2 * (1 + camera_mid_offset_percent))
            x_offset = (left_x2 + right_x2) / 2 - mid

        y_offset = int(height / 2)
        angle_to_mid_radian = math.atan(x_offset / y_offset)
        angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
        return angle_to_mid_deg + 90

    def publish_drive_command(self, steering_angle):
        ackermann = AckermannDrive()
        ackermann.speed = 0.5
        ackermann.steering_angle = int(steering_angle)
        self.publisher.publish(ackermann)

    def save_debug_images(self, bgr_image, steering_angle):
        # ... (saving logic as before, but using self.usb_mount and self.message_count)
        self.message_count += 1

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = LaneFollowerNode()
    node.run()