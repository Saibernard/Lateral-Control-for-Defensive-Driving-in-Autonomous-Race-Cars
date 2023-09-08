#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import tf2_ros
import csv
import cv2
import numpy as np
from pupil_apriltags import Detector

class Camera(Node):
    """
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')
        # TODO: create ROS subscribers and publishers
        self.opp_pose_pub = self.create_publisher(Float32MultiArray, '/cam', 10)
        self.timer = self.create_timer(0, self.timer_callback)

        self.i = 0

        self.init_april()

    def init_april(self):
        # Create a detector
        self.detector = Detector()

        # Open video capture
        self.cap = cv2.VideoCapture(4)

        # Threshold values for x and y coordinates
        x_threshold = 100
        y_threshold = 100

        # Known size of the AprilTag in millimeters (replace with your tag's size)
        self.tag_size_mm = 200

        # Diagonal field of view of the camera in degrees (replace with your camera's value)
        fov_degrees = 70

        # Get the image dimensions
        width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

        # Calculate the focal length in pixels
        self.focal_length_px = (width / 2) / (0.5 * (1 / np.tan(np.deg2rad(fov_degrees / 2))))

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect the AprilTags in the frame
        tags = self.detector.detect(gray)

        # Process each detected tag
        for tag in tags:
            # Compute the center of the tag
            cx = int((tag.corners[0][0] + tag.corners[2][0]) / 2)
            cy = int((tag.corners[0][1] + tag.corners[2][1]) / 2)
            cx_check = int((tag.corners[0][0] + tag.corners[1][0]) / 2)
            cy_check = int((tag.corners[0][1] + tag.corners[1][1]) / 2)
        
            perimeter = np.linalg.norm(np.diff(tag.corners, axis=1))

            distance_mm = (self.tag_size_mm * self.focal_length_px) / perimeter
                
            #pixel input distance
            distance_x, distance_y = self.get_distance(cx_check, cy_check)

            msg = Float32MultiArray()
            msg.data = [distance_mm / 10, distance_x, distance_y]
            print(f"publishing {msg.data}")
            self.opp_pose_pub.publish(msg)

    def get_distance(self, x, y):  # input pixel x,y
        Hmount = 8.5
        intrinsic = np.array(
            [[694.71543085, 0, 449.37540776], [0, 695.54961208, 258.64705743], [0, 0, 1]])  # From calibration

        x_car = intrinsic[1][1] * Hmount / (y - intrinsic[1][2])
        y_car = - x_car * (x - intrinsic[0][2]) / intrinsic[0][0]
        return x_car, y_car
        

def main(args=None):
    rclpy.init(args=args)
    print("Cam Initialized")
    cam_node = Camera()
    rclpy.spin(cam_node)

    cam_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
