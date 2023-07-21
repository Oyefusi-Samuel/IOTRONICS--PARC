#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Twist

class ImageCapture:
    def __init__(self):
        self.left_image = None
        self.right_image = None
        self.bridge = CvBridge()
        self.left_sub = rospy.Subscriber('/left_camera/image_raw', Image, self.left_callback)
        self.right_sub = rospy.Subscriber('/right_camera/image_raw', Image, self.right_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()

    def left_callback(self, data):
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def right_callback(self, data):
        try:
            self.right_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def obstacle_detected(self):
        # Obstacle detection logic for the front only
        front_distance_threshold = 1.5  # meters

        return self.check_front_obstacle(front_distance_threshold)

    def check_front_obstacle(self, distance_threshold):
        # Placeholder logic for front obstacle detection.
        # In a real-world scenario, you'd use the actual sensor data.
        # For now, we'll assume an obstacle is detected if either left or right image is not None.
        return self.left_image is not None or self.right_image is not None

    def capture_images(self):
        rospy.sleep(1)  # Wait for the camera topics to start publishing
        rate = rospy.Rate(10)  # Adjust the rate as needed
        distance = 0  # Distance traveled
        desired_distance = 9  # Desired distance to travel in meters

        while distance < desired_distance and not rospy.is_shutdown():
            if self.left_image is not None and self.right_image is not None:
                # Display or process the images as desired
                cv2.imshow("Left Camera", self.left_image)
                cv2.imshow("Right Camera", self.right_image)
                cv2.waitKey(1)  # Adjust the delay as needed

            # Check for obstacles in front
            if self.obstacle_detected():
                # If an obstacle is detected, stop the robot and wait for a short time
                self.twist.linear.x = 0.0
                self.cmd_vel_pub.publish(self.twist)
                rospy.sleep(1)

            # Move the robot forward
            self.twist.linear.x = 0.8  # Adjust the linear velocity as needed
            self.twist.angular.z = 0.25
            self.cmd_vel_pub.publish(self.twist)

            # Calculate the distance traveled based on linear velocity and time
            distance += abs(self.twist.linear.x) * 0.1  # 0.1 is the sleep duration (1 / rate)

            rate.sleep()

        # Stop the robot after reaching the desired distance
        self.twist.linear.x = 0.0
        self.cmd_vel_pub.publish(self.twist)

if __name__ == '__main__':
    rospy.init_node('image_capture_node')
    image_capture = ImageCapture()
    image_capture.capture_images()
