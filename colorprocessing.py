#!/usr/bin/env python

# Import necessary libraries
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Twist

# Define the ImageCapture class
class ImageCapture:
    def __init__(self):
        # Initialize variables for left and right images
        self.left_image = None
        self.right_image = None

        # Create a CvBridge object to convert ROS Image messages to OpenCV format
        self.bridge = CvBridge()

        # Set up subscribers to receive left and right camera images
        self.left_sub = rospy.Subscriber('/left_camera/image_raw', Image, self.left_callback)
        self.right_sub = rospy.Subscriber('/right_camera/image_raw', Image, self.right_callback)

        # Set up publisher to send Twist messages for robot control
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Initialize a Twist message for controlling the robot's movement
        self.twist = Twist()

    # Callback function for the left camera image subscriber
    def left_callback(self, data):
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    # Callback function for the right camera image subscriber
    def right_callback(self, data):
        try:
            self.right_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    # Method to capture images and move the robot forward
    def capture_images(self):
        rospy.sleep(1)  # Wait for the camera topics to start publishing
        rate = rospy.Rate(10)  # Adjust the rate as needed
        distance = 0  # Distance traveled
        desired_distance = 9  # Desired distance to travel in meters

        # Loop until the robot reaches the desired distance or ROS is shutdown
        while distance < desired_distance and not rospy.is_shutdown():
            # Check if both left and right images are available
            if self.left_image is not None and self.right_image is not None:
                # Display or process the images as desired
                cv2.imshow("Left Camera", self.left_image)
                cv2.imshow("Right Camera", self.right_image)
                cv2.waitKey(1)  # Adjust the delay as needed

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

# Main function to initialize the node and start image capture
if __name__ == '__main__':
    rospy.init_node('image_capture_node')  # Initialize the ROS node with the name 'image_capture_node'
    image_capture = ImageCapture()  # Create an instance of the ImageCapture class
    image_capture.capture_images()  # Call the capture_images method to start capturing images and moving the robot
