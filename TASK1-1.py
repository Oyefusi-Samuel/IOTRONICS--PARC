#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

pub = None


def callback_laser(msg):
    # 120 degrees into 3 regions
    regions = {
        'right': min(min(msg.ranges[0:2]), 10),  # Calculate minimum range for the right region
        'front': min(min(msg.ranges[3:5]), 10),  # Calculate minimum range for the front region
        'left': min(min(msg.ranges[6:9]), 10),  # Calculate minimum range for the left region
    }

    take_action(regions)


def take_action(regions):
    threshold_dist = 1.5  # Threshold distance for obstacle detection
    linear_speed = 0.6  # Linear speed for forward/backward movement
    angular_speed = 1  # Angular speed for rotation

    msg = Twist()  # Create a Twist message object
    linear_x = 0  # Initialize linear velocity
    angular_z = 0  # Initialize angular velocity

    state_description = ''  # Initialize the state description variable

    if regions['front'] > threshold_dist and regions['left'] > threshold_dist and regions['right'] > threshold_dist:
        # No obstacle case
        state_description = 'case 1 - no obstacle'
        linear_x = linear_speed
        angular_z = 0
    elif regions['front'] < threshold_dist and regions['left'] < threshold_dist and regions['right'] < threshold_dist:
        # Obstacles in front, left, and right case
        state_description = 'case 7 - front and left and right'
        linear_x = -linear_speed
        angular_z = angular_speed
    elif regions['front'] < threshold_dist and regions['left'] > threshold_dist and regions['right'] > threshold_dist:
        # Obstacle in front case
        state_description = 'case 2 - front'
        linear_x = 0
        angular_z = angular_speed
    elif regions['front'] > threshold_dist and regions['left'] > threshold_dist and regions['right'] < threshold_dist:
        # Obstacle on the right case
        state_description = 'case 3 - right'
        linear_x = 0
        angular_z = -angular_speed
    elif regions['front'] > threshold_dist and regions['left'] < threshold_dist and regions['right'] > threshold_dist:
        # Obstacle on the left case
        state_description = 'case 4 - left'
        linear_x = 0
        angular_z = angular_speed
    elif regions['front'] < threshold_dist and regions['left'] > threshold_dist and regions['right'] < threshold_dist:
        # Obstacles in front and right case
        state_description = 'case 5 - front and right'
        linear_x = 0
        angular_z = -angular_speed
    elif regions['front'] < threshold_dist and regions['left'] < threshold_dist and regions['right'] > threshold_dist:
        # Obstacles in front and left case
        state_description = 'case 6 - front and left'
        linear_x = 0
        angular_z = angular_speed
    elif regions['front'] > threshold_dist and regions['left'] < threshold_dist and regions['right'] < threshold_dist:
        # Obstacles on the left and right case
        state_description = 'case 8 - left and right'
        linear_x = linear_speed
        angular_z = 0
    else:
        # Unknown case
        state_description = 'unknown case'
        rospy.loginfo(regions)

    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)


def main():
    global pub

    rospy.init_node('crop_row_navigation')  # Initialize the ROS node

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)  # Create a publisher for Twist messages

    sub = rospy.Subscriber('/scan', LaserScan, callback_laser)  # Create a subscriber for LaserScan messages

    rospy.spin()  # Enter the ROS event loop


if _name_ == '_main_':
    main()