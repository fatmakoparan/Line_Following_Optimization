#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import os
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan

# Constants
STRAIGHT_SPD = 0.25  # Speed for moving straight
TURN_SPEED = 0.15    # Speed for light turning
ANGULAR_VEL = 1.25   # Angular velocity for turning
SAFE_DISTANCE = 1.0  # Obstacle avoidance distance
WAIT_TIME = 10       # Waiting time after detecting an obstacle (seconds)
SHOULDER_DISTANCE = 0.4  # Maximum lateral deviation from the lane (40 cm)

# Target coordinates
target_x = 3.59106
target_y = -2.3663
target_reached = False  # Has the target been reached?

# Global Variables
obstacle_detected = False  # Is there an obstacle?
sound_played = False       # Has the obstacle sound been played?
stop_time = 0              # Waiting time after obstacle detection
in_lane = False            # Is the robot in the lane?
lane_offset = 0            # Distance from the lane center
current_x = 0.0            # Current x position
current_y = 0.0            # Current y position
current_yaw = 0.0          # Current yaw angle

# Odometry callback to get current yaw and position
def odometry_callback(data):
    global current_x, current_y, current_yaw, target_reached

    # Update position and orientation
    current_x = data.pose.pose.position.x
    current_y = data.pose.pose.position.y
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    _, _, current_yaw = euler_from_quaternion(orientation_list)

    # Check proximity to the target
    if abs(current_x - target_x) < 0.1 and abs(current_y - target_y) < 0.1:
        rospy.loginfo("Target reached!")
        target_reached = True

# LIDAR callback for obstacle detection
def lidar_callback(data):
    global obstacle_detected, sound_played, stop_time

    min_range = min(min(data.ranges[:15] + data.ranges[-15:]), data.range_max)
    if min_range < SAFE_DISTANCE:
        obstacle_detected = True
        if not sound_played:
            rospy.loginfo("Obstacle detected, playing sound...")
            os.system('aplay /home/fatmak/catkin_ws/src/sounds/alert.wav')
            sound_played = True
            etrafindan_dolan()
    else:
        obstacle_detected = False
        sound_played = False

# Turn a specified angle based on yaw
def turn_angle(target_angle):
    global current_yaw

    move = Twist()
    target_yaw = current_yaw + target_angle

    # Keep angle within [-pi, pi]
    if target_yaw > 3.14159:
        target_yaw -= 2 * 3.14159
    elif target_yaw < -3.14159:
        target_yaw += 2 * 3.14159

    # Turn until the target angle is reached
    rate = rospy.Rate(10)  # 10 Hz kontrol
    while abs(target_yaw - current_yaw) > 0.05:
        move.angular.z = 0.5 if target_angle > 0 else -0.5
        pub.publish(move)
        rate.sleep()

    # Stop after the turn is complete
    move.angular.z = 0
    pub.publish(move)
    rospy.sleep(0.1)  # Dönüş tamamlandıktan sonra duraksama

# Function to move a specified distance in a straight line
def go_straight(distance):
    global current_x, current_y

    # Record the starting position
    initial_x = current_x
    initial_y = current_y

    move = Twist()
    move.linear.x = STRAIGHT_SPD
    pub.publish(move)

    rate = rospy.Rate(10)  # 10 Hz döngü hızıyla kontrol sağla

    # Move the desired distance
    while not rospy.is_shutdown():
        current_distance = ((current_x - initial_x) ** 2 + (current_y - initial_y) ** 2) ** 0.5
        if current_distance >= distance:
            break
        pub.publish(move)  # Her döngüde hız komutunu tekrar yayınla
        rate.sleep()

    # Stop moving
    move.linear.x = 0
    pub.publish(move)

# Function to navigate around an obstacle
def etrafindan_dolan():
    rospy.loginfo("Navigating around obstacle...")

    # Step 1: Turn 90 degrees right
    turn_angle(-3.14159 / 2)

    # Step 2: Move forward 50 cm
    go_straight(0.6)

    # Step 3: Turn 90 degrees left
    turn_angle(3.14159 / 2)

    # Step 4: Move forward 2.2 meters
    go_straight(2.2)

    # Step 5: Turn 90 degrees left
    turn_angle(3.14159 / 2)

    # Step 6: Move forward 50 cm
    go_straight(0.6)

    # Step 7: Turn 90 degrees right
    turn_angle(-3.14159 / 2)

    rospy.loginfo("Finished navigating around obstacle, returning to lane tracking.")

# Camera callback for lane tracking
def camera_callback(data):
    global in_lane, target_reached, lane_offset

    if target_reached:
        # Stop the robot if the target is reached
        move = Twist()
        move.linear.x = 0
        move.angular.z = 0
        pub.publish(move)
        return

    try:
        img_grayscale = bridge.imgmsg_to_cv2(data, 'mono8')
        _, img_bin = cv2.threshold(img_grayscale, 128, 255, cv2.THRESH_BINARY_INV)

        # Split the bottom of the image in half
        bottom_half = img_bin[-100:]  # Bottom part of the image
        left_half = bottom_half[:, :bottom_half.shape[1] // 2]
        right_half = bottom_half[:, bottom_half.shape[1] // 2:]

        # Check for lane presence on the left or right
        left_detected = np.sum(left_half) > 1000
        right_detected = np.sum(right_half) > 1000

        move = Twist()

        if not obstacle_detected:
            if left_detected and not right_detected:
                # If the lane is on the left, turn left slightly
                lane_offset = 0.5  # Estimate distance from lane center (left)
                if lane_offset >= SHOULDER_DISTANCE:
                    move.linear.x = STRAIGHT_SPD / 2
                    move.angular.z = 0.5  # Slightly turn left
                    rospy.loginfo("Lane on left, turning left.")
                else:
                    move.linear.x = STRAIGHT_SPD
                    move.angular.z = 0
                    rospy.loginfo("Near lane, moving straight.")
            elif right_detected and not left_detected:
                # If the lane is on the right, turn right slightly
                lane_offset = -0.5  # Estimate distance from lane center (right)
                if abs(lane_offset) >= SHOULDER_DISTANCE:
                    move.linear.x = STRAIGHT_SPD / 2
                    move.angular.z = -0.5  # Slightly turn right
                    rospy.loginfo("Lane on right, turning right.")
                else:
                    move.linear.x = STRAIGHT_SPD
                    move.angular.z = 0
                    rospy.loginfo("Near lane, moving straight.")
            elif left_detected and right_detected:
                # Lane is centered, move straight
                lane_offset = 0
                move.linear.x = STRAIGHT_SPD
                move.angular.z = 0
                rospy.loginfo("Lane centered, moving straight.")
            else:
                # No lane detected, search slowly
                move.linear.x = TURN_SPEED / 2
                rospy.loginfo("No lane detected, searching.")

        pub.publish(move)

    except CvBridgeError as e:
        print(e)

if __name__ == "__main__":
    rospy.init_node('move_robot')

    bridge = CvBridge()

    rospy.Subscriber('/camera/rgb/image_raw', Image, camera_callback)
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    rospy.Subscriber('/odom', Odometry, odometry_callback)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    rospy.spin()

