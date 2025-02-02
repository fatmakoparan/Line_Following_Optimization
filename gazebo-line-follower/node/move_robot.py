#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import os
import math
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
WAIT_TIME_AFTER_STOP = 2  # Waiting time after reaching the target (seconds)
WAIT_TIME = 10       # Waiting time after detecting an obstacle (seconds)
LANE_CHANGE_DISTANCE = 0.5  # Distance to move into the left lane (meters)
SHOULDER_DISTANCE = 0.4  # Maximum lateral deviation from the lane (40 cm)

# Target coordinates
target_b_x = 1.79
target_b_y = 2.41
target_b_reached = False  # Has the target been reached?

# Global Variables
obstacle_detected = False  # Is there an obstacle?
sound_played = False       # Has the obstacle sound been played?
stop_time = 0              # Waiting time after obstacle detection
in_lane = False            # Is the robot in the lane?
lane_offset = 0            # Distance from the lane center
current_x = 0.0            # Current x position
current_y = 0.0            # Current y position
current_yaw = 0.0          # Current yaw angle


def turn_angle(target_angle):
    global current_yaw

    move = Twist()
    initial_yaw = current_yaw  # Başlangıç yönelim açısını kaydet
    target_yaw = initial_yaw + target_angle  # Yeni hedef yönelim açısı

    # Açıyı [-pi, pi] aralığında normalize et
    target_yaw = math.atan2(math.sin(target_yaw), math.cos(target_yaw))

    rospy.loginfo(f"Turning to target yaw: {math.degrees(target_yaw)} degrees (Current: {math.degrees(current_yaw)})")

    rate = rospy.Rate(10)  # 10 Hz kontrol
    timeout = rospy.Time.now() + rospy.Duration(7)  # Maks 7 saniye içinde bitmeli

    while abs(math.atan2(math.sin(target_yaw - current_yaw), math.cos(target_yaw - current_yaw))) > 0.05:  
        if rospy.Time.now() > timeout:  # Sonsuz döngüyü engellemek için max süre
            rospy.logwarn(f"Turn timeout reached! Stopping turn. (Final Yaw: {math.degrees(current_yaw)})")
            move = Twist()
            move.linear.x = 0
            move.angular.z = 0
            break
        move.angular.z = 0.2 if target_yaw > current_yaw else -0.2  # Daha düşük hızda dönüş
        pub.publish(move)
        rate.sleep()

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
    
def go_straight1():
    move = Twist()
    move.linear.x = STRAIGHT_SPD  # Düz ilerleme hızı
    move.angular.z = 0  # Açısal hareket yok

    rospy.loginfo("Moving straight for 4 seconds...")

    rate = rospy.Rate(10)  # 10 Hz yayınlama
    start_time = rospy.Time.now()
    
    while rospy.Time.now() - start_time < rospy.Duration(4):
        pub.publish(move)
        rate.sleep()

    # Hareketi durdur
    stop_moving()
    rospy.loginfo("Stopped after 4 seconds of straight movement.")


def stop_moving():
    move = Twist()
    move.linear.x = 0
    move.angular.z = 0
    pub.publish(move)
    rospy.sleep(0.5)  # Tam durmasını bekliyoruz

def check_lane():
    """Kameradan gelen görüntüyü analiz ederek önünde şerit olup olmadığını kontrol eder."""
    try:
        img_msg = rospy.wait_for_message('/camera/rgb/image_raw', Image, timeout=1)
        img = bridge.imgmsg_to_cv2(img_msg, 'bgr8')

        # Görüntüyü griye çevir ve binary (siyah-beyaz) yap
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, img_bin = cv2.threshold(img_gray, 128, 255, cv2.THRESH_BINARY)

        # Görüntünün alt kısmını analiz et (önündeki şeridi kontrol ediyoruz)
        bottom_half = img_bin[-50:, :]  # Alt 50 pikseli al
        white_pixels = np.sum(bottom_half == 255)  # Beyaz pikselleri say

        rospy.loginfo(f"Detected white pixels: {white_pixels}")

        return white_pixels > 1000  # Eğer yeterli beyaz piksel varsa şerit vardır

    except CvBridgeError as e:
        rospy.logerr(f"Camera error: {e}")
        return False
    except rospy.ROSException:
        rospy.logwarn("No camera image received.")
        return False

def stop_and_turn_left():
    rospy.loginfo("Starting left turn maneuver...")

    # Step 1: 15 derece sola dön
    turn_angle(math.radians(15))  # 15 derece dönüş

    # Hareketi durdur
    stop_moving()

    # Step 2: 4 saniye düz git
    go_straight1()

    # Hareketi durdur
    stop_moving()
    
    # Step 1: 30-45 derece sola dön
    turn_angle(math.radians(20))  # 15 derece dönüş

    # Step 2: 5 saniye düz git
    go_straight1()
    
    rospy.loginfo("Finished navigating around obstacle, returning to lane tracking.")

# Odometry callback to get current position
def odometry_callback(data):
    global current_x, current_y, current_yaw, target_b_reached

    # Update current position and orientation
    current_x = data.pose.pose.position.x
    current_y = data.pose.pose.position.y

    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    _, _, current_yaw = euler_from_quaternion(orientation_list)

    # Log current position
    #rospy.loginfo(f"Current position: x={current_x:.2f}, y={current_y:.2f}, yaw={current_yaw:.2f}")

    # Check if robot reached the target
    if not target_b_reached and abs(current_x - target_b_x) < 0.1 and abs(current_y - target_b_y) < 0.1:
        rospy.loginfo("Target B reached! Preparing to stop and turn left.")
        target_b_reached = True
        stop_and_turn_left()
    
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
    global in_lane, target_b_reached, lane_offset

    if target_b_reached:
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
