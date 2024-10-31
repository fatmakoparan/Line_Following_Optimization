#! /usr/bin/env python3

import rospy
import numpy as np
import cv2
import os
import time
from scipy.ndimage import center_of_mass
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan

# Constants
TURN_LEFT_SPD = 0.1
TURN_RIGHT_SPD = 0.125
STRAIGHT_SPD = 0.25
ANGULAR_VEL = 1.25
OFFSET_Y = 500
SAFE_DISTANCE = 0.5  # Distance threshold for obstacle detection (in meters)
WAIT_TIME = 10  # Engel tespit edildikten sonra bekleme süresi (saniye)

# Global Variables
prev = (400, 400)  # Holds previous CoM
obstacle_detected = False  # Keeps track of obstacle status
sound_played = False  # Tracks if sound has been played for the current obstacle
stop_time = 0  # Engel algılandıktan sonra bekleme için zamanlayıcı
avoiding_obstacle = False  # Engel etrafından dolanma durumu

# LIDAR callback for obstacle detection
def lidar_callback(data):
    global obstacle_detected, sound_played, stop_time, avoiding_obstacle

    # Check minimum range in front of the robot
    min_range = min(min(data.ranges[:15] + data.ranges[-15:]), data.range_max)  # Look at +/-15 degrees
    if min_range < SAFE_DISTANCE:
        obstacle_detected = True

        # Eğer ses çalınmadıysa terminalden sesi çal
        if not sound_played:
            rospy.loginfo("Obstacle detected! Playing sound...")
            stop_time = time.time()  # Engelin algılandığı zamanı kaydet
            os.system('aplay /home/fatmak/catkin_ws/src/sounds/alert.wav')  # Ses dosyasının yolu
            sound_played = True

        current_time = time.time()

        # 10 saniye boyunca dur, ses çal
        if current_time - stop_time < WAIT_TIME:
            move = Twist()
            move.linear.x = 0
            move.angular.z = 0
            pub.publish(move)
            print("Obstacle detected! Waiting for 10 seconds...")

        # 10 saniye dolduktan sonra etrafından dolanmaya başla
        else:
            print("Checking left and right for best path...")
            # Sağ ve sol açılarda mesafe ölç
            left_range = min(data.ranges[45:55])  # 50 derece sola bak
            right_range = min(data.ranges[-55:-45])  # 50 derece sağa bak

            move = Twist()
            if left_range > right_range:
                print("Turning left to avoid obstacle...")
                move.linear.x = 0.1
                move.angular.z = ANGULAR_VEL  # Sola dön
            else:
                print("Turning right to avoid obstacle...")
                move.linear.x = 0.1
                move.angular.z = -ANGULAR_VEL  # Sağa dön

            pub.publish(move)
            avoiding_obstacle = True  # Engelden kaçınıyor durumda olduğunu belirt

    else:
        obstacle_detected = False
        sound_played = False  # Eğer engel kalmadıysa sıfırla
        avoiding_obstacle = False  # Engel etrafından dolanma bitmiş demektir

# Kamera callback (şerit takibi)
def camera_callback(data):
    global prev

    try:
        # Process grayscale and binary mask
        img_grayscale = bridge.imgmsg_to_cv2(data, 'mono8')
        _, img_bin = cv2.threshold(img_grayscale, 128, 1, cv2.THRESH_BINARY_INV)

        # Compute center of mass of bottom 300 rows
        coords_bin = center_of_mass(img_bin[-300:])
        y = coords_bin[0] + OFFSET_Y
        x = coords_bin[1]

        # if CoM is NaN, take previous iteration's value of CoM
        if np.isnan(x) or np.isnan(y):
            x = prev[0]
            y = prev[1]
        else:
            prev = (x, y)

        print(f"Center of Mass: {(x,y)}")  # For debugging

        # Yeni hareket komutu için Twist objesi oluştur
        move = Twist()

        # Engel yoksa şerit takibine devam et
        if not avoiding_obstacle and not obstacle_detected:
            # Ağırlık merkezine göre şerit takibi yap
            if x < 350:  # Şerit sol tarafta, sola dön
                move.linear.x = TURN_LEFT_SPD
                move.angular.z = ANGULAR_VEL
            elif 350 <= x <= 450:  # Şerit ortada, düz git
                move.linear.x = STRAIGHT_SPD
                move.angular.z = 0
            else:  # Şerit sağda, sağa dön
                move.linear.x = TURN_RIGHT_SPD
                move.angular.z = -ANGULAR_VEL

        # Hareket komutunu yayınla
        pub.publish(move)

    except CvBridgeError as e:
        print(e)

if __name__ == '__main__':
    rospy.init_node('move_robot')

    # Bridge for converting image
    bridge = CvBridge()

    # Subscribers
    rospy.Subscriber('/camera/rgb/image_raw', Image, camera_callback)  # Kamera için
    rospy.Subscriber('/scan', LaserScan, lidar_callback)  # LIDAR için

    # Publisher for velocity commands
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    rospy.spin()
