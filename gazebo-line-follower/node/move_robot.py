#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import os
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan

# Constants
STRAIGHT_SPD = 0.25  # Şeritte ilerleme hızı
TURN_SPEED = 0.15    # Dönüş hızında hafif hareket
ANGULAR_VEL = 1.25   # Dönüş açısal hızı
SAFE_DISTANCE = 1.0  # Engelden kaçınma mesafesi
WAIT_TIME = 10       # Engel tespit edildikten sonra bekleme süresi (saniye)
SHOULDER_DISTANCE = 0.4  # Şeritten en fazla uzaklaşabileceği mesafe (40 cm)

# Bitiş noktası koordinatları
target_x = 3.59106  # M noktası x
target_y = -2.3663  # M noktası y
target_reached = False  # Bitişe ulaşıldı mı?

# Global Variables
obstacle_detected = False  # Engel var mı
sound_played = False       # Engel sesi çalındı mı
stop_time = 0              # Engel sonrası bekleme zamanı
in_lane = False            # Şerit içinde mi?
lane_offset = 0            # Şerit merkezinden olan uzaklık
current_yaw = 0.0          # Mevcut yaw açısı

# Odometry callback to get current yaw
def odometry_callback(data):
    global current_yaw, target_reached

    # Yaw açısını hesapla
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    _, _, current_yaw = euler_from_quaternion(orientation_list)

    # Mevcut konum bilgisi
    current_x = data.pose.pose.position.x
    current_y = data.pose.pose.position.y

    # Bitişe yakınlığı kontrol et
    if abs(current_x - target_x) < 0.1 and abs(current_y - target_y) < 0.1:
        rospy.loginfo("Hedefe ulaşıldı!")
        target_reached = True

# LIDAR callback for obstacle detection
def lidar_callback(data):
    global obstacle_detected, sound_played, stop_time

    min_range = min(min(data.ranges[:15] + data.ranges[-15:]), data.range_max)
    if min_range < SAFE_DISTANCE:
        obstacle_detected = True
        if not sound_played:
            rospy.loginfo("Engel algılandı, ses çalıyor...")
            os.system('aplay /home/fatmak/catkin_ws/src/sounds/alert.wav')
            sound_played = True
            etrafindan_dolan()
    else:
        obstacle_detected = False
        sound_played = False

# Yaw açısı ile 90 derece dönme fonksiyonu
def turn_angle(target_angle):
    global current_yaw

    move = Twist()
    target_yaw = current_yaw + target_angle

    # Açının [-pi, pi] aralığında kalmasını sağla
    if target_yaw > 3.14159:
        target_yaw -= 2 * 3.14159
    elif target_yaw < -3.14159:
        target_yaw += 2 * 3.14159

    # Hedef açıya ulaşana kadar dön
    while abs(target_yaw - current_yaw) > 0.05:
        move.angular.z = 0.5 if target_angle > 0 else -0.5
        pub.publish(move)
        rospy.sleep(0.1)

    # Dönüş tamamlandıktan sonra dur
    move.angular.z = 0
    pub.publish(move)

# Engelin etrafından dolanma fonksiyonu
def etrafindan_dolan():
    rospy.loginfo("Engelin etrafından dolanıyor...")

    # Adım 1: 90 derece sağa dön
    turn_angle(-3.14159 / 2)

    # Adım 2: 50 cm ileri git
    move = Twist()
    move.linear.x = STRAIGHT_SPD
    pub.publish(move)
    rospy.sleep(2)  # Yaklaşık 50 cm ileri gitmek için bekleme

    # Adım 3: 90 derece sola dön
    turn_angle(3.14159 / 2)

    # Adım 4: 2 metre düz ilerle
    move = Twist()
    move.linear.x = STRAIGHT_SPD
    pub.publish(move)
    rospy.sleep(8)  # Yaklaşık 2 metre ileri gitmek için bekleme

    # Adım 5: 90 derece sola dön
    turn_angle(3.14159 / 2)

    # Adım 6: 50 cm düz ilerle
    move = Twist()
    move.linear.x = STRAIGHT_SPD
    pub.publish(move)
    rospy.sleep(2)  # Yaklaşık 50 cm ileri gitmek için bekleme

    # Adım 7: 90 derece sağa dön
    turn_angle(-3.14159 / 2)

    rospy.loginfo("Engelin etrafından dolanma tamamlandı, şerit takibine dönüyor...")

# Kamera callback (şerit takibi)
def camera_callback(data):
    global in_lane, target_reached, lane_offset

    if target_reached:
        # Hedefe ulaşıldıysa robotu durdur
        move = Twist()
        move.linear.x = 0
        move.angular.z = 0
        pub.publish(move)
        return

    try:
        img_grayscale = bridge.imgmsg_to_cv2(data, 'mono8')
        _, img_bin = cv2.threshold(img_grayscale, 128, 255, cv2.THRESH_BINARY_INV)

        # Görüntünün alt kısmını ikiye bölelim
        bottom_half = img_bin[-100:]  # Görüntünün alt kısmı
        left_half = bottom_half[:, :bottom_half.shape[1] // 2]
        right_half = bottom_half[:, bottom_half.shape[1] // 2:]

        # Sol veya sağda şerit olup olmadığını kontrol et
        left_detected = np.sum(left_half) > 1000
        right_detected = np.sum(right_half) > 1000

        move = Twist()

        if not obstacle_detected:
            if left_detected and not right_detected:
                # Şerit solda kaldıysa, sola dönerek yaklaş
                lane_offset = 0.5  # Şerit merkezine uzaklık tahmini (solda)
                if lane_offset >= SHOULDER_DISTANCE:
                    move.linear.x = STRAIGHT_SPD / 2
                    move.angular.z = 0.5  # Hafif sola yönel
                    rospy.loginfo("Şerit solda, sola yöneliyor.")
                else:
                    move.linear.x = STRAIGHT_SPD
                    move.angular.z = 0
                    rospy.loginfo("Şeride yakın, düz ilerliyor.")
            elif right_detected and not left_detected:
                # Şerit sağda kaldıysa, sağa dönerek yaklaş
                lane_offset = -0.5  # Şerit merkezine uzaklık tahmini (sağda)
                if abs(lane_offset) >= SHOULDER_DISTANCE:
                    move.linear.x = STRAIGHT_SPD / 2
                    move.angular.z = -0.5  # Hafif sağa yönel
                    rospy.loginfo("Şerit sağda, sağa yöneliyor.")
                else:
                    move.linear.x = STRAIGHT_SPD
                    move.angular.z = 0
                    rospy.loginfo("Şeride yakın, düz ilerliyor.")
            elif left_detected and right_detected:
                # Şerit ortada, düz git
                lane_offset = 0  # Şeridin merkezinde
                move.linear.x = STRAIGHT_SPD
                move.angular.z = 0
                rospy.loginfo("Şerit ortada, düz ilerliyor.")
            else:
                # Şerit algılanmadı, yavaş ilerleyerek arama yap
                move.linear.x = TURN_SPEED / 2
                rospy.loginfo("Şerit algılanmadı, arıyor.")

        pub.publish(move)

    except CvBridgeError as e:
        print(e)

if __name__ == "__main__":
    rospy.init_node('move_robot')

    bridge = CvBridge()

    rospy.Subscriber('/camera/rgb/image_raw', Image, camera_callback)
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    rospy.Subscriber('/odom', Odometry, odometry_callback)  # Odometry verisi

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    rospy.spin()

