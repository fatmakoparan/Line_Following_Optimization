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

# --- Sabitler ---
STRAIGHT_SPD = 0.25       # İleri hız
TURN_SPEED = 0.15         # Hafif dönüş hızı
ANGULAR_VEL = 1.25        # (rad/s) zaman temelli dönüş hızı
SAFE_DISTANCE = 0.5       # Engel algılama eşiği
LANE_CHANGE_DISTANCE = 0.5  # Şeride geçiş uzunluğu
SHOULDER_DISTANCE = 0.4     # Şerit merkezinden max sapma

# --- Hedef Koordinatlar ve Durum Bayrakları ---
target_x = -3.39
target_y = 2.02
target_b_reached = False    # Hedefe ulaşıldı mı?
turned = False              # 90° dönüş yapıldı mı?

# --- Küresel Pozisyon ve Yön ---
current_x = 0.0
current_y = 0.0
current_yaw = 0.0

# --- Diğer Global Bayraklar ---
obstacle_detected = False
sound_played = False

# Zaman temelli dönüş fonksiyonu (bloklamaz, odometri almaya devam eder)
def turn_angle(target_angle):
    move = Twist()
    direction = 1 if target_angle > 0 else -1
    move.angular.z = direction * ANGULAR_VEL

    duration = abs(target_angle) / ANGULAR_VEL
    t0 = rospy.Time.now().to_sec()
    rate = rospy.Rate(20)
    while rospy.Time.now().to_sec() - t0 < duration and not rospy.is_shutdown():
        pub.publish(move)
        rate.sleep()
    pub.publish(Twist())

# Belirli mesafede düz gitme
def go_straight(distance, speed=STRAIGHT_SPD):
    global current_x, current_y
    x0, y0 = current_x, current_y
    move = Twist()
    move.linear.x = speed
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        if math.hypot(current_x - x0, current_y - y0) >= distance:
            break
        pub.publish(move)
        rate.sleep()
    pub.publish(Twist())

# Hedefe ulaşıldıktan sonra sağa sola dönüp şeride geçiş
def stop_and_turn_left():
    rospy.loginfo("🛑 Stopping and turning left...")
    pub.publish(Twist())
    rospy.sleep(1.0)

    # 90° sola
    turn_angle(math.radians(90))

    # Şeride gir
    rospy.loginfo(f"⬆️ Moving straight {LANE_CHANGE_DISTANCE}m after turn")
    go_straight(LANE_CHANGE_DISTANCE)

    rospy.loginfo("🚗 Resuming lane tracking (open-loop)")
    go_straight(2.0)

# Odometri callback: pozisyon-güncelleme ve hedef kontrolü
def odometry_callback(msg):
    global current_x, current_y, current_yaw, target_b_reached, turned
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    _, _, current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    rospy.loginfo("📍 (x: %.2f, y: %.2f), Yaw: %.2f°",
                  current_x, current_y, math.degrees(current_yaw))

    if not target_b_reached:
        # Hedefe yakınlık kontrolünü daha hassas yapalım
        if math.isclose(current_x, target_x, abs_tol=0.1) and math.isclose(current_y, target_y, abs_tol=0.1):
            rospy.loginfo("🚀 Target reached!")
            target_b_reached = True

    # Robot belirtilen konumdayken 90 derece sola dönsün ve düz gitmeye devam etsin
    if target_b_reached and not turned:
        # Hedef konumda dönüş yapmak için daha hassas kontrol
        if math.isclose(current_x, -3.27, abs_tol=0.05) and math.isclose(current_y, 1.25, abs_tol=0.05):
            rospy.loginfo("🛑 Stopping and turning left...")
            pub.publish(Twist())  # Robotu durdur
            rospy.sleep(1.0)  # Kısa bir bekleme

            # 90 derece sola dön (mevcut işlevi değiştirmeden)
            turn_angle(math.radians(90))

            # Şimdi düz gitmeye devam et
            rospy.loginfo(f"⬆️ Moving straight after turn")
            go_straight(0.2)  # Düz gitme mesafesini 0.2 metreye ayarladım

            turned = True

# LIDAR callback: engel algılama ve dolanma
def lidar_callback(data):
    global obstacle_detected, sound_played
    front_min = min(min(data.ranges[:15] + data.ranges[-15:]), data.range_max)
    if front_min < SAFE_DISTANCE:
        obstacle_detected = True
        if not sound_played:
            rospy.loginfo("🔊 Obstacle detected! Playing sound.")
            os.system('aplay /home/fatma/catkin_ws/src/sounds/alert.wav')
            sound_played = True
            etrafindan_dolan()
    else:
        obstacle_detected = False
        sound_played = False

# Engel etrafından dolanma adımları
def etrafindan_dolan():
    rospy.loginfo("↪️ Navigating around obstacle...")
    turn_angle(-math.pi/2)
    go_straight(0.3)
    turn_angle(math.pi/2)
    go_straight(0.6)
    turn_angle(math.pi/2)
    go_straight(0.3)
    turn_angle(-math.pi/2)
    rospy.loginfo("✅ Finished obstacle avoidance.")

# Kamera callback: şerit takibi
def camera_callback(data):
    global obstacle_detected
    if obstacle_detected or target_b_reached:
        return

    try:
        img = bridge.imgmsg_to_cv2(data, 'mono8')
        _, bin_img = cv2.threshold(img, 128, 255, cv2.THRESH_BINARY_INV)
        bottom = bin_img[-100:]  # Alt kısmı alıyoruz çünkü şerit genellikle alt kısmadır

        # Sol ve sağ kısmı ayırarak şerit algılamayı yapalım
        left, right = bottom[:, :bottom.shape[1]//2], bottom[:, bottom.shape[1]//2:]

        ld, rd = np.sum(left) > 1000, np.sum(right) > 1000
        move = Twist()

        if ld and not rd:
            move.angular.z = 0.5
            rospy.loginfo("🔄 Lane on left, turning left.")
        elif rd and not ld:
            move.angular.z = -0.5
            rospy.loginfo("🔄 Lane on right, turning right.")
        else:
            move.linear.x = STRAIGHT_SPD
            rospy.loginfo("⬆️ Moving straight.")

        pub.publish(move)

    except CvBridgeError as e:
        rospy.logwarn(f"⚠️ CV Bridge error: {e}")

if __name__ == "__main__":
    rospy.init_node('move_robot')

    # Publisher & CvBridge
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    bridge = CvBridge()

    # Subscriber’lar
    rospy.Subscriber('/odom', Odometry, odometry_callback)
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    rospy.Subscriber('/camera/rgb/image_raw', Image, camera_callback)

    # Ana döngü: dönüşü bir kez yönet
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if target_b_reached and not turned:
            stop_and_turn_left()
            turned = True
        rate.sleep()
