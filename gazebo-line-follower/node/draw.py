#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import time

# Görseli yükle
image_path = "/home/fatma/Downloads/paths.jpeg"
img = cv2.imread(image_path)
overlay = img.copy()

# Renk tanımlamaları (BGR formatında)
green =  (118, 239, 186)  # A* Algoritması
blue = (202, 61, 8)   # Dijkstra Algoritması
pink = (8, 61, 202)   # Optimize değil
start_color = (128, 0, 128)  # Başlangıç noktası (Mor)
end_color = (0, 0, 255)  # Bitiş noktası (Kırmızı)

# Koordinat tanımlamaları
points = {
    1: (40, 44),  2: (40, 218),  3: (401, 44),
    4: (401, 127), 5: (226, 127), 6: (401, 218),
    7: (316, 218), 8: (136, 218), 9: (226, 218)
}

# Çizgileri renklendiren fonksiyon
def draw_path(image, path, color, label, custom_text_pos=None, thickness=3, delay=0.5):
    for i in range(len(path) - 1):
        start, end = points[path[i]], points[path[i + 1]]
        
        # Çizgiyi çiz
        cv2.line(image, start, end, color, thickness)
        
        # Küçük ok ekle
        cv2.arrowedLine(image, start, end, color, thickness=2, tipLength=0.1) 
        
        # Görüntüyü göster
        cv2.imshow("Path Animation", image)
        cv2.waitKey(int(delay * 1000))  # Milisaniye cinsinden bekletme
    
    # Eğer özel bir yazı konumu verilmişse onu kullan, yoksa otomatik hesapla
    if custom_text_pos:
        mid_x, mid_y = custom_text_pos
    else:
        mid_x, mid_y = 0, 0
        for p in path:
            mid_x += points[p][0]
            mid_y += points[p][1]
        mid_x //= len(path)
        mid_y //= len(path)

    # Yazıyı ekleyelim
    cv2.putText(image, label, (mid_x, mid_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    return image

# Yeşil yol (A* Algoritması) → (1 → 2 → 8 → 9)
overlay = draw_path(overlay, [1, 2, 8, 9], green, "A* Algoritmasi")

# Mavi yol (Dijkstra Algoritması) → (1 → 3 → 4 → 6 → 7 → 9)
overlay = draw_path(overlay, [1, 3, 4, 6, 7, 9], blue, "Dijkstra Algoritmasi", custom_text_pos=(143, 28))

# Pembe yol (Optimize değil) → (4 → 5 → 9)
overlay = draw_path(overlay, [4, 5, 9], pink, "Optimize Degil")

# Başlangıç ve bitiş noktalarını ekle
cv2.circle(overlay, points[1], 5, start_color, -1)  # Başlangıç noktası (Mor, Küçük)
cv2.circle(overlay, points[9], 5, end_color, -1)  # Bitiş noktası (Kırmızı, Küçük)

# Son görüntüyü göster
cv2.imshow("Final Path", overlay)
cv2.waitKey(0)
cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('draw_path_node', anonymous=True)
    rospy.loginfo("Path drawing node is running...")
    rospy.spin()  # ROS node'unun çalışmasını sağla

