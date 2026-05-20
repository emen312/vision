import cv2
import numpy as np
from picamera2 import Picamera2

# 初始化摄像头
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

# 创建窗口
cv2.namedWindow("Trackbars")

# 创建6个滑块：H_min, H_max, S_min, S_max, V_min, V_max
cv2.createTrackbar("L - H", "Trackbars", 0, 179, lambda x: None)
cv2.createTrackbar("L - S", "Trackbars", 0, 255, lambda x: None)
cv2.createTrackbar("L - V", "Trackbars", 0, 255, lambda x: None)
cv2.createTrackbar("U - H", "Trackbars", 179, 179, lambda x: None)
cv2.createTrackbar("U - S", "Trackbars", 255, 255, lambda x: None)
cv2.createTrackbar("U - V", "Trackbars", 255, 255, lambda x: None)

while True:
    frame = picam2.capture_array()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 获取当前滑块的值
    l_h = cv2.getTrackbarPos("L - H", "Trackbars")
    l_s = cv2.getTrackbarPos("L - S", "Trackbars")
    l_v = cv2.getTrackbarPos("L - V", "Trackbars")
    u_h = cv2.getTrackbarPos("U - H", "Trackbars")
    u_s = cv2.getTrackbarPos("U - S", "Trackbars")
    u_v = cv2.getTrackbarPos("U - V", "Trackbars")
    
    # 生成掩码
    lower_bound = np.array([l_h, l_s, l_v])
    upper_bound = np.array([u_h, u_s, u_v])
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    
    # 显示
    cv2.imshow("Original", frame)
    cv2.imshow("Mask", mask)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()