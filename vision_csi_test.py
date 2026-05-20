import os
os.environ["QT_QPA_PLATFORM"] = "xcb"  # 强制使用 X11 后端
import cv2
import numpy as np
from picamera2 import Picamera2

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (320, 240)}))
picam2.start()

print("按 'q' 键退出程序。")

# 设定颜色范围（以红色为例）
lower_red = np.array([171, 148, 128])
upper_red = np.array([179, 255, 255])

while True:
    frame = picam2.capture_array()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_red, upper_red)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) > 500:
            x, y, w, h = cv2.boundingRect(largest_contour)
            center_x = x + w // 2
            center_y = y + h // 2
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.circle(frame, (center_x, center_y), 5, (0, 255, 255), -1)
            print(f"目标中心: {center_x}, {center_y}")
    
    # 显示画面
    cv2.imshow("Camera Feed", frame)
    cv2.imshow("Mask", mask)
    
    # 按 'q' 键退出
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()