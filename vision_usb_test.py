import os
# 强制指定显示设备，告诉 Qt 去哪里画窗口
os.environ['DISPLAY'] = ':0'      # 大多数 VNC 是 :0，如果不行改成 :1
os.environ['QT_QPA_PLATFORM'] = 'xcb'  # 强制 Qt 使用 X11 后端

import cv2
import numpy as np

cap = cv2.VideoCapture(1, cv2.CAP_V4L2) # 继续用 V4L2 驱动

if not cap.isOpened():
    print("无法打开摄像头！")
    exit()

print("摄像头已启动。按 'q' 键退出程序。")

# === 颜色阈值设置 ===
lower_red1 = np.array([0, 50, 50])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 50, 50])
upper_red2 = np.array([180, 255, 255])

while True:
    ret, frame = cap.read()
    if not ret:
        print("无法获取画面")
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) > 200:
            x, y, w, h = cv2.boundingRect(largest_contour)
            center_x = x + w // 2
            center_y = y + h // 2

            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)

            print(f"中心点坐标: X={center_x}, Y={center_y}")

    # === 正常显示窗口 ===
    cv2.imshow("USB Camera Feed", frame)
    # 如果你觉得两个窗口太卡，可以把下面这行注释掉
    cv2.imshow("Mask", mask)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("程序结束")