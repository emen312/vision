import os
# ===== 强制设置显示环境（关键！保证窗口能弹出） =====
os.environ["DISPLAY"] = ":0"           # 大多数 VNC 是 :0，如果不行改成 :1
os.environ["QT_QPA_PLATFORM"] = "xcb"  # 强制使用 X11 后端

import cv2
import numpy as np  # ===== 1. 共享内存配置（使用 /dev/shm 避免权限问题） =====
from picamera2 import Picamera2
import mmap
import struct

# ===== 1. 共享内存配置（使用 /dev/shm 避免权限问题） =====
SHM_NAME = "/dev/shm/vision_shm"
SHM_SIZE = 32

try:
    os.unlink(SHM_NAME)
except FileNotFoundError:
    pass
shm_fd = os.open(SHM_NAME, os.O_CREAT | os.O_RDWR, 0o666)
os.ftruncate(shm_fd, SHM_SIZE)
shm = mmap.mmap(shm_fd, SHM_SIZE, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
print(f"✅ 共享内存 {SHM_NAME} 已创建")

# ===== 2. 摄像头初始化 =====
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (320, 240)}))
picam2.start()

print("按 'q' 键退出程序。")

# ===== 3. 颜色阈值（完全保留你的参数） =====
lower_red = np.array([171, 148, 128])
upper_red = np.array([179, 255, 255])

while True:
    frame = picam2.capture_array()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_red, upper_red)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # ===== 状态变量初始化 =====
    target_x, target_y, target_area = 0, 0, 0
    target_exist = 0
    locked = 0
    mode = 1
    reserved = 0
    
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) > 500:
            x, y, w, h = cv2.boundingRect(largest_contour)
            center_x = x + w // 2
            center_y = y + h // 2
            target_area = cv2.contourArea(largest_contour)
            
            # ===== 更新状态 =====
            target_x = center_x
            target_y = center_y
            target_exist = 1
            if target_area > 2000:
                locked = 1
            
            # ===== 画框与中心点（你原来的样式） =====
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.circle(frame, (center_x, center_y), 5, (0, 255, 255), -1)
            print(f"目标中心: {center_x}, {center_y}, 面积: {int(target_area)}, 锁定: {locked}")
            
            # ===== 协议打包 =====
            packet = struct.pack('>HHHBBBB', target_x, target_y, int(target_area), target_exist, locked, mode, reserved)
            shm.seek(0)
            shm.write(packet)
            shm.flush()
    
    # ===== 显示窗口（你的初稿：两个窗口） =====
    cv2.imshow("Camera Feed", frame)
    cv2.imshow("Mask", mask)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# ===== 清理 =====
picam2.stop()
cv2.destroyAllWindows()
shm.close()
os.close(shm_fd)
try:
    os.unlink(SHM_NAME)
except FileNotFoundError:
    pass
print("程序结束")