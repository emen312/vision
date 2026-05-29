import cv2
import numpy as np
import serial
import struct
import time
import math
import socket

# =========================================================
# 1. 串口配置
# =========================================================

SERIAL_ENABLE = False
SERIAL_PORT = '/dev/ttyUSB0'
BAUDRATE = 115200

# =========================================================
# IPC 配置：Python 视觉进程 -> LibXR writer.cpp
# =========================================================

IPC_ENABLE = True

IPC_TARGET_IP = "127.0.0.1"
IPC_TARGET_PORT = 5005

# =========================================================
# 2. 摄像头配置
# =========================================================

CAMERA_ID = 0

FRAME_WIDTH = 320
FRAME_HEIGHT = 240
CAMERA_FPS = 30

PROCESS_WIDTH = 320
PROCESS_HEIGHT = 240

# 调试时 True，正式运行建议 False
SHOW_IMAGE = True
SHOW_MASK = True

FLIP_MODE = None

SEND_HZ = 20

DROP_OLD_FRAMES = 0

MAX_READ_FAILS = 30
CAMERA_REOPEN_INTERVAL = 1.0

# =========================================================
# 2.1 摄像头手动控制
# =========================================================

DISABLE_AUTOFOCUS = True
MANUAL_FOCUS_VALUE = 35

DISABLE_AUTO_EXPOSURE = False
MANUAL_EXPOSURE_VALUE = -6

# 如果 MJPG 偶发 imdecode 报错，可以改成 'YUYV'
CAMERA_FOURCC = 'MJPG'

# =========================================================
# 3. 协议配置
# =========================================================

FRAME_HEADER_1 = 0x55
FRAME_HEADER_2 = 0xAA

PROTOCOL_VERSION = 0x01
MSG_TARGET_INFO = 0x01

# =========================================================
# 4. 白色矩形识别参数
# =========================================================

WHITE_S_MAX = 90
WHITE_V_MIN = 135

GRAY_THRESHOLD = 190

# 轮廓面积范围
MIN_AREA = 180
MAX_AREA = 12000

# 外接矩形面积范围，用于过滤很小的误识别框
MIN_RECT_AREA = 900
MAX_RECT_AREA = 30000

MIN_TARGET_W = 20
MIN_TARGET_H = 20

MAX_TARGET_W = 220
MAX_TARGET_H = 170

MIN_ASPECT_RATIO = 0.45
MAX_ASPECT_RATIO = 2.80

# 轮廓面积 / 外接矩形面积
MIN_EXTENT = 0.52

# 轮廓面积 / 凸包面积
MIN_SOLIDITY = 0.86

# 非四边形兜底时使用的矩形填充率
MIN_RECTANGULARITY = 0.72

# 圆形度，过滤白炽灯/圆形反光
MAX_CIRCULARITY = 0.80

APPROX_EPSILON_RATIO = 0.025

TARGET_EXPAND_PIXELS = 0

BORDER_MARGIN = 3

IGNORE_TOP_Y = 60
IGNORE_BOTTOM_Y = 235

MAX_TARGET_JUMP = 40

MAX_SIZE_CHANGE_RATIO = 0.45

# 不长时间保留旧目标，避免目标没了还继续画框
TARGET_HOLD_TIME = 0.05

SMOOTH_ALPHA = 0.25

LOCK_ERROR_PIXEL = 20

# 置信度低于这个值，不认为 found=1
MIN_ACCEPT_CONFIDENCE = 520

# =========================================================
# 5. 预生成常量
# =========================================================

cv2.setUseOptimized(True)

SCALE_X = FRAME_WIDTH / PROCESS_WIDTH
SCALE_Y = FRAME_HEIGHT / PROCESS_HEIGHT

SEND_INTERVAL = 1.0 / SEND_HZ

KERNEL_CLOSE = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
KERNEL_OPEN = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
KERNEL_DILATE = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))

LOWER_WHITE = np.array([0, 0, WHITE_V_MIN], dtype=np.uint8)
UPPER_WHITE = np.array([180, WHITE_S_MAX, 255], dtype=np.uint8)

# =========================================================
# 6. CRC16-Modbus
# =========================================================

def crc16_modbus(data):
    crc = 0xFFFF

    for byte in data:
        crc ^= byte

        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1

    return crc & 0xFFFF


# =========================================================
# 7. 协议打包
# =========================================================

def build_target_packet(
    frame_id,
    target_x,
    target_y,
    width,
    height,
    area,
    yaw_error,
    pitch_error,
    confidence,
    found,
    locked
):
    timestamp = int(time.time() * 1000) & 0xFFFFFFFF

    payload = struct.pack(
        '<IHHHHHHHIhhHBBBB',

        timestamp,
        frame_id & 0xFFFF,

        FRAME_WIDTH,
        FRAME_HEIGHT,

        int(target_x) & 0xFFFF,
        int(target_y) & 0xFFFF,

        int(width) & 0xFFFF,
        int(height) & 0xFFFF,

        int(area) & 0xFFFFFFFF,

        int(yaw_error),
        int(pitch_error),

        int(confidence) & 0xFFFF,

        int(found) & 0xFF,
        int(locked) & 0xFF,

        0,
        0
    )

    return payload


def build_protocol_frame(msg_type, seq, payload):
    version = PROTOCOL_VERSION
    length = len(payload)

    crc_data = bytes([
        version,
        msg_type,
        seq & 0xFF,
        length & 0xFF
    ]) + payload

    crc = crc16_modbus(crc_data)

    packet = (
        bytes([FRAME_HEADER_1, FRAME_HEADER_2])
        + crc_data
        + crc.to_bytes(2, byteorder='little')
    )

    return packet


def send_target_ipc(
    ipc_sock,
    frame_id,
    cx,
    cy,
    w,
    h,
    area,
    yaw_error,
    pitch_error,
    confidence,
    found,
    locked
):
    if ipc_sock is None:
        return

    payload = build_target_packet(
        frame_id=frame_id % 65535,
        target_x=cx,
        target_y=cy,
        width=w,
        height=h,
        area=area,
        yaw_error=yaw_error,
        pitch_error=pitch_error,
        confidence=confidence,
        found=found,
        locked=locked
    )

    packet = build_protocol_frame(
        MSG_TARGET_INFO,
        frame_id % 256,
        payload
    )

    try:
        ipc_sock.sendto(
            packet,
            (IPC_TARGET_IP, IPC_TARGET_PORT)
        )
    except BlockingIOError:
        pass
    except Exception:
        pass


def init_ipc_sender():
    if not IPC_ENABLE:
        return None

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)

    print(f"[INFO] IPC UDP sender -> {IPC_TARGET_IP}:{IPC_TARGET_PORT}")

    return sock


def send_target_packet(
    ser,
    frame_id,
    cx,
    cy,
    w,
    h,
    area,
    yaw_error,
    pitch_error,
    confidence,
    found,
    locked
):
    payload = build_target_packet(
        frame_id=frame_id % 65535,
        target_x=cx,
        target_y=cy,
        width=w,
        height=h,
        area=area,
        yaw_error=yaw_error,
        pitch_error=pitch_error,
        confidence=confidence,
        found=found,
        locked=locked
    )

    packet = build_protocol_frame(
        MSG_TARGET_INFO,
        frame_id % 256,
        payload
    )

    if ser is not None:
        try:
            ser.write(packet)
        except Exception:
            pass


# =========================================================
# 8. 四点排序
# =========================================================

def order_points(pts):
    pts = pts.reshape(4, 2)

    rect = np.zeros((4, 2), dtype=np.float32)

    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1)

    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]

    return rect


# =========================================================
# 9. 对角线交点
# =========================================================

def diagonal_intersection(rect):
    p1 = rect[0]
    p2 = rect[2]
    p3 = rect[1]
    p4 = rect[3]

    A = np.array([
        [p2[0] - p1[0], p3[0] - p4[0]],
        [p2[1] - p1[1], p3[1] - p4[1]]
    ], dtype=np.float32)

    b = np.array([
        p3[0] - p1[0],
        p3[1] - p1[1]
    ], dtype=np.float32)

    det = np.linalg.det(A)

    if abs(det) < 1e-6:
        return np.mean(rect, axis=0)

    t = np.linalg.solve(A, b)

    return p1 + t[0] * (p2 - p1)


# =========================================================
# 9.1 角点亚像素修正
# =========================================================

def refine_corners(mask, rect_pts):
    if rect_pts is None:
        return rect_pts

    corners = rect_pts.astype(np.float32).reshape(-1, 1, 2)

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        20,
        0.03
    )

    try:
        refined = cv2.cornerSubPix(
            mask,
            corners,
            (5, 5),
            (-1, -1),
            criteria
        )

        refined = refined.reshape(4, 2)

        return order_points(refined)

    except cv2.error:
        return rect_pts


# =========================================================
# 10. 是否贴边
# =========================================================

def touch_border(x, y, w, h, img_w, img_h):
    if x <= BORDER_MARGIN:
        return True

    if y <= BORDER_MARGIN:
        return True

    if x + w >= img_w - BORDER_MARGIN:
        return True

    if y + h >= img_h - BORDER_MARGIN:
        return True

    return False


# =========================================================
# 11. 创建白色 mask
# =========================================================

def make_white_mask(frame_small):
    blur = cv2.GaussianBlur(frame_small, (3, 3), 0)

    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    mask_hsv = cv2.inRange(
        hsv,
        LOWER_WHITE,
        UPPER_WHITE
    )

    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)

    _, mask_gray = cv2.threshold(
        gray,
        GRAY_THRESHOLD,
        255,
        cv2.THRESH_BINARY
    )

    mask = cv2.bitwise_and(mask_hsv, mask_gray)

    mask[:IGNORE_TOP_Y, :] = 0
    mask[IGNORE_BOTTOM_Y:, :] = 0

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        KERNEL_CLOSE,
        iterations=1
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        KERNEL_OPEN,
        iterations=1
    )

    mask = cv2.dilate(
        mask,
        KERNEL_DILATE,
        iterations=1
    )

    return mask


# =========================================================
# 12. 轮廓转候选目标
# =========================================================

def contour_to_candidate(cnt, img_w, img_h, mask):
    area = cv2.contourArea(cnt)

    if area < MIN_AREA or area > MAX_AREA:
        return None

    perimeter = cv2.arcLength(cnt, True)

    if perimeter <= 0:
        return None

    x0, y0, w0, h0 = cv2.boundingRect(cnt)

    rect_area0 = w0 * h0

    if rect_area0 < MIN_RECT_AREA or rect_area0 > MAX_RECT_AREA:
        return None

    if w0 < MIN_TARGET_W or h0 < MIN_TARGET_H:
        return None

    if w0 > MAX_TARGET_W or h0 > MAX_TARGET_H:
        return None

    if touch_border(x0, y0, w0, h0, img_w, img_h):
        return None

    raw_cx = x0 + w0 / 2.0
    raw_cy = y0 + h0 / 2.0

    if raw_cy < IGNORE_TOP_Y:
        return None

    if raw_cy > IGNORE_BOTTOM_Y:
        return None

    aspect = max(w0, h0) / max(1, min(w0, h0))

    if aspect < MIN_ASPECT_RATIO or aspect > MAX_ASPECT_RATIO:
        return None

    if rect_area0 <= 0:
        return None

    extent = area / float(rect_area0)

    if extent < MIN_EXTENT:
        return None

    mask_roi = mask[y0:y0 + h0, x0:x0 + w0]

    if mask_roi.size <= 0:
        return None

    mask_fill = cv2.countNonZero(mask_roi) / float(rect_area0)

    if mask_fill < 0.45:
        return None

    hull = cv2.convexHull(cnt)
    hull_area = cv2.contourArea(hull)

    if hull_area <= 0:
        return None

    solidity = area / float(hull_area)

    if solidity < MIN_SOLIDITY:
        return None

    circularity = 4.0 * math.pi * area / (perimeter * perimeter)

    if circularity > MAX_CIRCULARITY:
        return None

    approx = cv2.approxPolyDP(
        cnt,
        APPROX_EPSILON_RATIO * perimeter,
        True
    )

    use_quad = False

    if len(approx) == 4 and cv2.isContourConvex(approx):
        rect_pts = order_points(approx)
        rect_pts = refine_corners(mask, rect_pts)
        use_quad = True

    else:
        if len(approx) < 4 or len(approx) > 6:
            return None

        min_rect = cv2.minAreaRect(cnt)

        rect_w = min_rect[1][0]
        rect_h = min_rect[1][1]

        if rect_w <= 1 or rect_h <= 1:
            return None

        min_rect_area = rect_w * rect_h

        if min_rect_area <= 0:
            return None

        rectangularity = area / float(min_rect_area)

        if rectangularity < MIN_RECTANGULARITY:
            return None

        if circularity > 0.76:
            return None

        box = cv2.boxPoints(min_rect)
        rect_pts = order_points(box.astype(np.float32))
        rect_pts = refine_corners(mask, rect_pts)

    x, y, w, h = cv2.boundingRect(rect_pts.astype(np.int32))

    final_box_area = w * h

    if final_box_area < MIN_RECT_AREA or final_box_area > MAX_RECT_AREA:
        return None

    expand = TARGET_EXPAND_PIXELS

    x = max(0, x - expand)
    y = max(0, y - expand)
    w = min(img_w - x, w + expand * 2)
    h = min(img_h - y, h + expand * 2)

    if w < MIN_TARGET_W or h < MIN_TARGET_H:
        return None

    if w > MAX_TARGET_W or h > MAX_TARGET_H:
        return None

    diag_center = diagonal_intersection(rect_pts)

    moments = cv2.moments(cnt)

    if abs(moments["m00"]) > 1e-6:
        moment_center = np.array([
            moments["m10"] / moments["m00"],
            moments["m01"] / moments["m00"]
        ], dtype=np.float32)

        center = 0.80 * diag_center + 0.20 * moment_center
    else:
        center = diag_center

    if center[1] < IGNORE_TOP_Y:
        return None

    if center[1] > IGNORE_BOTTOM_Y:
        return None

    final_rect_area = w * h

    if final_rect_area <= 0:
        return None

    final_extent = area / float(final_rect_area)

    if final_extent < MIN_EXTENT * 0.90:
        return None

    return {
        "rect": rect_pts,
        "center": center,
        "x": x,
        "y": y,
        "w": w,
        "h": h,
        "area": area,
        "rect_area": final_rect_area,
        "extent": extent,
        "solidity": solidity,
        "circularity": circularity,
        "mask_fill": mask_fill,
        "use_quad": use_quad
    }


# =========================================================
# 13. 寻找最佳白色矩形
# =========================================================

def find_best_white_rect(frame_small, last_center=None):
    mask = make_white_mask(frame_small)

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    img_h, img_w = mask.shape[:2]

    image_center = np.array([
        img_w / 2.0,
        img_h / 2.0
    ], dtype=np.float32)

    max_dist = math.sqrt(
        image_center[0] * image_center[0]
        + image_center[1] * image_center[1]
    )

    best = None
    best_score = -1.0

    for cnt in contours:
        candidate = contour_to_candidate(
            cnt,
            img_w,
            img_h,
            mask
        )

        if candidate is None:
            continue

        center = candidate["center"]

        if last_center is not None:
            jump = np.linalg.norm(center - last_center)

            if jump > MAX_TARGET_JUMP:
                continue

            track_score = 1.0 - min(jump / MAX_TARGET_JUMP, 1.0)
        else:
            track_score = 0.0

        center_dist = np.linalg.norm(center - image_center)
        center_score = 1.0 - min(center_dist / max_dist, 1.0)

        area_score = min(candidate["area"] / 5000.0, 1.0)
        extent_score = min(candidate["extent"], 1.0)
        solidity_score = min(candidate["solidity"], 1.0)
        fill_score = min(candidate.get("mask_fill", 0.0), 1.0)

        quad_bonus = 0.20 if candidate["use_quad"] else 0.0

        y = center[1]
        vertical_score = 1.0 - abs(y - img_h * 0.55) / (img_h * 0.55)
        vertical_score = max(0.0, min(1.0, vertical_score))

        if last_center is not None:
            score = (
                track_score * 0.42
                + extent_score * 0.18
                + solidity_score * 0.16
                + fill_score * 0.12
                + vertical_score * 0.06
                + area_score * 0.06
                + quad_bonus
            )
        else:
            score = (
                extent_score * 0.28
                + solidity_score * 0.22
                + fill_score * 0.18
                + vertical_score * 0.12
                + area_score * 0.10
                + center_score * 0.10
                + quad_bonus
            )

        candidate["score"] = score
        candidate["mask"] = mask

        if score > best_score:
            best_score = score
            best = candidate

    return best, mask


# =========================================================
# 14. 平滑目标
# =========================================================

def smooth_target(current, last_smooth):
    if current is None:
        return last_smooth

    if last_smooth is None:
        return current.copy()

    last_w = max(1, last_smooth["w"])
    last_h = max(1, last_smooth["h"])

    cur_w = max(1, current["w"])
    cur_h = max(1, current["h"])

    w_change = abs(cur_w - last_w) / float(last_w)
    h_change = abs(cur_h - last_h) / float(last_h)

    if w_change > MAX_SIZE_CHANGE_RATIO or h_change > MAX_SIZE_CHANGE_RATIO:
        hold = last_smooth.copy()
        hold["score"] = min(last_smooth.get("score", 0.3), 0.3)
        return hold

    smooth = current.copy()

    alpha = SMOOTH_ALPHA

    smooth["rect"] = (
        alpha * current["rect"]
        + (1.0 - alpha) * last_smooth["rect"]
    )

    smooth["center"] = (
        alpha * current["center"]
        + (1.0 - alpha) * last_smooth["center"]
    )

    x, y, w, h = cv2.boundingRect(
        smooth["rect"].astype(np.int32)
    )

    smooth["x"] = x
    smooth["y"] = y
    smooth["w"] = w
    smooth["h"] = h

    smooth["area"] = (
        alpha * current["area"]
        + (1.0 - alpha) * last_smooth["area"]
    )

    smooth["score"] = current["score"]

    return smooth


# =========================================================
# 15. 摄像头控制
# =========================================================

def set_camera_manual_controls(cap):
    if cap is None:
        return

    if DISABLE_AUTOFOCUS:
        try:
            cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            cap.set(cv2.CAP_PROP_FOCUS, MANUAL_FOCUS_VALUE)

            print("[INFO] Autofocus disabled")
            print(f"[INFO] Manual focus value: {MANUAL_FOCUS_VALUE}")
        except Exception as e:
            print(f"[WARN] set focus failed: {e}")

    if DISABLE_AUTO_EXPOSURE:
        try:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            cap.set(cv2.CAP_PROP_EXPOSURE, MANUAL_EXPOSURE_VALUE)

            print("[INFO] Auto exposure disabled")
            print(f"[INFO] Manual exposure value: {MANUAL_EXPOSURE_VALUE}")
        except Exception as e:
            print(f"[WARN] set exposure failed: {e}")


def open_camera():
    cap = cv2.VideoCapture(
        CAMERA_ID,
        cv2.CAP_V4L2
    )

    if not cap.isOpened():
        return None

    cap.set(
        cv2.CAP_PROP_FOURCC,
        cv2.VideoWriter_fourcc(*CAMERA_FOURCC)
    )

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    set_camera_manual_controls(cap)

    for _ in range(5):
        try:
            cap.grab()
        except cv2.error:
            pass

    real_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    real_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    real_fps = cap.get(cv2.CAP_PROP_FPS)
    real_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))

    fourcc_str = ''.join([
        chr((real_fourcc >> 8 * i) & 0xFF)
        for i in range(4)
    ])

    print("[INFO] Camera opened")
    print(f"[INFO] Size  : {real_w} x {real_h}")
    print(f"[INFO] FPS   : {real_fps}")
    print(f"[INFO] FOURCC: {fourcc_str}")

    return cap


def reopen_camera(cap):
    print("[WARN] Reopening camera...")

    try:
        if cap is not None:
            cap.release()
    except Exception:
        pass

    time.sleep(0.3)

    new_cap = open_camera()

    if new_cap is None:
        print("[ERROR] Camera reopen failed")
    else:
        print("[INFO] Camera reopened")

    return new_cap


# =========================================================
# 16. 主函数
# =========================================================

def main():
    if SERIAL_ENABLE:
        try:
            ser = serial.Serial(
                SERIAL_PORT,
                BAUDRATE,
                timeout=0,
                write_timeout=0
            )

            print(f"[INFO] 串口 {SERIAL_PORT} 打开成功")

        except Exception as e:
            print(f"[WARN] 串口打开失败: {e}")
            ser = None
    else:
        ser = None
        print("[INFO] Python 串口直连已关闭，视觉数据通过 IPC 发送给 LibXR")

    cap = open_camera()

    if cap is None:
        print("[ERROR] 摄像头打开失败")
        return

    ipc_sock = init_ipc_sender()

    read_fail_count = 0
    last_reopen_time = 0.0

    frame_id = 0

    last_send_time = 0.0
    last_print_time = 0.0

    fps_count = 0
    fps_time = time.time()
    fps = 0

    last_center_small = None
    last_target_time = 0.0
    last_smooth_target = None

    while True:
        now = time.time()

        try:
            for _ in range(DROP_OLD_FRAMES):
                cap.grab()
        except cv2.error as e:
            print(f"[WARN] cap.grab error: {e}")
            read_fail_count += 1
            continue

        try:
            ret, frame = cap.read()
        except cv2.error as e:
            print(f"[WARN] cap.read error: {e}")
            ret = False
            frame = None
        except Exception as e:
            print(f"[WARN] unknown camera read error: {e}")
            ret = False
            frame = None

        if not ret or frame is None:
            read_fail_count += 1

            if (
                read_fail_count >= MAX_READ_FAILS
                and now - last_reopen_time > CAMERA_REOPEN_INTERVAL
            ):
                last_reopen_time = now
                read_fail_count = 0
                cap = reopen_camera(cap)

                if cap is None:
                    time.sleep(0.5)
                    cap = open_camera()

            time.sleep(0.005)
            continue

        read_fail_count = 0

        if FLIP_MODE is not None:
            frame = cv2.flip(frame, FLIP_MODE)

        frame_id += 1

        if FRAME_WIDTH == PROCESS_WIDTH and FRAME_HEIGHT == PROCESS_HEIGHT:
            frame_small = frame
        else:
            frame_small = cv2.resize(
                frame,
                (PROCESS_WIDTH, PROCESS_HEIGHT),
                interpolation=cv2.INTER_LINEAR
            )

        target, mask = find_best_white_rect(
            frame_small,
            last_center_small
        )

        if target is None and now - last_target_time > TARGET_HOLD_TIME:
            target, mask = find_best_white_rect(
                frame_small,
                None
            )

        found = 0
        locked = 0
        confidence = 0

        cx = 0
        cy = 0
        w = 0
        h = 0
        area = 0
        yaw_error = 0
        pitch_error = 0

        draw_target = None

        if target is not None:
            last_target_time = now

            smooth = smooth_target(
                target,
                last_smooth_target
            )

            last_smooth_target = smooth
            last_center_small = smooth["center"]

            draw_target = smooth

            center_small = smooth["center"]

            cx = int(center_small[0] * SCALE_X)
            cy = int(center_small[1] * SCALE_Y)

            x = int(smooth["x"] * SCALE_X)
            y = int(smooth["y"] * SCALE_Y)
            w = int(smooth["w"] * SCALE_X)
            h = int(smooth["h"] * SCALE_Y)

            area = int(smooth["area"] * SCALE_X * SCALE_Y)

            yaw_error = cx - FRAME_WIDTH // 2
            pitch_error = cy - FRAME_HEIGHT // 2

            locked = 1 if (
                abs(yaw_error) < LOCK_ERROR_PIXEL
                and abs(pitch_error) < LOCK_ERROR_PIXEL
            ) else 0

            confidence = int(
                min(1000, max(0, smooth["score"] * 1000))
            )

            if confidence >= MIN_ACCEPT_CONFIDENCE:
                found = 1
            else:
                found = 0
                locked = 0
                confidence = 0
                draw_target = None
                last_center_small = None
                last_smooth_target = None

        elif last_smooth_target is not None and now - last_target_time < TARGET_HOLD_TIME:
            found = 0
            locked = 0
            confidence = 0
            draw_target = None

        else:
            last_center_small = None
            last_smooth_target = None

        if now - last_send_time >= SEND_INTERVAL:
            last_send_time = now

            if SERIAL_ENABLE:
                send_target_packet(
                    ser=ser,
                    frame_id=frame_id,
                    cx=cx,
                    cy=cy,
                    w=w,
                    h=h,
                    area=area,
                    yaw_error=yaw_error,
                    pitch_error=pitch_error,
                    confidence=confidence,
                    found=found,
                    locked=locked
                )

            send_target_ipc(
                ipc_sock=ipc_sock,
                frame_id=frame_id,
                cx=cx,
                cy=cy,
                w=w,
                h=h,
                area=area,
                yaw_error=yaw_error,
                pitch_error=pitch_error,
                confidence=confidence,
                found=found,
                locked=locked
            )

        if SHOW_IMAGE:
            if found and draw_target is not None:
                rect_draw = draw_target["rect"].copy()

                rect_draw[:, 0] *= SCALE_X
                rect_draw[:, 1] *= SCALE_Y
                rect_draw = rect_draw.astype(np.int32)

                cv2.polylines(
                    frame,
                    [rect_draw.reshape(-1, 1, 2)],
                    True,
                    (0, 255, 0),
                    2
                )

                for i, pt in enumerate(rect_draw):
                    cv2.circle(
                        frame,
                        tuple(pt),
                        4,
                        (255, 0, 0),
                        -1
                    )

                    cv2.putText(
                        frame,
                        f"P{i}",
                        tuple(pt + 4),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4,
                        (255, 255, 255),
                        1
                    )

                cv2.circle(
                    frame,
                    (cx, cy),
                    5,
                    (0, 0, 255),
                    -1
                )

                cv2.putText(
                    frame,
                    f"TARGET CX:{cx} CY:{cy}",
                    (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    1
                )

                cv2.putText(
                    frame,
                    f"ERR:{yaw_error},{pitch_error} CONF:{confidence} LOCK:{locked}",
                    (8, 46),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    1
                )

                cv2.putText(
                    frame,
                    f"AREA:{area} W:{w} H:{h}",
                    (8, 68),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    1
                )

            else:
                cv2.putText(
                    frame,
                    "NO TARGET",
                    (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2
                )

            cv2.putText(
                frame,
                f"FPS:{fps}",
                (8, 92),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1
            )

            cv2.imshow("White Rect Detection 320x240", frame)

            if SHOW_MASK:
                cv2.imshow("White Mask", mask)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break

        fps_count += 1

        if now - fps_time >= 1.0:
            fps = fps_count
            fps_count = 0
            fps_time = now

        if now - last_print_time >= 0.5:
            last_print_time = now

            if found:
                print(
                    f"FOUND CX={cx} CY={cy} "
                    f"W={w} H={h} "
                    f"AREA={area} "
                    f"CONF={confidence} "
                    f"LOCK={locked} "
                    f"FPS={fps}"
                )
            else:
                print(f"NO TARGET FPS={fps}")

    cap.release()

    if SHOW_IMAGE:
        cv2.destroyAllWindows()

    if ser is not None:
        ser.close()

    if ipc_sock is not None:
        ipc_sock.close()


if __name__ == '__main__':
    main()