import cv2
import numpy as np

def open_webcam(device):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("Não foi possível abrir a webcam")
        exit()
    return cap


def pre_process(frame):
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    gray_frame = clahe.apply(gray_frame)
    blur = cv2.GaussianBlur(gray_frame, (5, 5), 0)
    return blur


def adaptive_canny(blur):
    m = np.median(blur)
    lower = int(max(0, 0.66 * m))
    upper = int(min(255, 1.33 * m))
    edges = cv2.Canny(blur, lower, upper)
    kernel = np.ones((3, 3), np.uint8)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    return edges


def line_intersection(line1, line2):
    x1, y1, x2, y2 = line1
    x3, y3, x4, y4 = line2
    determ = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4)
    if determ == 0:
        return None
    px = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)) / determ
    py = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)) / determ
    return int(px), int(py)


def image_processing(frame):
    blur = pre_process(frame)
    edges = adaptive_canny(blur)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 120, minLineLength=80, maxLineGap=15)

    h, w = frame.shape[:2]
    center_x = w // 2
    cv2.line(frame, (center_x, 0), (center_x, h), (0, 255, 255), 2)

    step = 10
    area_left = 0
    area_right = 0

    center_line = (center_x, 0, center_x, h)
    v_center = np.array([0, h], dtype=float)

    min_angle = 90.0
    min_angle_line = None
    closest_inter = None
    max_inter_y = -1

    if lines is not None:
        for x1, y1, x2, y2 in lines[:, 0]:
            cv2.line(frame, (x1, y1), (x2, y2), (255, 199, 209), 1)

            inter = line_intersection(center_line, (x1, y1, x2, y2))
            if inter is not None:
                ix, iy = inter
                if 0 <= ix < w and 0 <= iy < h:
                    v_line = np.array([x2 - x1, y2 - y1], dtype=float)
                    dot = np.dot(v_center, v_line)
                    norms = np.linalg.norm(v_center) * np.linalg.norm(v_line)

                    if norms > 1e-6:
                        cos_theta = np.clip(dot / norms, -1.0, 1.0)
                        angle = float(np.degrees(np.arccos(cos_theta)))
                    else:
                        angle = 90.0

                    if angle < min_angle or (angle == min_angle and iy > max_inter_y):
                        min_angle = angle
                        min_angle_line = (x1, y1, x2, y2)
                        closest_inter = (ix, iy)
                        max_inter_y = iy

        for y in range(0, h, step):
            delta_xs = []
            for x1, y1, x2, y2 in lines[:, 0]:
                if x2 != x1:
                    m = (y2 - y1) / (x2 - x1)
                    b = y1 - m * x1
                    if abs(m) > 1e-6:
                        x_at_y = int((y - b) / m)
                    else:
                        x_at_y = x1
                else:
                    x_at_y = x1

                if 0 <= x_at_y < w:
                    delta_xs.append(x_at_y - center_x)

            if delta_xs:
                min_dx = min(delta_xs, key=lambda dx: abs(dx))
                x_min = center_x + min_dx
                color = (0, 255, 0) if min_dx >= 0 else (0, 0, 255)
                cv2.line(frame, (center_x, y), (x_min, y), color, 1)
                area = abs(min_dx) * step
                if min_dx < 0:
                    area_left += area
                else:
                    area_right += area

    if min_angle_line is not None and closest_inter is not None:
        x1, y1, x2, y2 = min_angle_line
        cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
        cv2.circle(frame, closest_inter, 6, (0, 255, 255), -1)
        cv2.putText(frame, f"Menor angulo: {min_angle:.2f}°", (10, h - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    return frame, area_left, area_right, closest_inter, min_angle
