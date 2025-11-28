import cv2
import numpy as np

params = (2,10,0.1)

def exp_model(y, a, b, c, type):
    if type == 1:
        return int(a * np.exp(b * y) + c)
        
    if type == 2:
        return int(np.log((y - c)/a)/b)
    
def estimate_distances(y_pixel_or_real_distance, params, type):
    a, b, c = params

    if type == 1:
        return int(exp_model(y_pixel_or_real_distance, a, b, c, 1))
        
    if type == 2:
        return int(exp_model(y_pixel_or_real_distance, a, b, c, 2))

def open_webcam(device):

    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)

    if not cap.isOpened():
        print("Não foi possível abrir a webcam")
        exit()

    return cap
    
def color_mask(frame):

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([15, 80, 80])
        upper_yellow = np.array([40, 200, 200])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        lower_white = np.array([0, 0, 250])
        upper_white = np.array([179, 30, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        r_channel = frame[:, :, 2]
        _, r_mask = cv2.threshold(r_channel, 200, 255, cv2.THRESH_BINARY)

        kernel = np.ones((5,5), np.uint8)

        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)

        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)

        mask = cv2.bitwise_or(yellow_mask, white_mask)
        mask = cv2.bitwise_or(mask, r_mask)

        masked = cv2.bitwise_and(frame, frame, mask=mask)

        return masked

def sobel_edges(frame, ksize=3, thresh=(50, 255)):
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)

        sobelx = cv2.Sobel(blur, cv2.CV_64F, 1, 0, ksize=ksize)
        sobely = cv2.Sobel(blur, cv2.CV_64F, 0, 1, ksize=ksize)
        mag = np.sqrt(sobelx**2 + sobely**2)
        mag = np.uint8(255 * mag / np.max(mag))

        _, binary = cv2.threshold(mag, thresh[0], thresh[1], cv2.THRESH_BINARY)
        return binary

def region_of_interest(img):

        h, w = img.shape[:2]
        
        vertices = np.array([[
            (int(0.1*w), h),
            (int(0.1*w), int(0.3*h)),
            (int(0.9*w), int(0.3*h)),
            (int(0.9*w), h)
        ]], dtype=np.int32)

        mask = np.zeros_like(img)
        cv2.fillPoly(mask, vertices, 255)
        masked = cv2.bitwise_and(img, mask)
        return masked


def line_intersection(line1, line2):
        tolerance = 1e-6
        x1, y1, x2, y2 = line1
        x3, y3, x4, y4 = line2
        determ = (x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4)
        
        if abs(determ) <= tolerance:
            px = (x3 + x4)/ 2
            py = y3
            return int(px), int(py)

        px = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)) / determ
        py = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)) / determ
        return int(px), int(py)


def image_processing(frame):

        masked_frame = color_mask(frame)
        edges = sobel_edges(masked_frame, ksize=3, thresh=(50,255))
        roi_edges = region_of_interest(edges)
        lines = cv2.HoughLinesP(roi_edges, 1, np.pi/180, 200, minLineLength=10, maxLineGap=10)

        h, w = frame.shape[:2]
        center_x = w // 2
        ref_y = 2* h // 3
        cv2.line(frame, (center_x, 0), (center_x, h), (255, 0, 0), 2)
        cv2.line(frame, (0, ref_y), (w, ref_y), (255, 0, 0), 2)

        center_line_x = (center_x, 0, center_x, h)
        ref_line_y = (0, ref_y, w, ref_y)

        v_center_x = np.array([0, h], dtype=float)
        v_ref_line_y = np.array([0,w], dtype=float)

        min_angle_c = 90.0
        min_angle_l = 90.0

        min_angle_line_c = None
        min_angle_line_l = None

        max_inter_c = -1
        max_inter_l = -1

        if lines is not None:
            for x1, y1, x2, y2 in lines[:, 0]:
                cv2.line(frame, (x1, y1), (x2, y2), (255, 199, 209), 3)

                inter_central = line_intersection(center_line_x, (x1, y1, x2, y2))
                inter_lateral = line_intersection(ref_line_y, (x1, y1, x2, y2))

                if inter_central is not None and inter_lateral is not None:
                    ic_x, ic_y = inter_central
                    il_x, il_y = inter_lateral

                if 0 <= ic_x < w and 0 <= ic_y < h:

                    #cv2.line(frame, (ix_x, ix_y), (iy_x, iy_y), (238, 130, 238), 3)

                    v_line = np.array([x2 - x1, y2 - y1], dtype=float)
                    dot_c = np.dot(v_center_x, v_line)
                    norms_c = np.linalg.norm(v_center_x) * np.linalg.norm(v_line)

                    if norms_c > 1e-6:
                        cos_theta_c = np.clip(dot_c / norms_c, -1.0, 1.0)
                        angle_c = np.degrees(np.arccos(cos_theta_c))
                        cv2.circle(frame, inter_central, 6, (0, 255, 255), -1)

                        cross_c = v_center_x[0] * v_line[1] - v_center_x[1] * v_line[0]

                        if cross_c < 0:
                            angle_c = -angle_c 

                    else:
                        angle_c = 90.0
                        cv2.circle(frame, inter_central, 6, (0, 255, 255), -1)

                    if angle_c < min_angle_c or (angle_c == min_angle_c and ic_y > max_inter_c):
                        min_angle_c = angle_c
                        min_angle_line_c = (x1, y1, x2, y2)
                        closest_inter_c = (ic_x, ic_y)
                        cv2.circle(frame, closest_inter_c, 6, (0, 255, 255), -1)
                        max_inter_c = ic_y
                
                if 0 <= il_x < w and 0 <= il_y < h:
                     
                     ref_line = (x1 - estimate_distances(0.2375, params, 2), y1, 
                                 x2 - estimate_distances(0.2375, params, 2), y2)
                     
                     cv2.line(frame, ((x1 - estimate_distances(0.2375, params, 2)), y1), 
                              ((x2 - estimate_distances(0.2375, params, 2)), y2), (0, 255, 0), 2)

        if min_angle_line_c is not None and closest_inter_c is not None:
            x1, y1, x2, y2 = min_angle_line_c
            #cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.putText(frame, f"Menor angulo C: {min_angle_c:.2f}°", (10, h - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        return frame