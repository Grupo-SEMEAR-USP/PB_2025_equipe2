import cv2
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Point


class VisionNode():

    def __init__(self):

        rospy.init_node('vision_node', anonymous=True)

        self.pub_min_angle = rospy.Publisher('/vision/min_angle', Float32, queue_size=10)
        self.pub_intersection = rospy.Publisher('/vision/intersection', Point, queue_size=10)

        self.closest_inter = 0
        self.min_angle = 0

        self.rate = rospy.Rate(10)

    def open_webcam(self, device):

        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)

        if not cap.isOpened():
            rospy.logwarn("Não foi possível abrir a webcam")
            exit()

        return cap
    
    def color_mask(self, frame):

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

    def sobel_edges(self, frame, ksize=3, thresh=(50, 255)):
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)

        sobelx = cv2.Sobel(blur, cv2.CV_64F, 1, 0, ksize=ksize)
        sobely = cv2.Sobel(blur, cv2.CV_64F, 0, 1, ksize=ksize)
        mag = np.sqrt(sobelx**2 + sobely**2)
        mag = np.uint8(255 * mag / np.max(mag))

        _, binary = cv2.threshold(mag, thresh[0], thresh[1], cv2.THRESH_BINARY)
        return binary

    def region_of_interest(self, img):

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


    def line_intersection(self, line1, line2):
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


    def image_processing(self, frame):

        masked_frame = self.color_mask(frame)

        edges = self.sobel_edges(masked_frame, ksize=3, thresh=(50,255))

        roi_edges = self.region_of_interest(edges)
    
        lines = cv2.HoughLinesP(roi_edges, 1, np.pi/180, 200, minLineLength=10, maxLineGap=10)

        h, w = frame.shape[:2]
        center_x = w // 2
        ref_y = 2* h // 3
        cv2.line(frame, (center_x, 0), (center_x, h), (255, 0, 0), 2)
        cv2.line(frame, (0, ref_y), (w, ref_y), (255, 0, 0), 2)

        center_line_x = (center_x, 0, center_x, h)
        ref_line_y = (0, ref_y, w, ref_y)
        v_center_x = np.array([0, h], dtype=float)

        min_angle = 90.0
        min_angle_line = None
        closest_inter = None
        max_inter_y = -1

        if lines is not None:
            for x1, y1, x2, y2 in lines[:, 0]:
                cv2.line(frame, (x1, y1), (x2, y2), (255, 199, 209), 3)

                inter_x = self.line_intersection(center_line_x, (x1, y1, x2, y2))
                inter_y = self.line_intersection(ref_line_y, (x1, y1, x2, y2))

                if inter_x is not None and inter_y is not None:
                    ix_x, ix_y = inter_x
                    iy_x, iy_y = inter_y

                if 0 <= ix_x < w and 0 <= ix_y < h and iy_x < w and iy_y < h:

                    #cv2.line(frame, (ix_x, ix_y), (iy_x, iy_y), (238, 130, 238), 3)

                    v_line = np.array([x2 - x1, y2 - y1], dtype=float)
                    dot = np.dot(v_center_x, v_line)
                    norms = np.linalg.norm(v_center_x) * np.linalg.norm(v_line)

                    if norms > 1e-6:
                        cos_theta = np.clip(dot / norms, -1.0, 1.0)
                        angle = float(np.degrees(np.arccos(cos_theta)))
                    else:
                        angle = 90.0
                        cv2.circle(frame, closest_inter, 6, (0, 255, 255), -1)

                    if angle < min_angle or (angle == min_angle and ix_y > max_inter_y):
                        self.min_angle = angle
                        min_angle_line = (x1, y1, x2, y2)
                        self.closest_inter = (ix_x, ix_y)
                        cv2.circle(frame, closest_inter, 6, (0, 255, 255), -1)
                        max_inter_y = ix_y

        if min_angle_line is not None and closest_inter is not None:
            x1, y1, x2, y2 = min_angle_line
            #cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.putText(frame, f"Menor angulo: {min_angle:.2f}°", (10, h - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        return frame


    def run(self):

        cap = self.open_webcam(2)

        while not rospy.is_shutdown():

            ret, frame = cap.read()

            if not ret:
                    
                rospy.logwarn("Não foi possível capturar frame")
                break

            frame = self.image_processing(frame)

            self.pub_min_angle.publish(self.min_angle)
            
            intersec_msg = Point()
            if self.closest_inter is not None:
                intersec_msg.x = float(self.closest_inter[0])
                intersec_msg.y = float(self.closest_inter[1])
                intersec_msg.z = 0.0
            
            else:
                intersec_msg.x = intersec_msg.y = intersec_msg.z = 0.0
            
            self.pub_intersection.publish(intersec_msg)


            cv2.imshow('Linhas', frame)

            if cv2.waitKey(1) == ord('q'):
                break

            self.rate.sleep()

        cap.release()
        cv2.destroyAllWindows()
    

if __name__ == '__main__':

    try:
        v = VisionNode()
        v.run()
    
    except rospy.ROSInterruptException:
        pass