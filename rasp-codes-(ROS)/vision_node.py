import cv2
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Point


class VisionNode():

    def __init__(self):

        rospy.init_node('vision_node', anonymous=True)

        self.pub_min_angle_c = rospy.Publisher('/vision/min_angle_c', Float32, queue_size=10)
        self.pub_min_angle_r = rospy.Publisher('/vision/min_angle_r', Float32, queue_size=10)
        self.pub_intersection_c = rospy.Publisher('/vision/intersection_c', Point, queue_size=10)

        self.closest_inter_c = Point()
        self.min_angle_c = 90

        self.closest_inter_r = Point()
        self.min_angle_r = 90

        self.rate = rospy.Rate(10)

        self.params = (3.406410147905296e-16,0.9999999998947352,1.0)

    def exp_model(self, y, a, b, c, type):
        if type == 1:
            return a * np.exp(b * y) + c
            
        if type == 2:
            return np.log((y - c)/a)/b
        
    def estimate_distances(self,y_pixel_or_real_distance, type):
        a, b, c = self.params

        return self.exp_model(y_pixel_or_real_distance, a, b, c, type)

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
        #roi_edges = self.region_of_interest(edges)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 200, minLineLength=10, maxLineGap=10)

        h, w = frame.shape[:2]
        center_x = w // 2
        ref_y = 2* h // 3
        cv2.line(frame, (center_x, 0), (center_x, h), (255, 0, 0), 2)
        cv2.line(frame, (0, ref_y), (w, ref_y), (255, 0, 0), 2)

        center_line_x = (center_x, 0, center_x, h)
        ref_line_y = (0, ref_y, w, ref_y)

        v_center_x = np.array([0, h], dtype=float)

        min_angle_line_c = None
        min_angle_line_r = None

        max_inter_c = -1
        max_inter_r = -1

        ic_y = 0

        if lines is not None:
            for x1, y1, x2, y2 in lines[:, 0]:
                cv2.line(frame, (x1, y1), (x2, y2), (255, 199, 209), 3)

                inter_central = self.line_intersection(center_line_x, (x1, y1, x2, y2))
                inter_lateral = self.line_intersection(ref_line_y, (x1, y1, x2, y2))

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

                    if angle_c < self.min_angle_c or (angle_c == self.min_angle_c and ic_y > max_inter_c):

                        self.min_angle_c = angle_c

                        self.closest_inter_c.x = ic_x
                        self.closest_inter_c.y = ic_y
                        self.closest_inter_c.z = 0
                        closest_inter_c_visual = (ic_x, ic_y)

                        self.pub_min_angle_c.publish(self.min_angle_c)
                        self.pub_intersection_c.publish(self.closest_inter_c)

                        cv2.circle(frame, closest_inter_c_visual, 6, (0, 255, 255), -1)
                        max_inter_c = ic_y

                if 0 <= il_x < w and 0 <= il_y < h:
                     
                    ref_line = (x1 - 300, y1, 
                                x2 - 300, y2)
                     
                    v_ref_line = np.array([(x2 - 300) - (x1 - 300),
                                        y2 - y1], dtype=float)
                     
                    cv2.line(frame, (x1 - 300, y1), 
                              (x2 - 300, y2), (0, 255, 0), 2)
                     
                    inter_ref = self.line_intersection(center_line_x, ref_line)

                    if inter_ref is not None:
                        ir_x, ir_y = inter_ref
                    
                    dot_r = np.dot(v_ref_line, v_center_x)
                    norms_r = np.linalg.norm(v_ref_line) * np.linalg.norm(v_center_x)

                    if norms_r > 1e-6:
                        cos_theta_l = np.clip(dot_r / norms_r, -1.0, 1.0)
                        angle_r = np.degrees(np.arccos(cos_theta_l))
                        cv2.circle(frame, inter_central, 6, (0, 255, 255), -1)

                        cross_r = v_center_x[0] * v_ref_line[1] - v_center_x[1] * v_ref_line[0]

                        if cross_r < 0:
                            angle_r = -angle_r 

                    else:
                        angle_r = 90.0
                        cv2.circle(frame, inter_central, 6, (0, 255, 255), -1)

                    if angle_r < self.min_angle_r or (angle_r == self.min_angle_r and ir_y > max_inter_r):
                        self.min_angle_r = angle_r
                        self.closest_inter_r.x = ir_x
                        self.closest_inter_r.y = ir_y
                        self.closest_inter_r.z = 0
                        closest_inter_r_visual = (ir_x, ir_y)

                        self.pub_min_angle_r.publish(self.min_angle_r)

                        cv2.circle(frame, closest_inter_r_visual, 6, (0, 255, 255), -1)
                        max_inter_r = ir_y

        if min_angle_line_c is not None and self.closest_inter_c is not None:
            x1, y1, x2, y2 = min_angle_line_c
            #cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.putText(frame, f"Menor angulo: {abs(self.min_angle_c):.2f}°", (10, h - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        if min_angle_line_r is not None and self.closest_inter_r is not None:
            x1, y1, x2, y2 = min_angle_line_r
            #cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.putText(frame, f"Menor angulo R: {abs(self.min_angle_r):.2f}°", (10, h - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        return frame

    def run(self):

        cap = self.open_webcam(2)

        while not rospy.is_shutdown():

            ret, frame = cap.read()
            frame = cv2.rotate(frame, cv2.ROTATE_180)

            if not ret:
                    
                rospy.logwarn("Não foi possível capturar frame")
                break

            frame = self.image_processing(frame)

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