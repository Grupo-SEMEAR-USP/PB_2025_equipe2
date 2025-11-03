import cv2
import sys
import os
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Point

script_dir = os.path.dirname(os.path.realpath(__file__))

sys.path.append(os.path.join(script_dir, 'lib', 'visao_nav'))

from lib.visao_nav import open_webcam, image_processing

def vision_node():

    rospy.init_node('vision_node', anonymous=True)

    pub_area_l = rospy.Publisher('/vision/area_l', Float32, queue_size=10)
    pub_area_r = rospy.Publisher('/vision/area_r', Float32, queue_size=10)
    pub_min_angle = rospy.Publisher('/vision/min_angle', Float32, queue_size=10)
    pub_intersection = rospy.Publisher('/vision/intersection', Point, queue_size=10)

    cap = open_webcam(2)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        ret, frame = cap.read()

        if not ret:
                
            rospy.logwarn("Não foi possível capturar frame")
            break

        frame, area_left, area_right, closest_inter, min_angle, h = image_processing(frame)

        pub_area_l.publish(area_left)
        pub_area_r.publish(area_right)
        pub_min_angle.publish(min_angle)
        
        intersec_msg = Point()
        if closest_inter is not None:
            intersec_msg.x = float(closest_inter[0])
            intersec_msg.y = float(closest_inter[1])
            intersec_msg.z = 0.0
        
        else:
            intersec_msg.x = intersec_msg.y = intersec_msg.z = 0.0
        
        pub_intersection.publish(intersec_msg)


        cv2.imshow('Linhas', frame)

        if cv2.waitKey(1) == ord('q'):
            break

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()
    

if __name__ == '__main__':

    try:
        vision_node()
    
    except rospy.ROSInterruptException:
        pass