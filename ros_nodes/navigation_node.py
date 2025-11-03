import cv2
import sys
import os
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from message_filters import Subscriber, ApproximateTimeSynchronizer

script_dir = os.path.dirname(os.path.realpath(__file__))

sys.path.append(os.path.join(script_dir, 'lib', 'visao_nav'))

from lib.visao_nav import collision_manager, area_comp, ang_comp

def vision_callback(area_l_msg, area_r_msg, min_angle_msg, intersection_msg):

    area_l = area_l_msg.data
    area_r = area_r_msg.data
    min_angle = min_angle_msg.data
    intersection = intersection_msg.data

def navigator_node():

    rospy.init_node('navigator_node', anonymous= True)

    sub_area_l = Subscriber('/vision/area_l', Float32)
    sub_area_r = Subscriber('/vision/area_r', Float32)
    sub_min_angle = Subscriber('/vision/min_angle', Float32)
    sub_intersection = Subscriber('/vision/intersection', Point)

    ats = ApproximateTimeSynchronizer([sub_area_l, sub_area_r, sub_min_angle, sub_intersection],
                                      queue_size= 10, slop= 0.1)
    
    ats.registerCallback(vision_callback)
    
    rospy.loginfo('Navegador iniciado...')

    rospy.spin()

if __name__ == "__main__":

    try:
        navigator_node()
    
    except rospy.ROSInterruptException:
        pass
