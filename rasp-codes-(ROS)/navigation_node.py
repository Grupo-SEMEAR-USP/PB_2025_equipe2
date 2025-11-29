import rospy
import time
import numpy as np
import message_filters
from std_msgs.msg import Float32
from geometry_msgs.msg import Point, Twist

from hardware_interface import RobotHardwareInterface


class NavigationNode():

    def __init__(self):
        
        rospy.init_node('navigator_node', anonymous=True)
        self.rate = rospy.Rate(10)

        self.callibrate_params = (3.406410147905296e-16,0.9999999998947352,1.0)

        self.wheel_radius = 0.0485          # EM METROS
        self.base_width = 0.38              # EM METROS
        self.max_speed = 0.2                # EM METROS/SEGUNDO
        self.std_vel = self.max_speed/2     # EM METROS/SEGUNDO

        self.current_vel = Twist()
        self.current_vel.linear.x = 0       # EM METROS/SEGUNDO
        self.current_vel.angular.z = 0      # EM RAD/SEGUNDO

        self.target_vel = Twist()
        self.target_vel.linear.x = 0        # EM METROS/SEGUNDO
        self.target_vel.angular.z = 0       # EM RAD/SEGUNDO

        self.angle = 0.0                    # EM GRAUS(?)

        self.cmd_pub = rospy.Publisher('/cmd/vel', Twist, queue_size=10)

        rospy.Subscriber('/esp/current_vel', Twist, self.current_vel_cb)

        sub_angle_c = message_filters.Subscriber('/vision/min_angle_c', Float32)
        sub_angle_r = message_filters.Subscriber('/vision/min_angle_r', Float32)
        sub_intersection_c = message_filters.Subscriber('/vision/intersection_c', Point())

        self.ts = message_filters.ApproximateTimeSynchronizer([sub_angle_c, sub_angle_r, sub_intersection_c],
                                                         queue_size=10, slop= 0.1)
        
        self.ts.registerCallback(self.synced_cb)

    def current_vel_cb(self, msg):

        self.current_vel.linear.x = msg.linear.x
        self.current_vel.angular.z = msg.angular.z

    def synched_cb(self, angle_c_msg, angle_r_msg, intersection_c_msg):

        self.angle_c = angle_c_msg.data
        self.angle_r = angle_r_msg.data
        self.intersection_c = intersection_c_msg.data

        self.update_navigation()

    def exp_model(y, a, b, c, type):
        if type == 1:
            return a * np.exp(b * y) + c
        
        if type == 2:
            return np.log((y - c)/a)/b
    
    def estimate_distances(self, y_pixel_or_real_distance, params, type):
        a, b, c = params

        if type == 1:
            return float(self.exp_model(y_pixel_or_real_distance, a, b, c, 1))
        
        if type == 2:
            return float(self.exp_model(y_pixel_or_real_distance, a, b, c, 2))
    
    
    def collision_manager(self, params, frame_height= 360):
        
        _, y, _ = self.intersection_c
        y = np.clip(y, 0, frame_height-1)               # EM PIXEIS

        self.D = self.estimate_distances(y, params)     # EM METROS
    

    def update_navigation(self):

        self.collision_manager(self.params, 360)
        self.correction_protocol()

        self.rate.sleep()

        # if self.D is not None:
        #     if self.D > 75*np.sqrt(2)/2:
        #         self.correction_protocol()

        #     else:
        #         self.turn_protocol()
        
        # else:
        #     self.correction_protocol()

    def correction_protocol(self):
        
        kp = 5
        error = self.angle_r - 90.0
        
        self.target_vel.linear.x = self.max_speed
        self.target_vel.angular.z = kp*-1*error

        self.cmd_pub(self.target_vel)

    # def turn_protocol(self):

    #     target_vel = Twist()
    #     target_vel.linear.x = 0
    #     target_vel.angular.z = np.pi

    #     self.cmd_pub.publish(target_vel)
    #     time.sleep(1)

    
    def run(self):

        rospy.loginfo('Navegador iniciado...')
        
        while not rospy.is_shutdown():

            self.update_navigation()

            rospy.spin()


if __name__ == "__main__":

    try:
        hw = RobotHardwareInterface()
        hw._read_data_loop()

        nav = NavigationNode()
        nav.run()
    
    except rospy.ROSInterruptException:
        pass

