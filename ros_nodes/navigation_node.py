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

        self.callibrate_params = (0,0,0)
        self.correction_lim = 0.0

        self.wheel_radius = 0.0485          # EM METROS
        self.base_width = 0.38              # EM METROS
        self.max_speed = 15                 # EM METROS/SEGUNDO
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

        sub_angle = message_filters.Subscriber('/vision/min_angle', Float32)
        sub_intersection = message_filters.Subscriber('/vision/intersection', Point())

        self.ts = message_filters.ApproximateTimeSynchronizer([sub_angle, sub_intersection],
                                                         queue_size=10, slop= 0.1)
        
        self.ts.registerCallback(self.synced_cb)

    def current_vel_cb(self, msg):

        self.current_vel.linear.x = msg.linear.x
        self.current_vel.angular.z = msg.angular.z

    def synched_cb(self, angle_msg, intersection_msg):

        self.angle = angle_msg.data
        self.intersection = intersection_msg.data

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
    
    
    def collision_manager(self, closest_inter, params, current_vel, frame_height= 360):

        if closest_inter is None:
            status = 'I'
            return status
        
        _, y = closest_inter
        y = np.clip(y, 0, frame_height-1)               # EM PIXEIS

        self.D = self.estimate_distances(y, params)     # EM METROS

        self.collision_time = self.D / self.current_vel.linear.x if current_vel.linear.x != 0 else -1   # EM SEGUNDOS

        if self.D <= (self.base_width/2):
            status = 'C'
        
        elif 0 < self.collision_time <= 0.5:
            status = 'C'

        elif (self.base_width/2) <= self.D <= (0.475 + (self.base_width/2)):
            status = 'T'

        elif (0.475 + (self.base_width/2)) <= self.D <= 1:
            status = 'W'

        elif self.D > 1:
            status = 'F'

        return status
    

    def update_navigation(self):

        status = self.collision_manager(self.intersection, self.params, self.current_vel.linear.x, 360)

        if status == 'C':
            self.critical_protocol()

        elif status == 'T':
            self.turn_protocol()
        
        else:
            self.drive_protocol(status)

        
    def critical_protocol(self):

        self.target_vel.linear.x = 0
        self.target_vel.angular.z = 0
        self.cmd_pub.publish(self.target_vel)

        self.rate.sleep()

        error = (self.base_width/2) - self.D

        if error > 0:

            self.target_vel.linear.x = -1*error
            self.target_vel.angular.z = 0
            self.cmd_pub.publish(self.target_vel)

            time.sleep(1)
        
        else:
            self.rate.sleep()

    def drive_protocol(self, status):

        if status == 'I':
            self.target_vel.linear.x = self.std_vel
            self.target_vel.angular.z = 0

        elif status == 'F':
             
            if 80 < self.angle < 100:
                self.target_vel.linear.x = self.max_speed
                self.target_vel.angular.z = 0
                
            else:
                self.target_vel.linear.x = self.max_speed
                self.target_vel.angular.z = (self.angle*np.pi)/180 if self.angle < 90 else -1*(self.angle*np.pi)/180
            
        elif status == 'W':

            if 85 < self.angle < 95:
                self.target_vel.linear.x = self.std_vel
                self.target_vel.angular.z = 0
                
            else:
                self.target_vel.linear.x = 0
                self.target_vel.angular.z = (self.angle*np.pi)/180 if self.angle < 90 else -1*(self.angle*np.pi)/180
           

        self.cmd_pub.publish(self.target_vel)
        self.rate.sleep()

    def turn_protocol(self):

        target_vel = Twist()
        target_vel.linear.x = 0
        target_vel.angular.z = np.pi

        self.cmd_pub.publish(target_vel)
        time.sleep(1)

    
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


