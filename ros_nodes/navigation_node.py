import rospy
import numpy as np
import message_filters
from std_msgs.msg import Float32
from geometry_msgs.msg import Point, Twist


class NavigationNode():

    def __init__(self):
        
        rospy.init_node('navigator_node', anonymous=True)

        self.callibrate_params = (0,0,0)
        self.correction_lim = 0.0

        self.wheel_radius = 0
        self.base_width = 0

        self.current_vel = Twist()
        self.current_vel.linear.x = 0
        self.current_vel.angular.z = 0

        self.area_l = 0.0
        self.area_r = 0.0
        self.angle = 0.0
        self.intersection = Point()

        self.cmd_pub = rospy.Publisher('/cmd/target_vel', Twist, queue_size=10)

        rospy.Subscriber('/cmd/current_vel', Twist, self.current_vel_cb)

        sub_area_l = message_filters.Subscriber('/vision/area_l', Float32)
        sub_area_r = message_filters.Subscriber('/vision/area_r', Float32)
        sub_angle = message_filters.Subscriber('/vision/min_angle', Float32)
        sub_intersection = message_filters.Subscriber('/vision/intersection', Point)

        self.ts = message_filters.ApproximateTimeSynchronizer([sub_area_l, sub_area_r, sub_angle, sub_intersection],
                                                         queue_size=10, slop= 0.1)
        
        self.ts.registerCallback(self.synced_cb)

    def current_vel_cb(self, msg):

        self.current_vel.linear.x = msg.linear.x
        self.current_vel.angular.z = msg.angular.z

    def synced_cb(self, area_l_msg, area_r_msg, angle_msg, intersection_msg):

        self.area_l = area_l_msg.data
        self.area_r = area_r_msg.data
        self.angle = angle_msg.data
        self.intersection = intersection_msg.data

        self.update_navigation()

    
    def exp_model(y, a, b, c):
        return a * np.exp(b * y) + c
    
    def estimate_distances(self, y_pixel, params):
        a, b, c = params
        return float(self.exp_model(y_pixel, a, b, c))
    
    
    def collision_manager(self, closest_inter, params, current_vel, frame_height= 360):

        if closest_inter is None:
            status = 'I'
            return status
        
        _, y = closest_inter
        y = np.clip(y, 0, frame_height-1)

        D = self.estimate_distances(y, params)
        D_rad = D / self.w_radius

        collision_time = D_rad / current_vel if current_vel != 0 else -1

        if D <= self.base_width:
            status = 'C'

        elif D > self.base_width:

            if collision_time >= 1.5:
                status = 'F'
            
            elif 0.5 <= collision_time < 1.5:
                status = 'W'
            
            elif 0 < collision_time < 0.5:
                status = 'C'
            
            else:
                status = 'I'

        return status
    
    
    def area_comp(self, area_l, area_r):

        if area_l > area_r:

            larger_side = area_l
            f = area_l / area_r

        elif area_r > area_l:

            larger_side = area_r
            f = area_r / area_l

        else:
            f = 1
            larger_side = None

        return f, larger_side
    

    def update_navigation(self):

        status = self.collision_manager(self.intersection, self.params, self.current_vel.linear.x, 360)
        f, larger_side = self.area_comp(self.area_l, self.area_r)

        if status == 'C':
            self.critical_protocol()

        elif status == 'I':
            if f <= self.correction_lim:
                self.straight_protocol()

            elif f > self.correction_lim:
                self.angular_correction_protocol(f, larger_side, self.angle, status)
        
        elif status == 'F':
            if self.angle <= 80 or self.angle >= 100:
                self.angular_correction_protocol(f, larger_side, self.angle, status)
        
    def critical_protocol(self):
        pass

    def straight_protocol(self):
        pass

    def angular_correction_protocol(self, f, larger_side, angle, status):
        pass

    def run(self):
        
        while not rospy.is_shutdown():

            rospy.loginfo('Navegador iniciado...')

            rospy.spin()

if __name__ == "__main__":

    try:
        NavigationNode()
    
    except rospy.ROSInterruptException:
        pass
