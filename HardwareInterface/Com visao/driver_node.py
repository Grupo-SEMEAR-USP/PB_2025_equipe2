import rospy
import math
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros # biblioteca p publica transformações
from tf.transformations import quaternion_from_euler

# importar interface de hardware
from hardware_interface import RobotHardwareInterface # type: ignore


WHEEL_SEPARATION = 0.30 # 30 cm distancia entre as rodas (definir)
WHEEL_RADIUS = 0.05    # 5 cm teste raio da roda 


class RobotDriverNode:
    # nó pra ser a ponte entre ROS e hardware

    def __init__(self):
        rospy.init_node('robot_driver_node')
        print("Nó Driver do Robô iniciado.")

        self.hw_interface = RobotHardwareInterface()

        # Verificar conexão
        if not self.hw_interface.is_connected():
            rospy.logwarn("Hardware não conectado. Usando modo simulação.")

        # posicao / orientação estimada (pose)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # tempo ultima atualização de odometria
        self.last_time = rospy.Time.now()

        # subscriber e publisher
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/cmd_vel_from_vision', Twist, self.vision_cmd_callback)  # ← MOVIDO para cá
        self.odom_publisher = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # timer p mandar odometria a 20hz
        rospy.Timer(rospy.Duration(1.0 / 20.0), self.publish_odometry)


    def cmd_vel_callback(self, msg):
        # callback p toda vez q um comando de velocidade chega no /cmd_vel

        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # converter veloc linear e angular p veloc rodas 
        left_wheel_vel = (linear_vel / WHEEL_RADIUS) + (angular_vel * WHEEL_SEPARATION / (2 * WHEEL_RADIUS))
        right_wheel_vel = (linear_vel / WHEEL_RADIUS) - (angular_vel * WHEEL_SEPARATION / (2 * WHEEL_RADIUS))
        
        self.hw_interface.send_velocity_command(left_wheel_vel, right_wheel_vel)


    def vision_cmd_callback(self, msg):
        """Recebe comandos da visão computacional"""
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # Converter para velocidades das rodas 
        left_wheel_vel = (linear_vel / WHEEL_RADIUS) + (angular_vel * WHEEL_SEPARATION / (2 * WHEEL_RADIUS))
        right_wheel_vel = (linear_vel / WHEEL_RADIUS) - (angular_vel * WHEEL_SEPARATION / (2 * WHEEL_RADIUS))
        
        self.hw_interface.send_velocity_command(left_wheel_vel, right_wheel_vel)
        
        rospy.loginfo(f"Controle VISÃO: linear={linear_vel:.2f}, angular={angular_vel:.2f}")


    def publish_odometry(self, event=None):
        # função chamada pelo timer p ler estado do hardware
        # calcular odomtria e mandar no ROS

        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # pegar veloc atuais das rodas do hardware interface
        left_wheel_vel, right_wheel_vel = self.hw_interface.get_current_state()

        # calcular veloc linear e angular
        linear_velocity = (WHEEL_RADIUS / 2) * (right_wheel_vel + left_wheel_vel)
        angular_velocity = (WHEEL_RADIUS / WHEEL_SEPARATION) * (right_wheel_vel - left_wheel_vel)

        # integrar veloc p calcular nova pose
        delta_x = linear_velocity * math.cos(self.theta) * dt
        delta_y = linear_velocity * math.sin(self.theta) * dt
        delta_theta = angular_velocity * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # publicar transformação 
        # odom -> base_link
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = "odom" # frame pai
        t.child_frame_id = "base_link" # frame filho

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        q = quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

        # mensagem odometria
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # posicao
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*q)

        # Veloc
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = angular_velocity

        self.odom_publisher.publish(odom)


    def run(self):
        # manter nó

        rospy.on_shutdown(self.hw_interface.close)
        rospy.spin()



if __name__ == '__main__':
    try:
        driver_node = RobotDriverNode()
        driver_node.run()
    except rospy.ROSInterruptException:
        print("Nó Driver encerrado.")
