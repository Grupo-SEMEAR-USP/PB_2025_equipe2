import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge

# Importa o código dos seus amigos
from vib import image_processing, open_webcam # type: ignore

class VisionNode:
    def __init__(self):
        rospy.init_node('vision_node')
        
        self.bridge = CvBridge()
        self.cap = None
        
        # Inicializar webcam
        try:
            self.cap = open_webcam(0)  # /dev/video0
            rospy.loginfo("Webcam inicializada com sucesso!")
        except Exception as e:
            rospy.logerr(f"Erro ao abrir webcam: {e}")
            return
        
        # Publishers para resultados da visão
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_from_vision', Twist, queue_size=10)
        self.area_left_pub = rospy.Publisher('/vision/area_left', Float32, queue_size=10)
        self.area_right_pub = rospy.Publisher('/vision/area_right', Float32, queue_size=10)
        self.intersection_pub = rospy.Publisher('/vision/intersection', String, queue_size=10)
        self.angle_pub = rospy.Publisher('/vision/angle', Float32, queue_size=10)
        
        # Publisher para imagem processada (opcional)
        self.image_pub = rospy.Publisher('/vision/processed_image', Image, queue_size=10)
        
        rospy.loginfo("Nó de visão computacional iniciado!")
        
        # Taxa de processamento (10Hz)
        self.rate = rospy.Rate(10)
        
    def process_frame(self):
        """Processa um frame da webcam usando o código dos seus amigos"""
        if self.cap is None:
            return
            
        ret, frame = self.cap.read()
        if not ret:
            rospy.logwarn("Não foi possível ler frame da webcam")
            return
        
        # Usa a função image_processing do vib.py
        processed_frame, area_left, area_right, intersection, angle = image_processing(frame)
        
        # Publicar resultados
        self.publish_results(area_left, area_right, intersection, angle)
        
        # Publicar imagem processada (opcional)
        self.publish_processed_image(processed_frame)
        
        # Gerar comando de controle baseado na visão
        self.generate_control_command(area_left, area_right, intersection, angle)
        
    def publish_results(self, area_left, area_right, intersection, angle):
        """Publica os resultados do processamento visual"""
        self.area_left_pub.publish(Float32(area_left))
        self.area_right_pub.publish(Float32(area_right))
        self.angle_pub.publish(Float32(angle))
        
        if intersection:
            inter_str = f"({intersection[0]}, {intersection[1]})"
            self.intersection_pub.publish(String(inter_str))
        
        # Log para debug
        rospy.loginfo(f"Área Esq: {area_left:.1f}, Área Dir: {area_right:.1f}, Ângulo: {angle:.1f}°")
    
    def publish_processed_image(self, frame):
        """Publica a imagem processada (opcional)"""
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.image_pub.publish(img_msg)
        except Exception as e:
            rospy.logerr(f"Erro ao publicar imagem: {e}")
    
    def generate_control_command(self, area_left, area_right, intersection, angle):
        """Gera comandos de velocidade baseado na detecção visual"""
        twist = Twist()
        
        # LÓGICA DE CONTROLE SIMPLES:
        # Seguir linhas baseado na área detectada
        
        total_area = area_left + area_right
        if total_area > 0:
            # Se tem mais área à direita, virar à direita
            if area_right > area_left * 1.5:  # 50% mais área à direita
                twist.angular.z = -0.3  # Virar à direita
                twist.linear.x = 0.1
                rospy.loginfo("📏 Virando DIREITA - Mais área à direita")
            
            # Se tem mais área à esquerda, virar à esquerda
            elif area_left > area_right * 1.5:  # 50% mais área à esquerda
                twist.angular.z = 0.3   # Virar à esquerda
                twist.linear.x = 0.1
                rospy.loginfo("📏 Virando ESQUERDA - Mais área à esquerda")
            
            # Seguir em frente se áreas balanceadas
            else:
                twist.linear.x = 0.2
                twist.angular.z = 0.0
                rospy.loginfo("📏 Seguindo em FRENTE - Áreas balanceadas")
        
        # Publicar comando
        self.cmd_vel_pub.publish(twist)
    
    def run(self):
        """Loop principal"""
        while not rospy.is_shutdown():
            self.process_frame()
            self.rate.sleep()
        
        # Liberar webcam ao fechar
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        node = VisionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
