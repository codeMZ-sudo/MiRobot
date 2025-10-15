#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock     #import para poder tomar medida de tiempo en simulación
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import pygame
import time
import math
import numpy as np

class JoystickTeleop:
    def __init__(self):
        rospy.init_node('realtime_teleop_joystick')
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)  #cmd_vel_joystick      
        self.pub_reset = rospy.Publisher('reset_robot', Bool, queue_size=10)
        
        # ROS Subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.current_pose = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
        
        # Umbral de ejes
        self.axis_deadzone = 0.25
        
        #Config distancia
        self.distancia = 1.0
        self.distancia_step = 0.1
        self.last_distancia_adj = rospy.Time.now().to_sec()
        self.adjust_cooldown = 0.075  # segundos
        
        # Mapeo de botones según tu especificación
        BTN_X = 3
        BTN_L1 = 6
        BTN_R1 = 7
        BTN_R3 = 14

        self.btn_map = {
            'X': BTN_X,
            'L1': BTN_L1,
            'R1': BTN_R1,
            'R3': BTN_R3,
        }
        
        self.pose = PoseStamped()
        self.pose.header.stamp = rospy.Time.now()
        self.pose.header.frame_id = "odom"

        # Inicializar pygame y joystick
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            rospy.logerr("No se detecta joystick conectado. Abortando.")
            raise SystemExit("Joystick no conectado")

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        rospy.loginfo(f"Joystick detectado: {self.joystick.get_name()}")
        
        print("\n------------------------------------------------:")
        print("\nCONTROLES JOYSTICK EN TIEMPO REAL:")
        print("  Eje analógicos izquierdo: Dirección de movimiento")
        print("  Eje analógicos derecho: Orientación del robot")
        print("  L1 / R1: Reducir / Incrementar distancia desde le robot al punto de llegada")
        print("  Sin entrada relevante: robot parado")

        # Estado previo para evitar spam innecesario en log
        self.prev_message = ""
        
        ''' Siempre agregar estas lineas para tener la medida de tiempo en simulación '''   #desde aqui
        
        self.running_on_sim = rospy.get_param('running_on_sim', False)      #######
        if self.running_on_sim:                                             #######
            rospy.Subscriber('/clock', Clock, self.clock_callback)          #######
            if not rospy.get_param('/use_sim_time', False):                 #######
                rospy.set_param('/use_sim_time', True)                      #######
            self.sim_time = rospy.Time.now()                                #######
        
    def clock_callback(self, msg):                                          #######
        self.sim_time = msg.clock                                           ####### 
    
    def get_time(self):
        #permite obtener el tiempo segun la bse de tiempo real o simulación llamando a esta función
        return self.sim_time.to_sec() if self.running_on_sim else (rospy.Time.now()).to_sec()   #######        #hasta aqui

    def odom_callback(self, msg):
        """
        Update current robot pose from odometry.
        """
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extract orientation (convert quaternion to euler)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, 
                          orientation_q.z, orientation_q.w]
        (_, _, theta) = euler_from_quaternion(orientation_list)
        
        self.current_pose = np.array([x, y, theta])
    
    def run(self):
        rate = rospy.Rate(20)  # 20 Hz
        while not rospy.is_shutdown():
            pygame.event.pump()  # actualizar estado de pygame

            # Leer botones (True/False)
            btns = {name: bool(self.joystick.get_button(idx)) for name, idx in self.btn_map.items()}

            now = self.get_time()
            
            if self.running_on_sim:
                if btns['R3']:
                    msg = Bool(data=True)
                    self.pub_reset.publish(msg)
            

            # 1. Direccional: ejes
            axis_x1 = self.joystick.get_axis(0)  # izquierda-derecha
            axis_y1 = self.joystick.get_axis(1)  # adelante-atras (generalmente negativo hacia adelante)
            
            axis_x2 = self.joystick.get_axis(2)  # izquierda-derecha
            axis_y2 = self.joystick.get_axis(3)  # adelante-atras (generalmente negativo hacia adelante)

            '''Lectura analogicos para dirección de movimiento'''
            # Adelante / atrás
            if axis_y1 < -self.axis_deadzone or axis_y1 > self.axis_deadzone:
                head_y = -axis_y1
            else:
                head_y = 0.0

            # Izquierda / derecha
            if axis_x1 < -self.axis_deadzone or axis_x1 > self.axis_deadzone:
                head_x = axis_x1
            else:
                head_x = 0.0
                
            ang_head = np.arctan2(head_y,head_x)

            pos_x = self.distancia*np.sin(ang_head) + self.current_pose[0]
            pos_y = -self.distancia*np.cos(ang_head) + self.current_pose[1]
            
            '''Lectura analogicos para orientación robot'''
            # Adelante / atrás
            if axis_y2 < -self.axis_deadzone or axis_y2 > self.axis_deadzone:
                ang_y = -axis_y2
            else:
                ang_y = 0.0
                
            # Izquierda / derecha
            if axis_x2 < -self.axis_deadzone or axis_x2 > self.axis_deadzone:
                ang_x = axis_x2
            else:
                ang_x = 0.0
            
            ang = np.arctan2(-ang_x,ang_y)

            # 2. L1 / R1: ajustar distancia hasta goal
            if btns['L1'] and (now - self.last_distancia_adj) >= self.adjust_cooldown:
                self.distancia = max(0.1, self.distancia - self.distancia_step)
                self.last_distancia_adj = now
                print(f"Distancia reducida: {self.distancia:.2f} m")
            elif btns['R1'] and (now - self.last_distancia_adj) >= self.adjust_cooldown:
                self.distancia = min(2.0, self.distancia + self.distancia_step)
                self.last_distancia_adj = now
                print(f"Distancia aumentada: {self.distancia:.2f} m")

            # 5. Si no se pulsa nada relevante, queda todo en cero (ya por default)
            if (head_x == 0.0 and head_y == 0.0 and ang_x == 0.0 and ang_y == 0.0):
                mensaje = "Robot parado (sin entrada activa)"
                self.pose.pose.position.z = 1.0
            else:
                self.pose.pose.position.x = pos_x
                self.pose.pose.position.y = pos_y
                self.pose.pose.position.z = 0.0   
                # convert yaw (around Z) to quaternion (roll=pitch=0)
                q = quaternion_from_euler(0.0, 0.0, ang)  # returns [x,y,z,w]
                self.pose.pose.orientation.x = q[0]
                self.pose.pose.orientation.y = q[1]
                self.pose.pose.orientation.z = q[2]
                self.pose.pose.orientation.w = q[3]
                
                self.pose.header.stamp = rospy.Time.now()
                
                #mensaje = f"Pose referida al mundo: ({self.pose.pose.position.x},{self.pose.pose.position.y}), Orientación: {ang}"
                
            # Publicar
            self.pub.publish(self.pose)

            if mensaje != self.prev_message:
                print(mensaje)
                self.prev_message = mensaje

            rate.sleep()

if __name__ == '__main__':
    try:
        node = JoystickTeleop()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except SystemExit as e:
        print(f"Saliendo: {e}")
