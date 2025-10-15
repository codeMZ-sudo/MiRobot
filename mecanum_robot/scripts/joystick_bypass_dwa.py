#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock     #import para poder tomar medida de tiempo en simulación
from std_msgs.msg import Bool
import pygame
import time
import math

class JoystickTeleop:
    def __init__(self):
        rospy.init_node('realtime_teleop_joystick')
        self.pub = rospy.Publisher('/cmd_vel_joystick', Twist, queue_size=1)  #cmd_vel_joystick
        self.twist = Twist()
        
        self.pub_reset = rospy.Publisher('reset_robot', Bool, queue_size=10)
        
        '''Modo joystick'''
        
        # Velocidades iniciales y pasos
        self.linear_speed = 0.1   # m/s
        self.angular_speed = 0.5   # rad/s
        self.linear_step = 0.05
        self.angular_step = 0.25

        # Para limitar la tasa de ajuste de velocidades (evitar cambios demasiado rápidos)
        self.last_ang_adj = 0.0
        self.last_lin_adj = 0.0
        self.adjust_cooldown = 0.075  # segundos
        
        '''Modo pruebas de velocidad'''
        
        # Velocidades iniciales y pasos
        self.tiempo= 15.0 # s
        self.distancia = 3.0   # m
        
        self.tiempo_step = 2.5 #s
        self.distancia_step = 0.5 #m
        
        self.speed = self.distancia/self.tiempo 
        
        self.flag_start = False
        self.flag_stop = False
        
        self.direccion = 1
        
        self.selec_mode = True

        # Umbral de ejes
        self.axis_deadzone = 0.5

        # Mapeo de botones según tu especificación
        BTN_X = 1
        BTN_CUADRADO = 3
        BTN_L1 = 8
        BTN_R1 = 9
        BTN_L2 = 6
        BTN_R2 = 7
        BTN_START = 11
        BTN_SELECT = 10
        BTN_L3 = 13
        BTN_R3 = 14

        self.btn_map = {
            'X': BTN_X,
            'CUADRADO': BTN_CUADRADO,
            'L1': BTN_L1,
            'R1': BTN_R1,
            'L2': BTN_L2,
            'R2': BTN_R2,
            'START': BTN_START,
            'SELECT': BTN_SELECT,
            'L3': BTN_L3,
            'R3': BTN_R3,
        }

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
        print("  SELECT: Selección de modo (Joystick/Pruebas)")
        print("  Default Joystick")
        
        print("\n------------------------------------------------:")
        print("\nCONTROLES JOYSTICK EN TIEMPO REAL:")
        print("  Ejes analógicos: Adelante/Atrás/Izquierda/Derecha")
        print("  L1 / R1 : Menor / Mayor velocidad angular")
        print("  L2 / R2 : Menor / Mayor velocidad lineal")
        print("  X / Cuadrado : Giro horario / antihorario")
        print("  Start : Paro de emergencia")
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
    
    def stop(self):
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.angular.z = 0.0
        # publicamos inmediatamente y saltamos ajustes
        self.pub.publish(self.twist)  

    def run(self):
        rate = rospy.Rate(50)  # 20 Hz
        while not rospy.is_shutdown():
            pygame.event.pump()  # actualizar estado de pygame

            self.twist = Twist()  # Default: parado

            # Leer botones (True/False)
            btns = {name: bool(self.joystick.get_button(idx)) for name, idx in self.btn_map.items()}

            now = self.get_time()
            
            if self.running_on_sim:
                if btns['R3']:
                    msg = Bool(data=True)
                    self.pub_reset.publish(msg)
            
            # 6. Start = paro de emergencia
            if btns['SELECT']:
                self.selec_mode = not self.selec_mode
                mensaje = "Joystick" if self.selec_mode else "Modo pruebas de velocidad"
                rospy.logwarn(f"Cambio de modo a: {mensaje}")
                if self.selec_mode:
                    print("\n------------------------------------------------:")
                    print("\nCONTROLES JOYSTICK EN TIEMPO REAL:")
                    print("  Ejes analógicos: Adelante/Atrás/Izquierda/Derecha")
                    print("  L1 / R1 : Menor / Mayor velocidad angular")
                    print("  L2 / R2 : Menor / Mayor velocidad lineal")
                    print("  X / Cuadrado : Giro horario / antihorario")
                    print("  Start : Paro de emergencia")
                    print("  Sin entrada relevante: robot parado")
                    self.flag_start = False
                    self.flag_stop = False
                else:
                    print("\n------------------------------------------------:")
                    print("\nCONTROLES PRUEBA DE VELOCIDAD:")
                    print("  START / X : Inicir / Detener prueba")
                    print("  L1 / R1 : Menor / Mayor distancia")
                    print("  L2 / R2 : Menor / Mayor duración prueba")
                    print("  Ejes analógicos: Seleccionar dirección de movimiento")
                    print("\n")
                    print(f"Parametros de prueba - Distancia: {self.distancia}, en tiempo: {self.tiempo}")
                rospy.sleep(0.5)
            
            if self.selec_mode:
                
                # 6. Start = paro de emergencia
                if btns['START']:
                    #twist = Twist()  # todo cero
                    self.twist.linear.x = 0.0
                    self.twist.linear.y = 0.0
                    self.twist.angular.z = 0.0
                    # publicamos inmediatamente y saltamos ajustes
                    self.pub.publish(self.twist)
                    if mensaje != self.prev_message:
                        print(mensaje)
                        self.prev_message = mensaje
                    rate.sleep()
                    continue  # siguiente ciclo

                # 1. Direccional: ejes
                axis_x = self.joystick.get_axis(0)  # izquierda-derecha
                axis_y = self.joystick.get_axis(1)  # adelante-atras (generalmente negativo hacia adelante)

                movimiento = None
                # Adelante / atrás
                if axis_y < -self.axis_deadzone:
                    self.twist.linear.x = self.linear_speed
                    movimiento = "Adelante"
                elif axis_y > self.axis_deadzone:
                    self.twist.linear.x = -self.linear_speed
                    movimiento = "Atrás"

                # Izquierda / derecha (omnidireccional en y)
                if axis_x < -self.axis_deadzone:
                    self.twist.linear.y = self.linear_speed
                    movimiento = "Izquierda"
                elif axis_x > self.axis_deadzone:
                    self.twist.linear.y = -self.linear_speed
                    movimiento = "Derecha"

                # 4. Giro horario / antihorario por botones X y cuadrado
                if btns['X']:
                    # Giro horario: en ROS convencional, giro horario es negativo en z
                    self.twist.angular.z = -self.angular_speed
                    movimiento = "Giro horario"
                elif btns['CUADRADO']:
                    self.twist.angular.z = self.angular_speed
                    movimiento = "Giro antihorario"

                # 2. L1 / R1: ajustar velocidad angular
                if btns['L1'] and (now - self.last_ang_adj) >= self.adjust_cooldown:
                    self.angular_speed = max(0.25, self.angular_speed - self.angular_step)
                    self.last_ang_adj = now
                    print(f"Velocidad angular reducida: {self.angular_speed:.2f} rad/s")
                elif btns['R1'] and (now - self.last_ang_adj) >= self.adjust_cooldown:
                    self.angular_speed = min(1.5, self.angular_speed + self.angular_step)
                    self.last_ang_adj = now
                    print(f"Velocidad angular aumentada: {self.angular_speed:.2f} rad/s")

                # 3. L2 / R2: ajustar velocidad lineal
                if btns['L2'] and (now - self.last_lin_adj) >= self.adjust_cooldown:
                    self.linear_speed = max(0.05, self.linear_speed - self.linear_step)
                    self.last_lin_adj = now
                    print(f"Velocidad lineal reducida: {self.linear_speed:.2f} m/s")
                elif btns['R2'] and (now - self.last_lin_adj) >= self.adjust_cooldown:
                    self.linear_speed = min(0.5, self.linear_speed + self.linear_step)
                    self.last_lin_adj = now
                    print(f"Velocidad lineal aumentada: {self.linear_speed:.2f} m/s")

                # 5. Si no se pulsa nada relevante, queda todo en cero (ya por default)
                if (self.twist.linear.x == 0.0 and self.twist.linear.y == 0.0 and self.twist.angular.z == 0.0):
                    mensaje = "Robot parado (sin entrada activa)"
                else:
                    # Determinar mensaje con prioridad: giro > movimiento lineal
                    if self.twist.angular.z != 0.0 and (btns['X'] or btns['CUADRADO']):
                        mensaje = "Giro horario" if btns['X'] else "Giro antihorario"
                    elif movimiento:
                        mensaje = movimiento
                    else:
                        mensaje = "Comando activo"
            else:
                # 6. Detener prueba
                if btns['CUADRADO']:
                    self.stop()
                    self.flag_stop = True
                    mensaje = "Prueba detenida por el usuario"
                
                # 2. L1 / R1: ajustar distancia
                if btns['L1'] and (now - self.last_ang_adj) >= self.adjust_cooldown and not self.flag_start :
                    self.distancia = max(1.0, self.distancia - self.distancia_step)
                    self.last_ang_adj = now
                    print(f"Distancia reducida: {self.distancia:.2f} m")
                    #Ajustar velocidad
                    self.speed = self.distancia/self.tiempo 
                elif btns['R1'] and (now - self.last_ang_adj) >= self.adjust_cooldown and not self.flag_start:
                    self.distancia = min(9.0, self.distancia + self.distancia_step)
                    self.last_ang_adj = now
                    print(f"Distancia aumentada: {self.distancia:.2f} m")
                    #Ajustar velocidad
                    self.speed = self.distancia/self.tiempo 

                # 3. L2 / R2: ajustar tiempo
                if btns['L2'] and (now - self.last_lin_adj) >= self.adjust_cooldown and not self.flag_start :
                    self.tiempo = max(5.0, self.tiempo - self.tiempo_step)
                    self.last_lin_adj = now
                    print(f"Duración prueba reducida: {self.tiempo:.2f} s")
                    #Ajustar velocidad
                    self.speed = self.distancia/self.tiempo 
                elif btns['R2'] and (now - self.last_lin_adj) >= self.adjust_cooldown and not self.flag_start:
                    self.tiempo = min(60.0, self.tiempo + self.tiempo_step)
                    self.last_lin_adj = now
                    print(f"Duración prueba aumentada: {self.tiempo:.2f} s")
                    #Ajustar velocidad
                    self.speed = self.distancia/self.tiempo
                
                #Seleccionar dirección de movimiento
                # 1. Direccional: ejes
                axis_x = self.joystick.get_axis(0)  # izquierda-derecha
                axis_y = self.joystick.get_axis(1)  # adelante-atras (generalmente negativo hacia adelante)
                
                if btns['L3'] and (now - self.last_lin_adj) >= self.adjust_cooldown and not self.flag_start :
                    self.direccion = 0       #Rotación horaria
                    mensaje = "Dirección de rotación horaria"
                else:
                    # Adelante / atrás
                    if axis_y < -self.axis_deadzone:
                        self.direccion = 1       #Adelante
                        mensaje = "Dirección de movimiento en X"
                    elif axis_y > self.axis_deadzone:
                        self.direccion = 2       #Atras
                        mensaje = "Dirección de movimiento en -X"

                    # Izquierda / derecha (omnidireccional en y)
                    if axis_x < -self.axis_deadzone:
                        self.direccion = 3       #Izquierda
                        mensaje = "Dirección de movimiento en Y"
                    elif axis_x > self.axis_deadzone:
                        self.direccion = 4       #Derecha
                        mensaje = "Dirección de movimiento en -Y"
                    
                # 4. Start = prueba
                if btns['START']:
                    if self.direccion != 0:
                        mensaje = (f"Iniciando Prueba \n Distancia: {self.distancia}m, en tiempo: {self.tiempo}s")
                    else:
                        vueltas = 2.0*math.pi
                        tiempo = 7.5
                        speed_ang = vueltas/tiempo
                        mensaje = (f"Iniciando Prueba \n Vueltas: {self.distancia}, en tiempo: 7.5s")
                    self.flag_start = True
                    self.init_time = self.get_time()
                
                if self.flag_start and not self.flag_stop:
                    if self.direccion == 0:
                        if (self.get_time() - self.init_time) < tiempo:
                            self.twist.linear.y =  0.0
                            self.twist.linear.x =  0.0
                            self.twist.angular.z = speed_ang
                        else:
                            print(f"Duración final del recorrido: {(self.get_time() - self.init_time)}")
                            self.flag_stop = True
                            self.stop()
                    else:   
                        if (self.get_time() - self.init_time) < self.tiempo:
                            self.twist.angular.z = 0.0
                            if self.direccion == 1:
                                self.twist.linear.x =  self.speed
                                self.twist.linear.y =  0.0
                            elif self.direccion == 2:
                                self.twist.linear.x =  -self.speed
                                self.twist.linear.y =  0.0
                            elif self.direccion == 3:
                                self.twist.linear.y =  self.speed
                                self.twist.linear.x =  0.0
                                self.twist.angular.z = 0.0
                            elif self.direccion == 4:
                                self.twist.linear.y =  -self.speed
                                self.twist.linear.x =  0.0
                            else:
                                self.twist.linear.y =  0.0
                                self.twist.linear.x =  0.0
                            
                            self.pub.publish(self.twist)
                        else:
                            print(f"Duración final del recorrido: {(self.get_time() - self.init_time)}")
                            self.flag_stop = True
                            self.stop()
                
            # Publicar
            self.pub.publish(self.twist)

            if mensaje != self.prev_message:
                print(mensaje)
                self.prev_message = mensaje

            rate.sleep()

        # On shutdown: detener
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.angular.z = 0.0
        self.pub.publish(self.twist)
        print("Nodo detenido. Motores parados.")

if __name__ == '__main__':
    try:
        node = JoystickTeleop()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except SystemExit as e:
        print(f"Saliendo: {e}")
