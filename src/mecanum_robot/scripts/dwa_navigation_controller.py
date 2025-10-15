#!/usr/bin/env python3
import rospy
import math
import sys
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from roboclaw_3 import Roboclaw
from tf2_ros import TransformBroadcaster
from tf.transformations import quaternion_from_euler

class MecanumController:
    def __init__(self):
        rospy.init_node('mecanum_controller', log_level=rospy.INFO)
        
        # 1. Configuración física del robot (modifica estos valores según tu robot)
        self.wheel_diameter = 0.152  # 15.2 cm
        self.wheel_radius = self.wheel_diameter / 2.0
        self.L = 0.125   # 12.5 cm (semidistancia longitudinal)
        self.w = 0.285   # 28.5 cm (semidistancia transversal)
        
        # 2. Configuración de motores y encoders
        self.encoder_ppr_motor = 1000      # PPR en el eje del motor
        self.gear_ratio = 18              # Relación de reducción
        self.encoder_ppr_wheel = self.encoder_ppr_motor * self.gear_ratio
        self.wheel_circumference = math.pi * self.wheel_diameter
        self.meters_per_tick = self.wheel_circumference / self.encoder_ppr_wheel
        
        # 3. Configuración RoboClaw (verifica puerto y direcciones)
        self.addresses = [0x80, 0x81, 0x83, 0x82]  # [FL, FR, RL, RR]
        self.rc = Roboclaw("/dev/ttyAMA0", 115200)
        if not self.rc.Open():
            rospy.logerr("No se pudo conectar a RoboClaw")
            rospy.signal_shutdown("Error de conexión RoboClaw")
            return
        
        # 4. Variables de estado
        self.x = 0.0      # Posición X (metros)
        self.y = 0.0      # Posición Y (metros)
        self.theta = 0.0  # Orientación (radianes)
        self.last_encoders = [0]*4
        self.last_time = rospy.Time.now()
        self.current_twist = Twist()
        
        # 5. Factores de control (ajusta según necesidades)
        self.acceleration = 70000 
        self.scale_factor = 40000 / 0.07  # Conversión m/s -> ticks
        
        # 6. Configuración ROS
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10) # Era odom
        self.odom = Odometry()
        self.transform = TransformStamped() # % Lo comento porque la transformada lo va a publicar ekf.yaml
        self.tf_broadcaster = TransformBroadcaster() # % Lo comento porque la transformada lo va a publicar ekf.yaml
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        rospy.on_shutdown(self.stop_motors)

    def cmd_vel_callback(self, msg):
        """Almacena el último comando de velocidad."""
        self.current_twist = msg

    def send_motor_commands(self):
        """Convierte Twist en comandos para motores usando cinemática inversa."""
        try:
            vx = self.current_twist.linear.x
            vy = self.current_twist.linear.y
            wz = self.current_twist.angular.z
            
            # Cinemática inversa para ruedas mecanum
            # Orden de las ruedas: [FL, FR, RL, RR]
            wheel_speeds_mps = [
                vx - vy - wz * (self.L + self.w), # FL
                vx + vy + wz * (self.L + self.w), # FR
                vx + vy - wz * (self.L + self.w), # RL
                vx - vy + wz * (self.L + self.w)  # RR
            ]
            
            wheel_ticks = [int(speed * self.scale_factor) for speed in wheel_speeds_mps]
            
            for addr, ticks in zip(self.addresses, wheel_ticks):
                self.rc.SpeedAccelM1(addr, self.acceleration, ticks)
                
        except Exception as e:
            rospy.logerr(f"Error en send_motor_commands: {e}")

    def read_encoders(self):
        """Lee los valores actuales de los encoders con manejo de errores."""
        try:
            return [self.rc.ReadEncM1(addr)[1] for addr in self.addresses]
        except Exception as e:
            rospy.logwarn(f"Error leyendo encoders: {e}")
            return self.last_encoders

    def update_odometry(self):
        """Actualiza la odometría usando lecturas de encoders."""
        try:
            now = rospy.Time.now()
            dt = (now - self.last_time).to_sec()
            if dt <= 0:
                return
            
            current_encoders = self.read_encoders()
            delta_ticks = [cur - last for cur, last in zip(current_encoders, self.last_encoders)]
            self.last_encoders = current_encoders
            
            # Cinemática directa
            delta_meters = [ticks * self.meters_per_tick for ticks in delta_ticks]

            # Orden de las ruedas: [FL, FR, RL, RR]
            delta_fl = delta_meters[0]
            delta_fr = delta_meters[1]
            delta_rl = delta_meters[2]
            delta_rr = delta_meters[3]

            # Fórmulas de cinemática directa
            # Los signos han sido corregidos para que los movimientos se grafiquen correctamente en RViz
            dx_local = (delta_fl + delta_fr + delta_rl + delta_rr) / 4.0
            dy_local = (-delta_fl + delta_fr + delta_rl - delta_rr) / 4.0
            dtheta = (-delta_fl + delta_fr - delta_rl + delta_rr) / (4.0 * (self.L + self.w))

            # Transformar las velocidades locales al marco de referencia global
            self.x += (dx_local * math.cos(self.theta) - dy_local * math.sin(self.theta))
            self.y += (dx_local * math.sin(self.theta) + dy_local * math.cos(self.theta))
            self.theta = math.atan2(math.sin(self.theta + dtheta), math.cos(self.theta + dtheta))
            
            self.last_time = now
            
        except Exception as e:
            rospy.logerr(f"Error en update_odometry: {e}")

    def publish_odometry(self):
        """Publica odometría y transformada TF."""
        try:
            self.odom.header.stamp = rospy.Time.now()
            self.odom.header.frame_id = "/odom" # /
            self.odom.child_frame_id = "/base_link" # /
            
            self.odom.pose.pose.position.x = self.x
            self.odom.pose.pose.position.y = self.y
            q = quaternion_from_euler(0, 0, self.theta)
            self.odom.pose.pose.orientation = Quaternion(*q)
            
            # Covarianza (ajustar según precisión real)


            self.odom.pose.covariance = [
                                    0.1, 0, 0, 0, 0, 0, # 0.01
                                    0, 0.1, 0, 0, 0, 0, # 0.01
                                    0, 0, 0.1, 0, 0, 0, # 0.01
                                    0, 0, 0, 0.1, 0, 0, # 0.01
                                    0, 0, 0, 0, 0.1, 0, # 0.01
                                    0, 0, 0, 0, 0, 0.1] # 0.01
            self.odom.twist.covariance = [
                                    0.1, 0, 0, 0, 0, 0, # 0.01
                                    0, 0.1, 0, 0, 0, 0, # 0.01
                                    0, 0, 0.1, 0, 0, 0, # 0.01
                                    0, 0, 0, 0.1, 0, 0, # 0.01
                                    0, 0, 0, 0, 0.1, 0, # 0.01
                                    0, 0, 0, 0, 0, 0.1] # 0.01
            
            self.odom_pub.publish(self.odom)
            
            # Publicar transformada TF
            # Lo comento porque ahora va a publicar la transformada el archivo ekf.yaml
            self.transform.header.stamp = self.odom.header.stamp
            self.transform.header.frame_id = "/odom"
            self.transform.child_frame_id = "/base_link"
            self.transform.transform.translation.x = self.x
            self.transform.transform.translation.y = self.y
            self.transform.transform.rotation = self.odom.pose.pose.orientation
            
            #self.tf_broadcaster.sendTransform(self.transform)
            
        except Exception as e:
            rospy.logerr(f"Error en publish_odometry: {e}")

    def stop_motors(self):
        """Detiene todos los motores de forma segura."""
        rospy.loginfo("Deteniendo motores...")
        for addr in self.addresses:
            try:
                self.rc.SpeedM1(addr, 0)
            except:
                pass

    def run(self):
        """Bucle principal del controlador."""
        rate = rospy.Rate(50)  # 50 Hz
        while not rospy.is_shutdown():
            try:
                self.send_motor_commands()
                self.update_odometry()
                self.publish_odometry()
                rate.sleep()
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                rospy.logerr(f"Error en run: {e}")
                continue

if __name__ == '__main__':
    try:
        controller = MecanumController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
