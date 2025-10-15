#!/usr/bin/env python3
# -*- coding: utf--8 -*-

import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock     #import para poder tomar medida de tiempo en simulación

def ang_wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

class SafetyDWAFull:
    def __init__(self):
        rospy.init_node("safety_dwa_full")

        # ---------- Parámetros ----------
        self.safe_distance     = rospy.get_param("~safe_distance", 0.50)
        self.slowdown_distance = rospy.get_param("~slowdown_distance", 0.80)
        self.sector_half_width = rospy.get_param("~sector_half_width", math.radians(35.0))
        self.search_step       = rospy.get_param("~search_step", math.radians(5.0))

        self.max_vx  = rospy.get_param("~max_vx", 0.5)
        self.max_vy  = rospy.get_param("~max_vy", 0.5)
        self.max_wz  = rospy.get_param("~max_wz", 1.5)

        self.steer_gain    = rospy.get_param("~steer_gain", 1.8)
        self.angular_gain  = rospy.get_param("~angular_gain", 1.2)
        self.lateral_gain  = rospy.get_param("~lateral_gain", 1.75)

        self.lp_alpha      = rospy.get_param("~lowpass_alpha", 0.4)

        joy_topic  = rospy.get_param("~joy_cmd_topic", "/cmd_vel_joystick")
        scan_topic = rospy.get_param("~scan_topic", "/scan")
        out_topic  = rospy.get_param("~out_cmd_topic", "/cmd_vel")

        # Estado
        self.last_joy  = Twist()
        self.have_scan = False
        self.angles    = None
        self.ranges    = None
        self.y_f       = Twist()

        rospy.Subscriber(joy_topic, Twist, self.cb_joy, queue_size=10)
        rospy.Subscriber(scan_topic, LaserScan, self.cb_scan, queue_size=10)
        self.pub = rospy.Publisher(out_topic, Twist, queue_size=20)

        rospy.loginfo("SafetyDWAFull listo. safe=%.2fm slowdown=%.2fm",
                      self.safe_distance, self.slowdown_distance)
        self.rate = rospy.Rate(30)
        
        self.DWA_active = False
        self.DWA_active_time = rospy.Time.now().to_sec()
        
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

    # ---------------- Callbacks ----------------
    def cb_joy(self, msg):
        msg.linear.x  = np.clip(msg.linear.x,  -self.max_vx, self.max_vx)
        msg.linear.y  = np.clip(msg.linear.y,  -self.max_vy, self.max_vy)
        msg.angular.z = np.clip(msg.angular.z, -self.max_wz, self.max_wz)
        self.last_joy = msg

    def cb_scan(self, scan: LaserScan):
        r = np.array(scan.ranges, dtype=np.float32)
        r[np.isinf(r)] = scan.range_max
        r[np.isnan(r)] = scan.range_max
        self.ranges = np.clip(r, scan.range_min, scan.range_max)
        n = len(self.ranges)
        self.angles = scan.angle_min + np.arange(n, dtype=np.float32) * scan.angle_increment
        self.have_scan = True

    # ---------------- Utilities ----------------
    def clearance_along(self, theta, half_width):
        if not self.have_scan or self.angles is None:
            return float("inf")
        da = np.vectorize(lambda a: ang_wrap(a - theta))(self.angles)
        mask = np.abs(da) <= half_width
        if not np.any(mask):
            return float("inf")
        return float(np.min(self.ranges[mask]))

    def best_free_direction(self):
        thetas = np.arange(-math.pi, math.pi + 1e-6, self.search_step, dtype=np.float32)
        best_theta, best_clear = 0.0, -1.0
        for th in thetas:
            c = self.clearance_along(th, self.sector_half_width)
            if c > best_clear:
                best_clear, best_theta = c, th
        return best_theta, best_clear

    # ---------------- Core ----------------
    def compute_safe_cmd(self):
        if not self.have_scan:
            return self.lowpass(self.last_joy)

        vx_d, vy_d, wz_d = self.last_joy.linear.x, self.last_joy.linear.y, self.last_joy.angular.z
        
        # V1. Mantenerse quieto si el joystick está en cero
        if abs(vx_d) < 1e-4 and abs(vy_d) < 1e-4 and abs(wz_d) < 1e-4:
            return Twist()

        # V2. **NUEVA LÓGICA:** Aplicar la evitación de obstáculos omnidireccional
        vx, vy, wz = vx_d, vy_d, wz_d
        
        # Calcular el ángulo de la velocidad deseada
        theta_cmd = math.atan2(vy_d, vx_d)
        
        # Obtener la distancia al obstáculo más cercano en la dirección del movimiento
        d_cmd = self.clearance_along(theta_cmd, self.sector_half_width)

        if d_cmd < self.slowdown_distance:
            s = (d_cmd - self.safe_distance) / (self.slowdown_distance - self.safe_distance + 1e-6)
            s = float(np.clip(s, 0.0, 1.0))
            
            # Reducir la velocidad lineal de forma progresiva
            vx *= s
            vy *= s
            
            # Si el obstáculo está en la zona segura, iniciar el rodeo
            if d_cmd < self.safe_distance:
                theta_best, d_best = self.best_free_direction()
                
                # Calcular el ángulo de desviación
                err = ang_wrap(theta_best - theta_cmd)
                
                # Ajustar las velocidades para rodear el obstáculo
                # La velocidad lineal ya se redujo con 's'
                vy = np.clip(self.lateral_gain * err, -self.max_vy, self.max_vy)
                wz = np.clip(self.angular_gain * err, -self.max_wz, self.max_wz)
                
        # V3. Bloqueo de rotación si hay un obstáculo muy cerca mientras se va al frente
        # (Esto se mantiene como una capa de seguridad adicional)
        d_front = self.clearance_along(0.0, self.sector_half_width)
        if abs(vx_d) < 1e-4 and abs(vy_d) < 1e-4 and d_front < self.safe_distance:
            wz = 0.0

        # Limites finales
        out = Twist()
        out.linear.x  = float(np.clip(vx, -self.max_vx, self.max_vx))
        out.linear.y  = float(np.clip(vy, -self.max_vy, self.max_vy))
        out.angular.z = float(np.clip(wz, -self.max_wz, self.max_wz))
        
        if out.linear.x == vx_d and out.linear.y == vy_d and out.angular.z == wz_d:
            if self.DWA_active:
                self.DWA_active_time = self.get_time() - self.DWA_active_time
                print(f"DWA activo durante: {self.DWA_active_time} segundos")
                self.DWA_active = False
            self.DWA_active_time = self.get_time()
        else:
            if not self.DWA_active:
                print("Inicio de DWA")
            self.DWA_active = True
        
        return self.lowpass(out)

    def lowpass(self, cmd: Twist):
        a = float(np.clip(self.lp_alpha, 0.0, 1.0))
        self.y_f.linear.x  = a*cmd.linear.x  + (1-a)*self.y_f.linear.x
        self.y_f.linear.y  = a*cmd.linear.y  + (1-a)*self.y_f.linear.y
        self.y_f.angular.z = a*cmd.angular.z + (1-a)*self.y_f.angular.z
        return self.y_f

    def spin(self):
        while not rospy.is_shutdown():
            safe = self.compute_safe_cmd()
            self.pub.publish(safe)
            self.rate.sleep()

if __name__ == "__main__":
    node = SafetyDWAFull()
    node.spin()
