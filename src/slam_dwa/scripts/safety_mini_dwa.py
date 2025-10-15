#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def ang_wrap(a):
    """Normaliza ángulo a [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))

class MiniDWAReactive:
    def __init__(self):
        rospy.init_node("safety_mini_dwa")

        # ---------- Parámetros ----------
        # Distancias (m)
        self.safe_distance      = rospy.get_param("~safe_distance",       0.40)  # bloqueo total
        self.slowdown_distance  = rospy.get_param("~slowdown_distance",   0.80)  # frenado suave

        # Ventanas / discretización (rad)
        self.sector_half_width  = rospy.get_param("~sector_half_width", math.radians(10.0))
        self.search_step        = rospy.get_param("~search_step",       math.radians(5.0))

        # Ganancias de desvío
        self.prefer_lateral     = rospy.get_param("~prefer_lateral", True)
        self.steer_gain         = rospy.get_param("~steer_gain", 0.8)
        self.angular_gain       = rospy.get_param("~angular_gain", 1.2)
        self.lateral_gain       = rospy.get_param("~lateral_gain", 0.8)

        # Límites absolutos
        self.max_vx             = rospy.get_param("~max_vx", 0.1)
        self.max_vy             = rospy.get_param("~max_vy", 0.1)
        self.max_wz             = rospy.get_param("~max_wz", 0.1)

        # Suavizado
        self.lp_alpha           = rospy.get_param("~lowpass_alpha", 0.5)

        # Tópicos
        joy_topic   = rospy.get_param("~joy_cmd_topic", "/cmd_vel_joystick")
        scan_topic  = rospy.get_param("~scan_topic", "/scan")
        out_topic   = rospy.get_param("~out_cmd_topic", "/cmd_vel")

        # Estado
        self.last_joy   = Twist()
        self.have_scan  = False
        self.angles     = None
        self.ranges     = None
        self.y_f        = Twist()

        # ROS I/O
        rospy.Subscriber(joy_topic,  Twist,      self.cb_joy,   queue_size=10)
        rospy.Subscriber(scan_topic, LaserScan,  self.cb_scan,  queue_size=10)
        self.pub = rospy.Publisher(out_topic, Twist, queue_size=20)

        rospy.loginfo("safety_mini_dwa listo. safe=%.2fm slowdown=%.2fm", 
                      self.safe_distance, self.slowdown_distance)

        self.rate = rospy.Rate(30)

    # ----------------- Callbacks -----------------
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

    # ----------------- Utilidades -----------------
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

    # ----------------- Núcleo del filtro -----------------
    def compute_safe_cmd(self):
        if not self.have_scan:
            return self.lowpass(self.last_joy)

        vx_d, vy_d, wz_d = self.last_joy.linear.x, self.last_joy.linear.y, self.last_joy.angular.z

        # Caso especial: solo giro (sin movimiento lineal)
        if abs(vx_d) < 1e-4 and abs(vy_d) < 1e-4:
            out = Twist()
            out.angular.z = wz_d

            # ⚠️ Revisa obstáculos al frente para bloquear la rotación
            d_front = self.clearance_along(0.0, self.sector_half_width)  # 0 rad = frente
            if d_front < self.safe_distance:
                out.angular.z = 0.0
                rospy.logwarn("Bloqueo de rotación: obstáculo a %.2fm" % d_front)

            return self.lowpass(out)

        # Dirección deseada
        theta_cmd = math.atan2(vy_d, vx_d)
        d_cmd = self.clearance_along(theta_cmd, self.sector_half_width)

        vx, vy, wz = vx_d, vy_d, wz_d

        # 1) Frenado progresivo
        if d_cmd < self.slowdown_distance:
            s = (d_cmd - self.safe_distance) / max(1e-6, (self.slowdown_distance - self.safe_distance))
            s = float(np.clip(s, 0.0, 1.0))
            vx *= s
            vy *= s
            wz *= 0.5 + 0.5 * s

        # 2) Bloqueo y desvío
        if d_cmd < self.safe_distance:
            vx, vy = 0.0, 0.0
            theta_best, d_best = self.best_free_direction()
            err = ang_wrap(theta_best - theta_cmd)

            if self.prefer_lateral:
                vy = np.clip(self.lateral_gain * err, -self.max_vy, self.max_vy)
                wz = np.clip(self.angular_gain * err * 0.5, -self.max_wz, self.max_wz)
            else:
                wz = np.clip(self.angular_gain * err, -self.max_wz, self.max_wz)

            if d_best < self.safe_distance * 0.8:
                vx, vy = 0.0, 0.0
                wz = np.sign(wz) * min(abs(wz), 0.6)

        # Límite por joystick
        vx = np.clip(vx, -abs(self.last_joy.linear.x),  abs(self.last_joy.linear.x))
        vy = np.clip(vy, -abs(self.last_joy.linear.y),  abs(self.last_joy.linear.y))
        wz = np.clip(wz, -abs(self.last_joy.angular.z), abs(self.last_joy.angular.z))

        out = Twist()
        out.linear.x  = float(np.clip(vx, -self.max_vx, self.max_vx))
        out.linear.y  = float(np.clip(vy, -self.max_vy, self.max_vy))
        out.angular.z = float(np.clip(wz, -self.max_wz, self.max_wz))
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
    node = MiniDWAReactive()
    node.spin()

