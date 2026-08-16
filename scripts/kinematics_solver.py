#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class KinematicsSolver(Node):
    def __init__(self):
        super().__init__('kinematics_solver')

        # Publishing to the Gazebo Controller
        self.joint_pub = self.create_publisher(
            Float64MultiArray, 
            '/continuum_controller/commands', 
            10
        )

        # Subscriptions
        # Mode 1: Direct Tendon Control (from Dashboard Hardware/Calibration tabs)
        self.tendon_sub = self.create_subscription(Float64MultiArray, '/tendon_lengths', self.tendon_callback, 10)
        
        # Mode 2: Task-Space/IK Control (Targeting Bending Angle, Plane, and Contraction)
        # Format: [Total_Bending_Angle_deg, Bending_Plane_Angle_rad, Linear_Contraction_mm]
        self.ik_sub = self.create_subscription(Float64MultiArray, '/ik_targets', self.ik_callback, 10)

        self.n_disks = 10
        self.d = 0.0073  # Tendon distance from center (7.3 mm)
        self.L0 = 0.23   # Rest length (23 cm)
        
        # Initialize tendon lengths (in meters) to resting state
        self.l1 = self.L0; self.l2 = self.L0; self.l3 = self.L0; self.l4 = self.L0

        self.timer = self.create_timer(1.0 / 30.0, self.publish_kinematics)

    def tendon_callback(self, msg):
        """Direct tendon length mode (meters)"""
        if len(msg.data) == 4:
            self.l1, self.l2, self.l3, self.l4 = msg.data

    def ik_callback(self, msg):
        """Inverse Kinematics Mode: Solves Task-Space inputs to Tendon Lengths (meters)"""
        if len(msg.data) == 3:
            theta_deg = msg.data[0]       # Total bending angle in degrees
            phi = msg.data[1]             # Bending plane angle (radians)
            contraction_mm = msg.data[2]  # Uniform backbone contraction (mm)
            
            # Convert inputs to SI units
            theta = math.radians(theta_deg)
            s = (self.L0 * 1000.0 - contraction_mm) / 1000.0  # Active arc length (s) in meters
            
            if theta < 0.001:
                # Pure contraction/straight configuration
                self.l1 = s
                self.l2 = s
                self.l3 = s
                self.l4 = s
            else:
                # Exact Discrete Kinematics from Paper (Equations 5, 6, 7, 8)
                k = theta / s         # Curvature k = theta / s
                n = self.n_disks      # Number of disks (10)
                d = self.d            # Tendon offset radius (0.01 meters for your hardware)
                
                # The shared chord length multiplier: (2n) * sin(ks / 2n)
                chord_factor = (2 * n) * math.sin((k * s) / (2 * n))
                
                # Eq 5: Tendon 1
                self.l1 = chord_factor * ((1 / k) - d * math.cos(phi))
                
                # Eq 6: Tendon 2 (Phase shifted by pi/2)
                self.l2 = chord_factor * ((1 / k) - d * math.cos((math.pi / 2) - phi))
                
                # Eq 7: Tendon 3 (Phase shifted by pi)
                self.l3 = chord_factor * ((1 / k) - d * math.cos(math.pi - phi))
                
                # Eq 8: Tendon 4 (Phase shifted by 3pi/2)
                self.l4 = chord_factor * ((1 / k) - d * math.cos((3 * math.pi / 2) - phi))
                
                # Failsafe bounds: prevent tendons from exceeding the resting length or going overly slack
                self.l1 = max(0.54 * self.L0, min(self.L0, self.l1))
                self.l2 = max(0.54 * self.L0, min(self.L0, self.l2))
                self.l3 = max(0.54 * self.L0, min(self.L0, self.l3))
                self.l4 = max(0.54 * self.L0, min(self.L0, self.l4))

    def publish_kinematics(self):
        # Forward Kinematics conversion to drive the continuum disks in Gazebo
        delta_l13 = self.l3 - self.l1
        delta_l24 = self.l4 - self.l2
        s = (self.l1 + self.l2 + self.l3 + self.l4) / 4.0
        
        numerator = math.sqrt(delta_l13**2 + delta_l24**2)
        k = numerator / (self.d * 2.0 * s) if s > 0 else 0.0
        phi = math.atan2(delta_l24, delta_l13)
        theta = k * s
        
        theta_per_disk = theta / self.n_disks
        pitch_per_disk = theta_per_disk * math.cos(phi)
        yaw_per_disk = theta_per_disk * math.sin(phi)
        
        contraction_total = self.L0 - s
        contraction_per_disk = -(contraction_total / self.n_disks)
        
        # Pack the 30 numbers into the command array for the ROS2 control plugin
        msg = Float64MultiArray()
        for i in range(self.n_disks):
            msg.data.extend([pitch_per_disk, yaw_per_disk, contraction_per_disk])
            
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KinematicsSolver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()