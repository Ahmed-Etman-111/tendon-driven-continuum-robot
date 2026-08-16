#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import math
import cv2
import numpy as np
from tf2_ros import Buffer, TransformListener

class DashboardGUI(Node):
    def __init__(self):
        super().__init__('dashboard_gui')
        
        # Publishers
        self.publisher_ = self.create_publisher(Float64MultiArray, '/tendon_lengths', 10)
        self.ik_pub = self.create_publisher(Float64MultiArray, '/ik_targets', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera Subscriber and Image Bridge
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(ROSImage, '/camera/image_raw', self.image_callback, 10)
        self.tk_image = None
        
        # --- SEPARATED HARDWARE PARAMETERS ---
        self.L0 = 0.23              # Length of the robot: 23 cm = 0.23 m
        self.pulley_radius = 0.01   # Motor pulley radius (for motor degree to mm conversion)
        self.d = 0.0073             # Disk tendon routing offset (for bending curve kinematics)
        # -------------------------------------

        # A dedicated 30Hz timer to prevent network flooding
        self.pub_timer = self.create_timer(1.0 / 30.0, self.update_system)
        
        # Initialize Tkinter Window
        self.root = tk.Tk()
        self.root.title("Continuum Robot Controller")
        self.root.geometry("600x800")
        self.setup_ui()
        
        self.root.after(33, self.ros_loop)

    def setup_ui(self):
        style = ttk.Style()
        style.theme_use('clam')
        
        # The Video Monitor
        self.video_frame = ttk.LabelFrame(self.root, text="Endoscope View")
        self.video_frame.pack(fill=tk.BOTH, expand=False, padx=10, pady=5)
        
        self.video_lbl = ttk.Label(self.video_frame, text="Waiting for camera feed...")
        self.video_lbl.pack()

        # THE LIGHT DIMMER
        self.light_val = tk.DoubleVar(value=1.0) 
        ttk.Label(self.video_frame, text="Light Intensity").pack(pady=(5, 0))
        self.light_slider = ttk.Scale(
            self.video_frame, from_=0.0, to=1.0, orient=tk.HORIZONTAL, 
            variable=self.light_val, command=self.update_light_label
        )
        self.light_slider.pack(fill=tk.X, padx=20, pady=5)
        self.light_lbl = ttk.Label(self.video_frame, text="100%")
        self.light_lbl.pack(pady=(0, 5))

        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # ==========================================
        # TAB 1: HARDWARE MODE
        # ==========================================
        self.tab_hw = ttk.Frame(self.notebook, padding="15")
        self.notebook.add(self.tab_hw, text="Hardware Mode (Motors)")
        
        ttk.Label(self.tab_hw, text="Motors 1&3: Pitch (Degrees)").pack(anchor=tk.W, pady=2)
        self.pitch_slider = ttk.Scale(self.tab_hw, from_=-180.0, to=180.0, orient=tk.HORIZONTAL)
        self.pitch_slider.pack(fill=tk.X, pady=2)
        self.pitch_lbl = ttk.Label(self.tab_hw, text="0.0°")
        self.pitch_lbl.pack(anchor=tk.E)

        ttk.Label(self.tab_hw, text="Motors 2&4: Yaw (Degrees)").pack(anchor=tk.W, pady=2)
        self.yaw_slider = ttk.Scale(self.tab_hw, from_=-180.0, to=180.0, orient=tk.HORIZONTAL)
        self.yaw_slider.pack(fill=tk.X, pady=2)
        self.yaw_lbl = ttk.Label(self.tab_hw, text="0.0°")
        self.yaw_lbl.pack(anchor=tk.E)

        ttk.Label(self.tab_hw, text="Linear Spring Contraction (%)").pack(anchor=tk.W, pady=2)
        self.contract_slider = ttk.Scale(self.tab_hw, from_=0.0, to=46.0, orient=tk.HORIZONTAL)
        self.contract_slider.pack(fill=tk.X, pady=2)
        self.contract_lbl = ttk.Label(self.tab_hw, text="0.0%")
        self.contract_lbl.pack(anchor=tk.E)

        # Tendon Length Visualization Frame (Hardware Mode)
        self.hw_length_frame = ttk.LabelFrame(self.tab_hw, text="Calculated Tendon Lengths")
        self.hw_length_frame.pack(fill=tk.X, pady=5, padx=5)

        self.hw_l1_lbl = ttk.Label(self.hw_length_frame, text="L1: 230.0 mm", font=("Courier", 10))
        self.hw_l1_lbl.grid(row=0, column=0, padx=20, pady=5)
        self.hw_l2_lbl = ttk.Label(self.hw_length_frame, text="L2: 230.0 mm", font=("Courier", 10))
        self.hw_l2_lbl.grid(row=0, column=1, padx=20, pady=5)
        self.hw_l3_lbl = ttk.Label(self.hw_length_frame, text="L3: 230.0 mm", font=("Courier", 10))
        self.hw_l3_lbl.grid(row=1, column=0, padx=20, pady=5)
        self.hw_l4_lbl = ttk.Label(self.hw_length_frame, text="L4: 230.0 mm", font=("Courier", 10))
        self.hw_l4_lbl.grid(row=1, column=1, padx=20, pady=5)

        # Coordinates Visualization Frame (Hardware Mode)
        self.hw_coord_frame = ttk.LabelFrame(self.tab_hw, text="Calculated Tip Coordinates (XYZ)")
        self.hw_coord_frame.pack(fill=tk.X, pady=5, padx=5)

        self.hw_x_lbl = ttk.Label(self.hw_coord_frame, text="X: 0.0 mm", font=("Courier", 10, "bold"), foreground="blue")
        self.hw_x_lbl.grid(row=0, column=0, padx=15, pady=5)
        self.hw_y_lbl = ttk.Label(self.hw_coord_frame, text="Y: 0.0 mm", font=("Courier", 10, "bold"), foreground="blue")
        self.hw_y_lbl.grid(row=0, column=1, padx=15, pady=5)
        self.hw_z_lbl = ttk.Label(self.hw_coord_frame, text="Z: 230.0 mm", font=("Courier", 10, "bold"), foreground="blue")
        self.hw_z_lbl.grid(row=0, column=2, padx=15, pady=5)

        # --- ADD THESE NEW LABELS FOR ACTUAL GAZEBO POSITION ---
        self.actual_x_lbl = ttk.Label(self.hw_coord_frame, text="X (Real): 0.0 mm", font=("Courier", 10, "bold"), foreground="red")
        self.actual_x_lbl.grid(row=1, column=0, padx=15, pady=5)
        self.actual_y_lbl = ttk.Label(self.hw_coord_frame, text="Y (Real): 0.0 mm", font=("Courier", 10, "bold"), foreground="red")
        self.actual_y_lbl.grid(row=1, column=1, padx=15, pady=5)
        self.actual_z_lbl = ttk.Label(self.hw_coord_frame, text="Z (Real): 230.0 mm", font=("Courier", 10, "bold"), foreground="red")
        self.actual_z_lbl.grid(row=1, column=2, padx=15, pady=5)

        ttk.Button(self.tab_hw, text="Reset Hardware to Zero", command=self.reset_hw).pack(pady=10)

        # ==========================================
        # TAB 2: CALIBRATION MODE
        # ==========================================
        self.tab_cal = ttk.Frame(self.notebook, padding="15")
        self.notebook.add(self.tab_cal, text="Calibration Mode (Raw L)")
        
        base_len_mm = self.L0 * 1000.0       
        min_len = base_len_mm * 0.54         
        
        ttk.Label(self.tab_cal, text="Tendon 1 Length (mm)").pack(anchor=tk.W, pady=2)
        self.l1_slider = ttk.Scale(self.tab_cal, from_=min_len, to=base_len_mm, orient=tk.HORIZONTAL)
        self.l1_slider.pack(fill=tk.X, pady=2)
        self.l1_lbl = ttk.Label(self.tab_cal, text=f"{base_len_mm:.1f} mm")
        self.l1_lbl.pack(anchor=tk.E)

        ttk.Label(self.tab_cal, text="Tendon 2 Length (mm)").pack(anchor=tk.W, pady=2)
        self.l2_slider = ttk.Scale(self.tab_cal, from_=min_len, to=base_len_mm, orient=tk.HORIZONTAL)
        self.l2_slider.pack(fill=tk.X, pady=2)
        self.l2_lbl = ttk.Label(self.tab_cal, text=f"{base_len_mm:.1f} mm")
        self.l2_lbl.pack(anchor=tk.E)

        ttk.Label(self.tab_cal, text="Tendon 3 Length (mm)").pack(anchor=tk.W, pady=2)
        self.l3_slider = ttk.Scale(self.tab_cal, from_=min_len, to=base_len_mm, orient=tk.HORIZONTAL)
        self.l3_slider.pack(fill=tk.X, pady=2)
        self.l3_lbl = ttk.Label(self.tab_cal, text=f"{base_len_mm:.1f} mm")
        self.l3_lbl.pack(anchor=tk.E)

        ttk.Label(self.tab_cal, text="Tendon 4 Length (mm)").pack(anchor=tk.W, pady=2)
        self.l4_slider = ttk.Scale(self.tab_cal, from_=min_len, to=base_len_mm, orient=tk.HORIZONTAL)
        self.l4_slider.pack(fill=tk.X, pady=2)
        self.l4_lbl = ttk.Label(self.tab_cal, text=f"{base_len_mm:.1f} mm")
        self.l4_lbl.pack(anchor=tk.E)

        ttk.Button(self.tab_cal, text="Reset Tendons to 100mm", command=self.reset_cal).pack(pady=15)

        # ==========================================
        # TAB 3: INVERSE KINEMATICS MODE (XYZ)
        # ==========================================
        self.tab_ik = ttk.Frame(self.notebook, padding="15")
        self.notebook.add(self.tab_ik, text="Task Space (XYZ)")

        ttk.Label(self.tab_ik, text="X Target (mm) [Tip Deflection]").pack(anchor=tk.W, pady=(5, 2))
        self.ik_x_var = tk.StringVar(value="0.0")
        self.ik_x_entry = ttk.Entry(self.tab_ik, textvariable=self.ik_x_var, font=("Courier", 10))
        self.ik_x_entry.pack(fill=tk.X, pady=2)

        ttk.Label(self.tab_ik, text="Y Target (mm) [Tip Deflection]").pack(anchor=tk.W, pady=(5, 2))
        self.ik_y_var = tk.StringVar(value="0.0")
        self.ik_y_entry = ttk.Entry(self.tab_ik, textvariable=self.ik_y_var, font=("Courier", 10))
        self.ik_y_entry.pack(fill=tk.X, pady=2)

        # CHANGED: 270.0 is now 230.0
        ttk.Label(self.tab_ik, text="Z Target (mm) [Base=0, Default Tip=230.0]").pack(anchor=tk.W, pady=(5, 2))
        self.ik_z_var = tk.StringVar(value="230.0")
        self.ik_z_entry = ttk.Entry(self.tab_ik, textvariable=self.ik_z_var, font=("Courier", 10))
        self.ik_z_entry.pack(fill=tk.X, pady=2)

        ttk.Button(self.tab_ik, text="Send IK Target", command=self.send_ik_command).pack(pady=15)
        self.ik_status_lbl = ttk.Label(self.tab_ik, text="Status: Ready", font=("Arial", 9, "bold"))
        self.ik_status_lbl.pack(pady=5)

        self.length_frame = ttk.LabelFrame(self.tab_ik, text="Calculated Tendon Lengths")
        self.length_frame.pack(fill=tk.X, pady=10, padx=5)

        self.ik_l1_lbl = ttk.Label(self.length_frame, text="L1: 230.0 mm", font=("Courier", 10))
        self.ik_l1_lbl.grid(row=0, column=0, padx=20, pady=5)
        self.ik_l2_lbl = ttk.Label(self.length_frame, text="L2: 230.0 mm", font=("Courier", 10))
        self.ik_l2_lbl.grid(row=0, column=1, padx=20, pady=5)
        self.ik_l3_lbl = ttk.Label(self.length_frame, text="L3: 230.0 mm", font=("Courier", 10))
        self.ik_l3_lbl.grid(row=1, column=0, padx=20, pady=5)
        self.ik_l4_lbl = ttk.Label(self.length_frame, text="L4: 230.0 mm", font=("Courier", 10))
        self.ik_l4_lbl.grid(row=1, column=1, padx=20, pady=5)

        # --- CORRECTED REAL COORDINATES SECTION ---
        # Created a new sub-frame so we can safely use .grid() without crashing the .pack() layout
        self.ik_actual_frame = ttk.LabelFrame(self.tab_ik, text="Actual Gazebo Tip Coordinates (XYZ)")
        self.ik_actual_frame.pack(fill=tk.X, pady=10, padx=5)

        self.ik_actual_x_lbl = ttk.Label(self.ik_actual_frame, text="X (Real): 0.0 mm", font=("Courier", 10, "bold"), foreground="red")
        self.ik_actual_x_lbl.grid(row=0, column=0, padx=15, pady=5) 
        
        self.ik_actual_y_lbl = ttk.Label(self.ik_actual_frame, text="Y (Real): 0.0 mm", font=("Courier", 10, "bold"), foreground="red")
        self.ik_actual_y_lbl.grid(row=0, column=1, padx=15, pady=5)
        
        self.ik_actual_z_lbl = ttk.Label(self.ik_actual_frame, text="Z (Real): 230.0 mm", font=("Courier", 10, "bold"), foreground="red")
        self.ik_actual_z_lbl.grid(row=0, column=2, padx=15, pady=5)
        # ------------------------------------------

        self.reset_hw()
        self.reset_cal()

    def update_light_label(self, value):
        intensity = float(value)
        percent = int(intensity * 100)
        self.light_lbl.config(text=f"{percent}%")
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            current_brightness = self.light_val.get()
            cv_image = cv2.convertScaleAbs(cv_image, alpha=current_brightness, beta=0)
            
            pil_image = Image.fromarray(cv_image)
            self.tk_image = ImageTk.PhotoImage(image=pil_image)
            self.video_lbl.config(image=self.tk_image, text="")
        except Exception as e:
            self.get_logger().error(f"Camera rendering error: {e}")

    def reset_hw(self):
        self.pitch_slider.set(0.0)
        self.yaw_slider.set(0.0)
        self.contract_slider.set(0.0)

    def reset_cal(self):
        base_len = self.L0 * 1000.0
        self.l1_slider.set(base_len)
        self.l2_slider.set(base_len)
        self.l3_slider.set(base_len)
        self.l4_slider.set(base_len)

    def send_ik_command(self):
        try:
            x = float(self.ik_x_var.get())
            y = float(self.ik_y_var.get())
            z = float(self.ik_z_var.get())
        except ValueError:
            self.ik_status_lbl.config(text="Error: Please enter valid numbers!", foreground="red")
            return
        
        L0_mm = self.L0 * 1000.0
        max_contract_mm = L0_mm * 0.46
        
        if z <= 0.1:
            self.ik_status_lbl.config(text="Error: Z-coordinate must be > 0", foreground="red")
            return

        r = math.sqrt(x**2 + y**2)
        phi = math.atan2(y, x)
        
        theta_rad = 2.0 * math.atan(r / z)
        theta_deg = math.degrees(theta_rad)
        
        if theta_rad < 0.001:
            Lc_mm = z
        else:
            Lc_mm = z * (theta_rad / math.sin(theta_rad))
            
        contraction_mm = L0_mm - Lc_mm
        
        if contraction_mm < -0.01: 
            self.ik_status_lbl.config(text=f"Unreachable: Outside workspace (Needs Lc={Lc_mm:.1f}mm)", foreground="red")
            return
        elif contraction_mm > max_contract_mm:
            self.ik_status_lbl.config(text=f"Unreachable: Exceeds max spring contraction", foreground="red")
            return
            
        contraction_mm = max(0.0, contraction_mm) 
        Lc_mm = min(L0_mm, Lc_mm) 

        s_m = Lc_mm / 1000.0
        k = theta_rad / s_m if s_m > 0.001 else 0
        n = 10
        # --- FIXED: Use the disk offset (d) for bending math ---
        disk_offset_m = self.d 

        if theta_rad < 0.001:
            l1 = l2 = l3 = l4 = Lc_mm
        else:
            chord_factor = (2 * n) * math.sin((k * s_m) / (2 * n))
            l1_m = chord_factor * ((1 / k) - disk_offset_m * math.cos(phi))
            l2_m = chord_factor * ((1 / k) - disk_offset_m * math.cos((math.pi / 2) - phi))
            l3_m = chord_factor * ((1 / k) - disk_offset_m * math.cos(math.pi - phi))
            l4_m = chord_factor * ((1 / k) - disk_offset_m * math.cos((3 * math.pi / 2) - phi))
            l1, l2, l3, l4 = l1_m*1000, l2_m*1000, l3_m*1000, l4_m*1000
        # -------------------------------------------------------

        # --- FIXED: Dynamic bounds based on actual resting length ---
        min_length = L0_mm * (1.0 - 0.46)  # 54% of resting length
        max_length = L0_mm                 # 100% of resting length
        
        l1 = max(min_length, min(max_length, l1))
        l2 = max(min_length, min(max_length, l2))
        l3 = max(min_length, min(max_length, l3))
        l4 = max(min_length, min(max_length, l4))
        # ------------------------------------------------------------

        self.ik_l1_lbl.config(text=f"L1: {l1:.1f} mm")
        self.ik_l2_lbl.config(text=f"L2: {l2:.1f} mm")
        self.ik_l3_lbl.config(text=f"L3: {l3:.1f} mm")
        self.ik_l4_lbl.config(text=f"L4: {l4:.1f} mm")
        
        msg = Float64MultiArray()
        msg.data = [theta_deg, phi, contraction_mm]
        self.ik_pub.publish(msg)
        
        self.ik_status_lbl.config(text=f"Executed -> Angle: {theta_deg:.1f}°, Lc: {Lc_mm:.1f}mm", foreground="green")

    def update_system(self):
        try:
            current_tab = self.notebook.index(self.notebook.select())
        except tk.TclError:
            return 
        
        msg = Float64MultiArray()
        L0_mm = self.L0 * 1000.0  

        if current_tab == 0:
            # TAB 1: Hardware Mode (Motors)
            pitch_deg = self.pitch_slider.get()
            yaw_deg = self.yaw_slider.get()
            contract_pct = self.contract_slider.get()
            
            L_c_mm = L0_mm * (1.0 - contract_pct / 100.0)  
            min_tendon_limit = L0_mm * 0.54  
            max_tendon_limit = L0_mm * 1.20  
            
            pulley_circumference_mm = 2 * math.pi * (self.pulley_radius * 1000.0)
            
            max_allowed_delta_mm = min((L_c_mm - min_tendon_limit), (max_tendon_limit - L_c_mm))
            max_allowed_deg = (max_allowed_delta_mm / pulley_circumference_mm) * 360.0
            max_allowed_deg = max(0.0, max_allowed_deg)
            
            safe_pitch = max(-max_allowed_deg, min(max_allowed_deg, pitch_deg))
            safe_yaw = max(-max_allowed_deg, min(max_allowed_deg, yaw_deg))
            
            if safe_pitch != pitch_deg:
                self.pitch_slider.set(safe_pitch)
            if safe_yaw != yaw_deg:
                self.yaw_slider.set(safe_yaw)

            self.pitch_lbl.config(text=f"{safe_pitch:.1f}°")
            self.yaw_lbl.config(text=f"{safe_yaw:.1f}°")
            self.contract_lbl.config(text=f"{contract_pct:.1f}%")
            
            delta_l_pitch = (safe_pitch / 360.0) * pulley_circumference_mm
            delta_l_yaw = (safe_yaw / 360.0) * pulley_circumference_mm
            uniform_offset = L0_mm * (contract_pct / 100.0)
            
            # --- CORRECTED ANTAGONISTIC/SYMMETRIC TENDON MAPPING ---
            # Tendon 1 (+X), Tendon 2 (+Y), Tendon 3 (-X), Tendon 4 (-Y)
            l1 = L0_mm - uniform_offset - delta_l_pitch
            l2 = L0_mm - uniform_offset - delta_l_yaw
            l3 = L0_mm - uniform_offset + delta_l_pitch
            l4 = L0_mm - uniform_offset + delta_l_yaw  # <--- FIXED: Changed minus to plus
            # --------------------------------------------------------
            
            self.hw_l1_lbl.config(text=f"L1: {l1:.1f} mm")
            self.hw_l2_lbl.config(text=f"L2: {l2:.1f} mm")
            self.hw_l3_lbl.config(text=f"L3: {l3:.1f} mm")
            self.hw_l4_lbl.config(text=f"L4: {l4:.1f} mm")
            
            # Convert physical lengths to meters for kinematics mapping
            l1_m, l2_m, l3_m, l4_m = l1/1000.0, l2/1000.0, l3/1000.0, l4/1000.0
            
            # Map the four tendon lengths to arc parameters (s, k, phi) acc. to Table 1
            delta_l13 = l3_m - l1_m
            delta_l24 = l4_m - l2_m
            s_m = (l1_m + l2_m + l3_m + l4_m) / 4.0  # Arc Length (s)
            
            numerator = math.sqrt(delta_l13**2 + delta_l24**2)
            k = numerator / (self.d * 2.0 * s_m) if s_m > 0.0001 else 0.0  # Curvature (k)
            phi = math.atan2(delta_l24, delta_l13)                        # Angle of Curvature (phi)
            theta_rad = k * s_m                                           # Bending angle (theta)
            
            # Map arc parameters to X, Y, Z coordinates using Homogeneous Transform Matrix (Equation 4)
            if theta_rad < 0.001:
                tip_x, tip_y, tip_z = 0.0, 0.0, s_m * 1000.0
            else:
                r_deflection = (1.0 - math.cos(theta_rad)) / k
                tip_x = r_deflection * math.cos(phi) * 1000.0
                tip_y = - (r_deflection * math.sin(phi) * 1000.0)
                tip_z = (math.sin(theta_rad) / k) * 1000.0

            self.hw_x_lbl.config(text=f"X: {tip_x:.1f} mm")
            self.hw_y_lbl.config(text=f"Y: {tip_y:.1f} mm")
            self.hw_z_lbl.config(text=f"Z: {tip_z:.1f} mm")
            
            msg.data = [l1_m, l2_m, l3_m, l4_m]
            self.publisher_.publish(msg)

        elif current_tab == 1:
            l1_mm = min(self.l1_slider.get(), L0_mm) 
            l2_mm = min(self.l2_slider.get(), L0_mm)
            l3_mm = min(self.l3_slider.get(), L0_mm)
            l4_mm = min(self.l4_slider.get(), L0_mm)
            
            self.l1_slider.set(l1_mm)
            self.l2_slider.set(l2_mm)
            self.l3_slider.set(l3_mm)
            self.l4_slider.set(l4_mm)
            
            self.l1_lbl.config(text=f"{l1_mm:.1f} mm")
            self.l2_lbl.config(text=f"{l2_mm:.1f} mm")
            self.l3_lbl.config(text=f"{l3_mm:.1f} mm")
            self.l4_lbl.config(text=f"{l4_mm:.1f} mm")


            
            msg.data = [l1_mm / 1000.0, l2_mm / 1000.0, l3_mm / 1000.0, l4_mm / 1000.0]
            self.publisher_.publish(msg)

    def ros_loop(self):
        rclpy.spin_once(self, timeout_sec=0.0)
        
        try:
            t = self.tf_buffer.lookup_transform(
                'base_link', 
                'disk_10', 
                rclpy.time.Time()
            )
            
            actual_x = t.transform.translation.x * 1000.0
            actual_y = t.transform.translation.y * 1000.0
            actual_z = t.transform.translation.z * 1000.0
            
            # (Your existing hardware tab updates)
            self.actual_x_lbl.config(text=f"X (Real): {actual_x:.1f} mm")
            self.actual_y_lbl.config(text=f"Y (Real): {actual_y:.1f} mm")
            self.actual_z_lbl.config(text=f"Z (Real): {actual_z:.1f} mm")
            
            # --- ADD THESE LINES TO UPDATE THE IK TAB ---
            self.ik_actual_x_lbl.config(text=f"X (Real): {actual_x:.1f} mm")
            self.ik_actual_y_lbl.config(text=f"Y (Real): {actual_y:.1f} mm")
            self.ik_actual_z_lbl.config(text=f"Z (Real): {actual_z:.1f} mm")
            # --------------------------------------------
            
        except Exception as e:
            pass

        self.root.after(33, self.ros_loop)

def main(args=None):
    rclpy.init(args=args)
    node = DashboardGUI()
    node.root.mainloop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()