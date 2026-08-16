#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import tf2_ros
from tf2_ros import TransformException

class SystemVisualizer(Node):
    def __init__(self):
        super().__init__('system_visualizer')
        
        self.marker_pub = self.create_publisher(MarkerArray, '/continuum_rods', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
       # 4 Tendons at 0, 90, 180, 270 degrees (5.0 mm from center)
        self.tendon_offsets = [(0.005, 0.0), (0.0, 0.005), (-0.005, 0.0), (0.0, -0.005)]
        
        # 4 Torsional Rods shifted by 45 degrees (5.0 mm from center)
        self.rod_offsets = [(0.00354, 0.00354), (-0.00354, 0.00354), (-0.00354, -0.00354), (0.00354, -0.00354)]
        
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def get_transformed_point(self, t, offset_x, offset_y, offset_z=0.0):
        """Calculates exact 3D coordinates applying local X, Y, and Z offsets."""
        qx = t.transform.rotation.x
        qy = t.transform.rotation.y
        qz = t.transform.rotation.z
        qw = t.transform.rotation.w
        
        # Quaternion rotation matrix optimizations
        x2 = qx + qx; y2 = qy + qy; z2 = qz + qz
        xx = qx * x2; xy = qx * y2; xz = qx * z2
        yy = qy * y2; yz = qy * z2; zz = qz * z2
        wx = qw * x2; wy = qw * y2; wz = qw * z2

        # Rotate the local offset vector into the global frame
        dx = offset_x * (1.0 - (yy + zz)) + offset_y * (xy - wz) + offset_z * (xz + wy)
        dy = offset_x * (xy + wz) + offset_y * (1.0 - (xx + zz)) + offset_z * (yz - wx)
        dz = offset_x * (xz - wy) + offset_y * (yz + wx) + offset_z * (1.0 - (xx + yy))

        p = Point()
        p.x = t.transform.translation.x + dx
        p.y = t.transform.translation.y + dy
        p.z = t.transform.translation.z + dz
        return p

    def timer_callback(self):
        marker_array = MarkerArray()
        marker_id = 0
        links = ['base_link'] + [f'disk_{i}' for i in range(1, 11)]
        
        # --- 1. DRAW THE 4 TENDONS ---
        for offset in self.tendon_offsets:
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'tendons'
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # Tendon styling: 0.4mm thick, bright gold/yellow
            marker.scale.x = 0.0004
            marker.color.r = 0.9
            marker.color.g = 0.8
            marker.color.b = 0.1
            marker.color.a = 1.0
            
            for link_name in links:
                try:
                    t = self.tf_buffer.lookup_transform('base_link', link_name, rclpy.time.Time())
                    marker.points.append(self.get_transformed_point(t, offset[0], offset[1], 0.0))
                except TransformException:
                    pass
            marker_array.markers.append(marker)

        # --- 2. DRAW THE 4 STAINLESS STEEL RODS ---
        for offset in self.rod_offsets:
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'rods'
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # Rod styling: 0.7mm thick, shiny steel grey
            marker.scale.x = 0.0007
            marker.color.r = 0.6
            marker.color.g = 0.6
            marker.color.b = 0.6
            marker.color.a = 1.0
            
            for link_name in links:
                try:
                    t = self.tf_buffer.lookup_transform('base_link', link_name, rclpy.time.Time())
                    # Add standard hole position
                    marker.points.append(self.get_transformed_point(t, offset[0], offset[1], 0.0))
                    
                    # PROTRUSION: If this is the last disk, project the rod 10mm out the top
                    if link_name == 'disk_10':
                        marker.points.append(self.get_transformed_point(t, offset[0], offset[1], 0.01))
                except TransformException:
                    pass
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = SystemVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()