#!/usr/bin/env python3
"""Marker Generator Node - Generate printable ArUco/AprilTag markers."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
import os


class MarkerGeneratorNode(Node):
    def __init__(self):
        super().__init__('marker_generator_node')
        
        self.declare_parameter('marker_type', 'aruco')
        self.declare_parameter('marker_ids', [0, 1, 2, 3, 4])
        self.declare_parameter('marker_size', 200)  # pixels
        self.declare_parameter('output_directory', './markers')
        self.declare_parameter('dictionary_id', 'DICT_4X4_50')
        
        marker_type = self.get_parameter('marker_type').value
        marker_ids = self.get_parameter('marker_ids').value
        marker_size = self.get_parameter('marker_size').value
        output_dir = self.get_parameter('output_directory').value
        dict_name = self.get_parameter('dictionary_id').value
        
        os.makedirs(output_dir, exist_ok=True)
        
        if marker_type == 'aruco':
            self.generate_aruco_markers(marker_ids, marker_size, output_dir, dict_name)
        
        self.status_pub = self.create_publisher(String, 'marker_generator/status', 10)
        
        status_msg = String()
        status_msg.data = f'Generated {len(marker_ids)} {marker_type} markers in {output_dir}'
        self.status_pub.publish(status_msg)
        
        self.get_logger().info(status_msg.data)

    def generate_aruco_markers(self, ids, size, output_dir, dict_name):
        aruco_dicts = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
            'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
        }
        
        aruco_dict = cv2.aruco.getPredefinedDictionary(
            aruco_dicts.get(dict_name, cv2.aruco.DICT_4X4_50)
        )
        
        for marker_id in ids:
            marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, size)
            
            # Add border
            border_size = 20
            bordered = cv2.copyMakeBorder(marker_image, border_size, border_size,
                                         border_size, border_size,
                                         cv2.BORDER_CONSTANT, value=255)
            
            # Add ID text
            cv2.putText(bordered, f'ID: {marker_id}', (10, bordered.shape[0] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, 0, 2)
            
            filename = os.path.join(output_dir, f'aruco_marker_{marker_id}.png')
            cv2.imwrite(filename, bordered)
            
            self.get_logger().info(f'Generated: {filename}')


def main(args=None):
    rclpy.init(args=args)
    node = MarkerGeneratorNode()
    
    # Run once and shutdown
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
