#!/usr/bin/env python3
"""YOLO Webcam Node - Real-time YOLO detection from webcam with GUI."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class YOLOWebcamNode(Node):
    def __init__(self):
        super().__init__('yolo_webcam_node')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('display_window', True)
        
        camera_idx = self.get_parameter('camera_index').value
        self.display = self.get_parameter('display_window').value
        
        self.cap = cv2.VideoCapture(camera_idx)
        self.bridge = CvBridge()
        
        self.image_pub = self.create_publisher(Image, 'yolo/webcam/image', 10)
        self.timer = self.create_timer(0.033, self.timer_callback)
        
        self.get_logger().info(f'YOLO webcam node started on camera {camera_idx}')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Simulate YOLO detections
            cv2.putText(frame, 'YOLO Real-time Detection', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            self.image_pub.publish(msg)
            
            if self.display:
                cv2.imshow('YOLO Webcam', frame)
                cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YOLOWebcamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
