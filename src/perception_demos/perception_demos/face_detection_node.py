#!/usr/bin/env python3
"""Face Detection Node - Detect human faces using Haar cascades or DNN."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import cv2


class FaceDetectionNode(Node):
    def __init__(self):
        super().__init__('face_detection_node')
        
        self.declare_parameter('method', 'haar')  # haar or dnn
        self.declare_parameter('scale_factor', 1.1)
        self.declare_parameter('min_neighbors', 5)
        self.declare_parameter('min_size', 30)
        
        method = self.get_parameter('method').value
        scale_factor = self.get_parameter('scale_factor').value
        min_neighbors = self.get_parameter('min_neighbors').value
        min_size = self.get_parameter('min_size').value
        
        self.bridge = CvBridge()
        
        # Load Haar Cascade
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )
        self.eye_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_eye.xml'
        )
        
        self.scale_factor = scale_factor
        self.min_neighbors = min_neighbors
        self.min_size = (min_size, min_size)
        
        self.image_sub = self.create_subscription(Image, 'camera/image_raw',
                                                  self.image_callback, 10)
        self.faces_pub = self.create_publisher(PoseArray, 'faces/poses', 10)
        self.vis_pub = self.create_publisher(Image, 'faces/visualization', 10)
        
        self.get_logger().info(f'Face detection started (method={method})')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Detect faces
            faces = self.face_cascade.detectMultiScale(
                gray, self.scale_factor, self.min_neighbors, minSize=self.min_size
            )
            
            vis_image = cv_image.copy()
            pose_array = PoseArray()
            pose_array.header = msg.header
            
            for (x, y, w, h) in faces:
                # Draw face rectangle
                cv2.rectangle(vis_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                
                # Detect eyes within face ROI
                roi_gray = gray[y:y+h, x:x+w]
                eyes = self.eye_cascade.detectMultiScale(roi_gray)
                
                for (ex, ey, ew, eh) in eyes:
                    cv2.rectangle(vis_image, (x+ex, y+ey), (x+ex+ew, y+ey+eh), (255, 0, 0), 2)
                
                # Create pose
                pose = Pose()
                pose.position.x = float(x + w/2)
                pose.position.y = float(y + h/2)
                pose.position.z = float(w)  # Use width as rough size indicator
                pose_array.poses.append(pose)
                
                # Label
                cv2.putText(vis_image, f'Face {len(pose_array.poses)}', (x, y-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Add count
            cv2.putText(vis_image, f'Faces: {len(faces)}', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Publish
            self.faces_pub.publish(pose_array)
            
            vis_msg = self.bridge.cv2_to_imgmsg(vis_image, encoding='bgr8')
            vis_msg.header = msg.header
            self.vis_pub.publish(vis_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error detecting faces: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
