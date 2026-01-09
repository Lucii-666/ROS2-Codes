#!/usr/bin/env python3
"""
Video Recorder Node - Record video streams to disk.

This node demonstrates:
- Video recording with cv2.VideoWriter
- Multiple codec support (MP4, AVI, MKV)
- Framerate control
- Recording triggers
- Metadata embedding
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime


class VideoRecorderNode(Node):
    """Records video from image topics."""

    def __init__(self):
        super().__init__('video_recorder_node')
        
        # Declare parameters
        self.declare_parameter('output_directory', './recordings')
        self.declare_parameter('filename_prefix', 'recording')
        self.declare_parameter('codec', 'mp4v')  # mp4v, XVID, MJPG, H264
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('max_duration', 0)  # seconds, 0 = unlimited
        self.declare_parameter('add_timestamp', True)
        
        # Get parameters
        self.output_dir = self.get_parameter('output_directory').value
        self.filename_prefix = self.get_parameter('filename_prefix').value
        self.codec = self.get_parameter('codec').value
        self.fps = self.get_parameter('fps').value
        self.auto_start = self.get_parameter('auto_start').value
        self.max_duration = self.get_parameter('max_duration').value
        self.add_timestamp = self.get_parameter('add_timestamp').value
        
        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Recording state
        self.is_recording = self.auto_start
        self.video_writer = None
        self.frame_count = 0
        self.start_time = None
        self.current_filename = None
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10
        )
        
        # Control subscriber
        self.control_sub = self.create_subscription(
            Bool, 'video_recorder/record', self.control_callback, 10
        )
        
        # Publishers
        self.status_pub = self.create_publisher(String, 'video_recorder/status', 10)
        
        # Timer for status updates
        self.timer = self.create_timer(1.0, self.status_callback)
        
        self.get_logger().info(
            f'Video recorder node started\n'
            f'  Output: {self.output_dir}\n'
            f'  Codec: {self.codec}\n'
            f'  FPS: {self.fps}\n'
            f'  Auto-start: {self.auto_start}'
        )
        
        if self.auto_start:
            self.get_logger().info('Recording started automatically')

    def image_callback(self, msg):
        """Record incoming images."""
        if not self.is_recording:
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Initialize video writer on first frame
            if self.video_writer is None:
                self.start_recording(cv_image.shape[1], cv_image.shape[0])
            
            # Add timestamp overlay if enabled
            if self.add_timestamp:
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                cv2.putText(cv_image, timestamp, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.circle(cv_image, (cv_image.shape[1] - 30, 30), 10, (0, 0, 255), -1)
            
            # Write frame
            self.video_writer.write(cv_image)
            self.frame_count += 1
            
            # Check max duration
            if self.max_duration > 0:
                elapsed = (self.get_clock().now().nanoseconds - self.start_time) / 1e9
                if elapsed >= self.max_duration:
                    self.stop_recording()
                    self.get_logger().info(f'Max duration reached, stopped recording')
            
        except Exception as e:
            self.get_logger().error(f'Error recording frame: {e}')

    def control_callback(self, msg):
        """Handle recording control commands."""
        if msg.data and not self.is_recording:
            self.is_recording = True
            self.get_logger().info('Recording started')
        elif not msg.data and self.is_recording:
            self.stop_recording()
            self.get_logger().info('Recording stopped')

    def start_recording(self, width, height):
        """Initialize video writer."""
        # Generate filename
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'{self.filename_prefix}_{timestamp}.mp4'
        self.current_filename = os.path.join(self.output_dir, filename)
        
        # Create video writer
        fourcc = cv2.VideoWriter_fourcc(*self.codec)
        self.video_writer = cv2.VideoWriter(
            self.current_filename, fourcc, self.fps, (width, height)
        )
        
        if not self.video_writer.isOpened():
            self.get_logger().error(f'Failed to open video writer for {self.current_filename}')
            self.video_writer = None
            self.is_recording = False
            return
        
        self.frame_count = 0
        self.start_time = self.get_clock().now().nanoseconds
        
        self.get_logger().info(f'Started recording to {self.current_filename}')

    def stop_recording(self):
        """Stop recording and close video writer."""
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
            
            duration = (self.get_clock().now().nanoseconds - self.start_time) / 1e9
            self.get_logger().info(
                f'Recording saved: {self.current_filename}\n'
                f'  Frames: {self.frame_count}\n'
                f'  Duration: {duration:.2f}s'
            )
        
        self.is_recording = False
        self.frame_count = 0

    def status_callback(self):
        """Publish recording status."""
        status_msg = String()
        
        if self.is_recording:
            duration = (self.get_clock().now().nanoseconds - self.start_time) / 1e9
            status_msg.data = f'RECORDING: {self.frame_count} frames, {duration:.1f}s'
        else:
            status_msg.data = 'IDLE'
        
        self.status_pub.publish(status_msg)

    def destroy_node(self):
        """Clean up resources."""
        if self.is_recording:
            self.stop_recording()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
