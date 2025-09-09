#!/usr/bin/env python3
# Video Publisher Node for Lane Detection Testing

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from ament_index_python.packages import get_package_share_directory

class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__('image_publisher_node')
        
        # Declare parameters
        self.declare_parameter('video_path', '')
        self.declare_parameter('loop_video', True)
        self.declare_parameter('publish_rate', 30.0)
        
        # Get parameters
        video_path = self.get_parameter('video_path').value
        self.loop_video = self.get_parameter('loop_video').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # If no video path provided, try to find the video in common locations
        if not video_path:
            # Try common locations for the video file
            possible_paths = [
                os.path.expanduser('~/Downloads/road0.mp4'),  # Your video location
                os.path.expanduser('~/Videos/road0.mp4'),
                os.path.expanduser('~/road0.mp4'),
                os.path.join(get_package_share_directory('lka'), 'videos', 'road0.mp4')
            ]
            
            for path in possible_paths:
                if os.path.exists(path):
                    video_path = path
                    self.get_logger().info(f"Found video at: {video_path}")
                    break
            
            if not video_path:
                self.get_logger().error("No video file found in common locations")
                self.get_logger().info("Please specify the video path with: --ros-args -p video_path:=/path/to/video.mp4")
        
        # Initialize video capture
        self.cap = None
        if video_path:
            self.cap = cv2.VideoCapture(video_path)
            if not self.cap.isOpened():
                self.get_logger().error(f"Could not open video: {video_path}")
                self.cap = None
        
        # If video file didn't work, try camera
        if self.cap is None:
            self.get_logger().info("Trying to open camera...")
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                self.get_logger().error("Could not open camera either")
                raise RuntimeError("No video source available")
            else:
                self.get_logger().info("Using camera as video source")
        
        # Initialize bridge and publisher
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/RGBImage', 10)
        
        # Create timer for publishing frames
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_frame)
        
        self.get_logger().info("Image Publisher Node started")

    def publish_frame(self):
        ret, frame = self.cap.read()
        
        if ret:
            # Convert frame to ROS message and publish
            try:
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.publisher.publish(ros_image)
                self.get_logger().info("Published frame", throttle_duration_sec=2.0)  # Log every 2 seconds
            except Exception as e:
                self.get_logger().error(f"Error converting image: {e}")
        else:
            if self.loop_video:
                # Loop video if enabled
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                self.get_logger().info("Looping video...")
            else:
                self.get_logger().info("End of video reached")
                self.timer.cancel()

    def destroy_node(self):
        if self.cap:
            self.cap.release()
        self.get_logger().info("Image Publisher Node shutting down")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
