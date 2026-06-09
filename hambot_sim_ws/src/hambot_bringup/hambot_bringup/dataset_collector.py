#!/usr/bin/env python3
import os
import time
import cv2
import numpy as np
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image

class DatasetCollector(Node):
    def __init__(self):
        super().__init__('dataset_collector')
        
        # Parameters
        self.declare_parameter('save_directory', 'processed_rosmaster_photos/mask')
        self.declare_parameter('save_interval', 1.0)        # Save rate in seconds
        self.declare_parameter('target_gray_value', 15)     # Remap 255 -> 15 to match training script default
        
        self.save_dir = self.get_parameter('save_directory').value
        self.save_interval = self.get_parameter('save_interval').value
        self.target_gray_value = self.get_parameter('target_gray_value').value
        
        # Create directory if it does not exist
        os.makedirs(self.save_dir, exist_ok=True)
        
        self.last_save_time = 0.0
        
        # Determine the next available file index to avoid overwriting previous runs
        self.img_counter = self.get_next_file_index()
        
        self.sub = self.create_subscription(
            Image,
            '/camera/sidewalk_mask',
            self.image_callback,
            10
        )
        
        self.get_logger().info(
            f"Dataset Collector Node (No-cv_bridge Edition) Active.\n"
            f"Saving output to: {os.path.abspath(self.save_dir)}\n"
            f"Frame Interval: {self.save_interval}s\n"
            f"Mapped Target Gray Value: {self.target_gray_value}"
        )

    def get_next_file_index(self):
        """Scans the directory to find the next incremental frame index."""
        if not os.path.exists(self.save_dir):
            return 0
        files = os.listdir(self.save_dir)
        indices = []
        for f in files:
            if f.endswith('.png'):
                name_without_ext = os.path.splitext(f)[0]
                parts = name_without_ext.split('_')
                if len(parts) >= 2 and parts[-1].isdigit():
                    indices.append(int(parts[-1]))
        return max(indices) + 1 if indices else 0

    def image_callback(self, msg: Image):
        current_time = time.time()
        if current_time - self.last_save_time >= self.save_interval:
            try:
                # Direct conversion from mono8 raw bytes to numpy array (bypassing cv_bridge)
                if msg.encoding == 'mono8':
                    cv_img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
                elif msg.encoding == 'rgb8':
                    rgb_img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                    cv_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2GRAY)
                else:
                    # Fallback conversion
                    cv_img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
                
                # Copy the buffer to make it writeable (np.frombuffer generates a read-only view)
                cv_img = cv_img.copy()

                # Remap the mask's white pixels (255) to match the expected training value (e.g. 15)
                if self.target_gray_value != 255:
                    cv_img[cv_img == 255] = self.target_gray_value
                
                # Save as PNG
                filename = f"frame_{self.img_counter:05d}.png"
                filepath = os.path.join(self.save_dir, filename)
                cv2.imwrite(filepath, cv_img)
                
                self.get_logger().info(f"Successfully saved {filename}")
                
                self.img_counter += 1
                self.last_save_time = current_time
                
            except Exception as e:
                self.get_logger().error(f"Image save error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DatasetCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()