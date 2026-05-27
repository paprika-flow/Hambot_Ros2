#!/usr/bin/env python3
# hambot_sim_ws/src/hambot_bringup/hambot_bringup/sidewalk_segmenter.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

class SidewalkSegmenter(Node):
    def __init__(self):
        super().__init__('sidewalk_segmenter')
        
        # Subscribe directly to the raw labels map topic bridged from Gazebo
        self.subscription = self.create_subscription(
            Image,
            '/segmentation/labels_map',
            self.listener_callback,
            10)
            
        # Publisher for the processed binary mask
        self.publisher = self.create_publisher(Image, '/camera/sidewalk_mask', 10)
        self.get_logger().info("Sidewalk Segmenter Node (No-cv_bridge Edition) Initialized.")

    def listener_callback(self, data):
        try:
            # 1. Safely decode the raw ROS byte buffer directly into a NumPy array
            # This is standard and avoids C-extension compile mismatches.
            raw_data = np.frombuffer(data.data, dtype=np.uint8)
            
            # 2. Reshape the flat 1D array to a 3D image array based on message headers.
            # Gazebo's /segmentation/labels_map is standard RGB (3 channels)
            h, w = data.height, data.width
            img = raw_data.reshape((h, w, 3))
            
            # 3. Extract the Red Channel (Channel 0 in raw RGB) which holds the 8-bit label ID
            label_map = img[:, :, 0]
            
            # 4. Create the binary mask: 255 where label == 1 (Sidewalk), 0 elsewhere
            binary_mask = np.zeros_like(label_map, dtype=np.uint8)
            binary_mask[label_map == 1] = 255
            
            # 5. Pack the 2D array back into a standard single-channel (mono8) ROS 2 Image
            mask_msg = Image()
            mask_msg.header = data.header
            mask_msg.height = h
            mask_msg.width = w
            mask_msg.encoding = 'mono8'
            mask_msg.is_bigendian = data.is_bigendian
            mask_msg.step = w  # 1 byte per pixel for mono8 images
            mask_msg.data = binary_mask.tobytes()
            
            # 6. Publish the mask
            self.publisher.publish(mask_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing segmentation image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = SidewalkSegmenter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()