#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np

class LocalNavigator(Node):
    def __init__(self):
        super().__init__('local_navigator')
        
        # Subscribe to the binary sidewalk mask published by your segmenter
        self.image_sub = self.create_subscription(
            Image,
            '/camera/sidewalk_mask',
            self.image_callback,
            10)
            
        # Subscribe to high-level commands (e.g., to trigger turns at intersections)
        self.cmd_sub = self.create_subscription(
            String,
            '/navigator/command',
            self.command_callback,
            10)
            
        # Publisher for the physical or simulated chassis motor velocities
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Controller Parameters
        self.Kp = 0.8          # Proportional steering gain (adjust to control steering sharpness)
        self.linear_speed = 0.25 # Base forward speed (m/s)
        
        # Simple State Machine
        self.state = "FOLLOW_SIDEWALK" # Active states: "FOLLOW_SIDEWALK", "TURNING_LEFT"
        self.turn_start_time = None
        self.turn_duration = 3.2       # Estimated seconds to execute a 90-degree left turn
        
        self.get_logger().info("Local Navigator Node Initialized.")

    def command_callback(self, msg):
        command = msg.data.lower().strip()
        if command == "turn_left" and self.state == "FOLLOW_SIDEWALK":
            self.state = "TURNING_LEFT"
            self.turn_start_time = self.get_clock().now()
            self.get_logger().info("Command received: Executing left turn maneuver.")

    def image_callback(self, data):
        h, w = data.height, data.width
        raw_data = np.frombuffer(data.data, dtype=np.uint8)
        binary_mask = raw_data.reshape((h, w))
        
        twist = Twist()
        
        if self.state == "FOLLOW_SIDEWALK":
            # --- SIDEWALK CENTERING CONTROLLER ---
            # Define a Region of Interest (ROI) looking at the path directly in front of the robot
            # (e.g., the lower 60% to 90% of the camera's image vertical coordinate frame)
            roi_ymin, roi_ymax = int(h * 0.6), int(h * 0.9)
            roi = binary_mask[roi_ymin:roi_ymax, :]
            
            # Find coordinates of sidewalk pixels (255 represents sidewalk)
            white_pixels = np.argwhere(roi == 255)
            
            if len(white_pixels) > 0:
                # Calculate the column centroid of the sidewalk
                centroid_x = np.mean(white_pixels[:, 1])
                
                # Determine deviation from camera center:
                # Positive error means centroid is left of center; negative means right.
                error = (w / 2.0) - centroid_x
                normalized_error = error / (w / 2.0) # Map error to [-1.0, 1.0]
                
                # Command linear forward speed and proportional angular steering
                twist.linear.x = self.linear_speed
                twist.angular.z = self.Kp * normalized_error
                
                self.get_logger().info(
                    f"Centering: Error = {normalized_error:.2f} | Steering Z = {twist.angular.z:.2f}", 
                    throttle_duration_sec=1.5
                )
            else:
                # Safe state: If the camera loses sight of the sidewalk, halt forward motion
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().warn("Sidewalk lost! Halting robot safety override.", throttle_duration_sec=1.5)
                
        elif self.state == "TURNING_LEFT":
            # --- OPEN-LOOP TURNING OVERRIDE ---
            now = self.get_clock().now()
            elapsed_sec = (now - self.turn_start_time).nanoseconds / 1e9
            
            if elapsed_sec < self.turn_duration:
                # Drive slowly forward while pivoting counter-clockwise (positive angular z)
                twist.linear.x = 0.12
                twist.angular.z = 0.6 # Pivot speed (rad/s)
                self.get_logger().info(f"Turning left: {elapsed_sec:.1f}s / {self.turn_duration}s", throttle_duration_sec=1.0)
            else:
                # Turn duration complete. Re-engage visual closed-loop tracking.
                self.state = "FOLLOW_SIDEWALK"
                self.get_logger().info("Turn complete. Returning to sidewalk tracking.")
                
        # Publish calculated command velocity
        self.vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LocalNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop motors before shutting down node
        stop_msg = Twist()
        node.vel_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()