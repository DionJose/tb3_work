#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class FindFirstBlock(Node):
    def __init__(self):
        super().__init__('find_first_block')
        
        # 1. ROS2 Setup (Subscribers and Publishers)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.bridge = CvBridge()
        
        # Data variables
        self.target_found = False
        self.center_offset = 0
        self.min_front_dist = 10.0 # Start with a clear path

    def scan_callback(self, msg):
        # Check the front 20 degrees for obstacles (10 left, 10 right)
        # Filters out 0.0 values (sensor errors)
        front_ranges = msg.ranges[0:10] + msg.ranges[350:359]
        valid_ranges = [r for r in front_ranges if r > 0.05]
        if valid_ranges:
            self.min_front_dist = min(valid_ranges)

    def image_callback(self, msg):
        # --- YOUR PROVEN VISION LOGIC ---
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Blue Mask
        mask_blue = cv2.inRange(hsv, np.array([110, 150, 50]), np.array([125, 255, 255]))

        # Red Mask (Combined Low and High)
        mask_red_low = cv2.inRange(hsv, np.array([0, 100, 50]), np.array([10, 255, 255]))
        mask_red_high = cv2.inRange(hsv, np.array([165, 100, 50]), np.array([180, 255, 255]))
        mask_red = cv2.bitwise_or(mask_red_low, mask_red_high)

        # Combine both for targeting
        combined_mask = cv2.bitwise_or(mask_blue, mask_red)

        # --- TARGETING LOGIC ---
        moments = cv2.moments(combined_mask)
        if moments['m00'] > 500: # If we see enough colored pixels
            cx = int(moments['m10'] / moments['m00'])
            self.center_offset = cx - (msg.width / 2)
            self.target_found = True
            
            # Draw on screen so you can see what the robot is thinking
            cv2.circle(frame, (cx, int(msg.height/2)), 10, (0, 255, 0), -1)
        else:
            self.target_found = False

        # --- DRIVE DECISION ---
        self.decide_movement()

        # --- DISPLAY WINDOWS ---
        cv2.imshow('Robot View (Driving)', frame)
        cv2.imshow('Target Mask', combined_mask)
        cv2.waitKey(1)

    def decide_movement(self):
        move = Twist()

        # 1. SAFETY: If something is closer than 0.4m, STOP
        if self.min_front_dist < 0.4:
            self.get_logger().info('OBSTACLE REACHED! Stopping.')
            move.linear.x = 0.0
            move.angular.z = 0.0
        
        # 2. CHASE: If we see a block, drive toward it
        elif self.target_found:
            self.get_logger().info('Target Spotted! Aligning and Approaching...')
            move.linear.x = 0.15 # Slow forward
            # Angular speed is based on how far the block is from the center
            move.angular.z = -float(self.center_offset) / 300.0
        
        # 3. SEARCH: If we see nothing, spin 360
        else:
            self.get_logger().info('Searching... Spinning 360')
            move.linear.x = 0.0
            move.angular.z = 0.5

        self.cmd_vel_pub.publish(move)

def main(args=None):
    rclpy.init(args=args)
    node = FindFirstBlock()
    print("Navigator Node Started. Hunting for Red/Blue blocks...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()