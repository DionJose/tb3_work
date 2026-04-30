#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
import time
import math
import numpy as np
from sklearn.cluster import DBSCAN
from block_wall_detection import GazeboVisionNode 

class CompetitionMaster(Node):
    def __init__(self):
        super().__init__('competition_master')
        self.vision = GazeboVisionNode()
        
        self.WALL_IDS = {'red': 23, 'blue': 0} 

        self.state = "DISCOVERING"
        self.raw_sightings = {'red': [], 'blue': []} # Separate sightings by color
        self.final_tasks = []   
        self.current_task = None
        
        self.total_rotation = 0.0
        self.last_yaw = 0.0
        self.pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.first_yaw_set = False

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.image_callback, 10)
        self.timer = self.create_timer(0.05, self.run_state_machine)

    def odom_cb(self, msg):
        self.pose['x'] = msg.pose.pose.position.x
        self.pose['y'] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        current_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.pose['yaw'] = current_yaw

        if not self.first_yaw_set:
            self.last_yaw = current_yaw
            self.first_yaw_set = True
            return

        delta = current_yaw - self.last_yaw
        if delta > math.pi: delta -= 2*math.pi
        elif delta < -math.pi: delta += 2*math.pi
        self.total_rotation += abs(delta)
        self.last_yaw = current_yaw

    def image_callback(self, msg):
        frame = self.vision.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        data = self.vision.get_vision_data(frame)

        if self.state == "DISCOVERING" and data["color"]:
            # Project detection into global map
            bx = self.pose['x'] + 0.45 * math.cos(self.pose['yaw'])
            by = self.pose['y'] + 0.45 * math.sin(self.pose['yaw'])
            self.raw_sightings[data["color"]].append([bx, by])

        elif self.state == "PUSH" and self.current_task:
            target_id = self.WALL_IDS.get(self.current_task['color'])
            if data["current_marker_id"] == target_id or data["too_close"]:
                self.get_logger().info(f"Delivered {self.current_task['color']} block.")
                self.state = "BACKOFF"
                self.backoff_start = time.time()

    def cluster_blocks(self):
        """ Runs DBSCAN independently for each color to prevent merging """
        self.final_tasks = []
        # eps 0.15m is tighter for 2m arena to distinguish nearby blocks
        dbscan = DBSCAN(eps=0.15, min_samples=3) 

        for color in ['red', 'blue']:
            sightings = self.raw_sightings[color]
            if len(sightings) < 3: continue
            
            pts = np.array(sightings)
            clustering = dbscan.fit(pts)
            
            for label in set(clustering.labels_):
                if label == -1: continue # Ignore noise
                
                mask = (clustering.labels_ == label)
                center = np.mean(pts[mask], axis=0)
                self.final_tasks.append({'color': color, 'x': center[0], 'y': center[1]})
        
        self.get_logger().info(f"Clustering complete. Found {len(self.final_tasks)} distinct blocks.")

    def run_state_machine(self):
        move = Twist()
        
        if self.state == "DISCOVERING":
            move.angular.z = 0.5
            if self.total_rotation >= 6.3:
                self.cluster_blocks()
                self.state = "PROCESS_QUEUE"

        elif self.state == "PROCESS_QUEUE":
            if not self.final_tasks:
                self.get_logger().info("No misplaced blocks. Restarting scan.")
                self.state = "DISCOVERING"
                self.total_rotation = 0.0
                self.raw_sightings = {'red': [], 'blue': []}
            else:
                self.current_task = self.final_tasks.pop(0)
                self.state = "SEARCH"

        elif self.state == "SEARCH":
            # Turn toward the target color
            move.angular.z = 0.3
            # In a real scenario, you'd calculate the angle to self.current_task['x','y']
            # For now, we wait for visual lock to transition
            self.state = "PUSH"
            self.push_start = time.time()

        elif self.state == "PUSH":
            move.linear.x = 0.18
            if (time.time() - self.push_start) > 12.0:
                self.state = "BACKOFF"
                self.backoff_start = time.time()

        elif self.state == "BACKOFF":
            move.linear.x = -0.2
            if (time.time() - self.backoff_start) > 3.0:
                self.state = "PROCESS_QUEUE"
        
        self.cmd_vel_pub.publish(move)

def main():
    rclpy.init()
    node = CompetitionMaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())
        rclpy.shutdown()

if __name__ == '__main__':
    main()