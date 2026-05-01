#!/usr/bin/env python3
import math
import cv2
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32MultiArray
from block_wall_detection import VisionHelper

# ── 2m Arena Tuning ──────────────────────────────────────────────────────────
SPIN_SPEED       = 0.30   
APPROACH_SPEED   = 0.15   
PUSH_SPEED       = 0.12   
BACKOFF_SPEED    = 0.15   
TURN_KP          = 1.5
ALIGN_KP         = 0.004

APPROACH_DIST    = 0.22   # Get closer to the block before aligning
NEAR_WALL_DIST   = 0.18   # Safety stop for wall proximity (Odom)
PUSH_TIMEOUT     = 12.0   
BACKOFF_DIST     = 0.30   # Enough to clear block, small enough to not hit back wall

DBSCAN_EPS       = 0.12   
DBSCAN_MIN_PTS   = 2      
CORRECT_THRESH   = 0.20   # Block is "home" if within 20cm of wall
H_FOV_RAD        = 1.089  
DIST_EST         = 0.45   

# Wall coordinates for 2m x 2m arena (walls at ~1.0m)
WALL_POSE = {7: (0.0, 0.95), 42: (0.0, -0.95), 23: (0.95, 0.0), 0: (-0.95, 0.0)}

def _yaw_from_quat(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

def _adiff(a, b):
    d = a - b
    while d >  math.pi: d -= 2*math.pi
    while d < -math.pi: d += 2*math.pi
    return d

def _dbscan(points, eps, min_pts):
    if not points: return []
    final_clusters = []
    for color_key in ['red', 'blue']:
        color_points = [p for p in points if p[0] == color_key]
        if not color_points: continue
        xy = np.array([[p[1], p[2]] for p in color_points])
        n, labels, cluster_id = len(color_points), [-1] * len(color_points), 0
        def neighbours(i):
            return list(np.where(np.linalg.norm(xy - xy[i], axis=1) <= eps)[0])
        visited = [False] * n
        for i in range(n):
            if visited[i]: continue
            visited[i] = True
            nb = neighbours(i)
            if len(nb) < min_pts: continue
            labels[i] = cluster_id
            seed = set(nb) - {i}
            while seed:
                j = seed.pop()
                if not visited[j]:
                    visited[j] = True
                    nb2 = neighbours(j); 
                    if len(nb2) >= min_pts: seed |= set(nb2)
                if labels[j] == -1: labels[j] = cluster_id
            cluster_id += 1
        for cid in range(cluster_id):
            idxs = [i for i, l in enumerate(labels) if l == cid]
            cx = float(np.mean([color_points[i][1] for i in idxs]))
            cy = float(np.mean([color_points[i][2] for i in idxs]))
            final_clusters.append({'color': color_key, 'x': cx, 'y': cy})
    return final_clusters

class CompetitionMaster(Node):
    def __init__(self):
        super().__init__('competition_master')
        self.vision = VisionHelper()
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self._image_cb, 10)
        self.create_subscription(Int32MultiArray, '/arena/wall_ids', self._wall_ids_cb, 10)

        self.pose, self.prev_yaw, self.odom_ready = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}, None, False
        self.total_rotation, self.raw_detections, self.block_map = 0.0, [], []
        self.wall_id_red, self.wall_id_blue, self._vision = None, None, None
        self.state = 'WAIT_PARAMS'
        self.create_timer(0.05, self._tick)

    def _odom_cb(self, msg):
        p = msg.pose.pose
        yaw = _yaw_from_quat(p.orientation)
        self.pose = {'x': p.position.x, 'y': p.position.y, 'yaw': yaw}
        if self.prev_yaw is not None: self.total_rotation += abs(_adiff(yaw, self.prev_yaw))
        self.prev_yaw, self.odom_ready = yaw, True

    def _image_cb(self, msg):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is not None: self._vision = self.vision.analyse(frame)

    def _wall_ids_cb(self, msg):
        if len(msg.data) >= 2:
            self.wall_id_red, self.wall_id_blue = int(msg.data[0]), int(msg.data[1])

    def _cmd(self, linear=0.0, angular=0.0):
        t = Twist()
        t.linear.x, t.angular.z = float(linear), float(angular)
        self.pub_vel.publish(t)

    def _start_scan(self):
        self.total_rotation, self.raw_detections, self.state = 0.0, [], 'SPIN_SCAN'
        self.get_logger().info('Scanning arena...')

    def _on_correct_side(self, block):
        tid = self.wall_id_red if block['color'] == 'red' else self.wall_id_blue
        wp = WALL_POSE[tid]
        return math.hypot(block['x'] - wp[0], block['y'] - wp[1]) < CORRECT_THRESH

    def _tick(self):
        if not self.odom_ready: return
        v = self._vision

        if self.state == 'WAIT_PARAMS':
            if self.wall_id_red is not None: self._start_scan()

        elif self.state == 'SPIN_SCAN':
            self._cmd(angular=SPIN_SPEED)
            if v and v['color'] and v['center_x'] is not None:
                px_off = v['center_x'] - v['frame_w'] / 2
                bearing = self.pose['yaw'] + (px_off / v['frame_w']) * H_FOV_RAD
                bx = self.pose['x'] + DIST_EST * math.cos(bearing)
                by = self.pose['y'] + DIST_EST * math.sin(bearing)
                self.raw_detections.append((v['color'], bx, by))
            if self.total_rotation >= 2 * math.pi:
                self._cmd(0.0, 0.0)
                self.state = 'CLUSTER'

        elif self.state == 'CLUSTER':
            self.block_map = _dbscan(self.raw_detections, DBSCAN_EPS, DBSCAN_MIN_PTS)
            self.get_logger().info(f'Map updated: {len(self.block_map)} blocks found.')
            self.state = 'EVALUATE'

        elif self.state == 'EVALUATE':
            self.target_block = next((b for b in self.block_map if not self._on_correct_side(b)), None)
            if not self.target_block:
                self.get_logger().info('Arena cleared.'); self.state = 'DONE'; return
            self.target_marker_id = self.wall_id_red if self.target_block['color'] == 'red' else self.wall_id_blue
            self.state = 'TURN_TO_BLOCK'

        elif self.state == 'TURN_TO_BLOCK':
            dx, dy = self.target_block['x'] - self.pose['x'], self.target_block['y'] - self.pose['y']
            err = _adiff(math.atan2(dy, dx), self.pose['yaw'])
            if abs(err) < 0.08: self.state = 'APPROACH'
            else: self._cmd(angular=TURN_KP * err)

        elif self.state == 'APPROACH':
            dx, dy = self.target_block['x'] - self.pose['x'], self.target_block['y'] - self.pose['y']
            dist = math.hypot(dx, dy)
            if dist < APPROACH_DIST:
                self.state = 'ALIGN'
                return
            # Use visual steering if block is in sight
            if v and v['color'] == self.target_block['color']:
                px_err = v['center_x'] - v['frame_w'] / 2
                self._cmd(linear=APPROACH_SPEED, angular=-ALIGN_KP * px_err * 3)
            else:
                err = _adiff(math.atan2(dy, dx), self.pose['yaw'])
                self._cmd(linear=APPROACH_SPEED, angular=TURN_KP * 0.5 * err)

        elif self.state == 'ALIGN':
            if v and v['color'] == self.target_block['color']:
                px_err = v['center_x'] - v['frame_w'] / 2
                if abs(px_err) < 20:
                    wp = WALL_POSE[self.target_marker_id]
                    self.push_heading = math.atan2(wp[1] - self.pose['y'], wp[0] - self.pose['x'])
                    self.state = 'TURN_TO_WALL'
                else: self._cmd(angular=-ALIGN_KP * px_err)
            else: self.state = 'APPROACH'

        elif self.state == 'TURN_TO_WALL':
            err = _adiff(self.push_heading, self.pose['yaw'])
            if abs(err) < 0.08:
                self.push_start_time = time.time(); self.state = 'PUSH'
            else: self._cmd(angular=TURN_KP * err)

        elif self.state == 'PUSH':
            wp = WALL_POSE[self.target_marker_id]
            d_wall = math.hypot(wp[0] - self.pose['x'], wp[1] - self.pose['y'])
            
            # FIXED: Only stop if Odom says we are at the wall OR vision confirms marker is CLOSE
            at_wall = d_wall < NEAR_WALL_DIST
            marker_hit = (v and v['marker_id'] == self.target_marker_id and v['too_close'])
            
            if at_wall or marker_hit or (time.time()-self.push_start_time) > PUSH_TIMEOUT:
                self.backoff_start_x, self.backoff_start_y = self.pose['x'], self.pose['y']
                self.state = 'BACKOFF'
            else:
                err = _adiff(self.push_heading, self.pose['yaw'])
                self._cmd(linear=PUSH_SPEED, angular=TURN_KP * 0.3 * err)

        elif self.state == 'BACKOFF':
            d = math.hypot(self.pose['x'] - self.backoff_start_x, self.pose['y'] - self.backoff_start_y)
            if d >= BACKOFF_DIST: self._start_scan()
            else: self._cmd(linear=-BACKOFF_SPEED)

        elif self.state == 'DONE': self._cmd(0.0, 0.0)

def main():
    rclpy.init()
    node = CompetitionMaster()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.pub_vel.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()