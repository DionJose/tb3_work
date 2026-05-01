#!/usr/bin/env python3
"""
sort_blocks.py
--------------
Sort coloured blocks into their correct half of the arena by pushing.

Strategy:
  INIT_SPIN     - spin a full circle, see ArUco markers, resolve sides
  SEARCHING     - spin in place looking for a misplaced block
  APPROACHING   - drive toward the chosen block (slows as it gets close)
  CAPTURE_PUSH  - block disappeared under camera; drive a short distance to
                  ensure it is between the arms
  ALIGN_TO_GOAL - spin clockwise until facing the correct half
  CARRY         - drive straight until past midline + buffer
  RELEASE       - reverse to leave the block in place
  VERIFY_DONE   - spin a full circle; if no misplaced blocks remain → DONE
  DONE          - stop

Localisation: /odom (Gazebo ground truth in sim).
Sides: red side X-sign decided from red marker id (23 → +1, 0 → -1).

Run:
  ros2 run turtlebot3_task sort_blocks
or override marker IDs:
  ros2 run turtlebot3_task sort_blocks --ros-args -p red_marker:=0 -p blue_marker:=23
"""
import math
import rclpy
import rclpy.duration
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np


# ---- TUNABLE PARAMETERS ----------------------------------------------------
BLOCK_AREA_MIN          = 500
APPROACH_SPEED_FAR      = 0.14
APPROACH_SPEED_NEAR     = 0.07
NEAR_AREA_THRESHOLD     = 4000      # block area at which to slow down
COMMIT_AREA_THRESHOLD   = 8000      # block area at which capture is plausible
COMMIT_OFFSET_THRESHOLD = 80        # block offset (px) below which capture is plausible
CAPTURE_PUSH_DURATION   = 1.2       # s: drive forward to seat the block in arms
ALIGN_ANGULAR_SPEED     = 0.5       # rad/s, always positive (option b: clockwise)
ALIGN_YAW_TOLERANCE     = 0.10      # rad: ~6 degrees
CARRY_SPEED             = 0.10
CARRY_BUFFER            = 0.30      # how far past midline to deposit the block (m)
CARRY_TIMEOUT           = 12.0      # safety: max seconds in CARRY before giving up
RELEASE_REVERSE_SPEED   = 0.10
RELEASE_REVERSE_TIME    = 1.5
SEARCH_ANGULAR_SPEED    = 0.4
INIT_SPIN_ANGULAR_SPEED = 0.4
ALIGN_GAIN              = 0.005
SAFE_FRONT_STOP         = 0.18      # too close to a wall while carrying
# ----------------------------------------------------------------------------


class BlockSorter(Node):
    def __init__(self):
        super().__init__('block_sorter')

        # ---- ROS parameters
        self.declare_parameter('red_marker', 23)
        self.declare_parameter('blue_marker', 0)
        self.red_marker_id = self.get_parameter('red_marker').value
        self.blue_marker_id = self.get_parameter('blue_marker').value

        # ---- Subscribers / publishers
        self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bridge = CvBridge()

        # ---- ArUco
        try:
            self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
            self.aruco_params = aruco.DetectorParameters()
            self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        except AttributeError:
            self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
            self.aruco_params = aruco.DetectorParameters_create()
            self.aruco_detector = None

        # ---- State
        self.state = 'INIT_SPIN'
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.front_dist = 10.0
        self.frame_width = 640

        self.red_blocks_pixel = []
        self.blue_blocks_pixel = []
        self.detected_marker_ids = set()

        # Side resolution
        self.red_side_x_sign = None  # +1 or -1
        self.sides_known = False

        # INIT_SPIN tracking
        self.init_spin_start_yaw = None
        self.init_spin_accumulated = 0.0
        self.init_spin_last_yaw = None

        # Targeting state
        self.target_colour = None
        self.last_target_area = 0
        self.last_target_offset = 0
        self.max_target_area_seen = 0
        self.min_target_offset_at_max = 0

        # CAPTURE_PUSH
        self.capture_push_until = None

        # ALIGN_TO_GOAL
        self.target_goal_yaw = None      # world yaw the robot should face

        # CARRY
        self.carry_start_time = None

        # RELEASE
        self.release_start_time = None

        # VERIFY_DONE — uses same accumulator as init spin
        self.verify_spin_accumulated = 0.0
        self.verify_spin_last_yaw = None
        self.verify_saw_misplaced = False

        # Decision loop
        self.timer = self.create_timer(0.1, self.decide)

        self.get_logger().info(
            f'Block sorter started. red_marker={self.red_marker_id}, blue_marker={self.blue_marker_id}'
        )

    # ===== CALLBACKS ========================================================
    def odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_cb(self, msg):
        front = list(msg.ranges[0:10]) + list(msg.ranges[-10:])
        valid = [r for r in front if 0.05 < r < 10.0]
        self.front_dist = min(valid) if valid else 10.0

    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.frame_width = frame.shape[1]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # ----- COLOUR MASKS (DO NOT CHANGE THESE NUMBERS) -------------------
        mask_blue = cv2.inRange(hsv, np.array([110, 150, 50]), np.array([125, 255, 255]))
        mask_red_low = cv2.inRange(hsv, np.array([0, 100, 50]), np.array([10, 255, 255]))
        mask_red_high = cv2.inRange(hsv, np.array([165, 100, 50]), np.array([180, 255, 255]))
        mask_red = cv2.bitwise_or(mask_red_low, mask_red_high)
        # --------------------------------------------------------------------

        self.red_blocks_pixel = self._extract_blocks(mask_red)
        self.blue_blocks_pixel = self._extract_blocks(mask_blue)

        for cx, area, x, y, w, h in self.red_blocks_pixel:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
        for cx, area, x, y, w, h in self.blue_blocks_pixel:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)

        # ArUco
        if self.aruco_detector:
            corners, ids, _ = self.aruco_detector.detectMarkers(frame)
        else:
            corners, ids, _ = aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

        self.detected_marker_ids = set()
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            for mid in ids.flatten():
                self.detected_marker_ids.add(int(mid))

        # HUD
        cv2.putText(frame, f'STATE: {self.state}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f'POS: ({self.robot_x:.2f}, {self.robot_y:.2f}) yaw={math.degrees(self.robot_yaw):.0f}',
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        if self.sides_known:
            cv2.putText(frame,
                        f'RED side: X{">"if self.red_side_x_sign>0 else "<"} 0',
                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        # Log correctly-placed blocks (once per cycle, not every frame)
        if self.sides_known:
            for cx, area, *_ in self.red_blocks_pixel:
                wx, _ = self.estimate_block_world_xy(cx)
                if self.is_correct_side('red', wx):
                    self.get_logger().info('RED block in correct place', throttle_duration_sec=2.0)
                    break
            for cx, area, *_ in self.blue_blocks_pixel:
                wx, _ = self.estimate_block_world_xy(cx)
                if self.is_correct_side('blue', wx):
                    self.get_logger().info('BLUE block in correct place', throttle_duration_sec=2.0)
                    break

        cv2.imshow('Robot View', frame)
        cv2.waitKey(1)

    def _extract_blocks(self, mask):
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for cnt in cnts:
            area = cv2.contourArea(cnt)
            if area < BLOCK_AREA_MIN:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            cx = x + w // 2
            out.append((cx, area, x, y, w, h))
        return out

    # ===== HELPERS ==========================================================
    def _resolve_sides_if_possible(self):
        if self.sides_known:
            return
        if self.red_marker_id == 23:
            self.red_side_x_sign = +1
        elif self.red_marker_id == 0:
            self.red_side_x_sign = -1
        else:
            self.get_logger().error(
                f'red_marker={self.red_marker_id} cannot define a side; must be 23 (east) or 0 (west).'
            )
            return
        self.sides_known = True
        self.get_logger().info(
            f'Sides resolved: red is X{"+" if self.red_side_x_sign>0 else "-"}, '
            f'blue is X{"-" if self.red_side_x_sign>0 else "+"}'
        )

    def is_correct_side(self, colour, world_x):
        if not self.sides_known:
            return True
        if colour == 'red':
            return (world_x * self.red_side_x_sign) > 0
        return (world_x * self.red_side_x_sign) < 0

    def estimate_block_world_xy(self, pixel_cx):
        FOV_RAD = math.radians(60.0)
        bearing = (pixel_cx - self.frame_width / 2.0) / self.frame_width * FOV_RAD
        distance = max(0.2, min(self.front_dist, 2.0))
        bx = self.robot_x + distance * math.cos(self.robot_yaw - bearing)
        by = self.robot_y + distance * math.sin(self.robot_yaw - bearing)
        return bx, by

    def _accumulate_yaw_change(self, last_yaw_attr, accum_attr):
        """Track total absolute yaw rotated; for detecting full 360° spins."""
        last = getattr(self, last_yaw_attr)
        if last is None:
            setattr(self, last_yaw_attr, self.robot_yaw)
            return getattr(self, accum_attr)
        d = self.robot_yaw - last
        # wrap to [-pi, pi]
        d = (d + math.pi) % (2 * math.pi) - math.pi
        new_total = getattr(self, accum_attr) + abs(d)
        setattr(self, accum_attr, new_total)
        setattr(self, last_yaw_attr, self.robot_yaw)
        return new_total

    def _publish_cmd(self, lin, ang):
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        self.cmd_vel_pub.publish(msg)

    # ===== STATE MACHINE ====================================================
    def decide(self):
        if self.state == 'INIT_SPIN':
            self._do_init_spin()
        elif self.state == 'SEARCHING':
            self._do_searching()
        elif self.state == 'APPROACHING':
            self._do_approaching()
        elif self.state == 'CAPTURE_PUSH':
            self._do_capture_push()
        elif self.state == 'ALIGN_TO_GOAL':
            self._do_align_to_goal()
        elif self.state == 'CARRY':
            self._do_carry()
        elif self.state == 'RELEASE':
            self._do_release()
        elif self.state == 'VERIFY_DONE':
            self._do_verify_done()
        elif self.state == 'DONE':
            self._publish_cmd(0.0, 0.0)

    # ----- INIT_SPIN: full 360° to map sides + survey -----------------------
    def _do_init_spin(self):
        self._resolve_sides_if_possible()  # static resolution from marker id

        accumulated = self._accumulate_yaw_change('init_spin_last_yaw', 'init_spin_accumulated')
        if accumulated >= 2 * math.pi * 1.05:  # full circle plus a touch
            if self.sides_known:
                self.get_logger().info('Initial survey complete. Searching for misplaced blocks.')
                self.state = 'SEARCHING'
            else:
                self.get_logger().error('Sides could not be resolved (bad marker id?). Halting.')
                self.state = 'DONE'
            return

        self._publish_cmd(0.0, INIT_SPIN_ANGULAR_SPEED)

    # ----- SEARCHING: spin until a misplaced block is in view ---------------
    def _do_searching(self):
        target = self._pick_misplaced_block()
        if target is not None:
            self.target_colour = target[0]
            self.last_target_area = target[2]
            self.last_target_offset = target[1] - self.frame_width / 2.0
            self.max_target_area_seen = target[2]
            self.min_target_offset_at_max = target[1] - self.frame_width / 2.0
            self.get_logger().info(f'Target acquired: misplaced {self.target_colour} block.')
            self.state = 'APPROACHING'
            return
        self._publish_cmd(0.0, SEARCH_ANGULAR_SPEED)

    def _pick_misplaced_block(self):
        candidates = []
        for cx, area, *_ in self.red_blocks_pixel:
            wx, _ = self.estimate_block_world_xy(cx)
            if not self.is_correct_side('red', wx):
                candidates.append(('red', cx, area))
        for cx, area, *_ in self.blue_blocks_pixel:
            wx, _ = self.estimate_block_world_xy(cx)
            if not self.is_correct_side('blue', wx):
                candidates.append(('blue', cx, area))
        if not candidates:
            return None
        candidates.sort(key=lambda c: c[2], reverse=True)
        return candidates[0]

    # ----- APPROACHING ------------------------------------------------------
    def _do_approaching(self):
        blocks = self.red_blocks_pixel if self.target_colour == 'red' else self.blue_blocks_pixel

        if blocks:
            blocks_sorted = sorted(blocks, key=lambda b: b[1], reverse=True)
            cx, area, *_ = blocks_sorted[0]
            offset = cx - self.frame_width / 2.0
            self.last_target_area = area
            self.last_target_offset = offset
            # Track peak — when the block was largest in our view
            if area > self.max_target_area_seen:
                self.max_target_area_seen = area
                self.min_target_offset_at_max = offset
            self.get_logger().info(f'tracking: area={area:.0f} offset={offset:.0f}')

            speed = APPROACH_SPEED_NEAR if area > NEAR_AREA_THRESHOLD else APPROACH_SPEED_FAR
            angular = -ALIGN_GAIN * offset
            self._publish_cmd(speed, angular)
            return

        # Lost sight — was the block big and centred?
        if (self.max_target_area_seen > COMMIT_AREA_THRESHOLD
                and abs(self.min_target_offset_at_max) < COMMIT_OFFSET_THRESHOLD):
            self.get_logger().info('Block under camera — committing capture push.')
            self.capture_push_until = self.get_clock().now() + \
                rclpy.duration.Duration(seconds=CAPTURE_PUSH_DURATION)
            self.state = 'CAPTURE_PUSH'
            return

        self.get_logger().info(
            f'Lost target. last_area={self.last_target_area:.0f}, '
            f'last_offset={self.last_target_offset:.0f}.'
        )
        self.last_target_area = 0
        self.last_target_offset = 0
        self.max_target_area_seen = 0
        self.min_target_offset_at_max = 0
        self.state = 'SEARCHING'

    # ----- CAPTURE_PUSH: brief forward drive to seat the block in arms ------
    def _do_capture_push(self):
        now = self.get_clock().now()
        if now >= self.capture_push_until:
            self.capture_push_until = None
            self.target_goal_yaw = self._goal_yaw_for_colour(self.target_colour)
            self.get_logger().info(
                f'Block captured. Aligning toward goal yaw '
                f'{math.degrees(self.target_goal_yaw):.0f}°.'
            )
            self.state = 'ALIGN_TO_GOAL'
            return
        self._publish_cmd(APPROACH_SPEED_NEAR, 0.0)

    def _goal_yaw_for_colour(self, colour):
        # red side X-sign tells us which direction red lives
        red_x = self.red_side_x_sign
        if colour == 'red':
            return 0.0 if red_x > 0 else math.pi   # face +X or -X
        else:
            return math.pi if red_x > 0 else 0.0   # face the opposite

    # ----- ALIGN_TO_GOAL: spin clockwise (option b) until facing goal yaw ---
    def _do_align_to_goal(self):
        # angular error in [-pi, pi]
        err = (self.target_goal_yaw - self.robot_yaw + math.pi) % (2 * math.pi) - math.pi

        if abs(err) < ALIGN_YAW_TOLERANCE:
            self.get_logger().info('Aligned. Carrying block toward goal.')
            self.carry_start_time = self.get_clock().now()
            self.state = 'CARRY'
            return

        # Option b: always spin clockwise (negative yaw rate in REP-103 = right turn)
        self._publish_cmd(0.0, -ALIGN_ANGULAR_SPEED)

    # ----- CARRY: drive straight until past midline + buffer ----------------
    def _do_carry(self):
        # Determine target X based on colour
        red_x = self.red_side_x_sign
        if self.target_colour == 'red':
            target_x = red_x * CARRY_BUFFER
        else:
            target_x = -red_x * CARRY_BUFFER

        # Have we crossed?
        if (target_x > 0 and self.robot_x >= target_x) or \
           (target_x < 0 and self.robot_x <= target_x):
            self.get_logger().info(f'Reached deposit zone (x={self.robot_x:.2f}). Releasing.')
            self.release_start_time = self.get_clock().now()
            self.state = 'RELEASE'
            return

        # Safety: too close to a wall
        if self.front_dist < SAFE_FRONT_STOP:
            self.get_logger().warn('Wall too close while carrying. Releasing here.')
            self.release_start_time = self.get_clock().now()
            self.state = 'RELEASE'
            return

        # Timeout
        elapsed = (self.get_clock().now() - self.carry_start_time).nanoseconds * 1e-9
        if elapsed > CARRY_TIMEOUT:
            self.get_logger().warn('Carry timeout. Releasing.')
            self.release_start_time = self.get_clock().now()
            self.state = 'RELEASE'
            return

        self._publish_cmd(CARRY_SPEED, 0.0)

    # ----- RELEASE: reverse for a fixed time --------------------------------
    def _do_release(self):
        elapsed = (self.get_clock().now() - self.release_start_time).nanoseconds * 1e-9
        if elapsed > RELEASE_REVERSE_TIME:
            # Reset verify-spin accumulator and start verification
            self.verify_spin_accumulated = 0.0
            self.verify_spin_last_yaw = None
            self.verify_saw_misplaced = False
            self.get_logger().info('Block released. Verifying remaining misplaced blocks.')
            self.state = 'VERIFY_DONE'
            return
        self._publish_cmd(-RELEASE_REVERSE_SPEED, 0.0)

    # ----- VERIFY_DONE: full 360°; if any misplaced seen, resume work -------
    def _do_verify_done(self):
        # Did we see a misplaced block this frame?
        if self._pick_misplaced_block() is not None:
            self.verify_saw_misplaced = True

        accumulated = self._accumulate_yaw_change('verify_spin_last_yaw', 'verify_spin_accumulated')
        if accumulated >= 2 * math.pi * 1.05:
            if self.verify_saw_misplaced:
                self.get_logger().info('Misplaced block(s) still present. Resuming search.')
                self.state = 'SEARCHING'
            else:
                self.get_logger().info('No misplaced blocks remain. Task complete.')
                self.state = 'DONE'
            return

        self._publish_cmd(0.0, SEARCH_ANGULAR_SPEED)


def main(args=None):
    rclpy.init(args=args)
    node = BlockSorter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cmd_vel_pub.publish(Twist())  # stop
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()