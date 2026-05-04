#!/usr/bin/env python3
"""
--------------
Sort coloured blocks into their correct half of the arena by pushing.

Strategy:
  INIT_SPIN            - full 360°, resolve sides from marker IDs
  SEARCHING            - spin until a misplaced block is visible
  APPROACHING          - drive at the closest misplaced block
  CAPTURE_PUSH         - block disappeared under camera; drive forward briefly
  ALIGN_TO_MARKER      - spin until target marker is centred in frame
  RECOVERY_REPOSITION  - if marker not found, move to open space and retry
  CARRY_TO_MARKER      - drive forward, visual servoing on the marker
  RELEASE              - reverse 0.30m to leave the block deposited
  VERIFY_DONE          - full 360° check; resume or finish
  DONE
"""
import math
import rclpy
import rclpy.duration
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, LaserScan, Image
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
NEAR_AREA_THRESHOLD     = 4000
COMMIT_AREA_THRESHOLD   = 8000
COMMIT_OFFSET_THRESHOLD = 80
CAPTURE_PUSH_DURATION   = 1.2

ALIGN_ANGULAR_SPEED     = 0.4       # marker align spin speed (clockwise)
ALIGN_PIXEL_TOLERANCE   = 50        # marker centred if within this many px of frame middle
ALIGN_TIMEOUT_REVS      = 1.2       # spins before giving up (×360°)

CARRY_SPEED             = 0.10
CARRY_GAIN              = 0.005     # angular correction per pixel offset on marker
CARRY_LIDAR_STOP        = 0.50      # stop carry when wall this close
CARRY_TIMEOUT           = 35

RECOVERY_CLEARANCE      = 0.50      # drive forward in recovery until 0.5m from any wall
RECOVERY_DRIVE_MAX      = 1.0       # safety cap on recovery drive distance
RECOVERY_DRIVE_SPEED    = 0.10
RECOVERY_TURN_SPEED     = 0.4
RECOVERY_MAX_ATTEMPTS   = 2

RELEASE_REVERSE_SPEED   = 0.10
RELEASE_REVERSE_TIME    = 4.5       # ~0.30m

SEARCH_ANGULAR_SPEED    = 0.4
INIT_SPIN_ANGULAR_SPEED = 0.4
ALIGN_GAIN              = 0.005     # block-approach steering
SAFE_FRONT_STOP         = 0.18      # legacy carry safety (still used as fallback)
# ----------------------------------------------------------------------------


class BlockSorter(Node):
    def __init__(self):
        super().__init__('block_sorter')

        # Parameters
        self.declare_parameter('red_marker', 23)
        self.declare_parameter('blue_marker', 0)
        self.red_marker_id = self.get_parameter('red_marker').value
        self.blue_marker_id = self.get_parameter('blue_marker').value

        # Subs / pubs
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.image_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bridge = CvBridge()

        # ArUco
        try:
            self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
            self.aruco_params = aruco.DetectorParameters()
            self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        except AttributeError:
            self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
            self.aruco_params = aruco.DetectorParameters_create()
            self.aruco_detector = None

        # State
        self.state = 'INIT_SPIN'
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.front_dist = 10.0
        self.full_scan_ranges = []  # full LaserScan ranges, used for "nearest wall" calc
        self.full_scan_angle_min = 0.0
        self.full_scan_angle_inc = 0.0
        self.frame_width = 640

        self.red_blocks_pixel = []
        self.blue_blocks_pixel = []
        self.detected_marker_pixels = {}   # id -> centre x in image
        self.detected_marker_widths = {}   # id -> width in pixels (proxy for distance)

        # Side resolution
        self.red_side_x_sign = None
        self.sides_known = False

        # INIT_SPIN
        self.init_spin_accumulated = 0.0
        self.init_spin_last_yaw = None

        # Approaching
        self.target_colour = None
        self.last_target_area = 0
        self.last_target_offset = 0
        self.max_target_area_seen = 0
        self.min_target_offset_at_max = 0

        # Capture push
        self.capture_push_until = None

        # Align to marker
        self.align_marker_id = None
        self.align_spin_accumulated = 0.0
        self.align_spin_last_yaw = None
        self.recovery_attempts_used = 0

        # Recovery reposition
        self.recovery_phase = None        # 'turn_away' | 'drive_forward' | None
        self.recovery_drive_start_x = None
        self.recovery_drive_start_y = None

        # Carry
        self.carry_start_time = None

        # Release
        self.release_start_time = None

        # Verify done
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

        self.full_scan_ranges = list(msg.ranges)
        self.full_scan_angle_min = msg.angle_min
        self.full_scan_angle_inc = msg.angle_increment

    def image_cb(self, msg):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
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

        self.detected_marker_pixels = {}
        self.detected_marker_widths = {}
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            for i, mid in enumerate(ids.flatten()):
                pts = corners[i].reshape(-1, 2)
                cx_marker = float(pts[:, 0].mean())
                width_marker = float(pts[:, 0].max() - pts[:, 0].min())
                self.detected_marker_pixels[int(mid)] = cx_marker
                self.detected_marker_widths[int(mid)] = width_marker

        # HUD
        cv2.putText(frame, f'STATE: {self.state}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f'POS: ({self.robot_x:.2f}, {self.robot_y:.2f})',
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        if self.sides_known:
            cv2.putText(frame, f'RED side: X{">"if self.red_side_x_sign>0 else "<"} 0',
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
                f'red_marker={self.red_marker_id} cannot define a side; must be 23 or 0.'
            )
            return
        self.sides_known = True
        self.get_logger().info(
            f'Sides resolved: red is X{"+" if self.red_side_x_sign>0 else "-"}'
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
        last = getattr(self, last_yaw_attr)
        if last is None:
            setattr(self, last_yaw_attr, self.robot_yaw)
            return getattr(self, accum_attr)
        d = self.robot_yaw - last
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

    def _min_distance_any_direction(self):
        """Return the smallest valid range across the full 360° LiDAR scan."""
        valid = [r for r in self.full_scan_ranges if 0.05 < r < 10.0]
        return min(valid) if valid else 10.0

    def _bearing_of_nearest_obstacle(self):
        """Return bearing (radians, in robot frame) of the closest LiDAR return."""
        if not self.full_scan_ranges:
            return 0.0
        best_idx = -1
        best_r = 1e9
        for i, r in enumerate(self.full_scan_ranges):
            if 0.05 < r < best_r:
                best_r = r
                best_idx = i
        if best_idx < 0:
            return 0.0
        return self.full_scan_angle_min + best_idx * self.full_scan_angle_inc

    # ===== STATE MACHINE ====================================================
    def decide(self):
        if self.state == 'INIT_SPIN':              self._do_init_spin()
        elif self.state == 'SEARCHING':            self._do_searching()
        elif self.state == 'APPROACHING':          self._do_approaching()
        elif self.state == 'CAPTURE_PUSH':         self._do_capture_push()
        elif self.state == 'ALIGN_TO_MARKER':      self._do_align_to_marker()
        elif self.state == 'RECOVERY_REPOSITION': self._do_recovery_reposition()
        elif self.state == 'CARRY_TO_MARKER':      self._do_carry_to_marker()
        elif self.state == 'RELEASE':              self._do_release()
        elif self.state == 'VERIFY_DONE':          self._do_verify_done()
        elif self.state == 'DONE':                 self._publish_cmd(0.0, 0.0)

    # ----- INIT_SPIN --------------------------------------------------------
    def _do_init_spin(self):
        self._resolve_sides_if_possible()
        accum = self._accumulate_yaw_change('init_spin_last_yaw', 'init_spin_accumulated')
        if accum >= 2 * math.pi * 1.05:
            if self.sides_known:
                self.get_logger().info('Initial survey complete. Searching.')
                self.state = 'SEARCHING'
            else:
                self.get_logger().error('Sides could not be resolved. Halting.')
                self.state = 'DONE'
            return
        self._publish_cmd(0.0, INIT_SPIN_ANGULAR_SPEED)

    # ----- SEARCHING --------------------------------------------------------
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
            if area > self.max_target_area_seen:
                self.max_target_area_seen = area
                self.min_target_offset_at_max = offset

            self.get_logger().info(f'tracking: area={area:.0f} offset={offset:.0f}')

            speed = APPROACH_SPEED_NEAR if area > NEAR_AREA_THRESHOLD else APPROACH_SPEED_FAR
            angular = -ALIGN_GAIN * offset
            self._publish_cmd(speed, angular)
            return

        if (self.max_target_area_seen > COMMIT_AREA_THRESHOLD
                and abs(self.min_target_offset_at_max) < COMMIT_OFFSET_THRESHOLD):
            self.get_logger().info('Block under camera — committing capture push.')
            self.capture_push_until = self.get_clock().now() + \
                rclpy.duration.Duration(seconds=CAPTURE_PUSH_DURATION)
            self.state = 'CAPTURE_PUSH'
            return

        self.get_logger().info(
            f'Lost target. last_area={self.last_target_area:.0f}, '
            f'last_offset={self.last_target_offset:.0f}. Returning to search.'
        )
        self.last_target_area = 0
        self.last_target_offset = 0
        self.max_target_area_seen = 0
        self.min_target_offset_at_max = 0
        self.state = 'SEARCHING'

    # ----- CAPTURE_PUSH -----------------------------------------------------
    def _do_capture_push(self):
        if self.get_clock().now() >= self.capture_push_until:
            self.capture_push_until = None
            # decide which marker to head toward
            self.align_marker_id = (
                self.red_marker_id if self.target_colour == 'red' else self.blue_marker_id
            )
            self.align_spin_accumulated = 0.0
            self.align_spin_last_yaw = None
            self.recovery_attempts_used = 0
            self.get_logger().info(
                f'Block captured. Aligning to marker id {self.align_marker_id}.'
            )
            self.state = 'ALIGN_TO_MARKER'
            return
        self._publish_cmd(APPROACH_SPEED_NEAR, 0.0)

    # ----- ALIGN_TO_MARKER ----------------------------------  ----------------
    def _do_align_to_marker(self):
        # Is the target marker visible?
        if self.align_marker_id in self.detected_marker_pixels:
            marker_cx = self.detected_marker_pixels[self.align_marker_id]
            offset = marker_cx - self.frame_width / 2.0
            if abs(offset) < ALIGN_PIXEL_TOLERANCE:
                self.get_logger().info(f'Marker {self.align_marker_id} centred. Carrying.')
                self.carry_start_time = self.get_clock().now()
                self.state = 'CARRY_TO_MARKER'
                return
            # Steer toward marker. If marker is right of centre (+offset), spin clockwise (-z).
            sign = -1.0 if offset > 0 else +1.0
            self._publish_cmd(0.0, sign * ALIGN_ANGULAR_SPEED)
            return

        # Marker not visible — keep spinning clockwise, accumulate
        accum = self._accumulate_yaw_change('align_spin_last_yaw', 'align_spin_accumulated')
        if accum >= 2 * math.pi * ALIGN_TIMEOUT_REVS:
            if self.recovery_attempts_used >= RECOVERY_MAX_ATTEMPTS:
                self.get_logger().warn(
                    f'Marker {self.align_marker_id} not found after {self.recovery_attempts_used} '
                    f'recovery attempts. Releasing block here.'
                )
                self.release_start_time = self.get_clock().now()
                self.state = 'RELEASE'
                return
            self.get_logger().warn(
                f'Marker {self.align_marker_id} not found. Recovery attempt '
                f'{self.recovery_attempts_used + 1}/{RECOVERY_MAX_ATTEMPTS}.'
            )
            self.recovery_attempts_used += 1
            self.recovery_phase = 'turn_away'
            self.state = 'RECOVERY_REPOSITION'
            return

        self._publish_cmd(0.0, -ALIGN_ANGULAR_SPEED)

    # ----- RECOVERY_REPOSITION ---------------------------------------------
    def _do_recovery_reposition(self):
        # Phase 1: turn so back of robot faces nearest wall (i.e. front faces away).
        if self.recovery_phase == 'turn_away':
            nearest_bearing = self._bearing_of_nearest_obstacle()
            # We want robot's front to face away from that bearing.
            # "Face away" yaw error in robot frame is: pi - nearest_bearing (wrapped)
            err = (math.pi - nearest_bearing + math.pi) % (2 * math.pi) - math.pi
            if abs(err) < 0.10:
                self.recovery_phase = 'drive_forward'
                self.recovery_drive_start_x = self.robot_x
                self.recovery_drive_start_y = self.robot_y
                self.get_logger().info('Recovery: facing away from nearest wall, driving forward.')
                return
            # Spin in the direction that reduces err
            self._publish_cmd(0.0, RECOVERY_TURN_SPEED if err > 0 else -RECOVERY_TURN_SPEED)
            return

        # Phase 2: drive forward until far enough from any wall, or until distance cap.
        if self.recovery_phase == 'drive_forward':
            travelled = math.hypot(
                self.robot_x - self.recovery_drive_start_x,
                self.robot_y - self.recovery_drive_start_y,
            )
            if (self._min_distance_any_direction() > RECOVERY_CLEARANCE
                    or travelled > RECOVERY_DRIVE_MAX):
                self.get_logger().info(
                    f'Recovery: repositioned (travelled {travelled:.2f}m). Re-aligning.'
                )
                self.recovery_phase = None
                # reset align-spin accumulator for the next attempt
                self.align_spin_accumulated = 0.0
                self.align_spin_last_yaw = None
                self.state = 'ALIGN_TO_MARKER'
                return
            self._publish_cmd(RECOVERY_DRIVE_SPEED, 0.0)
            return

    # ----- CARRY_TO_MARKER --------------------------------------------------
    def _do_carry_to_marker(self):
        self.get_logger().info(f'carry: front_dist={self.front_dist:.2f}m')
        # Stop conditions
        if self.front_dist < CARRY_LIDAR_STOP:
            self.get_logger().info(f'Carry: wall reached at {self.front_dist:.2f}m. Releasing.')
            self.release_start_time = self.get_clock().now()
            self.state = 'RELEASE'
            return

        elapsed = (self.get_clock().now() - self.carry_start_time).nanoseconds * 1e-9
        if elapsed > CARRY_TIMEOUT:
            self.get_logger().warn('Carry timeout. Releasing.')
            self.release_start_time = self.get_clock().now()
            self.state = 'RELEASE'
            return

        # Visual servoing on the marker if visible
        angular = 0.0
        if self.align_marker_id in self.detected_marker_pixels:
            marker_cx = self.detected_marker_pixels[self.align_marker_id]
            offset = marker_cx - self.frame_width / 2.0
            angular = -CARRY_GAIN * offset

        self._publish_cmd(CARRY_SPEED, angular)

    # ----- RELEASE ----------------------------------------------------------
    def _do_release(self):
        elapsed = (self.get_clock().now() - self.release_start_time).nanoseconds * 1e-9
        if elapsed > RELEASE_REVERSE_TIME:
            self.verify_spin_accumulated = 0.0
            self.verify_spin_last_yaw = None
            self.verify_saw_misplaced = False
            self.get_logger().info('Block released. Verifying.')
            self.state = 'VERIFY_DONE'
            return
        self._publish_cmd(-RELEASE_REVERSE_SPEED, 0.0)

    # ----- VERIFY_DONE ------------------------------------------------------
    def _do_verify_done(self):
        if self._pick_misplaced_block() is not None:
            self.verify_saw_misplaced = True
        accum = self._accumulate_yaw_change('verify_spin_last_yaw', 'verify_spin_accumulated')
        if accum >= 2 * math.pi * 1.05:
            if self.verify_saw_misplaced:
                self.get_logger().info('Misplaced block(s) remain. Resuming.')
                self.state = 'SEARCHING'
            else:
                self.get_logger().info('All blocks correctly placed. Done.')
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
    node.cmd_vel_pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()