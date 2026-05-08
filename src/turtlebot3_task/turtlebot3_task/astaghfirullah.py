#!/usr/bin/env python3
"""
Sort coloured blocks into their correct half of the arena by pushing.

Strategy:
  INIT_SPIN            - full 360°, record world bearings of all visible markers
  SEARCHING            - spin until a misplaced block is visible
  APPROACHING          - drive at the closest misplaced block
  CAPTURE_PUSH         - block disappeared under camera; drive forward briefly
  ALIGN_TO_MARKER      - spin until target marker is centred in frame
  RECOVERY_REPOSITION  - if marker not found, move to open space and retry
  CARRY_TO_MARKER      - drive forward, visual servoing on the marker
  RELEASE              - reverse to leave the block deposited
  RESURVEY             - re-record marker bearings after deposit
  FINAL_VERIFY         - after expected deposits reached, confirm no misplaced blocks
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
COMMIT_AREA_THRESHOLD   = 9500
COMMIT_OFFSET_THRESHOLD = 80
CAPTURE_PUSH_DURATION   = 1.2

ALIGN_ANGULAR_SPEED     = 0.3
ALIGN_PIXEL_TOLERANCE   = 50
ALIGN_TIMEOUT_REVS      = 1.2

CARRY_SPEED             = 0.10
CARRY_GAIN              = 0.005
CARRY_LIDAR_STOP        = 0.50
CARRY_TIMEOUT           = 35

RECOVERY_CLEARANCE      = 0.50
RECOVERY_DRIVE_MAX      = 1.0
RECOVERY_DRIVE_SPEED    = 0.10
RECOVERY_TURN_SPEED     = 0.4
RECOVERY_MAX_ATTEMPTS   = 2

RELEASE_REVERSE_SPEED   = 0.10
RELEASE_REVERSE_TIME    = 4.5

SEARCH_ANGULAR_SPEED    = 0.4
INIT_SPIN_ANGULAR_SPEED = 0.4
ALIGN_GAIN              = 0.005
SAFE_FRONT_STOP         = 0.18

EXPECTED_DEPOSITS       = 2

# Search watchdog
SEARCH_TIMEOUT_REVS     = 2.0
APPROACH_MIN_VALID_TIME = 1.0

# Survey behaviour
SURVEY_MIN_RADIANS      = math.radians(90.0)
SURVEY_TIMEOUT_REVS     = 1.5

# Final verify behaviour — after expected deposits, do one full 360° check.
# If a misplaced block is detected and confirmed by >0.8s in APPROACHING,
# resume normal flow; otherwise declare done.
FINAL_VERIFY_RADIANS              = 2 * math.pi
FINAL_VERIFY_APPROACH_THRESHOLD   = 0.8
# ----------------------------------------------------------------------------


class BlockSorter(Node):
    def __init__(self):
        super().__init__('block_sorter')

        self.declare_parameter('red_marker', 23)
        self.declare_parameter('blue_marker', 0)
        self.red_marker_id  = self.get_parameter('red_marker').value
        self.blue_marker_id = self.get_parameter('blue_marker').value

        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.image_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bridge = CvBridge()

        try:
            self.aruco_dict     = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
            self.aruco_params   = aruco.DetectorParameters()
            self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        except AttributeError:
            self.aruco_dict     = aruco.Dictionary_get(aruco.DICT_4X4_50)
            self.aruco_params   = aruco.DetectorParameters_create()
            self.aruco_detector = None

        self.state = 'INIT_SPIN'
        self.robot_x   = 0.0
        self.robot_y   = 0.0
        self.robot_yaw = 0.0
        self.front_dist = 10.0
        self.full_scan_ranges    = []
        self.full_scan_angle_min = 0.0
        self.full_scan_angle_inc = 0.0
        self.frame_width = 640

        self.red_blocks_pixel       = []
        self.blue_blocks_pixel      = []
        self.detected_marker_pixels = {}
        self.detected_marker_widths = {}
        self.marker_world_bearings  = {}

        self.red_side_x_sign = None
        self.sides_known     = False

        self.init_spin_accumulated = 0.0
        self.init_spin_last_yaw    = None

        self.target_colour            = None
        self.last_target_area         = 0
        self.last_target_offset       = 0
        self.max_target_area_seen     = 0
        self.min_target_offset_at_max = 0

        self.capture_push_until = None

        self.align_marker_id        = None
        self.align_spin_accumulated = 0.0
        self.align_spin_last_yaw    = None
        self.recovery_attempts_used = 0

        self.recovery_phase         = None
        self.recovery_drive_start_x = None
        self.recovery_drive_start_y = None

        self.carry_start_time   = None
        self.release_start_time = None

        self._search_slowdown_start = None

        self.deposits_made = 0

        # Search watchdog
        self.search_spin_accumulated = 0.0
        self.search_spin_last_yaw    = None
        self.approach_enter_time     = None
        self.approach_completed_recently = False

        # Resurvey
        self.resurvey_spin_accumulated = 0.0
        self.resurvey_spin_last_yaw    = None

        # Final verify
        self.final_verify_spin_accumulated = 0.0
        self.final_verify_spin_last_yaw    = None
        self.final_verify_approach_enter_time = None
        self.final_verify_real_approach_seen  = False

        self.timer = self.create_timer(0.1, self.decide)

        self.get_logger().info(
            f'Block sorter started. red_marker={self.red_marker_id}, '
            f'blue_marker={self.blue_marker_id}, expected_deposits={EXPECTED_DEPOSITS}')

    # ===== CALLBACKS ========================================================
    def odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def scan_cb(self, msg):
        front = list(msg.ranges[0:10]) + list(msg.ranges[-10:])
        valid = [r for r in front if 0.05 < r < 10.0]
        self.front_dist          = min(valid) if valid else 10.0
        self.full_scan_ranges    = list(msg.ranges)
        self.full_scan_angle_min = msg.angle_min
        self.full_scan_angle_inc = msg.angle_increment

    def image_cb(self, msg):
        frame_bright = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        self.frame_width = frame_bright.shape[1]
        hsv = cv2.cvtColor(frame_bright, cv2.COLOR_BGR2HSV)

        mask_blue = cv2.inRange(
            hsv, np.array([110, 100, 30]), np.array([135, 255, 255]))
        mask_red_low  = cv2.inRange(hsv, np.array([0,   60, 30]), np.array([15,  255, 255]))
        mask_red_high = cv2.inRange(hsv, np.array([155, 60, 30]), np.array([180, 255, 255]))
        mask_red = cv2.bitwise_or(mask_red_low, mask_red_high)

        self.red_blocks_pixel  = self._extract_blocks(mask_red)
        self.blue_blocks_pixel = self._extract_blocks(mask_blue)

        for cx, area, x, y, w, h in self.red_blocks_pixel:
            cv2.rectangle(frame_bright, (x, y), (x+w, y+h), (0, 0, 255), 2)
        for cx, area, x, y, w, h in self.blue_blocks_pixel:
            cv2.rectangle(frame_bright, (x, y), (x+w, y+h), (255, 0, 0), 2)

        if self.aruco_detector:
            corners, ids, _ = self.aruco_detector.detectMarkers(frame_bright)
        else:
            corners, ids, _ = aruco.detectMarkers(
                frame_bright, self.aruco_dict, parameters=self.aruco_params)

        self.detected_marker_pixels = {}
        self.detected_marker_widths = {}
        if ids is not None:
            aruco.drawDetectedMarkers(frame_bright, corners, ids)
            FOV_RAD = math.radians(60.0)
            for i, mid in enumerate(ids.flatten()):
                pts = corners[i].reshape(-1, 2)
                cx_marker = float(pts[:, 0].mean())
                width_marker = float(pts[:, 0].max() - pts[:, 0].min())
                self.detected_marker_pixels[int(mid)] = cx_marker
                self.detected_marker_widths[int(mid)] = width_marker

                # Record bearings during INIT_SPIN, RESURVEY, and FINAL_VERIFY
                if self.state in ('INIT_SPIN', 'RESURVEY', 'FINAL_VERIFY'):
                    cam_bearing = (cx_marker - self.frame_width / 2.0) / self.frame_width * FOV_RAD
                    self.marker_world_bearings[int(mid)] = self.robot_yaw - cam_bearing

        cv2.putText(frame_bright, f'STATE: {self.state}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame_bright, f'POS: ({self.robot_x:.2f}, {self.robot_y:.2f})',
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        cv2.putText(frame_bright, f'DEPOSITS: {self.deposits_made}/{EXPECTED_DEPOSITS}',
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        if self.sides_known and self.state != 'INIT_SPIN':
            for cx, area, *_ in self.red_blocks_pixel:
                bearing = self.block_world_bearing(cx)
                if self.is_correct_side('red', bearing):
                    self.get_logger().info(
                        f'RED correct: pixel_cx={cx}, bearing={math.degrees(bearing):.1f}°',
                        throttle_duration_sec=2.0)
                else:
                    self.get_logger().info(
                        f'RED MISPLACED: pixel_cx={cx}, bearing={math.degrees(bearing):.1f}°',
                        throttle_duration_sec=2.0)
                break
            for cx, area, *_ in self.blue_blocks_pixel:
                bearing = self.block_world_bearing(cx)
                if self.is_correct_side('blue', bearing):
                    self.get_logger().info(
                        f'BLUE correct: pixel_cx={cx}, bearing={math.degrees(bearing):.1f}°',
                        throttle_duration_sec=2.0)
                else:
                    self.get_logger().info(
                        f'BLUE MISPLACED: pixel_cx={cx}, bearing={math.degrees(bearing):.1f}°',
                        throttle_duration_sec=2.0)
                break

        cv2.imshow('Robot View', frame_bright)
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
                f'red_marker={self.red_marker_id} must be 23 or 0.')
            return
        self.sides_known = True
        self.get_logger().info(
            f'Sides resolved: red marker is id={self.red_marker_id}')

    def is_correct_side(self, colour, block_world_bearing):
        red_b  = self.marker_world_bearings.get(self.red_marker_id)
        blue_b = self.marker_world_bearings.get(self.blue_marker_id)
        if red_b is None or blue_b is None:
            return True

        def ang_diff(a, b):
            d = (a - b + math.pi) % (2 * math.pi) - math.pi
            return abs(d)

        d_red  = ang_diff(block_world_bearing, red_b)
        d_blue = ang_diff(block_world_bearing, blue_b)
        if colour == 'red':
            return d_red < d_blue
        return d_blue < d_red

    def block_world_bearing(self, pixel_cx):
        FOV_RAD = math.radians(60.0)
        cam_bearing = (pixel_cx - self.frame_width / 2.0) / self.frame_width * FOV_RAD
        return self.robot_yaw - cam_bearing

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
        msg.linear.x  = float(lin)
        msg.angular.z = float(ang)
        self.cmd_vel_pub.publish(msg)

    def _min_distance_any_direction(self):
        valid = [r for r in self.full_scan_ranges if 0.05 < r < 10.0]
        return min(valid) if valid else 10.0

    def _bearing_of_nearest_obstacle(self):
        if not self.full_scan_ranges:
            return 0.0
        best_idx, best_r = -1, 1e9
        for i, r in enumerate(self.full_scan_ranges):
            if 0.05 < r < best_r:
                best_r, best_idx = r, i
        if best_idx < 0:
            return 0.0
        return self.full_scan_angle_min + best_idx * self.full_scan_angle_inc

    def _both_markers_seen(self):
        return (self.red_marker_id  in self.marker_world_bearings
                and self.blue_marker_id in self.marker_world_bearings)

    # ===== STATE MACHINE ====================================================
    def decide(self):
        if   self.state == 'INIT_SPIN':            self._do_init_spin()
        elif self.state == 'SEARCHING':            self._do_searching()
        elif self.state == 'APPROACHING':          self._do_approaching()
        elif self.state == 'CAPTURE_PUSH':         self._do_capture_push()
        elif self.state == 'ALIGN_TO_MARKER':      self._do_align_to_marker()
        elif self.state == 'RECOVERY_REPOSITION':  self._do_recovery_reposition()
        elif self.state == 'CARRY_TO_MARKER':      self._do_carry_to_marker()
        elif self.state == 'RELEASE':              self._do_release()
        elif self.state == 'RESURVEY':             self._do_resurvey()
        elif self.state == 'FINAL_VERIFY':         self._do_final_verify()
        elif self.state == 'DONE':                 self._publish_cmd(0.0, 0.0)

    def _do_init_spin(self):
        self._resolve_sides_if_possible()
        if not self.sides_known:
            self.get_logger().error('Sides could not be resolved. Halting.')
            self.state = 'DONE'
            return

        accum = self._accumulate_yaw_change('init_spin_last_yaw', 'init_spin_accumulated')

        if accum >= SURVEY_MIN_RADIANS and self._both_markers_seen():
            self.get_logger().info(
                f'Init survey complete after {math.degrees(accum):.0f}°. Marker bearings: '
                f'red={math.degrees(self.marker_world_bearings[self.red_marker_id]):.1f}°, '
                f'blue={math.degrees(self.marker_world_bearings[self.blue_marker_id]):.1f}°')
            self.state = 'SEARCHING'
            return

        if accum >= 2 * math.pi * SURVEY_TIMEOUT_REVS:
            red_seen  = self.red_marker_id in self.marker_world_bearings
            blue_seen = self.blue_marker_id in self.marker_world_bearings
            self.get_logger().warn(
                f'Init spin timeout — red_seen={red_seen}, blue_seen={blue_seen}. '
                f'Resetting and trying again.')
            self.init_spin_accumulated = 0.0
            self.init_spin_last_yaw    = None
            return

        self._publish_cmd(0.0, INIT_SPIN_ANGULAR_SPEED)

    def _do_searching(self):
        accum = self._accumulate_yaw_change('search_spin_last_yaw', 'search_spin_accumulated')
        if accum >= 2 * math.pi * SEARCH_TIMEOUT_REVS and not self.approach_completed_recently:
            self.get_logger().info(
                f'Search watchdog: spun {SEARCH_TIMEOUT_REVS} revolutions without '
                f'a valid approach. Assuming done.')
            self.state = 'DONE'
            return

        for cx, area, *_ in self.red_blocks_pixel:
            bearing = self.block_world_bearing(cx)
            if not self.is_correct_side('red', bearing):
                self.get_logger().info('MISPLACED RED block detected',
                                       throttle_duration_sec=2.0)
                break
        for cx, area, *_ in self.blue_blocks_pixel:
            bearing = self.block_world_bearing(cx)
            if not self.is_correct_side('blue', bearing):
                self.get_logger().info('MISPLACED BLUE block detected',
                                       throttle_duration_sec=2.0)
                break

        target = self._pick_misplaced_block()
        if target is not None:
            if self._search_slowdown_start is None:
                self._search_slowdown_start = self.get_clock().now()
                self._publish_cmd(0.0, SEARCH_ANGULAR_SPEED * 0.3)
                return
            elapsed = (self.get_clock().now() - self._search_slowdown_start).nanoseconds * 1e-9
            if elapsed < 1.5:
                self._publish_cmd(0.0, SEARCH_ANGULAR_SPEED * 0.3)
                return
            self._search_slowdown_start = None
            self.target_colour            = target[0]
            self.last_target_area         = target[2]
            self.last_target_offset       = target[1] - self.frame_width / 2.0
            self.max_target_area_seen     = target[2]
            self.min_target_offset_at_max = target[1] - self.frame_width / 2.0
            self.approach_enter_time      = self.get_clock().now()
            self.approach_completed_recently = False
            self.get_logger().info(f'Target acquired: misplaced {self.target_colour} block.')
            self.state = 'APPROACHING'
            return

        self._search_slowdown_start = None
        self._publish_cmd(0.0, SEARCH_ANGULAR_SPEED)

    def _pick_misplaced_block(self):
        candidates = []
        for cx, area, *_ in self.red_blocks_pixel:
            bearing = self.block_world_bearing(cx)
            if not self.is_correct_side('red', bearing):
                candidates.append(('red', cx, area))
        for cx, area, *_ in self.blue_blocks_pixel:
            bearing = self.block_world_bearing(cx)
            if not self.is_correct_side('blue', bearing):
                candidates.append(('blue', cx, area))
        if not candidates:
            return None
        candidates.sort(key=lambda c: c[2], reverse=True)
        return candidates[0]

    def _do_approaching(self):
        if self.approach_enter_time is not None:
            elapsed_in_approach = (self.get_clock().now() - self.approach_enter_time).nanoseconds * 1e-9
            if elapsed_in_approach > APPROACH_MIN_VALID_TIME:
                self.approach_completed_recently = True
            # Also feed the FINAL_VERIFY signal: if this approach was kicked off
            # from FINAL_VERIFY and reaches threshold, mark it as a real approach
            if (self.final_verify_approach_enter_time is not None
                    and (self.get_clock().now() - self.final_verify_approach_enter_time).nanoseconds * 1e-9
                    > FINAL_VERIFY_APPROACH_THRESHOLD):
                self.final_verify_real_approach_seen = True

        blocks = self.red_blocks_pixel if self.target_colour == 'red' else self.blue_blocks_pixel

        if blocks:
            blocks_sorted = sorted(blocks, key=lambda b: b[1], reverse=True)
            cx, area, *_ = blocks_sorted[0]
            offset = cx - self.frame_width / 2.0
            self.last_target_area  = area
            self.last_target_offset = offset
            if area > self.max_target_area_seen:
                self.max_target_area_seen     = area
                self.min_target_offset_at_max = offset

            self.get_logger().info(f'tracking: area={area:.0f} offset={offset:.0f}')

            speed      = APPROACH_SPEED_NEAR if area > NEAR_AREA_THRESHOLD else APPROACH_SPEED_FAR
            gain_scale = 1.0 if area < NEAR_AREA_THRESHOLD else 0.5
            steering_offset = offset if abs(offset) >= 30 else 0
            angular    = -ALIGN_GAIN * steering_offset * gain_scale
            self._publish_cmd(speed, angular)
            return

        if (self.max_target_area_seen > COMMIT_AREA_THRESHOLD
                and abs(self.min_target_offset_at_max) < COMMIT_OFFSET_THRESHOLD):
            self.get_logger().info('Block under camera — committing capture push.')
            self.capture_push_until = (self.get_clock().now()
                                       + rclpy.duration.Duration(seconds=CAPTURE_PUSH_DURATION))
            self.state = 'CAPTURE_PUSH'
            return

        self.get_logger().info(
            f'Lost target. last_area={self.last_target_area:.0f}, '
            f'last_offset={self.last_target_offset:.0f}. Returning to search.')
        self.last_target_area         = 0
        self.last_target_offset       = 0
        self.max_target_area_seen     = 0
        self.min_target_offset_at_max = 0
        self.approach_enter_time      = None
        self.final_verify_approach_enter_time = None
        self.state = 'SEARCHING'

    def _do_capture_push(self):
        if self.get_clock().now() >= self.capture_push_until:
            self.capture_push_until     = None
            self.align_marker_id        = (self.red_marker_id
                                           if self.target_colour == 'red'
                                           else self.blue_marker_id)
            self.align_spin_accumulated = 0.0
            self.align_spin_last_yaw    = None
            self.recovery_attempts_used = 0
            self.get_logger().info(
                f'Block captured. Aligning to marker id {self.align_marker_id}.')
            self.state = 'ALIGN_TO_MARKER'
            return
        self._publish_cmd(APPROACH_SPEED_NEAR, 0.0)

    def _do_align_to_marker(self):
        if self.align_marker_id in self.detected_marker_pixels:
            marker_cx = self.detected_marker_pixels[self.align_marker_id]
            offset    = marker_cx - self.frame_width / 2.0
            if abs(offset) < ALIGN_PIXEL_TOLERANCE:
                self.get_logger().info(f'Marker {self.align_marker_id} centred. Carrying.')
                self.carry_start_time = self.get_clock().now()
                self.state = 'CARRY_TO_MARKER'
                return
            sign = -1.0 if offset > 0 else +1.0
            self._publish_cmd(0.0, sign * ALIGN_ANGULAR_SPEED)
            return

        accum = self._accumulate_yaw_change('align_spin_last_yaw', 'align_spin_accumulated')
        if accum >= 2 * math.pi * ALIGN_TIMEOUT_REVS:
            if self.recovery_attempts_used >= RECOVERY_MAX_ATTEMPTS:
                self.get_logger().warn(
                    f'Marker {self.align_marker_id} not found after '
                    f'{self.recovery_attempts_used} recovery attempts. Releasing.')
                self.release_start_time = self.get_clock().now()
                self.state = 'RELEASE'
                return
            self.get_logger().warn(
                f'Marker {self.align_marker_id} not found. Recovery attempt '
                f'{self.recovery_attempts_used + 1}/{RECOVERY_MAX_ATTEMPTS}.')
            self.recovery_attempts_used += 1
            self.recovery_phase          = 'turn_away'
            self.state = 'RECOVERY_REPOSITION'
            return

        self._publish_cmd(0.0, -ALIGN_ANGULAR_SPEED)

    def _do_recovery_reposition(self):
        if self.recovery_phase == 'turn_away':
            nearest_bearing = self._bearing_of_nearest_obstacle()
            err = (math.pi - nearest_bearing + math.pi) % (2 * math.pi) - math.pi
            if abs(err) < 0.10:
                self.recovery_phase         = 'drive_forward'
                self.recovery_drive_start_x = self.robot_x
                self.recovery_drive_start_y = self.robot_y
                self.get_logger().info('Recovery: facing away from wall, driving forward.')
                return
            self._publish_cmd(0.0, RECOVERY_TURN_SPEED if err > 0 else -RECOVERY_TURN_SPEED)
            return

        if self.recovery_phase == 'drive_forward':
            travelled = math.hypot(self.robot_x - self.recovery_drive_start_x,
                                   self.robot_y - self.recovery_drive_start_y)
            if (self._min_distance_any_direction() > RECOVERY_CLEARANCE
                    or travelled > RECOVERY_DRIVE_MAX):
                self.get_logger().info(
                    f'Recovery: repositioned ({travelled:.2f}m). Re-aligning.')
                self.recovery_phase         = None
                self.align_spin_accumulated = 0.0
                self.align_spin_last_yaw    = None
                self.state = 'ALIGN_TO_MARKER'
                return
            self._publish_cmd(RECOVERY_DRIVE_SPEED, 0.0)
            return

    def _do_carry_to_marker(self):
        self.get_logger().info(f'carry: front_dist={self.front_dist:.2f}m')
        if self.front_dist < CARRY_LIDAR_STOP:
            self.get_logger().info(
                f'Carry: wall reached at {self.front_dist:.2f}m. Releasing.')
            self.release_start_time = self.get_clock().now()
            self.state = 'RELEASE'
            return

        elapsed = (self.get_clock().now() - self.carry_start_time).nanoseconds * 1e-9
        if elapsed > CARRY_TIMEOUT:
            self.get_logger().warn('Carry timeout. Releasing.')
            self.release_start_time = self.get_clock().now()
            self.state = 'RELEASE'
            return

        angular = 0.0
        if self.align_marker_id in self.detected_marker_pixels:
            marker_cx = self.detected_marker_pixels[self.align_marker_id]
            offset    = marker_cx - self.frame_width / 2.0
            angular   = -CARRY_GAIN * offset
        self._publish_cmd(CARRY_SPEED, angular)

    def _do_release(self):
        elapsed = (self.get_clock().now() - self.release_start_time).nanoseconds * 1e-9
        if elapsed > RELEASE_REVERSE_TIME:
            self.deposits_made += 1
            self.get_logger().info(
                f'Block released. Deposits: {self.deposits_made}/{EXPECTED_DEPOSITS}')

            # Reset watchdog state for next cycle
            self.search_spin_accumulated = 0.0
            self.search_spin_last_yaw    = None
            self.approach_enter_time     = None
            self.approach_completed_recently = False

            # Clear bearings so re-survey records fresh ones
            self.marker_world_bearings   = {}

            if self.deposits_made >= EXPECTED_DEPOSITS:
                # Expected deposits hit — go to FINAL_VERIFY for a confirming spin
                self.final_verify_spin_accumulated = 0.0
                self.final_verify_spin_last_yaw    = None
                self.final_verify_approach_enter_time = None
                self.final_verify_real_approach_seen  = False
                self.get_logger().info(
                    f'Expected deposits reached. Entering FINAL_VERIFY.')
                self.state = 'FINAL_VERIFY'
                return

            # Else go to RESURVEY (intermediate deposit)
            self.resurvey_spin_accumulated = 0.0
            self.resurvey_spin_last_yaw    = None
            self.state = 'RESURVEY'
            return
        self._publish_cmd(-RELEASE_REVERSE_SPEED, 0.0)

    def _do_resurvey(self):
        accum = self._accumulate_yaw_change('resurvey_spin_last_yaw', 'resurvey_spin_accumulated')

        if accum >= SURVEY_MIN_RADIANS and self._both_markers_seen():
            self.get_logger().info(
                f'Resurvey complete after {math.degrees(accum):.0f}°. Marker bearings: '
                f'red={math.degrees(self.marker_world_bearings[self.red_marker_id]):.1f}°, '
                f'blue={math.degrees(self.marker_world_bearings[self.blue_marker_id]):.1f}°')
            self.state = 'SEARCHING'
            return

        if accum >= 2 * math.pi * SURVEY_TIMEOUT_REVS:
            red_seen  = self.red_marker_id  in self.marker_world_bearings
            blue_seen = self.blue_marker_id in self.marker_world_bearings
            self.get_logger().warn(
                f'Resurvey timeout — red_seen={red_seen}, blue_seen={blue_seen}. '
                f'Proceeding to search with whatever we have.')
            self.state = 'SEARCHING'
            return

        self._publish_cmd(0.0, INIT_SPIN_ANGULAR_SPEED)

    def _do_final_verify(self):
        """After expected deposits, do one full 360° spin to confirm no
        misplaced blocks remain. If a misplaced block is detected and the
        robot stays in APPROACHING for >0.8s (i.e. it's a real target,
        not a transient false positive), continue normal flow."""
        accum = self._accumulate_yaw_change('final_verify_spin_last_yaw',
                                             'final_verify_spin_accumulated')

        # Check for misplaced blocks; if found, transition to APPROACHING
        target = self._pick_misplaced_block()
        if target is not None:
            self.get_logger().info(
                f'FINAL_VERIFY: found possibly misplaced {target[0]} block. '
                f'Transitioning to approach to confirm.')
            self.target_colour            = target[0]
            self.last_target_area         = target[2]
            self.last_target_offset       = target[1] - self.frame_width / 2.0
            self.max_target_area_seen     = target[2]
            self.min_target_offset_at_max = target[1] - self.frame_width / 2.0
            self.approach_enter_time      = self.get_clock().now()
            self.approach_completed_recently = False
            self.final_verify_approach_enter_time = self.get_clock().now()
            # Note: we don't reset final_verify_real_approach_seen here — that
            # gets set inside _do_approaching once threshold time is reached
            self.state = 'APPROACHING'
            return

        # If we've completed the full verify spin without finding a real target, done
        if accum >= FINAL_VERIFY_RADIANS:
            self.get_logger().info(
                f'FINAL_VERIFY complete after {math.degrees(accum):.0f}°. '
                f'No misplaced blocks confirmed. Done.')
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
    finally:
        stop = Twist()
        for _ in range(10):
            node.cmd_vel_pub.publish(stop)
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()