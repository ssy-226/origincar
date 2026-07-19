#!/usr/bin/env python3
import time

import rclpy
from ai_msgs.msg import PerceptionTargets
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Int32


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


class ControlYolo(Node):
    """Convert monocular YOLO cone detections into stable avoidance commands."""

    TRACKED_TYPES = {'p', 'qrcode', 'zt'}

    def __init__(self):
        super().__init__('avoid')

        self.declare_parameter('y_p', 435)
        self.declare_parameter('require_start_signal', True)
        self.declare_parameter('start_topic', '/start')
        self.declare_parameter('y_qrcode', 167)
        self.declare_parameter('y_zt', 155)
        self.declare_parameter('v_avoid', 0.8)
        self.declare_parameter('v_avoid_min', 0.45)
        self.declare_parameter('kp_avoid', 0.0035)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('detection_confidence', 0.8)
        self.declare_parameter('detection_min_y', 119)
        self.declare_parameter('avoid_filter_alpha', 0.7)
        self.declare_parameter('avoid_direction_lock', 0.35)
        self.declare_parameter('avoid_release_timeout', 0.25)
        self.declare_parameter('max_angular', 5.0)
        self.declare_parameter('angular_slew_rate', 16.0)
        self.declare_parameter('turn_slowdown', 0.25)
        self.declare_parameter('p_confirm_frames', 3)
        self.declare_parameter('p_stop_after_qrcode', True)

        self.y_p = int(self.get_parameter('y_p').value)
        self.require_start_signal = bool(
            self.get_parameter('require_start_signal').value
        )
        self.start_topic = self.get_parameter('start_topic').value
        self.y_qrcode = int(self.get_parameter('y_qrcode').value)
        self.y_zt = int(self.get_parameter('y_zt').value)
        self.v_avoid = float(self.get_parameter('v_avoid').value)
        self.v_avoid_min = float(self.get_parameter('v_avoid_min').value)
        self.kp_avoid = float(self.get_parameter('kp_avoid').value)
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.detection_confidence = float(
            self.get_parameter('detection_confidence').value
        )
        self.detection_min_y = int(
            self.get_parameter('detection_min_y').value
        )
        self.avoid_filter_alpha = clamp(
            float(self.get_parameter('avoid_filter_alpha').value),
            0.0,
            1.0,
        )
        self.avoid_direction_lock = max(
            0.0,
            float(self.get_parameter('avoid_direction_lock').value),
        )
        self.avoid_release_timeout = max(
            0.0,
            float(self.get_parameter('avoid_release_timeout').value),
        )
        self.max_angular = max(
            0.1,
            float(self.get_parameter('max_angular').value),
        )
        self.angular_slew_rate = max(
            0.0,
            float(self.get_parameter('angular_slew_rate').value),
        )
        self.turn_slowdown = clamp(
            float(self.get_parameter('turn_slowdown').value),
            0.0,
            0.9,
        )
        self.p_confirm_frames = max(
            1,
            int(self.get_parameter('p_confirm_frames').value),
        )
        self.p_stop_after_qrcode = bool(
            self.get_parameter('p_stop_after_qrcode').value
        )

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.create_subscription(
            PerceptionTargets,
            '/detect',
            self.obstacle_callback,
            qos_profile,
        )
        self.start_subscription = self.create_subscription(
            Int32,
            self.start_topic,
            self.start_callback,
            10,
        )
        self.qrcode_subscription = self.create_subscription(
            Int32,
            '/qrcode_number',
            self.qrcode_callback,
            10,
        )

        self.pub_p = self.create_publisher(Int32, '/p', 10)
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel_yolo',
            qos_profile,
        )

        self.qrcode_seen = False
        self.started = not self.require_start_signal
        self.p_confirm_count = 0
        self.p_signal_sent = False

        self.avoid_direction = 0
        self.direction_lock_until = 0.0
        self.filtered_error = 0.0
        self.last_obstacle_time = float('-inf')
        self.last_control_time = None
        self.last_angular = 0.0
        self.last_debug_log = float('-inf')

        self.get_logger().info(
            'YOLO避障节点启动: '
            f'v={self.v_avoid:.2f}, v_min={self.v_avoid_min:.2f}, '
            f'kp={self.kp_avoid:.4f}, direction_lock='
            f'{self.avoid_direction_lock:.2f}s'
        )

    def start_callback(self, msg: Int32):
        if msg.data != 1 or self.started:
            return

        self.started = True
        self.reset_avoidance_state()
        self.get_logger().info('收到发车信号，启用锥桶和P点控制')

    def qrcode_callback(self, msg: Int32):
        if not self.started:
            return
        if msg.data in (3, 4):
            self.qrcode_seen = True

    def obstacle_callback(self, msg: PerceptionTargets):
        if not self.started:
            return
        try:
            detections = self.select_largest_detections(msg)
            self.process_p_stop(detections.get('p'))
            self.process_obstacle(
                cone_roi=detections.get('zt'),
                p_roi=detections.get('p'),
                qrcode_roi=detections.get('qrcode'),
            )
        except Exception as exc:
            self.get_logger().error(f'处理YOLO数据异常: {exc}')

    def select_largest_detections(self, msg: PerceptionTargets):
        best = {}
        best_area = {target_type: 0 for target_type in self.TRACKED_TYPES}

        for target in msg.targets:
            if target.type not in self.TRACKED_TYPES:
                continue

            for roi in target.rois:
                if roi.confidence < self.detection_confidence:
                    continue

                bottom_y = roi.rect.y_offset + roi.rect.height
                if not self.detection_min_y <= bottom_y < self.image_height:
                    continue

                area = roi.rect.width * roi.rect.height
                if area > best_area[target.type]:
                    best_area[target.type] = area
                    best[target.type] = roi

        return best

    def process_p_stop(self, p_roi):
        stop_allowed = not self.p_stop_after_qrcode or self.qrcode_seen
        close_enough = (
            p_roi is not None
            and p_roi.rect.y_offset + p_roi.rect.height >= self.y_p
        )

        if stop_allowed and close_enough:
            self.p_confirm_count += 1
        else:
            self.p_confirm_count = 0

        if (
            self.p_confirm_count >= self.p_confirm_frames
            and not self.p_signal_sent
        ):
            signal_msg = Int32()
            signal_msg.data = 1
            self.pub_p.publish(signal_msg)
            self.p_signal_sent = True
            bottom_y = p_roi.rect.y_offset + p_roi.rect.height
            self.get_logger().warn(
                f'P点连续确认{self.p_confirm_count}帧，发布停车信号 '
                f'(y={bottom_y})'
            )

    def process_obstacle(self, cone_roi, p_roi, qrcode_roi):
        now = time.monotonic()

        if cone_roi is None:
            if now - self.last_obstacle_time >= self.avoid_release_timeout:
                self.reset_avoidance_state()
            return

        cone_bottom = cone_roi.rect.y_offset + cone_roi.rect.height
        if cone_bottom < self.y_zt:
            if now - self.last_obstacle_time >= self.avoid_release_timeout:
                self.reset_avoidance_state()
            return

        self.last_obstacle_time = now
        cone_center = self.roi_center_x(cone_roi)

        if self.avoid_direction == 0 or now >= self.direction_lock_until:
            self.avoid_direction = self.choose_direction(
                cone_center,
                p_roi,
                qrcode_roi,
            )
            self.direction_lock_until = now + self.avoid_direction_lock

        if self.avoid_direction == -1:
            raw_error = (self.image_width - 1.0) - cone_center
        else:
            raw_error = -cone_center

        alpha = self.avoid_filter_alpha
        self.filtered_error = (
            alpha * raw_error + (1.0 - alpha) * self.filtered_error
        )

        desired_angular = clamp(
            self.filtered_error * self.kp_avoid,
            -self.max_angular,
            self.max_angular,
        )
        angular = self.limit_angular_change(desired_angular, now)

        turn_ratio = min(abs(angular) / self.max_angular, 1.0)
        speed_scale = 1.0 - self.turn_slowdown * turn_ratio
        linear = max(self.v_avoid_min, self.v_avoid * speed_scale)

        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.publisher.publish(twist)

        if now - self.last_debug_log >= 0.5:
            direction_text = '左绕' if self.avoid_direction == -1 else '右绕'
            self.get_logger().info(
                f'锥桶: y={cone_bottom}, x={cone_center:.1f}, '
                f'{direction_text}, v={linear:.2f}, w={angular:.2f}'
            )
            self.last_debug_log = now

    def choose_direction(self, cone_center, p_roi, qrcode_roi):
        """Keep the original reference-marker avoidance decision."""
        if p_roi is not None:
            reference_center = self.roi_center_x(p_roi)
            return -1 if reference_center <= cone_center else 1

        if qrcode_roi is not None:
            qrcode_bottom = (
                qrcode_roi.rect.y_offset + qrcode_roi.rect.height
            )
            if qrcode_bottom >= self.y_qrcode:
                reference_center = self.roi_center_x(qrcode_roi)
                return -1 if reference_center <= cone_center else 1

        return -1 if cone_center >= self.image_width / 2.0 else 1

    @staticmethod
    def roi_center_x(roi):
        return roi.rect.x_offset + roi.rect.width / 2.0

    def limit_angular_change(self, desired_angular, now):
        if self.last_control_time is None or self.angular_slew_rate <= 0.0:
            angular = desired_angular
        else:
            dt = clamp(now - self.last_control_time, 0.001, 0.25)
            max_change = self.angular_slew_rate * dt
            angular = clamp(
                desired_angular,
                self.last_angular - max_change,
                self.last_angular + max_change,
            )

        self.last_control_time = now
        self.last_angular = angular
        return angular

    def reset_avoidance_state(self):
        self.avoid_direction = 0
        self.direction_lock_until = 0.0
        self.filtered_error = 0.0
        self.last_control_time = None
        self.last_angular = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = ControlYolo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
