#!/usr/bin/env python3
import math
import time

import rclpy
from ai_msgs.msg import PerceptionTargets
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Int32


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


class ControlResnet(Node):
    """Select a route model and convert its track center into smooth velocity."""

    VALID_MODES = {'go', 's', 'n', 'back'}

    def __init__(self):
        super().__init__('line')

        self.declare_parameter('v_line', 0.8)
        self.declare_parameter('require_start_signal', True)
        self.declare_parameter('start_topic', '/start')
        self.declare_parameter('v_line_min', 0.45)
        self.declare_parameter('kp_line', 0.006)
        self.declare_parameter('y_line', 200)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('line_confidence', 0.8)
        self.declare_parameter('line_min_y', 119)
        self.declare_parameter('error_deadband', 3.0)
        self.declare_parameter('error_filter_alpha', 0.7)
        self.declare_parameter('max_angular', 5.0)
        self.declare_parameter('angular_slew_rate', 12.0)
        self.declare_parameter('turn_slowdown', 0.35)
        self.declare_parameter('qrcode_cooldown', 5.0)
        self.declare_parameter('return_delay', 8.0)

        self.v_line = float(self.get_parameter('v_line').value)
        self.require_start_signal = bool(
            self.get_parameter('require_start_signal').value
        )
        self.start_topic = self.get_parameter('start_topic').value
        self.v_line_min = float(self.get_parameter('v_line_min').value)
        self.kp_line = float(self.get_parameter('kp_line').value)
        self.y_line = int(self.get_parameter('y_line').value)
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.line_confidence = float(
            self.get_parameter('line_confidence').value
        )
        self.line_min_y = int(self.get_parameter('line_min_y').value)
        self.error_deadband = float(
            self.get_parameter('error_deadband').value
        )
        self.error_filter_alpha = clamp(
            float(self.get_parameter('error_filter_alpha').value),
            0.0,
            1.0,
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
        self.qrcode_cooldown = max(
            0.0,
            float(self.get_parameter('qrcode_cooldown').value),
        )
        self.return_delay = max(
            0.0,
            float(self.get_parameter('return_delay').value),
        )

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.current_mode = 'go'
        self.started = not self.require_start_signal
        self.filtered_error = 0.0
        self.last_angular = 0.0
        self.last_control_time = None
        self.last_qrcode_time = float('-inf')
        self.qr_scanned_flag = False
        self.qrcode_processed = False

        self.sub_back = self.create_subscription(
            PerceptionTargets,
            '/track/back',
            self.target_callback_back,
            qos_profile,
        )
        self.start_sub = self.create_subscription(
            Int32,
            self.start_topic,
            self.start_callback,
            10,
        )
        self.sub_go = self.create_subscription(
            PerceptionTargets,
            '/track/go',
            self.target_callback_go,
            qos_profile,
        )
        self.sub_s = self.create_subscription(
            PerceptionTargets,
            '/track/s',
            self.target_callback_s,
            qos_profile,
        )
        self.sub_n = self.create_subscription(
            PerceptionTargets,
            '/track/n',
            self.target_callback_n,
            qos_profile,
        )
        self.sub_qrcode = self.create_subscription(
            Int32,
            '/qrcode_number',
            self.qrcode_callback,
            10,
        )
        self.sub_obstacle = self.create_subscription(
            PerceptionTargets,
            '/detect',
            self.obstacle_callback,
            qos_profile,
        )

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel_resnet',
            qos_profile,
        )

        self.get_logger().info(
            '巡线节点启动: '
            f'mode=go, v={self.v_line:.2f}, v_min={self.v_line_min:.2f}, '
            f'kp={self.kp_line:.4f}'
        )

    def start_callback(self, msg: Int32):
        if msg.data != 1 or self.started:
            return

        self.started = True
        self.filtered_error = 0.0
        self.last_angular = 0.0
        self.last_control_time = None
        self.get_logger().info('收到发车信号，启用巡线和路线状态机')

    def set_mode(self, mode, reason):
        if mode not in self.VALID_MODES or mode == self.current_mode:
            return

        old_mode = self.current_mode
        self.current_mode = mode

        # Do not carry the previous model's steering history into a new route.
        self.filtered_error = 0.0
        self.last_angular = 0.0
        self.last_control_time = None

        self.get_logger().info(
            f'巡线模式切换: {old_mode} -> {mode}, 原因: {reason}'
        )

    def obstacle_callback(self, msg: PerceptionTargets):
        if (
            not self.started
            or not self.qr_scanned_flag
            or self.current_mode not in {'s', 'n'}
        ):
            return

        best_roi = None
        best_area = 0

        for target in msg.targets:
            if target.type != 'line':
                continue

            for roi in target.rois:
                if roi.confidence < self.line_confidence:
                    continue

                bottom_y = roi.rect.y_offset + roi.rect.height
                if not self.line_min_y <= bottom_y < self.image_height:
                    continue

                area = roi.rect.width * roi.rect.height
                if area > best_area:
                    best_area = area
                    best_roi = roi

        if best_roi is None:
            return

        bottom_y = best_roi.rect.y_offset + best_roi.rect.height
        elapsed = time.monotonic() - self.last_qrcode_time

        if bottom_y >= self.y_line and elapsed >= self.return_delay:
            self.qr_scanned_flag = False
            self.set_mode(
                'back',
                f'扫码后{elapsed:.1f}s检测到返航线(y={bottom_y})',
            )

    def qrcode_callback(self, msg: Int32):
        if not self.started:
            return
        if msg.data not in (3, 4):
            self.get_logger().warn(f'忽略未知二维码控制值: {msg.data}')
            return

        # The mission contains one QR code. ZBar can publish the same result
        # for many consecutive frames, so accept it only once per node run.
        if self.qrcode_processed:
            return

        now = time.monotonic()
        if now - self.last_qrcode_time < self.qrcode_cooldown:
            return

        self.last_qrcode_time = now
        self.qr_scanned_flag = True
        self.qrcode_processed = True

        if msg.data == 3:
            self.set_mode('s', '二维码为奇数/顺时针')
        else:
            self.set_mode('n', '二维码为偶数/逆时针')

    def target_callback_back(self, msg: PerceptionTargets):
        if self.current_mode == 'back':
            self.process_target(msg)

    def target_callback_go(self, msg: PerceptionTargets):
        if self.current_mode == 'go':
            self.process_target(msg)

    def target_callback_s(self, msg: PerceptionTargets):
        if self.current_mode == 's':
            self.process_target(msg)

    def target_callback_n(self, msg: PerceptionTargets):
        if self.current_mode == 'n':
            self.process_target(msg)

    def process_target(self, msg: PerceptionTargets):
        if not self.started:
            return
        target_x = self.extract_target_x(msg)
        if target_x is None:
            return

        self.execute_control(target_x)

    def extract_target_x(self, msg: PerceptionTargets):
        """Return the first finite track-center x coordinate in the message."""
        try:
            for target in msg.targets:
                for points_group in target.points:
                    for point in points_group.point:
                        target_x = float(point.x)
                        if (
                            math.isfinite(target_x)
                            and 0.0 <= target_x < self.image_width
                        ):
                            return target_x
        except Exception as exc:
            self.get_logger().warn(f'解析巡线坐标失败: {exc}')

        return None

    def execute_control(self, target_x):
        now = time.monotonic()
        center_x = self.image_width / 2.0
        raw_error = center_x - target_x

        if abs(raw_error) <= self.error_deadband:
            raw_error = 0.0

        alpha = self.error_filter_alpha
        self.filtered_error = (
            alpha * raw_error + (1.0 - alpha) * self.filtered_error
        )

        desired_angular = clamp(
            self.filtered_error * self.kp_line,
            -self.max_angular,
            self.max_angular,
        )

        angular = self.limit_angular_change(desired_angular, now)
        turn_ratio = min(abs(angular) / self.max_angular, 1.0)
        speed_scale = 1.0 - self.turn_slowdown * turn_ratio
        linear = max(self.v_line_min, self.v_line * speed_scale)

        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.publisher.publish(twist)

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


def main(args=None):
    rclpy.init(args=args)
    node = ControlResnet()
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
