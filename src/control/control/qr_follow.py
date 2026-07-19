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


class ControlQrcodeApproach(Node):
    """Temporarily steer toward an Aurora-detected QR code during go mode."""

    def __init__(self):
        super().__init__('qr_follow')

        self.declare_parameter(
            'detection_topic',
            '/aurora/person_detection',
        )
        self.declare_parameter('require_start_signal', True)
        self.declare_parameter('start_topic', '/start')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 400)
        self.declare_parameter('qrcode_confidence', 0.7)
        self.declare_parameter('qrcode_min_bottom_y', 40)
        self.declare_parameter('v_qrcode', 0.45)
        self.declare_parameter('v_qrcode_min', 0.25)
        self.declare_parameter('kp_qrcode', 0.005)
        self.declare_parameter('error_filter_alpha', 0.65)
        self.declare_parameter('error_deadband', 4.0)
        self.declare_parameter('max_angular', 2.5)
        self.declare_parameter('angular_slew_rate', 10.0)
        self.declare_parameter('turn_slowdown', 0.35)

        self.detection_topic = self.get_parameter('detection_topic').value
        self.require_start_signal = bool(
            self.get_parameter('require_start_signal').value
        )
        self.start_topic = self.get_parameter('start_topic').value
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.qrcode_confidence = float(
            self.get_parameter('qrcode_confidence').value
        )
        self.qrcode_min_bottom_y = int(
            self.get_parameter('qrcode_min_bottom_y').value
        )
        self.v_qrcode = float(self.get_parameter('v_qrcode').value)
        self.v_qrcode_min = float(
            self.get_parameter('v_qrcode_min').value
        )
        self.kp_qrcode = float(self.get_parameter('kp_qrcode').value)
        self.error_filter_alpha = clamp(
            float(self.get_parameter('error_filter_alpha').value),
            0.0,
            1.0,
        )
        self.error_deadband = float(
            self.get_parameter('error_deadband').value
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

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.detection_subscription = self.create_subscription(
            PerceptionTargets,
            self.detection_topic,
            self.detection_callback,
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
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel_qrcode_approach',
            qos_profile,
        )

        self.enabled = True
        self.started = not self.require_start_signal
        self.filtered_error = 0.0
        self.last_angular = 0.0
        self.last_control_time = None
        self.last_debug_log = float('-inf')

        self.get_logger().info(
            '二维码靠近节点启动: '
            f'detection={self.detection_topic}, v={self.v_qrcode:.2f}'
        )

    def start_callback(self, msg: Int32):
        if msg.data != 1 or self.started:
            return

        self.started = True
        self.filtered_error = 0.0
        self.last_angular = 0.0
        self.last_control_time = None
        self.get_logger().info('收到发车信号，启用二维码方向跟随')

    def qrcode_callback(self, msg: Int32):
        if not self.started:
            return
        if msg.data in (3, 4) and self.enabled:
            self.enabled = False
            self.get_logger().info('二维码已解码，停止二维码方向跟随')

    def detection_callback(self, msg: PerceptionTargets):
        if not self.started or not self.enabled:
            return

        best_roi = None
        best_area = 0

        for target in msg.targets:
            if target.type != 'qrcode':
                continue

            for roi in target.rois:
                if roi.confidence < self.qrcode_confidence:
                    continue

                bottom_y = roi.rect.y_offset + roi.rect.height
                if not self.qrcode_min_bottom_y <= bottom_y < self.image_height:
                    continue

                area = roi.rect.width * roi.rect.height
                if area > best_area:
                    best_area = area
                    best_roi = roi

        if best_roi is None:
            return

        self.publish_approach_command(best_roi)

    def publish_approach_command(self, roi):
        now = time.monotonic()
        target_x = roi.rect.x_offset + roi.rect.width / 2.0
        error = self.image_width / 2.0 - target_x

        if abs(error) <= self.error_deadband:
            error = 0.0

        alpha = self.error_filter_alpha
        self.filtered_error = (
            alpha * error + (1.0 - alpha) * self.filtered_error
        )

        desired_angular = clamp(
            self.filtered_error * self.kp_qrcode,
            -self.max_angular,
            self.max_angular,
        )
        angular = self.limit_angular_change(desired_angular, now)

        turn_ratio = min(abs(angular) / self.max_angular, 1.0)
        speed_scale = 1.0 - self.turn_slowdown * turn_ratio
        linear = max(self.v_qrcode_min, self.v_qrcode * speed_scale)

        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.publisher.publish(twist)

        if now - self.last_debug_log >= 0.5:
            bottom_y = roi.rect.y_offset + roi.rect.height
            self.get_logger().info(
                f'跟随二维码: x={target_x:.1f}, y={bottom_y}, '
                f'v={linear:.2f}, w={angular:.2f}'
            )
            self.last_debug_log = now

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
    node = ControlQrcodeApproach()
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
