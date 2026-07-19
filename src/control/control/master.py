#!/usr/bin/env python3
import signal
import time
from threading import Lock

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Int32


class ControlMaster(Node):
    """Arbitrate mission, obstacle and line-following velocity commands."""

    def __init__(self):
        super().__init__('master')

        self.declare_parameter('control_period', 0.03)
        self.declare_parameter('require_start_signal', True)
        self.declare_parameter('start_topic', '/start')
        self.declare_parameter('resnet_timeout', 0.25)
        self.declare_parameter('yolo_timeout', 0.18)
        self.declare_parameter('qrcode_approach_timeout', 0.20)
        self.declare_parameter('qrcode_cooldown', 5.0)
        self.declare_parameter('qr_stop_duration', 0.09)
        self.declare_parameter('qr_maneuver_duration', 1.11)
        self.declare_parameter('qr_reverse_speed', -0.8)
        self.declare_parameter('qr_angular_speed', 5.0)
        self.declare_parameter('person_event_topic', '/person_detected')
        self.declare_parameter('person_slowdown_factor', 0.45)
        self.declare_parameter('person_ramp_down', 0.35)
        self.declare_parameter('person_hold_duration', 0.80)
        self.declare_parameter('person_ramp_up', 0.80)
        self.declare_parameter('person_slowdown_cooldown', 2.0)

        self.control_period = max(
            0.01,
            float(self.get_parameter('control_period').value),
        )
        self.require_start_signal = bool(
            self.get_parameter('require_start_signal').value
        )
        self.start_topic = self.get_parameter('start_topic').value
        self.resnet_timeout = max(
            self.control_period,
            float(self.get_parameter('resnet_timeout').value),
        )
        self.yolo_timeout = max(
            self.control_period,
            float(self.get_parameter('yolo_timeout').value),
        )
        self.qrcode_approach_timeout = max(
            self.control_period,
            float(self.get_parameter('qrcode_approach_timeout').value),
        )
        self.qrcode_cooldown = max(
            0.0,
            float(self.get_parameter('qrcode_cooldown').value),
        )
        self.qr_stop_duration = max(
            0.0,
            float(self.get_parameter('qr_stop_duration').value),
        )
        self.qr_maneuver_duration = max(
            0.0,
            float(self.get_parameter('qr_maneuver_duration').value),
        )
        self.qr_reverse_speed = float(
            self.get_parameter('qr_reverse_speed').value
        )
        self.qr_angular_speed = float(
            self.get_parameter('qr_angular_speed').value
        )
        self.person_event_topic = self.get_parameter(
            'person_event_topic'
        ).value
        self.person_slowdown_factor = max(
            0.05,
            min(
                float(self.get_parameter('person_slowdown_factor').value),
                1.0,
            ),
        )
        self.person_ramp_down = max(
            0.0,
            float(self.get_parameter('person_ramp_down').value),
        )
        self.person_hold_duration = max(
            0.0,
            float(self.get_parameter('person_hold_duration').value),
        )
        self.person_ramp_up = max(
            0.0,
            float(self.get_parameter('person_ramp_up').value),
        )
        self.person_slowdown_cooldown = max(
            0.0,
            float(self.get_parameter('person_slowdown_cooldown').value),
        )

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.flag_p = False
        self.started = not self.require_start_signal
        self.flag_qrcode = False
        self.qrcode_processed = False
        self.qrcode_action_start = None
        self.last_qrcode_time = float('-inf')

        self.twist_lock = Lock()
        self.twist_yolo = Twist()
        self.twist_resnet = Twist()
        self.twist_qrcode_approach = Twist()
        self.last_yolo_time = float('-inf')
        self.last_resnet_time = float('-inf')
        self.last_qrcode_approach_time = float('-inf')

        self.person_slowdown_start = None
        self.last_person_event_time = float('-inf')

        self.active_source = None
        self.emergency_stopping = False

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_profile,
        )
        self.start_sub = self.create_subscription(
            Int32,
            self.start_topic,
            self.start_callback,
            10,
        )
        self.cmd_vel_resnet = self.create_subscription(
            Twist,
            '/cmd_vel_resnet',
            self.cmd_vel_resnet_callback,
            qos_profile,
        )
        self.cmd_vel_yolo = self.create_subscription(
            Twist,
            '/cmd_vel_yolo',
            self.cmd_vel_yolo_callback,
            qos_profile,
        )
        self.cmd_vel_qrcode_approach = self.create_subscription(
            Twist,
            '/cmd_vel_qrcode_approach',
            self.cmd_vel_qrcode_approach_callback,
            qos_profile,
        )
        self.qrcode_sub = self.create_subscription(
            Int32,
            '/qrcode_number',
            self.qrcode_callback,
            10,
        )
        self.p_sub = self.create_subscription(
            Int32,
            '/p',
            self.p_callback,
            10,
        )
        self.person_sub = self.create_subscription(
            Int32,
            self.person_event_topic,
            self.person_callback,
            10,
        )

        self.timer = self.create_timer(
            self.control_period,
            self.timer_callback,
        )

        self.get_logger().info(
            '总控制节点启动: '
            f'period={self.control_period:.2f}s, '
            f'yolo_timeout={self.yolo_timeout:.2f}s, '
            f'qr_approach_timeout={self.qrcode_approach_timeout:.2f}s, '
            f'resnet_timeout={self.resnet_timeout:.2f}s, '
            f'waiting_start={not self.started}'
        )

    def start_callback(self, msg: Int32):
        if msg.data != 1 or self.started:
            return

        self.started = True
        self.active_source = None
        with self.twist_lock:
            self.last_yolo_time = float('-inf')
            self.last_qrcode_approach_time = float('-inf')
            self.last_resnet_time = float('-inf')
        self.get_logger().warn('收到发车信号1，控制系统开始运行')

    @staticmethod
    def copy_twist(msg):
        copied = Twist()
        copied.linear.x = float(msg.linear.x)
        copied.linear.y = float(msg.linear.y)
        copied.linear.z = float(msg.linear.z)
        copied.angular.x = float(msg.angular.x)
        copied.angular.y = float(msg.angular.y)
        copied.angular.z = float(msg.angular.z)
        return copied

    @staticmethod
    def make_twist(linear_x=0.0, angular_z=0.0):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        return twist

    def p_callback(self, msg: Int32):
        if not self.started:
            return
        if msg.data == 1 and not self.flag_p:
            self.flag_p = True
            self.get_logger().warn('收到返航P点停车信号，锁定停车状态')

    def qrcode_callback(self, msg: Int32):
        if not self.started:
            return
        if (
            msg.data not in (3, 4)
            or self.flag_p
            or self.flag_qrcode
            or self.qrcode_processed
        ):
            return

        now = time.monotonic()
        if now - self.last_qrcode_time < self.qrcode_cooldown:
            return

        self.flag_qrcode = True
        self.qrcode_processed = True
        self.qrcode_action_start = now
        self.last_qrcode_time = now
        self.get_logger().info(
            f'接收到二维码控制值{msg.data}，执行入通道动作'
        )

    def cmd_vel_resnet_callback(self, msg: Twist):
        with self.twist_lock:
            self.twist_resnet = self.copy_twist(msg)
            self.last_resnet_time = time.monotonic()

    def cmd_vel_yolo_callback(self, msg: Twist):
        with self.twist_lock:
            self.twist_yolo = self.copy_twist(msg)
            self.last_yolo_time = time.monotonic()

    def cmd_vel_qrcode_approach_callback(self, msg: Twist):
        with self.twist_lock:
            self.twist_qrcode_approach = self.copy_twist(msg)
            self.last_qrcode_approach_time = time.monotonic()

    def person_callback(self, msg: Int32):
        if (
            not self.started
            or msg.data != 1
            or not self.qrcode_processed
            or self.flag_p
        ):
            return

        # Do not start the slowdown while the fixed QR entry maneuver is active.
        if self.flag_qrcode:
            return

        now = time.monotonic()
        if now - self.last_person_event_time < self.person_slowdown_cooldown:
            return

        self.person_slowdown_start = now
        self.last_person_event_time = now
        self.get_logger().info('识别到人形立牌，开始平滑减速')

    def timer_callback(self):
        """Priority: P stop > QR maneuver > YOLO avoidance > ResNet line."""
        now = time.monotonic()

        if not self.started:
            self.publish_command('waiting_start', self.make_twist())
            return

        if self.flag_p:
            self.publish_command('p_stop', self.make_twist())
            return

        qr_command = self.get_qrcode_command(now)
        if qr_command is not None:
            self.publish_command('qrcode', qr_command)
            return

        with self.twist_lock:
            yolo_age = now - self.last_yolo_time
            qrcode_approach_age = now - self.last_qrcode_approach_time
            resnet_age = now - self.last_resnet_time
            yolo_twist = self.copy_twist(self.twist_yolo)
            qrcode_approach_twist = self.copy_twist(
                self.twist_qrcode_approach
            )
            resnet_twist = self.copy_twist(self.twist_resnet)

        if yolo_age <= self.yolo_timeout:
            self.publish_command(
                'yolo',
                self.apply_person_slowdown(yolo_twist, now),
            )
        elif qrcode_approach_age <= self.qrcode_approach_timeout:
            self.publish_command(
                'qrcode_approach',
                self.apply_person_slowdown(qrcode_approach_twist, now),
            )
        elif resnet_age <= self.resnet_timeout:
            self.publish_command(
                'resnet',
                self.apply_person_slowdown(resnet_twist, now),
            )
        else:
            self.publish_command('watchdog_stop', self.make_twist())

    def apply_person_slowdown(self, twist, now):
        scale = self.get_person_speed_scale(now)
        if scale >= 1.0:
            return twist

        slowed = self.copy_twist(twist)
        slowed.linear.x *= scale
        slowed.linear.y *= scale
        return slowed

    def get_person_speed_scale(self, now):
        if self.person_slowdown_start is None:
            return 1.0

        elapsed = now - self.person_slowdown_start
        minimum = self.person_slowdown_factor

        if self.person_ramp_down > 0.0 and elapsed < self.person_ramp_down:
            progress = elapsed / self.person_ramp_down
            return 1.0 - (1.0 - minimum) * progress

        elapsed -= self.person_ramp_down
        if elapsed < self.person_hold_duration:
            return minimum

        elapsed -= self.person_hold_duration
        if self.person_ramp_up > 0.0 and elapsed < self.person_ramp_up:
            progress = elapsed / self.person_ramp_up
            return minimum + (1.0 - minimum) * progress

        self.person_slowdown_start = None
        self.get_logger().info('人形立牌减速结束，恢复原速度')
        return 1.0

    def get_qrcode_command(self, now):
        if not self.flag_qrcode or self.qrcode_action_start is None:
            return None

        elapsed = now - self.qrcode_action_start
        if elapsed < self.qr_stop_duration:
            return self.make_twist()

        maneuver_end = self.qr_stop_duration + self.qr_maneuver_duration
        if elapsed < maneuver_end:
            return self.make_twist(
                self.qr_reverse_speed,
                self.qr_angular_speed,
            )

        self.flag_qrcode = False
        self.qrcode_action_start = None
        self.get_logger().info('入通道动作完成，恢复感知控制')
        return None

    def publish_command(self, source, twist):
        self.publisher.publish(twist)
        if source != self.active_source:
            self.active_source = source
            self.get_logger().info(f'控制源切换为: {source}')

    def emergency_stop(self):
        if self.emergency_stopping:
            return

        self.emergency_stopping = True
        stop_twist = self.make_twist()
        self.get_logger().warn('收到退出信号，发送紧急停车指令')

        for _ in range(5):
            self.publisher.publish(stop_twist)
            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = ControlMaster()

    def emergency_stop_handler(_sig, _frame):
        node.emergency_stop()
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, emergency_stop_handler)
    signal.signal(signal.SIGTERM, emergency_stop_handler)

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
