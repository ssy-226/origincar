#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String


class QRCodeDisplay(Node):
    def __init__(self):
        super().__init__('qr_log')

        self.last_text = ''
        self.last_result = None

        self.text_sub = self.create_subscription(
            String,
            '/qrcode_text',
            self.text_callback,
            10
        )

        self.result_sub = self.create_subscription(
            Int32,
            '/qrcode_number',
            self.result_callback,
            10
        )

        self.get_logger().info('qr_log started, waiting for QR results')

    def text_callback(self, msg: String):
        self.last_text = msg.data.strip()
        self.print_status()

    def result_callback(self, msg: Int32):
        self.last_result = int(msg.data)
        self.print_status()

    def explain_result(self) -> str:
        if self.last_result == 3:
            return '奇数 / 顺时针 / 切换到 s 模式'
        if self.last_result == 4:
            return '偶数 / 逆时针 / 切换到 n 模式'
        if self.last_result is None:
            return '尚未收到映射结果'
        return f'未知映射值: {self.last_result}'

    def print_status(self):
        if not self.last_text and self.last_result is None:
            return

        text = self.last_text if self.last_text else '<empty>'
        explain = self.explain_result()

        self.get_logger().info(
            f'二维码原文: {text} | 控制值: {self.last_result} | 解释: {explain}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = QRCodeDisplay()
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
