#!/usr/bin/env python3
import os

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


SCREEN_WIDTH = 800
SCREEN_HEIGHT = 480
WINDOW_NAME = "Robot Display"

FONT_PATHS = [
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/arphic/uming.ttc",
]


def find_chinese_font():
    for font_path in FONT_PATHS:
        if os.path.exists(font_path):
            return font_path

    raise FileNotFoundError(
        "未找到中文字体，请安装：sudo apt install fonts-wqy-zenhei"
    )


def wrap_text_by_width(draw, text, font, max_width):
    lines = []

    for paragraph in text.split("\n"):
        if not paragraph:
            lines.append("")
            continue

        current_line = ""

        for character in paragraph:
            candidate = current_line + character
            bbox = draw.textbbox((0, 0), candidate, font=font)
            width = bbox[2] - bbox[0]

            if width <= max_width:
                current_line = candidate
            else:
                if current_line:
                    lines.append(current_line)
                current_line = character

        if current_line:
            lines.append(current_line)

    return lines


def explain_qrcode_direction(qr_text):
    if qr_text == "ClockWise":
        return "顺时针"

    if qr_text == "AntiClockWise":
        return "逆时针"

    try:
        number = int(qr_text)
        return "逆时针" if number % 2 == 0 else "顺时针"
    except ValueError:
        return "未知方向"


class DisplayNode(Node):
    def __init__(self):
        super().__init__("display")

        font_path = find_chinese_font()

        self.title_font = ImageFont.truetype(font_path, 42)
        self.qrcode_font = ImageFont.truetype(font_path, 34)
        self.vlm_title_font = ImageFont.truetype(font_path, 32)
        self.vlm_font = ImageFont.truetype(font_path, 30)

        self.qrcode_text = "二维码内容：等待识别"
        self.vlm_text = "大模型结果：等待识别"

        self.vlm_subscription = self.create_subscription(
            String,
            "/vision_language_model",
            self.vlm_callback,
            10,
        )

        self.qrcode_subscription = self.create_subscription(
            String,
            "/qrcode_text",
            self.qrcode_callback,
            10,
        )

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.moveWindow(WINDOW_NAME, 0, 0)
        cv2.resizeWindow(WINDOW_NAME, SCREEN_WIDTH, SCREEN_HEIGHT)
        cv2.setWindowProperty(
            WINDOW_NAME,
            cv2.WND_PROP_FULLSCREEN,
            cv2.WINDOW_FULLSCREEN,
        )

        self.timer = self.create_timer(0.1, self.render_screen)

        self.get_logger().info("显示节点已启动，等待 /qrcode_text 和 /vision_language_model")

    def vlm_callback(self, msg):
        text = msg.data.strip()

        if text == "start":
            self.vlm_text = "大模型结果：正在分析..."
        elif text == "error":
            self.vlm_text = "大模型结果：调用失败"
        elif text:
            self.vlm_text = "大模型结果：" + text

        self.get_logger().info(f"更新大模型显示内容：{text}")

    def qrcode_callback(self, msg):
        qr_text = msg.data.strip()

        if not qr_text:
            return

        direction = explain_qrcode_direction(qr_text)
        self.qrcode_text = f"二维码内容：{qr_text}，{direction}"

        self.get_logger().info(self.qrcode_text)

    def render_screen(self):
        image = Image.new(
            "RGB",
            (SCREEN_WIDTH, SCREEN_HEIGHT),
            (12, 27, 45),
        )

        draw = ImageDraw.Draw(image)

        draw.rectangle(
            (0, 0, SCREEN_WIDTH, 82),
            fill=(22, 72, 108),
        )

        draw.text(
            (36, 18),
            "小车识别结果显示",
            font=self.title_font,
            fill=(255, 255, 255),
        )

        draw.rectangle(
            (28, 108, 772, 188),
            fill=(34, 91, 80),
        )

        qrcode_lines = wrap_text_by_width(
            draw=draw,
            text=self.qrcode_text,
            font=self.qrcode_font,
            max_width=700,
        )

        for index, line in enumerate(qrcode_lines[:2]):
            draw.text(
                (46, 118 + index * 38),
                line,
                font=self.qrcode_font,
                fill=(255, 235, 130),
            )

        draw.text(
            (40, 225),
            "大模型结果：",
            font=self.vlm_title_font,
            fill=(160, 220, 255),
        )

        vlm_body = self.vlm_text.replace("大模型结果：", "", 1)

        lines = wrap_text_by_width(
            draw=draw,
            text=vlm_body,
            font=self.vlm_font,
            max_width=720,
        )

        start_y = 272
        line_height = 38
        max_lines = 5

        for index, line in enumerate(lines[:max_lines]):
            draw.text(
                (46, start_y + index * line_height),
                line,
                font=self.vlm_font,
                fill=(245, 248, 250),
            )

        if len(lines) > max_lines:
            draw.text(
                (710, 430),
                "...",
                font=self.vlm_font,
                fill=(245, 248, 250),
            )

        frame = cv2.cvtColor(
            np.asarray(image),
            cv2.COLOR_RGB2BGR,
        )

        cv2.imshow(WINDOW_NAME, frame)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 and rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DisplayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
