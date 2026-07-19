# 安装openai
# pip3 install openai==1.35.9 requests httpx==0.27.2 psutil flask -i http://mirrors.aliyun.com/pypi/simple/ --trusted-host mirrors.aliyun.com
# 图片自动上传，Token使用警告


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from ai_msgs.msg import PerceptionTargets
from cv_bridge import CvBridge

import cv2
import threading
import base64
import time
from openai import OpenAI

class PersonLLMNode(Node):
    def __init__(self):
        super().__init__('vlm')

        self.declare_parameter('image_topic', '/rgb/image_raw')
        self.declare_parameter(
            'detection_topic',
            '/aurora/person_detection'
        )
        self.declare_parameter('person_event_topic', '/person_detected')
        self.declare_parameter('person_trigger_cooldown', 3.0)
        self.image_topic = self.get_parameter('image_topic').value
        self.detection_topic = self.get_parameter('detection_topic').value
        self.person_event_topic = self.get_parameter(
            'person_event_topic'
        ).value
        self.person_trigger_cooldown = max(
            0.0,
            float(self.get_parameter('person_trigger_cooldown').value)
        )

        # 初始化大模型客户端
        self.client = OpenAI(
            base_url="https://ark.cn-beijing.volces.com/api/v3",
            api_key="ark-ef85d811-b52a-4585-9407-daadcd640459-7286c",
        )

        # QoS 设置：尽力而为，历史长度 1
        qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.bridge = CvBridge()

        # 订阅 Aurora RGB 原始图像。保持原始 640x400，不插值。
        self.img_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_best_effort
        )

        # 订阅 Aurora 独立 YOLO 的检测结果，确保检测框和照片来自同一相机
        self.target_sub = self.create_subscription(
            PerceptionTargets,
            self.detection_topic,
            self.target_callback,
            qos_best_effort
        )

        # 发布 VLM 描述结果
        self.publisher_ = self.create_publisher(
            String, 
            '/vision_language_model', 
            10
        )
        self.person_event_publisher = self.create_publisher(
            Int32,
            self.person_event_topic,
            10
        )

        # 共享数据与互斥锁
        self.latest_image = None
        self.lock = threading.Lock()
        
        # 状态标志位：防止大模型请求堆积阻塞
        self.is_calling_vlm = False
        self.last_person_trigger_time = float('-inf')
        
        self.get_logger().info(
            f'人形大模型节点已启动: image={self.image_topic}, '
            f'detection={self.detection_topic}'
        )

    def image_callback(self, msg: Image):
        try:
            color_img = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().warn(f'Aurora RGB图像转换失败: {e}')
            return

        # 保存 Aurora 原始分辨率图像，不做缩放或插值
        with self.lock:
            self.latest_image = color_img.copy()

    def target_callback(self, msg: PerceptionTargets):
        # 如果大模型正在推理中，直接丢弃当前帧，防止堆积卡死系统
        if self.is_calling_vlm:
            return

        # 获取与 Aurora 检测框对应的最新原始彩色图
        with self.lock:
            if self.latest_image is None:
                return
            color_img = self.latest_image.copy()

        img_height, img_width = color_img.shape[:2]

        # 筛选 Aurora YOLO 发布的 person 目标
        best_rect = None
        max_area = 0

        for target in msg.targets:      
            if target.type == 'person':
                for roi in target.rois:
                    if roi.confidence > 0.8:
                        bottom_y = roi.rect.y_offset + roi.rect.height
                        
                        if (120 - 1) <= bottom_y <= (img_height - 1):
                            area = roi.rect.width * roi.rect.height
                            
                            if area > max_area:
                                max_area = area
                                best_rect = roi.rect

        # 没有符合条件的 person，直接跳出
        if best_rect is None:
            return

        now = time.monotonic()
        if (
            now - self.last_person_trigger_time
            < self.person_trigger_cooldown
        ):
            return

        # 把检测框扩大 20%
        center_x = best_rect.x_offset + best_rect.width / 2.0
        center_y = best_rect.y_offset + best_rect.height / 2.0
        new_width = best_rect.width * 1.2
        new_height = best_rect.height * 1.2

        # 边界检查
        x_min = max(0, int(center_x - new_width / 2.0))
        y_min = max(0, int(center_y - new_height / 2.0))
        x_max = min(img_width, int(center_x + new_width / 2.0))
        y_max = min(img_height, int(center_y + new_height / 2.0))

        if x_max <= x_min or y_max <= y_min:
            return

        # 裁剪 ROI 区域
        cropped_roi = color_img[y_min:y_max, x_min:x_max]

        # 从 Aurora 原始 640x400 图像直接裁剪，不对人物照片进行插值缩放。
        # 仅通过 JPEG 质量参数控制上传体积。
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
        success, compressed_jpg = cv2.imencode('.jpg', cropped_roi, encode_param)
        
        if not success:
            return

        self.last_person_trigger_time = now
        person_event = Int32()
        person_event.data = 1
        self.person_event_publisher.publish(person_event)
        self.get_logger().info('已发布人形立牌减速事件')

        # 转换为 base64 字符串
        base64_image = base64.b64encode(compressed_jpg.tobytes()).decode("utf-8")

        # ==========================================
        # 阶段四：启动独立线程调用大模型
        # ==========================================
        # 开启推理锁，并把网络请求扔到后台线程执行，主线程继续高速运转
        self.is_calling_vlm = True
        threading.Thread(target=self.call_vlm, args=(base64_image,), daemon=True).start()

    def call_vlm(self, base64_image):
        """后台线程中执行的大模型请求"""
        try:
            # 报告请求开始
            response_msg = String()
            response_msg.data = "start"
            self.publisher_.publish(response_msg)
            self.get_logger().info("已发送图片至大模型，等待回复...")

            completion = self.client.chat.completions.create(
                model="doubao-seed-2-0-mini-260428",
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": "描述图片上的人,禁止说识别不清这类话,尽可能描述"},
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/jpeg;base64,{base64_image}",
                                }
                            },
                        ],
                    }
                ],
                max_tokens=300,
            )
            
            # 解析并发布结果
            result_text = completion.choices[0].message.content
            response_msg.data = result_text
            self.publisher_.publish(response_msg)
            self.get_logger().info(f"大模型回复: {result_text}")

        except Exception as e:
            self.get_logger().error(f"大模型调用异常: {str(e)}")
            response_msg = String()
            response_msg.data = "error"
            self.publisher_.publish(response_msg)
            
        finally:
            # 无论成功失败，释放推理锁，允许系统抓取下一张图
            self.is_calling_vlm = False

def main(args=None):
    rclpy.init(args=args)
    node = PersonLLMNode()
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
