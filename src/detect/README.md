# detect

同一个 YOLO 节点支持两种输入：

- 单目共享内存 NV12：用于锥桶、P点和黑线检测。
- Aurora `sensor_msgs/Image`：使用 letterbox，用于人形和二维码方向检测。

```bash
ros2 launch detect detect.launch.py
ros2 launch detect aurora.launch.py
```
