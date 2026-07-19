# 巡线与避障控制说明

## 发车信号

`control.launch.py` 默认启用 `/start` 门控。启动控制后小车持续接收零速度，巡线、避障、二维码靠近和任务状态机都处于等待状态。

在另一个终端发布整数 `1` 后才开始运行：

```bash
ros2 topic pub --once /start std_msgs/msg/Int32 "{data: 1}"
```

调试时如需启动后直接运行，可以使用：

```bash
ros2 launch control control.launch.py require_start_signal:=false
```

## 控制优先级

`master` 按以下优先级输出 `/cmd_vel`：

1. 返航 P 点停车
2. 二维码入通道动作
3. YOLO 锥桶避障
4. Aurora 二维码方向跟随
5. ResNet 巡线
6. 感知超时安全停车

二维码只在一次任务中触发一次。P 点默认必须在识别过二维码后连续确认 3 帧才会发布停车信号，避免小车从起点出发时误停。

## 主要优化

- 使用 `time.monotonic()` 计算控制时间，不受系统时间校准影响。
- ResNet 和 YOLO 使用独立、可配置的消息看门狗。
- 避障方向按秒锁定，不再依赖不稳定的检测帧数。
- 巡线和避障角速度均增加变化率限制，减少方向突跳。
- 转弯时自动降低线速度，直线时恢复设定速度。
- 目标检测只选择每一类别中面积最大的有效框。
- 二维码和 P 点增加任务阶段与防重复触发保护。
- go 阶段检测到 Aurora 二维码框时，临时跟随二维码方向；成功解码后立即退出跟随。
- 人形立牌识别后按斜坡减速、短暂保持、斜坡恢复，不会突然改变速度。

## 推荐起始参数

```bash
ros2 launch control control.launch.py \
  v_line:=0.8 \
  v_line_min:=0.45 \
  kp_line:=0.006 \
  line_filter_alpha:=0.7 \
  line_angular_slew_rate:=12.0 \
  line_turn_slowdown:=0.35 \
  v_avoid:=0.8 \
  v_avoid_min:=0.45 \
  kp_avoid:=0.0035 \
  avoid_direction_lock:=0.35 \
  avoid_angular_slew_rate:=16.0 \
  v_qrcode:=0.45 \
  kp_qrcode:=0.005 \
  qrcode_approach_timeout:=0.20 \
  person_slowdown_factor:=0.45 \
  person_ramp_down:=0.35 \
  person_hold_duration:=0.80 \
  person_ramp_up:=0.80 \
  resnet_timeout:=0.25 \
  yolo_timeout:=0.18 \
  y_zt:=157 \
  y_p:=435 \
  y_qrcode:=167 \
  y_line:=200
```

## 调参顺序

1. 先将 `v_line` 调低到 `0.5~0.6`，只调 `kp_line`，确保直线和普通弯道都能稳定跟踪。
2. 车在弯道冲出时，提高 `kp_line` 或提高 `line_turn_slowdown`。
3. 方向左右抖动时，降低 `kp_line`、降低 `line_filter_alpha`，或降低 `line_angular_slew_rate`。
4. 感知帧率低导致间歇停车时，适当提高 `resnet_timeout`，一般不要超过 `0.4s`。
5. 锥桶前左右反复选择时，提高 `avoid_direction_lock`，建议范围 `0.3~0.6s`。
6. 避障转向不足时提高 `kp_avoid`；转向过猛时降低 `kp_avoid` 或提高 `avoid_turn_slowdown`。
7. 二维码方向跟随不足时提高 `kp_qrcode`；左右摆动时降低 `kp_qrcode` 或 `qrcode_angular_slew_rate`。
8. 人形减速过强时提高 `person_slowdown_factor`；减速太突然时提高 `person_ramp_down`。

每次只调整一个参数，并记录赛道、速度和结果。首次实车测试应架空驱动轮或使用低速参数确认方向正负号正确。
