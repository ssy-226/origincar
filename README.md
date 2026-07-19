# 智能小车代码结构

```text
src/
├─ control/    巡线、避障、二维码靠近和总控制
├─ detect/     单目与 Aurora YOLO 检测
├─ display/    二维码与人形描述显示
├─ origincar/  原车底盘与第三方驱动（结构未改）
├─ qr/         Aurora RGB 二维码解码
├─ track/      go、s、n、back 四套巡线模型
└─ vlm/        人形照片裁剪和云端描述
```

除本次精简的感知话题外，其余控制、二维码、大模型和显示话题保持不变。

感知话题已精简为 `/track/go`、`/track/s`、`/track/n`、`/track/back` 和 `/detect`。控制启动后需要向 `/start` 发布整数 `1` 才会发车。

详细启动方法见 [启动命令.md](启动命令.md)。
