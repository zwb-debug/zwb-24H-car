# 2024_H 固件工程

MSPM0G3507（立创·地猛星）CCS 工程，2024 电赛 H 题自动行驶小车。

## 打开与编译

- **CCS**：File → Import Project → 选择本目录 → Build。
  引脚/外设全部在 `2024_H.syscfg` 里，改引脚不会影响 C 代码。
- **命令行**（不打开 CCS）：

  ```powershell
  powershell -ExecutionPolicy Bypass -File ..\tools\build.ps1
  ```

  产物在 `..\build\2024_H.out` / `.hex`。

## 文件说明

| 文件 | 作用 |
|---|---|
| `board.h` | 场地几何、机械参数、速度、PID 默认值（**先改这里**） |
| `main.c` | 初始化、按键、OLED 页面、1ms 时基 |
| `mission.c/.h` | 四道题的分段表与状态机 |
| `control.c/.h` | 轮速 PI、航向 PID、循迹 PD、转向 |
| `bsp_motor.c` | TB6612 双路 PWM + 方向 |
| `bsp_encoder.c` | 双路 AB 相四倍频（GPIOA 中断） |
| `bsp_gray.c` | 8 路数字灰度、加权位置 |
| `bsp_imu.c` | JY61P/WT61 的 0x55 0x53 帧解析 |
| `bsp_oled.c` / `font6x8.c` | SSD1306 128×64 + 6×8 字库 |
| `bsp_ui.c` | 按键消抖、蜂鸣器、LED |
| `telemetry.c` | 50Hz CSV 遥测 + 串口在线调参（自写整数格式化，不依赖 printf） |
| `startup_mspm0g350x_ticlang.c` | 启动文件（SDK 自带） |

## 默认引脚

见 `..\docs\02_硬件与接线.md`，或直接打开 `2024_H.syscfg`。

## 注意

- 本工程默认 **32MHz 内部时钟**，不依赖板载晶振；
- 默认 **2WD 差速**，轮距 12cm、轮径 6.5cm、编码器 13 线 1:30；
- 上电后先按 `docs/03_调试与标定.md` 标定，再跑题。
