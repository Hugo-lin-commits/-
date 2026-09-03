# 轴对称 X 三角一体翼 · 制导镖飞控（C++）

目标板：**Sipeed MaixCAM2** + **TDK ICM-42688**。控制律来自西交开源（三回路 + PNG/ISMCG），**板上跑 C++**，不用 Python。

`python_ref/` 是当初的算法草稿，**不要烧到板子上**。改参只改 `include/dart_fc/core.hpp`。

---

## 在电脑上先验证（不接板）

需要 g++（例如 `C:\c++\mingw64\bin\g++.exe`）：

```bat
cd /d 本仓库目录
g++ -std=c++17 -O2 -Iinclude examples\test_hw_remap.cpp -o test_hw_remap.exe
g++ -std=c++17 -O2 -Iinclude examples\closed_loop_demo.cpp -o closed_loop_demo.exe
test_hw_remap.exe
closed_loop_demo.exe
```

串级大约收到 −1°，三回路大约 0.9°（初值约 20° 滚转）。

---

## 在 MaixCAM2 上编译

用 [MaixCDK](https://github.com/sipeed/MaixCDK)（MaixPy 的 C++ 版），先保证官方 `hello_world` 能编过。

```bat
set MAIXCDK_PATH=你的MaixCDK路径
cd board\maixcam2
maixcdk build -p maixcam2
```

也可以把 `board/maixcam2` 拷进 `MaixCDK/projects/dart_fc`，并把仓库的 `include/` 放到能被 `main/CMakeLists.txt` 找到的位置（默认相对路径是仓库根下的 `include/`）。

接线：

- ICM-42688 → SPI2：B20 SCK、B18 MOSI、B19 MISO、B21 CS1
- 四路舵机 PWM 脚在 `HardwareParams::servo_pin`，按你们分电板改
- **不要用**板载 LSM6DSOWTR。WHO_AM_I 必须是 `0x47`
- 陀螺 ±2000 °/s，加计 ±16 g。发射后只积分陀螺

若编译报 `write_read` 找不到，对照本机 `maix_spi.hpp` 改 `MaixSpiBus::xfer`，控制律文件不用动。

视觉找灯接进 `VisionSample vis`（`valid/px/py/w/h`）。

---

## 目录

```
include/dart_fc.hpp               总头文件
include/dart_fc/core.hpp          参数、类型、坐标映射
include/dart_fc/control.hpp       状态机 / 导引 / 三回路 / 混控
include/dart_fc/hw.hpp            ICM-42688、AHRS
board/maixcam2/                   MaixCDK 工程（板上入口）
examples/                         PC 仿真（C++）
python_ref/                       旧 Python 草稿
```

回路：滚转指令恒 0，俯仰/偏航共用自动驾驶仪，饱和时保滚转。
