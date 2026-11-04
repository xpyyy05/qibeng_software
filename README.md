# qibeng_software

基于STM32G030F6P6的鱼缸气泵控制固件：四档 PWM 调速、WS2812 状态灯、电池电压监测与欠压保护、充电状态检测，以及基于 PA0 唤醒的低功耗待机。

仓库包含完整的 CubeMX 工程（`.ioc`）、Keil MDK 工程和业务源码，克隆后可直接用 Keil 打开编译，无需重新生成代码。

## 功能

**四档运行 + 停止**：0 档停止，1 档静音（约 16.5%），2 档约 33%，3 档约 66%，4 档 100%。

**反相 PWM 调速**：TIM1_CH4，20 kHz；档位切换使用约 10 CCR/ms 的非阻塞软启动斜坡。

**电池监测**：ADC1 + 循环 DMA 连续采样，使用 VREFINT 与出厂校准值实时修正 VDDA，8 次平均。

**欠压保护**：低于 2.8 V 持续 500 ms 进入欠压，高于等于 3.0 V 持续 500 ms 恢复（200 mV 滞回）；欠压时电池供电禁止电机。

**充电状态检测**：`N_CHRG` / `N_DONE` 两路低有效信号，解析为电池供电、充电中、已充满等状态。

**WS2812 指示**：SPI2 10 Mbit/s 编码驱动单颗 WS2812C，GRB 顺序，默认亮度 20%。

**低功耗**：连续 15 秒无操作且满足条件时进入 Standby，PA0（WKUP1）按键唤醒，唤醒后按复位启动并保持 0 档。

**最高档限时**：4 档连续运行 5 分钟后自动退回 3 档。

## 硬件

| 项目 | 说明 |
|---|---|
| MCU | STM32G030F6P6（Cortex-M0+，32 KB Flash / 8 KB RAM） |
| 指示灯 | WS2812C-2020-V6（立创编号 C55109522） |
| 电池 | 单节磷酸铁锂电池，PA1 经 1/2 电阻分压采样 |
| 电机 | 外置升压器 + 气泵电机，PA5 使能升压器，PA11 输出反相 PWM |

### 引脚

| CubeMX 标签 | 引脚 | 方向 / 外设 | 说明 |
|---|---|---|---|
| `KEY_EXTI` / `SYS_WKUP1` | PA0 | GPIO EXTI / WKUP1 | 内部下拉，上升沿有效；待机前切换为 WKUP1 |
| `ADC1_IN1` | PA1 | ADC1 输入 | 电池电压，1/2 分压 |
| `WS2812_DATA` | PA4 | SPI2_MOSI | WS2812 数据输出 |
| `BOOST_EN` | PA5 | GPIO 输出 | 高电平开启升压器 |
| `N_CHRG` | PA6 | GPIO 输入 | 低电平表示正在充电 |
| `N_DONE` | PA7 | GPIO 输入 | 低电平表示充电完成 |
| `WS2812_EN` | PA8 | GPIO 输出 | 高电平使能 WS2812 供电 |
| `N_MOTOR_PWM` | PA11 | TIM1_CH4 | 反相调速：CCR=999 停止，CCR=0 全速 |
| `SYS_SWDIO` | PA13 | SWD | 调试接口 |
| `SYS_SWCLK` | PA14 | SWD | 调试接口 |
| `SPI2_SCK` | PB8 | SPI2_SCK | WS2812 SPI 时钟 |

## 目录结构

```text
.
├── Core/
│   ├── Inc/                 # 应用与外设头文件
│   └── Src/                 # 应用逻辑与 CubeMX 生成的外设初始化
├── Drivers/
│   ├── CMSIS/               # Arm CMSIS（Apache-2.0）
│   └── STM32G0xx_HAL_Driver/# ST HAL 驱动（BSD-3-Clause）
├── MDK-ARM/                 # Keil MDK 工程
├── test/                    # 硬件自测代码（电机 / WS2812）
├── qibeng_software.ioc      # CubeMX 工程配置
├── stm32g030_air_pump_agent_spec.md  # 设计说明与实现记录
├── LICENSE
└── README.md
```

## 编译

1. 安装 **Keil MDK 5.x**（使用 Arm Compiler 5）。
2. 在 Keil Pack Installer 中安装 **Keil.STM32G0xx_DFP 1.4.0**。
3. 打开 [MDK-ARM/qibeng_software.uvprojx](MDK-ARM/qibeng_software.uvprojx)。
4. 选择目标并编译（F7）：

| Target | 用途 |
|---|---|
| `qibeng_software` | 主固件 |
| `qibeng_software_TestM` | 电机测试：0 → 16.5% → 33% → 66% → 100%，每档 3 秒 |
| `qibeng_software_TestWS` | WS2812 测试：红 → 绿 → 蓝 → 白循环，每色 500 ms |
| `qibeng_software_TestRed` | 红灯测试 |

编译产物位于 `MDK-ARM/<target>/<target>.hex`。Release 页面会提供预编译的 hex 文件。

## 使用

**按键**：短按循环切换 `0 → 1 → 2 → 3 → 4 → 0`。
**指示灯**：
插电供电且正常：绿色常亮；
电池供电且正常：蓝色常亮；
电池供电且确认欠压：红色常亮；
`N_CHRG` 与 `N_DONE` 同时为低（异常组合）：橙色；
1～4 档以 0.25 / 0.5 / 1 / 2 Hz 闪烁表示档位，0 档常亮。
**休眠**：连续 15 秒无按键操作且未插电时进入 Standby；按 PA0 唤醒，唤醒后为 0 档。
**欠压**：电池供电且确认欠压时强制 0 档并禁止电机；插电供电时不受此限制。

## CubeMX 注意事项

`qibeng_software.ioc` 是外设配置的唯一依据。用 CubeMX 重新生成代码前，请确认 **SPI2 DataSize 保持 8 bit**。若变为 16 bit，WS2812 的跨字节 SPI 编码会被打乱，颜色异常。
PA0 在运行时为 GPIO EXTI（下拉、上升沿），仅在进入低功耗前切换为 WKUP1。
**STM32G030 不支持 Shutdown 模式**（未定义 `PWR_SHDW_SUPPORT`），本工程使用 **Standby** 作为最低功耗状态，唤醒行为与 Shutdown 等价。

## 开发状态

主要功能已在实机验证。最近一批改动（静音档 16.5%、4 档 5 分钟自动退档、指示灯 20% 亮度）。详细设计约束与变更记录见 [stm32g030_air_pump_agent_spec.md](stm32g030_air_pump_agent_spec.md)。

## 许可证

- 本项目业务代码（`Core/Src` 中的 `app.c`、`battery.c`、`motor.c`、`power.c`、`ws2812.c` 及对应头文件、`test/`）采用 **MIT License**，见 [LICENSE](LICENSE)。
- `Drivers/STM32G0xx_HAL_Driver` 为 STMicroelectronics 的 **BSD-3-Clause** 许可，`Drivers/CMSIS` 为 Arm 的 **Apache-2.0** 许可，分别见各自目录下的 `LICENSE.txt`。
- CubeMX 生成的骨架文件保留 ST 原始版权头，按其自带许可使用。

## 贡献

欢迎提交 Issue 和 Pull Request。提交前请：

1. 保持 `.ioc` 与代码中的外设配置一致；
2. 说明改动涉及的硬件条件（供电方式、电池电压、电机档位）；
3. 若改动了时序、引脚或低功耗逻辑，请同步更新设计文档。

## 免责声明

本固件用于控制气泵电机与升压电路，涉及电池和较高电压。使用、改装或量产前请自行完成电气安全评估；作者不对因使用本软件造成的设备损坏或人身伤害承担责任。
