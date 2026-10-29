[![English](https://img.shields.io/badge/English-README-green)](readme.md)

# 安信可 LoRaWAN Ra-08 SDK

本仓库是安信可 Ra-08 和 Ra-08H 模组的 SoC 二次开发 SDK，目标芯片为 ASR6601CB，集成 32 位 48 MHz Arm Cortex-M4、128 KiB Flash 和 16 KiB SRAM。仓库包含外设示例、LoRa 点对点应用、LoRaWAN Class A/Class C/AT 应用、OTA 示例及上位机烧录工具。

## 从这里开始

- [代码入口](docs/CODE_ENTRY.zh.md)
- [架构说明](docs/ARCHITECTURE.zh.md)
- [构建与验证证据](docs/VALIDATION.zh.md)
- [固件变更记录](release_note_all.txt)
- [轮询示例](projects/ASR6601CB-EVAL/examples/lora/lora_myself_net/README.zh.md)

## 产品相关示例

| 分类 | 应用 | 用途 |
|---|---|---|
| LoRa | `lora_factory_test` | 产测/射频指令测试固件 |
| LoRa | `lora_myself_net` | 网关/节点轮询演示 |
| LoRa | `lora_test` | 定频射频测试固件 |
| LoRa | `pingpong` | 点对点 Ping/Pong 示例 |
| LoRa | `Ra-08-kit-LoraTest` | 开发板按键/LED 无线测试 |
| LoRaWAN | `class_a` | LoRaWAN Class A 示例 |
| LoRaWAN | `class_c` | LoRaWAN Class C 示例 |
| LoRaWAN | `lorawan_at` | LoRaWAN AT 应用；Bootloader/应用地址限制见变更记录 |

仓库还包含 60 个外设、密码算法、功耗、定时器、UART、OTA 和模板 Make 工程；它们属于平台示例，不是本次重点验证的 Ra-08 产品应用集合。

## 构建环境

请使用 Linux 或 WSL2，并安装 GNU Make、Python 3 和 GNU Arm Embedded 工具链。已验证环境为 Ubuntu 22.04、Make 4.3 和 `arm-none-eabi-gcc` 10.3.1。

构建单个示例：

```bash
source build/envsetup.sh
cd projects/ASR6601CB-EVAL/examples/lora/pingpong
make -j2
```

连续两轮构建全部 8 个产品工程并核对二进制一致性：

```bash
bash tools/validation/build_product_examples.sh
```

产物写入各应用的 `out/` 目录，并被 Git 忽略。

## 工具链子模块

`tools/toolchain` 是可选的固定版本子模块，保存历史工具链发行内容。系统中的 `arm-none-eabi-gcc` 优先使用，本次 Linux 构建也使用系统工具链。需要子模块时显式执行：

```bash
git submodule update --init --recursive
```

## 烧录

`make flash` 会调用 `build/scripts/tremo_loader.py`，串口和波特率来自 `build/make/common.mk` 的 `SERIAL_PORT` 与 `SERIAL_BAUDRATE`。执行前必须确认串口、固件地址和目标硬件。自动验证不会执行 `make flash`。

## 验证边界

8 个主要固件已完成可重复编译和链接，但这不等于启动、串口、LoRa/LoRaWAN 互通、网关入网、OTA、射频性能、功耗、距离或法规验证；这些项目必须使用目标硬件和合适仪器完成。

## 许可证状态

仓库没有统一的根许可证文件。不同源码和内嵌组件保留各自声明，其中包括 Semtech Revised BSD 内容及第三方/预编译库。再分发前应逐项核对，不能假定一个许可证覆盖整个仓库。
