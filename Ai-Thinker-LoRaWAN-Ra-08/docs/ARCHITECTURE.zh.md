[![English](https://img.shields.io/badge/English-Docs-green)](ARCHITECTURE.md)

# 架构说明

## 分层关系

```text
应用工程（projects/ASR6601CB-EVAL）
  |-- LoRa 产品应用
  |-- LoRaWAN Class A / Class C / AT 应用
  `-- 外设、OTA 和模板示例
             |
             v
协议与中间件组件（components/）
  |-- LoRaWAN MAC、区域参数和工具组件
  `-- 应用本地的无线/网络组件
             |
             v
平台服务（platform/）
  |-- 无线抽象、定时器、RTC 和板级服务
  `-- 启动、系统初始化和标准输入输出支持
             |
             v
ASR6601 设备驱动（drivers/）
  `-- RCC、GPIO、UART、电源、定时器、密码算法及其他外设
             |
             v
ASR6601CB 硬件 / Ra-08 或 Ra-08H 模组
```

## 构建架构

每个应用的 `Makefile` 定义源码、包含路径、宏、链接脚本和输出名称，然后包含 `build/make/common.mk`。公共 Make 层选择 `arm-none-eabi-*` 工具，为 Cortex-M4F 编译目标文件，使用工程链接脚本生成 ELF，再转换为 HEX 和 BIN。`build/envsetup.sh` 将仓库根目录导出为 `TREMO_SDK_PATH`。

启动路径是 `platform/system/startup_cm4.S` 中的 `Reset_Handler`，随后进入所选应用的 `main()`。后续根据工程进入 LoRa 无线状态机、LoRaWAN MAC 状态机、AT 固件状态机或外设示例。

## 无线与事件流

- LoRa 应用注册 `RadioEvents` 回调、配置无线参数，并在前台循环调用 `Radio.IrqProcess()`。
- Class A 和 Class C 初始化 `LoRaMac`，交换 MLME/MCPS 请求与指示，并在应用状态机中处理无线事件。
- `lorawan_at` 在 LoRaWAN/应用回调之上提供面向 AT 的状态机。
- `lora_myself_net` 将轮询协议放在应用本地的 `lora_driver` 和 `lora_net` 组件中。

## 外部与二进制边界

- `tools/toolchain` 是托管在 GitHub 之外的可选固定版本子模块；也可使用系统 GNU Arm 工具链完成构建。
- `drivers/crypto/lib/libcrypto.a` 是预编译库，本仓库无法提供该二进制的源码级可见性。
- Windows 烧录程序和 Python 烧录脚本属于主机工具，不是固件运行时组件。
- 仓库没有统一根许可证，各组件声明必须分别评估。

## 变更影响

修改 `drivers/`、`platform/`、共享 `components/` 或 `build/` 可能影响多个应用。应用目录内的修改通常较局部，但最终镜像仍由其 Makefile 和共享依赖共同决定。修改共享层后，至少应重新运行静态检查和全部 8 个产品工程构建。
