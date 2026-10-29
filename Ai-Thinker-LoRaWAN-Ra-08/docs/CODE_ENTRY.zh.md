[![English](https://img.shields.io/badge/English-Docs-green)](CODE_ENTRY.md)

# 代码入口

## 从复位到应用

GNU 链接脚本通过 `ENTRY(Reset_Handler)` 声明入口。`platform/system/startup_cm4.S` 初始化运行环境后调用各工程的 `main()`。下列应用都带有独立的 `Makefile`、`cfg/` 下的链接脚本以及 `src/main.c`。

## 主要产品应用

| 应用 | C 入口与下一阶段 | 运行职责 |
|---|---|---|
| `lora_factory_test` | `src/main.c:main()` -> `src/simple_cmd.c` 的 `SimpleCmdStart()` | 初始化硬件并运行产测/射频命令循环，其中包含无线中断处理。 |
| `lora_myself_net` | `src/main.c:main()` -> `components/lora_driver/lora_driver.c` 的 `Ra08KitLoraTestStart()` | 初始化硬件、无线回调和定时器驱动的网关/节点轮询逻辑。 |
| `lora_test` | `src/main.c:main()` -> `src/lora_test.c` 的 `tc_lora_test()` | 运行可选择的连续波、发送、接收和 CAD 类射频测试。 |
| `pingpong` | `src/main.c:main()` -> `src/pingpong.c` 的 `app_start()` | 配置无线回调，运行 Ping/Pong 状态机并处理无线中断。 |
| `Ra-08-kit-LoraTest` | `src/main.c:main()` -> `src/Ra08KitLoraTest.c` 的 `Ra08KitLoraTestStart()` | 运行开发板按键/LED 无线测试和中断处理循环。 |
| `class_a` | `src/main.c:main()` -> `src/classA.c` 的 `app_start()` | 以 CN470 初始化 LoRaMac，处理入网、发送及 MAC/无线事件。 |
| `class_c` | `src/main.c:main()` -> `src/classC.c` 的 `app_start()` | 以 CN470 初始化 LoRaMac，运行 Class C 入网、发送和接收流程。 |
| `lorawan_at` | `src/main.c:main()` -> `lora_init()` -> `lora_fsm()` | 初始化 AT/LoRaWAN 回调层并进入固件状态机。 |

表中路径按应用类别分别相对于 `projects/ASR6601CB-EVAL/examples/lora/` 或 `projects/ASR6601CB-EVAL/examples/lorawan/`。

## 其他示例

仓库另有 60 个外设、密码算法、功耗、定时器、UART、OTA 和模板 Make 工程。它们也从各自的 `src/main.c` 进入，但不属于本文记录的 8 个产品工程构建矩阵。

## 修改检查清单

1. 从目标应用的 `src/main.c` 开始，按上表继续跟踪下一阶段函数。
2. 检查对应 `Makefile` 中的源码列表、包含路径、编译宏和链接脚本。
3. 中断和无线事件继续沿用现有回调/IRQ 路径，避免在中断中阻塞。
4. 提交前运行仓库检查和两轮构建。
