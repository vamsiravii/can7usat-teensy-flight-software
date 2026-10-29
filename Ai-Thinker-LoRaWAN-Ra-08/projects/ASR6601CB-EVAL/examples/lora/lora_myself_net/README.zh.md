[![English](https://img.shields.io/badge/English-Docs-green)](README.md)

# Ra-08 轮询示例

该应用演示一个简单的 LoRa 网关/节点轮询网络。

## 选择角色

修改 `components/lora_driver/include/lora_config.h`：

```c
#define CONFIG_GATEWAY (1)   /* 网关 */
/* #define CONFIG_GATEWAY (1) */ /* 节点 */
```

在仓库环境中构建：

```bash
source build/envsetup.sh
cd projects/ASR6601CB-EVAL/examples/lora/lora_myself_net
make -j2
```

## 网关指令

以下数据帧把轮询节点数设置为 30：

```text
A0 F2 1E 01 11 22 85 A1
```

`A0` 和 `A1` 是帧边界，`F2` 表示轮询配置，`1E` 是节点数量，`85` 是校验和；其余字节在当前示例中保留。

典型状态信息包括 `ADDR x get ack timeout` 和 `Get data from: 1, set next call node: 2, polling nodes num: 2`。

## 节点指令

以下数据帧设置节点地址：

```text
A0 1E 00 00 00 00 A1
```

地址为 8 位无符号数。收到匹配请求后，日志会输出帧长度、本机/接收地址及载荷。

## 烧录与调试边界

历史 Windows 烧录工具位于 `tools/programmer/`。请按开发板文档进入下载模式，确认固件和串口后再擦除或写入设备。自动检查只构建网关/节点源码配置，不会操作烧录器、串口、LED 或射频硬件。

仓库相关问题请提交到 [Ai-Thinker LoRaWAN Ra-08 Issues](https://github.com/Ai-Thinker-Open/Ai-Thinker-LoRaWAN-Ra-08/issues)。
