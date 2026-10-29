[![English](https://img.shields.io/badge/English-Docs-green)](VALIDATION.md)

# 构建与验证证据

## 记录环境

- WSL2 中的 Ubuntu 22.04
- GNU Make 4.3
- `arm-none-eabi-gcc` 10.3.1
- Python 3
- 仓库基线：上游 `main` 提交 `5fdef9a2dc83ee9b089171ed6f98c8c24915e351`

## 命令

```bash
python3 tools/validation/validate_repository.py
bash tools/validation/build_product_examples.sh
```

构建脚本对每个产品应用连续执行两轮干净构建，拒绝编译器诊断，要求每个应用恰好生成一个 BIN，并比较两轮的大小和 SHA-256。脚本退出时清理产物。

## 两轮构建结果

| 应用 | BIN 字节数 | SHA-256 |
|---|---:|---|
| `lora_factory_test` | 35,380 | `65e894047a0e85870c19db75407e7502414e503b363daf0c3409d5c2fb43b8cc` |
| `lora_myself_net` | 24,268 | `c10f06260f4e9dc6f757cf18bb949fba1251f6fe128b1281e9c635b03b34536b` |
| `lora_test` | 26,008 | `ee2a46abfd60a6d7173803035070a3994e2bb1913934b7c91ed13894c13aeb5d` |
| `pingpong` | 22,400 | `e352e6f7256a670aeca9621762d633e3b6bfac20d1a0dbf01b80bcb830cdcd72` |
| `Ra-08-kit-LoraTest` | 23,208 | `c1881ecda91c119df9f6ad80706aeb373df33ce27d4551dddbfe72e59ee931ab` |
| `class_a` | 38,660 | `c56743897ad3f289023bfacb5de1bbdc9a98bdb1c8a4004caaa0363a95cfdc97` |
| `class_c` | 38,484 | `ee886c480fd212b9d6cf199cff23ea05a586c5c64ee1effd86c1fc126c095898` |
| `lorawan_at` | 64,356 | `036a344b27898b6d8217ae46510a7b4c01f275dce2f8bf2b51d72c318d500da3` |

两轮均得到上表结果，且没有 GCC 编译器诊断。Class A 和 Class C 构建显式选择 `LORAMAC_REGION_CN470`。

## 为通过验证而完成的修正

- 将当前工具链不存在的私有头文件 `stdint-gcc.h` 替换为标准 `stdint.h`。
- 为 `lora_myself_net` 补充缺失声明，并修正头文件保护宏、格式和运算优先级问题。
- 明确 Class A/Class C 的区域定义。
- 移除两个产品镜像中的编译日期/时间字符串，使重复构建可复现。
- 修正环境脚本的 shebang。

## 自动化覆盖

GitHub Actions 在 Ubuntu 22.04 上运行文档/结构检查和相同的两轮产品构建。静态检查覆盖中英文文档配对、产品入口文件、必要公共构建文件、工具链子模块声明、本地 Markdown 链接和仓库卫生。

## 未验证项目

自动化过程不烧录硬件，也不声称已经验证启动、UART 行为、LoRa 收发、LoRaWAN 入网/互通、网关兼容、OTA、射频输出或灵敏度、功耗、距离、环境可靠性或法规符合性。其余 60 个非产品 Make 工程已识别，但未加入本次构建矩阵；这些项目需要单独的硬件测试记录。
