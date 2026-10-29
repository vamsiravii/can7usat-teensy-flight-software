[![中文](https://img.shields.io/badge/中文-文档-blue)](ARCHITECTURE.zh.md)

# Architecture

## Layer map

```text
Application projects (projects/ASR6601CB-EVAL)
  |-- LoRa product applications
  |-- LoRaWAN Class A / Class C / AT applications
  `-- peripheral, OTA and template examples
             |
             v
Protocol and middleware components (components/)
  |-- LoRaWAN MAC, regional parameters and utilities
  `-- application-local radio/network components
             |
             v
Platform services (platform/)
  |-- radio abstraction, timers, RTC and board services
  `-- startup, system initialization and standard I/O support
             |
             v
ASR6601 device drivers (drivers/)
  `-- RCC, GPIO, UART, power, timers, crypto and other peripherals
             |
             v
ASR6601CB hardware / Ra-08 or Ra-08H module
```

## Build architecture

Each application `Makefile` defines its source files, include paths, macros, linker script and output name, then includes `build/make/common.mk`. The shared make layer selects `arm-none-eabi-*`, compiles Cortex-M4F objects, links an ELF with the project linker script, and converts it to HEX and BIN. `build/envsetup.sh` exports the repository root as `TREMO_SDK_PATH`.

The startup path is `Reset_Handler` in `platform/system/startup_cm4.S` followed by the selected application's `main()`. Runtime behavior then branches into a LoRa radio state machine, LoRaWAN MAC state machine, AT firmware state machine or peripheral example.

## Radio and event flow

- LoRa applications install `RadioEvents` callbacks, configure the radio and call `Radio.IrqProcess()` from their foreground loop.
- Class A and Class C initialize `LoRaMac`, exchange MLME/MCPS requests and indications, and process radio events in the application state machine.
- `lorawan_at` places an AT-facing state machine over the LoRaWAN/application callbacks.
- `lora_myself_net` keeps its polling protocol in application-local `lora_driver` and `lora_net` components.

## External and binary boundaries

- `tools/toolchain` is an optional pinned submodule hosted outside GitHub; builds can instead use a system GNU Arm toolchain.
- `drivers/crypto/lib/libcrypto.a` is precompiled, so this repository does not provide source-level visibility for that binary.
- Windows programming executables and the Python flashing script are host tools, not firmware runtime components.
- The repository has no single root license; notices must be evaluated per component.

## Change impact

A change in `drivers/`, `platform/`, shared `components/` or `build/` can affect many applications. A change below one application directory is usually more localized, but its Makefile and shared dependencies still determine the final image. At minimum, rerun the static validator and all eight product builds after changing a shared layer.
