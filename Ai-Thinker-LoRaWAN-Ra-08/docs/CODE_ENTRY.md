[![中文](https://img.shields.io/badge/中文-文档-blue)](CODE_ENTRY.zh.md)

# Code entry points

## Reset-to-application path

The GNU linker scripts declare `ENTRY(Reset_Handler)`. `platform/system/startup_cm4.S` initializes the runtime and calls each project's `main()` function. Every application below has its own `Makefile`, linker script under `cfg/`, and `src/main.c`.

## Primary product applications

| Application | C entry and next stage | Runtime responsibility |
|---|---|---|
| `lora_factory_test` | `src/main.c:main()` -> `SimpleCmdStart()` in `src/simple_cmd.c` | Initializes the board and runs the manufacturing/RF command loop, including radio IRQ processing. |
| `lora_myself_net` | `src/main.c:main()` -> `Ra08KitLoraTestStart()` in `components/lora_driver/lora_driver.c` | Initializes the board, radio callbacks and timer-driven gateway/node polling logic. |
| `lora_test` | `src/main.c:main()` -> `tc_lora_test()` in `src/lora_test.c` | Runs selectable continuous-wave, transmit, receive and CAD-oriented RF tests. |
| `pingpong` | `src/main.c:main()` -> `app_start()` in `src/pingpong.c` | Configures radio callbacks and alternates the Ping/Pong state machine while processing radio IRQs. |
| `Ra-08-kit-LoraTest` | `src/main.c:main()` -> `Ra08KitLoraTestStart()` in `src/Ra08KitLoraTest.c` | Runs the development-board key/LED radio test and radio IRQ loop. |
| `class_a` | `src/main.c:main()` -> `app_start()` in `src/classA.c` | Initializes LoRaMac for CN470, performs join/send state handling and processes MAC/radio events. |
| `class_c` | `src/main.c:main()` -> `app_start()` in `src/classC.c` | Initializes LoRaMac for CN470 and runs Class C join, send and receive processing. |
| `lorawan_at` | `src/main.c:main()` -> `lora_init()` -> `lora_fsm()` | Initializes the AT/LoRaWAN callback layer and enters its firmware state machine. |

Paths in the table are relative to `projects/ASR6601CB-EVAL/examples/lora/` or `projects/ASR6601CB-EVAL/examples/lorawan/`, according to the application category.

## Other examples

The repository contains 60 additional Make projects for peripherals, crypto, power, timers, UART, OTA and templates. Their local entry is likewise `src/main.c`, but they are outside the eight-product build matrix documented here.

## Modification checklist

1. Start at the selected application's `src/main.c` and follow the next-stage function listed above.
2. Review its `Makefile` for source lists, include paths, compile definitions and linker script.
3. Keep interrupt and radio-event work in the existing callback/IRQ path rather than blocking inside an interrupt.
4. Run the repository validator and the two-pass build before submitting a change.
