[![中文](https://img.shields.io/badge/中文-README-blue)](readme.zh.md)

# Ai-Thinker LoRaWAN Ra-08 SDK

This repository is the secondary-development SDK for Ai-Thinker Ra-08 and Ra-08H modules based on the ASR6601CB SoC. The target integrates a 32-bit 48 MHz Arm Cortex-M4, 128 KiB Flash and 16 KiB SRAM. The tree contains peripheral examples, LoRa point-to-point applications, LoRaWAN Class A/Class C/AT applications, OTA examples and host-side programming tools.

## Start here

- [Code entry points](docs/CODE_ENTRY.md)
- [Architecture](docs/ARCHITECTURE.md)
- [Build and validation evidence](docs/VALIDATION.md)
- [Firmware change history](release_note_all.txt)
- [Polling example](projects/ASR6601CB-EVAL/examples/lora/lora_myself_net/README.md)

## Product examples

| Category | Application | Purpose |
|---|---|---|
| LoRa | `lora_factory_test` | Manufacturing/RF command test firmware |
| LoRa | `lora_myself_net` | Gateway/node polling demonstration |
| LoRa | `lora_test` | Fixed-frequency RF test firmware |
| LoRa | `pingpong` | Point-to-point Ping/Pong example |
| LoRa | `Ra-08-kit-LoraTest` | Development-board key/LED radio test |
| LoRaWAN | `class_a` | LoRaWAN Class A example |
| LoRaWAN | `class_c` | LoRaWAN Class C example |
| LoRaWAN | `lorawan_at` | LoRaWAN AT application; see release notes for bootloader/app layout constraints |

The repository also includes 60 peripheral, crypto, power, timer, UART, OTA and template Make projects. They are platform examples rather than the primary Ra-08 application set audited here.

## Build environment

Use Linux or WSL2 with GNU Make, Python 3 and the GNU Arm Embedded toolchain. The validated environment uses Ubuntu 22.04, Make 4.3 and `arm-none-eabi-gcc` 10.3.1.

To build one example:

```bash
source build/envsetup.sh
cd projects/ASR6601CB-EVAL/examples/lora/pingpong
make -j2
```

To build all eight product examples twice and verify identical binaries:

```bash
bash tools/validation/build_product_examples.sh
```

Build output is written to each application's `out/` directory and is ignored by Git.

## Toolchain submodule

`tools/toolchain` is an optional pinned submodule containing the historical toolchain distribution. A system `arm-none-eabi-gcc` takes precedence and was used for the recorded Linux build. If the submodule is needed, initialize it explicitly:

```bash
git submodule update --init --recursive
```

## Flashing

`make flash` invokes `build/scripts/tremo_loader.py` using `SERIAL_PORT` and `SERIAL_BAUDRATE` from `build/make/common.mk`. Verify the serial port, image address and target hardware before using it. The automated validation never runs `make flash`.

## Validation boundary

The eight primary firmware images compile and link reproducibly. This does not verify boot, serial I/O, LoRa/LoRaWAN interoperability, gateway join, OTA, RF performance, power consumption, range or regulatory compliance. Those items require target hardware and suitable test equipment.

## License status

The repository has no single root license file. Individual files and bundled components contain their own notices, including Semtech Revised BSD material and third-party/prebuilt libraries. Review the notice for every component before redistribution; do not assume one license covers the entire repository.
