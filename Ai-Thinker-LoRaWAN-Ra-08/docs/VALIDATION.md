[![中文](https://img.shields.io/badge/中文-文档-blue)](VALIDATION.zh.md)

# Build and validation evidence

## Recorded environment

- Ubuntu 22.04 under WSL2
- GNU Make 4.3
- `arm-none-eabi-gcc` 10.3.1
- Python 3
- Repository baseline: upstream `main` commit `5fdef9a2dc83ee9b089171ed6f98c8c24915e351`

## Commands

```bash
python3 tools/validation/validate_repository.py
bash tools/validation/build_product_examples.sh
```

The build script performs a clean build of every product application twice, rejects compiler diagnostics, requires exactly one BIN per application, and compares the size and SHA-256 digest between passes. Outputs are cleaned when the script exits.

## Two-pass build results

| Application | BIN bytes | SHA-256 |
|---|---:|---|
| `lora_factory_test` | 35,380 | `65e894047a0e85870c19db75407e7502414e503b363daf0c3409d5c2fb43b8cc` |
| `lora_myself_net` | 24,268 | `c10f06260f4e9dc6f757cf18bb949fba1251f6fe128b1281e9c635b03b34536b` |
| `lora_test` | 26,008 | `ee2a46abfd60a6d7173803035070a3994e2bb1913934b7c91ed13894c13aeb5d` |
| `pingpong` | 22,400 | `e352e6f7256a670aeca9621762d633e3b6bfac20d1a0dbf01b80bcb830cdcd72` |
| `Ra-08-kit-LoraTest` | 23,208 | `c1881ecda91c119df9f6ad80706aeb373df33ce27d4551dddbfe72e59ee931ab` |
| `class_a` | 38,660 | `c56743897ad3f289023bfacb5de1bbdc9a98bdb1c8a4004caaa0363a95cfdc97` |
| `class_c` | 38,484 | `ee886c480fd212b9d6cf199cff23ea05a586c5c64ee1effd86c1fc126c095898` |
| `lorawan_at` | 64,356 | `036a344b27898b6d8217ae46510a7b4c01f275dce2f8bf2b51d72c318d500da3` |

Both passes produced the values above with no GCC compiler diagnostics. The Class A and Class C builds explicitly select `LORAMAC_REGION_CN470`.

## Corrections required for the verified build

- Replaced the unavailable private GCC header `stdint-gcc.h` with standard `stdint.h`.
- Added missing declarations and corrected header guards and format/precedence defects in `lora_myself_net`.
- Made the Class A/Class C regional definition explicit.
- Removed compile-date/time strings from two product images so repeat builds are deterministic.
- Corrected the environment script shebang.

## Automated coverage

The GitHub Actions workflow runs the documentation/structure validator and the same two-pass product build on Ubuntu 22.04. The static check verifies bilingual document pairs, product entry files, required shared build files, the toolchain submodule declaration, local Markdown links and repository hygiene.

## Not verified

No automated step flashes hardware or claims evidence for boot, UART behavior, LoRa packet exchange, LoRaWAN join/interoperability, gateway compatibility, OTA, RF output or sensitivity, power consumption, range, environmental reliability or regulatory compliance. The remaining 60 non-product Make projects were identified but not included in this build matrix. These items require separate hardware test records.
