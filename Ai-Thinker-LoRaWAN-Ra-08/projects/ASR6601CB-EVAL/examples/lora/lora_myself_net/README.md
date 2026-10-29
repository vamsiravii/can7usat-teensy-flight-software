[![中文](https://img.shields.io/badge/中文-文档-blue)](README.zh.md)

# Ra-08 polling example

This application demonstrates a simple LoRa gateway/node polling network.

## Select a role

Edit `components/lora_driver/include/lora_config.h`:

```c
#define CONFIG_GATEWAY (1)   /* gateway */
/* #define CONFIG_GATEWAY (1) */ /* node */
```

Build from the repository root environment:

```bash
source build/envsetup.sh
cd projects/ASR6601CB-EVAL/examples/lora/lora_myself_net
make -j2
```

## Gateway command

The following frame configures 30 polled nodes:

```text
A0 F2 1E 01 11 22 85 A1
```

`A0` and `A1` are the frame boundaries, `F2` selects polling configuration, `1E` is the node count and `85` is the checksum. Remaining bytes are reserved by the current example.

Typical status messages include `ADDR x get ack timeout` and `Get data from: 1, set next call node: 2, polling nodes num: 2`.

## Node command

The following frame assigns a node address:

```text
A0 1E 00 00 00 00 A1
```

The address is an unsigned 8-bit value. A matching request produces a receive log containing the frame length, local/received address and payload.

## Flash and debug boundary

The historical Windows programmer is under `tools/programmer/`. Enter download mode according to the board documentation, verify the selected image and port, and only then erase or program the device. Automated checks build the gateway/node source configuration but do not operate the programmer, serial port, LEDs or radio hardware.

Report repository-specific problems at the [Ai-Thinker LoRaWAN Ra-08 issue tracker](https://github.com/Ai-Thinker-Open/Ai-Thinker-LoRaWAN-Ra-08/issues).
