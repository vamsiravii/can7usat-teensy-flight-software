#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
export TREMO_SDK_PATH="$ROOT_DIR"

APPS=(
  "projects/ASR6601CB-EVAL/examples/lora/lora_factory_test"
  "projects/ASR6601CB-EVAL/examples/lora/lora_myself_net"
  "projects/ASR6601CB-EVAL/examples/lora/lora_test"
  "projects/ASR6601CB-EVAL/examples/lora/pingpong"
  "projects/ASR6601CB-EVAL/examples/lora/Ra-08-kit-LoraTest"
  "projects/ASR6601CB-EVAL/examples/lorawan/class_a"
  "projects/ASR6601CB-EVAL/examples/lorawan/class_c"
  "projects/ASR6601CB-EVAL/examples/lorawan/lorawan_at"
)

for command_name in arm-none-eabi-gcc arm-none-eabi-size make python3 sha256sum; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    echo "Missing required command: $command_name" >&2
    exit 2
  fi
done

declare -A FIRST_HASH
declare -A FIRST_SIZE
TEMP_DIR=$(mktemp -d)

cleanup() {
  for app in "${APPS[@]}"; do
    make -C "$ROOT_DIR/$app" clean >/dev/null 2>&1 || true
  done
  rm -rf "$TEMP_DIR"
}
trap cleanup EXIT

for run in 1 2; do
  echo "Build pass $run/2"
  for app in "${APPS[@]}"; do
    name=$(basename "$app")
    log_file="$TEMP_DIR/${name}-${run}.log"
    make -C "$ROOT_DIR/$app" clean >/dev/null
    if ! make -C "$ROOT_DIR/$app" -j2 2>&1 | tee "$log_file"; then
      echo "Build failed: $app" >&2
      exit 1
    fi
    if grep -Eqi ":[0-9]+:[0-9]+: (warning|error):" "$log_file"; then
      echo "Compiler diagnostics found in $app:" >&2
      grep -Ei ":[0-9]+:[0-9]+: (warning|error):" "$log_file" >&2
      exit 1
    fi

    mapfile -t bins < <(find "$ROOT_DIR/$app/out" -maxdepth 1 -type f -name "*.bin" -print)
    if [[ ${#bins[@]} -ne 1 ]]; then
      echo "Expected one BIN file for $app, found ${#bins[@]}" >&2
      exit 1
    fi
    size=$(stat -c %s "${bins[0]}")
    hash=$(sha256sum "${bins[0]}" | cut -d " " -f 1)

    if [[ $run -eq 1 ]]; then
      FIRST_SIZE["$app"]=$size
      FIRST_HASH["$app"]=$hash
    elif [[ ${FIRST_SIZE[$app]} != "$size" || ${FIRST_HASH[$app]} != "$hash" ]]; then
      echo "Non-reproducible output: $app" >&2
      exit 1
    fi
    printf '%s bytes=%s sha256=%s\n' "$app" "$size" "$hash"
  done
done

echo "All ${#APPS[@]} product examples built twice with identical binaries and no compiler diagnostics."
