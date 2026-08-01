#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
build_root=${BUILD_ROOT:-/tmp/bleserial-offline-validation}
fqbn=${FQBN:-esp32:esp32:nano_nora}
ringbuffer_dir=${RINGBUFFER_DIR:-"$repo_root/../UUtzinger_RingBuffer"}
logger_dir=${LOGGER_DIR:-"$repo_root/../UUtzinger_logger"}

for dependency in "$ringbuffer_dir" "$logger_dir"; do
  if [[ ! -f "$dependency/library.properties" ]]; then
    printf 'Missing Arduino library dependency: %s\n' "$dependency" >&2
    exit 1
  fi
done

compile_example() {
  local name=$1
  local sketch=$2
  shift 2

  arduino-cli compile \
    --warnings all \
    --fqbn "$fqbn" \
    --library "$ringbuffer_dir" \
    --library "$logger_dir" \
    --library "$repo_root" \
    --build-path "$build_root/$name" \
    "$@" \
    "$repo_root/$sketch"
}

compile_example minimal examples/BLESerial_minimal
compile_example demo examples/BLESerial_demo
compile_example comprehensive examples/BLESerial_comprehensive
compile_example latency examples/BLESerial_latency
compile_example text-stress examples/BLESerial_text_stress
compile_example comprehensive-debug examples/BLESerial_comprehensive \
  --build-property build.extra_flags=-DDEBUG
