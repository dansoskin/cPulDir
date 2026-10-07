#!/usr/bin/env bash
# Host tests for puldir. No HAL, no hardware. Run from bash: bash test/run.sh
set -euo pipefail
cd "$(dirname "$0")/.."
OUT="build/test"
mkdir -p "$OUT"
CFLAGS="-std=c99 -O1 -Wall -Wextra -Werror -Wdouble-promotion -Wshadow -I."

gcc $CFLAGS -o "$OUT/test_motion_profile" test/test_motion_profile.c motion_profile.c -lm
"$OUT/test_motion_profile"
echo "generator: all ok"
