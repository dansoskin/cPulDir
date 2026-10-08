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

LIB="motion_profile.c puldir.c puldir_pulse.c puldir_plan.c puldir_loop.c puldir_group.c puldir_console.c"
build_axis() {   # $1 = output name, rest = extra flags
    local out="$1"; shift
    gcc $CFLAGS -Itest "$@" -o "$OUT/$out" test/test_puldir.c test/fake_port.c $LIB -lm
    local cases
    cases=$("$OUT/$out" list | tr -d '\r')   # one process per case (D9); a failing list stops the script
    [ -n "$cases" ]
    for c in $cases; do "$OUT/$out" "$c"; done
}
build_axis test_puldir
build_axis test_puldir_5us -DPULDIR_PULSE_US=5u   # a 200 kHz tick
build_axis test_puldir_2k -DPULDIR_PULSE_US=10u -DPULDIR_PLAN_HZ=2000u   # the fastest plan rate D14 allows
build_axis test_puldir_wrap -DPULDIR_TICK_START=0xFFFFFC18u   # 1,000 ticks below the wrap (D9, test 22)
echo "axis: all ok"
