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
}
run_axis() {     # $1 = build, $2 $3 = pulse_us plan_us; one process per case (D9)
    local cases
    cases=$("$OUT/$1" list | tr -d '\r')   # a failing list stops the script
    [ -n "$cases" ]
    for c in $cases; do "$OUT/$1" "$c" "$2" "$3"; done
}
build_axis test_puldir
build_axis test_puldir_wrap -DPULDIR_TICK_START=0xFFFFFC18u   # 1,000 ticks below the wrap (D9, test 22)
run_axis test_puldir 20 1000        # a 50 kHz tick, the default
run_axis test_puldir 5 1000         # a 200 kHz tick
run_axis test_puldir 10 500         # a 2 kHz plan rate on a 100 kHz tick
run_axis test_puldir_wrap 20 1000
echo "axis: all ok"
