#!/bin/bash
# Runs the PID tuner against simulated drivetrains on this computer (not the
# robot). Rerun it after changing anything in src/tuner.cpp.
set -euo pipefail
cd "$(dirname "$0")/.."
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT

clang++ -std=c++20 -D_PROS_INCLUDE_LIBLVGL_LLEMU_HPP -Iinclude -Isrc \
    -c test/tuner_sim.cpp -o "$out/sim.o"

# The tuner references robot hardware (motors, chassis, tasks) the sim never
# calls. Give each of those symbols a placeholder so it links.
nm -u "$out/sim.o" | grep -E 'pros|lemlib|chassis|Motors|controller|driveInches|turnDegrees|applyDriverBrakes|driverBrakeHold' \
    | grep -v '_millis$\|_delay$' > "$out/wanted"
nm "$out/sim.o" | awk '$2 ~ /[TtDdBbSs]/ {print $3}' > "$out/defined"
grep -vxFf "$out/defined" "$out/wanted" | while read -r sym; do
    printf 'char stub_%s[8192] __asm__("%s");\n' "$(echo "$sym" | tr -c 'A-Za-z0-9_\n' '_')" "$sym"
done > "$out/stubs.c"

clang -c "$out/stubs.c" -o "$out/stubs.o"
clang++ "$out/sim.o" "$out/stubs.o" -o "$out/sim"
"$out/sim" | grep -E 'SUMMARY|drivetrains|passes'
