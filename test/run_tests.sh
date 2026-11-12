#!/bin/bash
# Runs the off-robot checks on this computer: the lift (presets, bottom limit,
# re-zeroing, top and backwards-sensor detection) against a simulated winch, and
# the PTO (shifting, joining the lift, slips) against a simulated PTO. Rerun
# after changing include/lift.hpp or include/pto.hpp.
set -uo pipefail
cd "$(dirname "$0")/.."
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT
status=0

clang++ -std=c++20 -Iinclude test/lift_check.cpp -o "$out/lift" && "$out/lift" || status=1
clang++ -std=c++20 -Iinclude test/pto_check.cpp -o "$out/pto" && "$out/pto" || status=1
exit $status
