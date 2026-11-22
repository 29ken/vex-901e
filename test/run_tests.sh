#!/bin/bash
# Runs the off-robot checks on this computer: the lift (presets, bottom limit,
# re-zeroing, top and backwards-sensor detection) against a simulated winch.
# Rerun after changing include/lift.hpp.
set -uo pipefail
cd "$(dirname "$0")/.."
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT
status=0

clang++ -std=c++20 -Iinclude test/lift_check.cpp -o "$out/lift" && "$out/lift" || status=1
exit $status
