#!/bin/sh
# usage: OUT=<a folder for the output> run.sh <label>   (Linux; the release
# test binary built with the driver registered in crates/godot-ext/tests/suite.rs)
# M74_4=1, SPECS or SPECS_FILE, WARM, GAP, RUN, CHECKS, JOBS pass through.
: "${OUT:?set OUT to a folder for <label>.out and <label>.err}"
ROOT=$(cd "$(dirname "$0")/../../../.." && pwd)
EXE=$(ls -t "$ROOT"/target/release/deps/suite-* | grep -v '\.d$' | xargs -I{} sh -c 'strings {} 2>/dev/null | grep -q m74_4_grid && echo {}' | head -1)
M74_4=1 nice -n 10 "$EXE" m74_4_grid --nocapture --test-threads 1 > "$OUT/$1.out" 2> "$OUT/$1.err"
echo "rc=$?"
grep -E '^(==|RESULT)' "$OUT/$1.out" | head -3
