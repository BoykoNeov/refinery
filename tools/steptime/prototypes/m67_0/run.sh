#!/bin/sh
# usage: run.sh <label> ; env PLANT SPECS etc. passed through
EXE=$(ls -t ${TARGET:-/w/temp/claude/m67_0/wt/target}/release/deps/suite-*.exe | head -1)
WEXE=$(cygpath -w "$EXE")
cmd //v:on //c "start /belownormal /b /wait $WEXE m67_0_grid --nocapture --test-threads 1 & exit !errorlevel!" > /w/temp/claude/m67_0/$1.out 2> /w/temp/claude/m67_0/$1.err
echo "rc=$?"
grep -E '^(==|RESULT)' /w/temp/claude/m67_0/$1.out
