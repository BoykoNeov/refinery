#!/bin/sh
# usage: run.sh <label> ; env PLANT SPECS etc. passed through
EXE=$(ls -t ${TARGET:-/w/temp/claude/m67_3/target}/release/deps/suite-*.exe | head -1)
WEXE=$(cygpath -w "$EXE")
cmd //v:on //c "start /belownormal /b /wait $WEXE m67_3_grid --nocapture --test-threads 1 & exit !errorlevel!" > ${OUT:-/w/temp/claude/m67_3}/$1.out 2> ${OUT:-/w/temp/claude/m67_3}/$1.err
echo "rc=$?"
grep -E '^(==|RESULT)' ${OUT:-/w/temp/claude/m67_3}/$1.out
