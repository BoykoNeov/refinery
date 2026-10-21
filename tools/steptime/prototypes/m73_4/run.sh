#!/bin/sh
# usage: TARGET=<the build folder holding release/deps/suite-*.exe>
#        OUT=<a folder for the output> run.sh <label>
# PLANT, SPECS (or SPECS_FILE) and the rest pass through to the driver.
: "${TARGET:?set TARGET to the cargo target folder the suite was built in}"
: "${OUT:?set OUT to a folder for <label>.out and <label>.err}"
EXE=$(ls -t "$TARGET"/release/deps/suite-*.exe | head -1)
WEXE=$(cygpath -w "$EXE")
cmd //v:on //c "start /belownormal /b /wait $WEXE m73_4_grid --nocapture --test-threads 1 & exit !errorlevel!" > "$OUT/$1.out" 2> "$OUT/$1.err"
echo "rc=$?"
grep -E '^(==|RESULT)' "$OUT/$1.out"
