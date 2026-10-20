#!/bin/sh
O=/w/temp/claude/m71-2
PLANT=cold SPECS_FILE=W:/temp/claude/m71-2/cold_rows.specs WARM=20 RUN=10000 JOBS=4 TARGET=/w/temp/claude/refinery-m71-2/target OUT=$O sh /w/Claude_projects/refinery/tools/steptime/prototypes/m67_3/run.sh cold_rows > /dev/null
echo "cold rows: $(grep -c ': RUNS' $O/cold_rows.out) run, $(grep -c ': FAIL' $O/cold_rows.out) fail"
grep ': FAIL' $O/cold_rows.out | cut -c1-160
