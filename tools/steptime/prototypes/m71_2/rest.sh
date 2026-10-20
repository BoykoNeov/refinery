#!/bin/sh
O=/w/temp/claude/m71-2
P=/w/Claude_projects/refinery/tools/steptime/prototypes/m67_3
T=/w/temp/claude/refinery-m71-2/target
PLANT=cold SPECS_FILE=W:/temp/claude/m71-2/cold_long.specs WARM=20 RUN=10000 JOBS=4 TARGET=$T OUT=$O sh $P/run.sh cold_long > /dev/null
echo "cold long: $(grep -c ': RUNS' $O/cold_long.out) run, $(grep -c ': FAIL' $O/cold_long.out) fail"
PLANT=cold SPECS_FILE=W:/temp/claude/m71-2/cold_held.specs WARM=10 RUN=2700 JOBS=4 TARGET=$T OUT=$O sh $P/run.sh cold_held > /dev/null
echo "cold held: $(grep -c ': RUNS' $O/cold_held.out) run, $(grep -c ': FAIL' $O/cold_held.out) fail"
PLANT=stripped SPECS='furnace=40+free=1;furnace=50+free=1' WARM=20 RUN=15000 JOBS=2 TARGET=$T OUT=$O sh $P/run.sh str_coils > /dev/null
grep RESULT $O/str_coils.out | cut -c1-120
