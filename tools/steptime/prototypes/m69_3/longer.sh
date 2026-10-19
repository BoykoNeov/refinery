#!/bin/sh
O=/w/temp/claude/m69-3
P=/w/Claude_projects/refinery/tools/steptime/prototypes/m67_3
T=/w/temp/claude/refinery-m69-3/target
PLANT=cold SPECS='file:50:413.9|steam=1+pa=0;file:65:456.5|steam=1+pa=0.25;file:52.5:421.1|steam=0.4+pa=0' WARM=20 RUN=20000 JOBS=2 TARGET=$T OUT=$O sh $P/run.sh cold_20k > /dev/null &
PLANT=stripped SPECS='furnace=40+free=1;furnace=50+free=1' WARM=20 RUN=15000 JOBS=2 TARGET=$T OUT=$O sh $P/run.sh str_coils > /dev/null
wait
grep -h RESULT $O/cold_20k.out $O/str_coils.out | cut -c1-150
