#!/bin/sh
O=/w/temp/claude/m69-3
P=/w/Claude_projects/refinery/tools/steptime/prototypes/m67_3
L='file:50:413.9|steam=1+pa=0;file:65:456.5|steam=1+pa=0.25;file:52.5:421.1|steam=0.4+pa=0'
H='furnace=58@490>pa=0+walk=2.5@50>pa=1'
for e in old new; do
  T=/w/temp/claude/refinery-m69-3/target; [ $e = old ] && T=/w/temp/claude/refinery-m69-3-old/target
  PLANT=cold SPECS="$L" WARM=20 RUN=10000 JOBS=3 TARGET=$T OUT=$O sh $P/run.sh diff_long_$e > /dev/null &
  PLANT=cold SPECS="$H" WARM=10 RUN=2700 JOBS=1 TARGET=$T OUT=$O sh $P/run.sh diff_held_$e > /dev/null
  wait
  grep -h RESULT $O/diff_long_$e.out $O/diff_held_$e.out | sed "s/^/$e /" | cut -c1-140
done
