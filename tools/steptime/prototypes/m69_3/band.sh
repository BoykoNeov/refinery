#!/bin/sh
# usage: band.sh <plant> — its inside and outside edge cells, 1 500 ticks each.
O=/w/temp/claude/m69-3
for side in inside outside; do
  PLANT=$1 SPECS_FILE=W:/temp/claude/m69-3/$1_$side.specs WARM=20 RUN=1500 JOBS=4 \
    TARGET=/w/temp/claude/refinery-m69-3/target OUT=$O \
    sh /w/Claude_projects/refinery/tools/steptime/prototypes/m67_3/run.sh $1_$side > /dev/null
  echo "$1 $side: $(grep -c ': RUNS' $O/$1_$side.out) run, $(grep -c ': FAIL' $O/$1_$side.out) fail, $(grep -c 'REFUSED' $O/$1_$side.out) refused"
done
