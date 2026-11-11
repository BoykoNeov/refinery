#!/bin/bash
EXE=$(ls /w/temp/claude/m74_6_1/wt/target/release/deps/suite-*.exe | head -1)
O=/w/temp/claude/m74_6_1/out
run() { # name, then env assignments
  local name=$1; shift
  env "$@" cmd //v:on //c "start /belownormal /b /wait $(cygpath -w $EXE) m74_6_0_empty --nocapture --test-threads 1 & exit !errorlevel!" > $O/$name.txt 2> $O/$name.err
  echo "$name done $(date +%T)"
}
# the starts from empty, steel at half, the guess, double
for s in 0.5 1 2; do
  run start_x$s PLANT=holdup M74_6_0_EMPTY=1 M74_6_0_STEEL=1 M74_6_1_STEEL_SCALE=$s WARM=1 JOBS=2 TRACE=600 \
    SPECS='tr=0~35999;tr=1@12000>tr=0~23999' &
done
# the feed interruption: a full column, feed and furnace shut, total reflux, the reboiler on
run interrupt PLANT=valved M74_6_0_STEEL=1 WARM=1000 JOBS=4 TRACE=600 \
  SPECS='valve=0+furnace=0+tr=1~18000;valve=0+furnace=0+tr=1+reb=40~18000;valve=0+furnace=0+tr=1@18000>valve=1+furnace=20+tr=0~18000' &
run interrupt_nosteel PLANT=valved WARM=1000 JOBS=3 TRACE=600 \
  SPECS='valve=0+furnace=0+tr=1~18000;valve=0+furnace=0+tr=1+reb=40~18000;valve=0+furnace=0+tr=1@18000>valve=1+furnace=20+tr=0~18000' &
wait
# the 70 held hours, the draining and the steel, at half and double
for s in 0.5 2; do
  run holds_drain_steel_x$s PLANT=holdup M74_6_0_DRAIN=1 M74_6_0_STEEL=1 M74_6_1_STEEL_SCALE=$s JOBS=14 \
    SPECS_FILE='W:\Claude_projects\refinery\tools\steptime\prototypes\m74_5_0\holds_specs.txt'
done
echo ALL DONE
