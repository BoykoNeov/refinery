#!/bin/sh
# base = current engine; draws = the probe with M71_DRAWS=1. Stripped held, stripped 58->65 MW, cold start-up, boiling held.
S=W:/Claude_projects/refinery/scenarios
run() { cmd //v:on //c "set OUT=W:/temp/claude/m71_0/out& $1 start /belownormal /b /wait W:\temp\claude\m71_0\target_$2\release\m71drv.exe $S/$3.toml $4 $5 & exit !errorlevel!" || echo FAIL $4; }
for v in base draws; do
  if [ $v = base ]; then b=base; e=""; else b=wt; e="set M71_DRAWS=1&"; fi
  run "$e set HELD=1&" $b crude_unit_stripped held_$v "0 0 2500"
  run "$e" $b crude_unit_stripped step_$v "1500 65 1800"
  run "$e set COLD=1&" $b crude_unit_cold cold_$v "0 0 2400"
  run "$e set HELD=1&" $b crude_unit_boiling boil_$v "0 0 2500"
done
