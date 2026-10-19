#!/bin/sh
# Alternates the committed (head) and new engines, 3 rounds: the stripped plant
# 1500 held, 58 -> 65 MW, 1800 after; and the cold start-up, 2400 ticks.
S=W:/Claude_projects/refinery/scenarios
for r in 1 2 3; do for v in head new; do
  cmd //v:on //c "set OUT=W:/temp/claude/m70_1/out& start /belownormal /b /wait W:\temp\claude\m70_1\target_$v\release\m70drv.exe $S/crude_unit_stripped.toml step_${v}_$r 1500 65 1800 & exit !errorlevel!" || echo FAIL $v $r
  cmd //v:on //c "set OUT=W:/temp/claude/m70_1/out& set COLD=1& start /belownormal /b /wait W:\temp\claude\m70_1\target_$v\release\m70drv.exe $S/crude_unit_cold.toml cold_${v}_$r 0 0 2400 & exit !errorlevel!" || echo FAIL cold $v $r
done; done
