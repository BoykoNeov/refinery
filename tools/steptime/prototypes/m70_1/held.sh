#!/bin/sh
S=W:/Claude_projects/refinery/scenarios
for r in 1 2 3; do for v in head new; do for p in wet_rundown flashing_rundown product_cooler water_slug; do
  cmd //v:on //c "set OUT=W:/temp/claude/m70_1/out& set HELD=1& start /belownormal /b /wait W:\temp\claude\m70_1\target_$v\release\m70drv.exe $S/$p.toml held_${p}_${v}_$r 0 0 2000 & exit !errorlevel!" || echo FAIL $p $v $r
done; done; done
