#!/bin/sh
# usage: run69.sh <mode 0|1|2>
m=$1
OUT=W:\temp\claude\m68-0-out
mkdir -p /w/temp/claude/m68-0-out/snap$m
cmd //v:on //c "set M69_PREDICT=$m& set SNAPDIR=W:/temp/claude/m68-0-out/snap$m& start /belownormal /b /wait W:\temp\claude\refinery-m68-0\tools\steptime\target\release\m69_0_count.exe W:\temp\claude\refinery-m68-0\scenarios crude_unit_boiling crude_unit_stripped crude_unit_cold & exit !errorlevel!" > /w/temp/claude/m68-0-out/predict$m.txt 2>&1
echo "mode $m rc=$?"
