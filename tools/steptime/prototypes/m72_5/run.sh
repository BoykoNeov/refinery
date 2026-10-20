#!/bin/sh
# run.sh <name> [plant ...] — runs the probe driver with the current PROBE_* env
# at below-normal priority; output in W:\temp\claude\m72_5_runs\<name>[.txt].
name=$1
shift
mkdir -p "/w/temp/claude/m72_5_runs/$name"
out="W:/temp/claude/m72_5_runs/$name"
cmd //v:on //c "start /belownormal /b /wait W:\\temp\\claude\\m72_5_probe\\tools\\steptime\\target\\release\\m72_5_count.exe W:/Claude_projects/refinery/scenarios $out $* & exit !errorlevel!" > "/w/temp/claude/m72_5_runs/$name.txt" 2>&1
echo "$name exit $?"
