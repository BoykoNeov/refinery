#!/bin/sh
# usage: run.sh <label> <scenario path> <hold> <MW> <after> [ENV=VAL ...]
label=$1; scen=$2; hold=$3; mw=$4; after=$5; shift 5
sets=""
for e in "$@"; do sets="$sets set $e&"; done
cmd //v:on //c "set OUT=W:/temp/claude/m70-0-out&$sets start /belownormal /b /wait W:\temp\claude\refinery-m70-0\tools\steptime\target\release\m70_0_step.exe $scen $label $hold $mw $after & exit !errorlevel!"
