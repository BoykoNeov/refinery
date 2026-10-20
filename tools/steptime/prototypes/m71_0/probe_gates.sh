#!/bin/sh
M='--manifest-path W:\temp\claude\m71_0\wt\Cargo.toml'
run() { cmd //v:on //c "set M71_DRAWS=1& set M71_SINCE=1& start /belownormal /b /wait $* & exit !errorlevel!"; echo "== rc=$? :: $*"; }
run cargo test -q --workspace $M
for s in newton simple; do
  run cargo run -q --release $M -p refinery-cli -- corpus W:/Claude_projects/refinery/scenarios --ticks 6000 --jobs 16 --solver $s --baseline W:/temp/claude/refinery-corpus/$s.json --out W:/temp/claude/m71_0/corpus_$s.json
done
