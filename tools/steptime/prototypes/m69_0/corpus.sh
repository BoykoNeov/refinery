#!/bin/sh
# Corpus, both fidelities, the predictor off (0) and guarded (2), against the
# rolling baseline (read only). Below-normal priority, 4 jobs.
O=/w/temp/claude/m68-0-out
cmd //v:on //c "start /belownormal /b /wait cargo build --release -p refinery-cli --manifest-path W:/temp/claude/refinery-m68-0/Cargo.toml -j 4 & exit !errorlevel!" > $O/corpus_build.txt 2>&1
echo "build rc=$?"
EXE=W:/temp/claude/refinery-m68-0/target/release/refinery.exe
for solver in newton simple; do
  for m in 0 2; do
    cmd //v:on //c "set M69_PREDICT=$m& start /belownormal /b /wait $EXE corpus W:/temp/claude/refinery-m68-0/scenarios --ticks 6000 --jobs 4 --solver $solver --baseline W:/temp/claude/refinery-corpus/$solver.json & exit !errorlevel!" > $O/corpus_${solver}_$m.txt 2>&1
    echo "$solver mode $m rc=$?"
  done
done
