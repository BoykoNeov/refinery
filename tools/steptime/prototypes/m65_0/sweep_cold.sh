# tick-1 cold window per water share (fine), with the feed's latent heat flow
B=W:/Claude_projects/refinery/target/release/refinery.exe
D=W:/temp/claude/m65_0
for w in 0.3 1 2 3 5 10 12 13 15 20; do
  ok=""; es=""
  for s in 0.2 0.25 0.3 0.35 0.4 0.45 0.5 0.55 0.6 0.65 0.7 0.75 0.8 0.85 0.9 0.95 1 1.1 1.2 1.3 1.4; do
    out=$($B run $D/wet_${w}_pa$s.toml --ticks 1 --snapshot-every 1 --out $D/c1.jsonl 2>&1)
    [ -z "$es" ] && es=$(echo "$out" | grep -o 'M65PROBE.*' | head -1)
    if echo "$out" | grep -q 'failed'; then
      if echo "$out" | grep -q flooded; then ok="$ok F$s"; fi
    else
      fl=$(grep -o '"column_flooded"' $D/c1.jsonl | head -1); ok="$ok ${fl:+F}$s"
    fi
  done
  echo "w=$w $es | runs at:$ok"
done
