"""M74.5.0: a batch on the patched engine against the same batch unpatched.

usage: compare.py <patched.out> <base.out>
Prints each spec: runs or fails on each engine, the stages that stopped
boiling (ticks, most at once, which, last tick, how many at the end), the
slowest tick, and the patched engine's failure if any.
"""
import re
import sys

sys.stdout.reconfigure(encoding="utf-8")


def load(path):
    out = {}
    for line in open(path, encoding="utf-8", errors="replace"):
        if line.startswith("RESULT "):
            spec, rest = line[7:].split(": ", 1)
            out[spec] = rest.strip()
    return out


patched, base = load(sys.argv[1]), load(sys.argv[2])
runs = lambda d: sum(v.startswith("RUNS") for v in d.values())
print(f"{len(patched)} specs: patched runs {runs(patched)}, unpatched runs {runs(base)}")
cold_re = re.compile(r"cold (\d+) ticks, (\d+) at most, stages ([\d,]*), last at (-?\d+), (\d+) at the end")
for spec, value in patched.items():
    other = base.get(spec, "")
    cold = cold_re.search(value)
    worst = re.search(r"worst ([\d.]+) ms", value)
    tag = ("RUN " if value.startswith("RUNS") else "FAIL") + " / unpatched " + (
        "RUN " if other.startswith("RUNS") else "FAIL"
    )
    detail = ""
    if cold:
        ticks, most, stages, last, end = cold.groups()
        detail = f"cold {ticks} ticks ({int(ticks) / 600:.1f} min), {most} at once, stages [{stages}], last at {last}, {end} at end"
    print(f"{tag}  {spec}  | {detail} | worst {worst.group(1) if worst else '-'} ms")
    if not value.startswith("RUNS"):
        print("      ", value[:300])
