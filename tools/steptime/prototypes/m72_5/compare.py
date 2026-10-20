"""compare.py <run A> <run B> — the probe's kept snapshots, A against B: per
snapshot, identical or the number of numeric leaves that differ and the largest
relative difference (|a-b| / max(|a|, |b|, 1e-12)), with where it is."""

import json
import sys

ROOT = "W:/temp/claude/m72_5_runs"


def leaves(value, path=""):
    if isinstance(value, dict):
        for k, v in value.items():
            yield from leaves(v, f"{path}.{k}")
    elif isinstance(value, list):
        for i, v in enumerate(value):
            yield from leaves(v, f"{path}[{i}]")
    else:
        yield path, value


def main():
    a_name, b_name = sys.argv[1], sys.argv[2]
    for plant in ["crude_unit", "crude_unit_loops"]:
        try:
            a = json.load(open(f"{ROOT}/{a_name}/{plant}.json"))
            b = json.load(open(f"{ROOT}/{b_name}/{plant}.json"))
        except FileNotFoundError:
            continue
        print(f"== {plant}: {a_name} vs {b_name}")
        for x, y in zip(a, b):
            la = dict(leaves(x["snapshot"]))
            lb = dict(leaves(y["snapshot"]))
            if la == lb:
                print(f"  {x['at']:40} identical")
                continue
            differ = 0
            worst = (0.0, "")
            for k in la.keys() | lb.keys():
                va, vb = la.get(k), lb.get(k)
                if va == vb:
                    continue
                differ += 1
                if isinstance(va, (int, float)) and isinstance(vb, (int, float)):
                    rel = abs(va - vb) / max(abs(va), abs(vb), 1e-12)
                    if rel > worst[0]:
                        worst = (rel, f"{k} {va:.6g} vs {vb:.6g}")
                else:
                    worst = (float("inf"), f"{k} {va!r} vs {vb!r}")
            print(f"  {x['at']:40} {differ:5} leaves differ, worst {worst[0]:.2e} at {worst[1][:140]}")


main()
