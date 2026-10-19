"""M70.1: how far each moved plant's answers moved, old engine against new.

Runs `refinery run <plant> --ticks 6000 --snapshot-every 600` on both CLIs
(both fidelities), then the largest relative difference over every numeric
field of every snapshot, with the field that set it. Values under 1e-6 in
magnitude are compared absolutely (a flow that is zero on one side).

python answers.py <new cli exe> <head cli exe> <plant:fidelity> ...
"""
import json, os, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

OUT = r'W:\temp\claude\m70_1\answers'
S = r'W:\Claude_projects\refinery\scenarios'
os.makedirs(OUT, exist_ok=True)
new_exe, head_exe = sys.argv[1], sys.argv[2]
cases = [a.split(':') for a in sys.argv[3:]]


def run(exe, tag, plant, fidelity):
    out = rf'{OUT}\{plant}_{fidelity}_{tag}.jsonl'
    cmd = (f'start /belownormal /b /wait {exe} run {S}\\{plant}.toml --ticks 6000 --solver {fidelity} '
           f'--snapshot-every 600 --out {out} & exit !errorlevel!')
    r = subprocess.run(['cmd', '/v:on', '/c', cmd], capture_output=True, text=True)
    assert r.returncode == 0, (plant, fidelity, tag, r.stderr[-500:])
    return out


def walk(a, b, path, worst):
    if isinstance(a, dict):
        for k in a:
            if k in b:
                walk(a[k], b[k], f'{path}.{k}', worst)
    elif isinstance(a, list):
        for i, (x, y) in enumerate(zip(a, b)):
            walk(x, y, f'{path}[{i}]', worst)
    elif isinstance(a, (int, float)) and not isinstance(a, bool) and isinstance(b, (int, float)):
        if 'iterations' in path or 'wall' in path or 're_solves' in path or 'residual' in path:
            return
        scale = max(abs(a), abs(b))
        d = abs(a - b) / scale if scale > 1e-6 else abs(a - b)
        if d > worst[0]:
            worst[:] = [d, path, a, b]


jobs = [(exe, tag, p, f) for p, f in cases for exe, tag in [(new_exe, 'new'), (head_exe, 'head')]]
with ThreadPoolExecutor(8) as pool:
    list(pool.map(lambda j: run(*j), jobs))
for plant, fidelity in cases:
    new = [json.loads(l) for l in open(rf'{OUT}\{plant}_{fidelity}_new.jsonl')]
    head = [json.loads(l) for l in open(rf'{OUT}\{plant}_{fidelity}_head.jsonl')]
    worst = [0.0, '', None, None]
    for a, b in zip(head, new):
        walk(a, b, '', worst)
    print(f'{plant:22} {fidelity:6} largest move {worst[0]:.1e} at {worst[1]} ({worst[2]} -> {worst[3]})', flush=True)
