"""M74.4's sample read (DESIGN §79.8): `m74_4_grid` output of single jumps.

Prints: runs and failures; each failure's kind and how long after the move it
came; failures by reboiler setting and by reflux; how many runs end at a
limit (a drum or base dry or full), with a level loop at its ceiling, or with
a tray passing nothing down; the slowest tick seen.

usage: analyse_sample.py <out file> [<out file> …]
"""
import re
import sys
from collections import Counter, defaultdict

lines = []
for path in sys.argv[1:]:
    lines += [l for l in open(path, encoding='utf-8') if l.startswith('RESULT ')]


def keys(spec):
    out = {}
    for move in spec.split('>'):
        for kv in move.split('@')[0].split('+'):
            k, v = kv.split('=')
            out[k] = float(v)
    return out


runs = [l for l in lines if ': RUNS ' in l]
fails = [l for l in lines if ': FAIL ' in l]
print(f'{len(lines)} runs: {len(runs)} ran, {len(fails)} failed')

kinds = Counter()
when = []
for l in fails:
    m = re.search(r'\((\d+) after the last move', l)
    when.append(int(m.group(1)))
    reason = re.search(r'"message":"([^"]*)"', l).group(1)
    reason = re.sub(r'[-\d.e]+', '#', reason)[:110]
    kinds[reason] += 1
for k, n in kinds.most_common():
    print(f'  {n:4d}  {k}')
if when:
    when.sort()
    print('  ticks after the move:', when)

by_reb = defaultdict(lambda: [0, 0])
by_reflux = defaultdict(lambda: [0, 0])
for l in lines:
    spec = l[len('RESULT '):l.index(': ')]
    k = keys(spec)
    band = 'free' if 'free' in k else '%d-%d MW' % (k['reb'] // 50 * 50, k['reb'] // 50 * 50 + 45)
    ok = ': RUNS ' in l
    by_reb[band][0 if ok else 1] += 1
    by_reflux[k['reflux']][0 if ok else 1] += 1
print('by reboiler (ran, failed):', dict(sorted(by_reb.items())))
print('by reflux (ran, failed):', {r: tuple(v) for r, v in sorted(by_reflux.items())})

end_limit = Counter()
dry_end = dry_any = at_max = 0
worst = 0.0
for l in runs:
    m = re.search(r'held|free', l.split('RUNS ')[1])
    head = l.split('RUNS ')[1].split(' | ')[0]
    for vessel in ('drum', 'base'):
        lim = re.search(vessel + r' [\d.]+ (\w+)', head).group(1)
        if lim != 'normal':
            end_limit[f'{vessel} {lim}'] += 1
    d = re.search(r'dry trays (\d+) ticks, (\d+) at most, last at (\d+)', l)
    if int(d.group(1)) > 0:
        dry_any += 1
        if int(d.group(3)) >= 12000 + (0 if 'free' not in l else 0) - 1:
            dry_end += 1
    a = re.search(r'atmax (\d+)/(\d+)', l)
    if int(a.group(1)) + int(a.group(2)) > 0:
        at_max += 1
    worst = max(worst, float(re.search(r'worst ([\d.]+) ms', l).group(1)))
print(f'of the {len(runs)} that ran: ending at a limit {dict(end_limit)}; '
      f'a level loop at its ceiling at some tick {at_max}; '
      f'a tray passing nothing down at some tick {dry_any}, to the end {dry_end}; '
      f'the slowest tick {worst:.0f} ms')
