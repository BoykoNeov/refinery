"""M74.4's checks before the keys' range is chosen (DESIGN §79.8): what the
300-jump sample did not reach.

- `holds`: the reboiler held at 0, 20, 40 and 215 MW (the free reboiler's
  ceiling, 2.5 times its settled 86.7 MW, §79.6) at the 16 corners of the
  other four keys, one jump, an hour after it (36 000 ticks).
- `free`: the reboiler freed at those 16 corners, and at 30 settings the tray
  plant's measured table refuses (`demo/column_region_trays.json`, random,
  seed 7) plus its gate's two refused points, one jump, two hours after it
  (72 000 ticks): the free column moves an e-fold every 40 minutes (§79.4).
- `walks`: 200 random walks of 10 presses 300 ticks apart on the screen's own
  steps, inside the keys' box — the furnace 5 MW, the reflux 0.5, the loops'
  flow 25 %, their return 20 K, the reboiler T/G 2.5 MW from its hold (or
  from what it runs at, free) and Y freeing it — 20 minutes after the last;
  40 of them start held high (130–215 MW) and free it on the base it ran
  dry, a minute to five on.

usage: specs_checks.py holds|free|walks   (prints the specs, `;`-separated)
"""
import itertools
import json
import random
import sys

CORNERS = list(itertools.product([0.0, 40.0], [0.5, 15.0], [0.0, 2.0], [-60.0, 60.0]))
REGION = 'W:/Claude_projects/refinery/demo/column_region_trays.json'


def g(x):
    return '%g' % x


def keys(f, r, p, t):
    return 'furnace=%s+reflux=%s+flow=%s+ret=%s' % (g(f), g(r), g(p), g(t))


kind = sys.argv[1]
specs = []
if kind == 'holds':
    for reb in (0.0, 20.0, 40.0, 215.0):
        for c in CORNERS:
            specs.append('reb=%s+%s~36000' % (g(reb), keys(*c)))
elif kind == 'free':
    for c in CORNERS:
        specs.append('free=1+%s~72000' % keys(*c))
    region = json.load(open(REGION))
    grid, names = region['grid'], region['keys']
    refused = []
    for idx in itertools.product(*[range(len(grid[k])) for k in names]):
        node = region['allowed']
        for i in idx[:-1]:
            node = node[i]
        if node[idx[-1]] == '0':
            refused.append(tuple(grid[k][i] for k, i in zip(names, idx)))
    rng = random.Random(7)
    for c in rng.sample(refused, 30) + [(20.0, 2.0, 1.0, 60.0), (0.0, 1.0, 0.5, 20.0)]:
        specs.append('free=1+%s~72000' % keys(*c))
elif kind == 'walks':
    rng = random.Random(11)
    box = {'furnace': (0.0, 40.0, 5.0), 'reflux': (0.5, 15.0, 0.5), 'flow': (0.0, 2.0, 0.25), 'ret': (-60.0, 60.0, 20.0)}
    for w in range(200):
        at = {'furnace': 20.0, 'reflux': 4.0, 'flow': 1.0, 'ret': 0.0}
        held = True
        moves = []
        if w < 40:
            moves.append('reb=%s@%d' % (g(rng.choice([130.0, 150.0, 175.0, 200.0, 215.0])), rng.choice([600, 1200, 1800, 3000])))
            moves.append('free=1@300')
            held = False
        for _ in range(10):
            key = rng.choice(['furnace', 'reflux', 'flow', 'ret', 'reb', 'reb', 'y'])
            if key == 'y':
                if held:
                    moves.append('free=1@300')
                    held = False
                else:
                    moves.append('rebstep=0@300')
                    held = True
                continue
            if key == 'reb':
                moves.append('rebstep=%s@300' % g(rng.choice([2.5, -2.5])))
                held = True
                continue
            low, high, step = box[key]
            value = at[key] + rng.choice([step, -step])
            if low - 1e-9 <= value <= high + 1e-9:
                at[key] = round(value, 6)
                moves.append('%s=%s@300' % (key, g(at[key])))
        moves[-1] = moves[-1].split('@')[0]
        specs.append('>'.join(moves) + '~12000')
print(';'.join(specs))
