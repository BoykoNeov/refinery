"""M74.4's sample (DESIGN §79.8): random one-jump settings of the holdup
plant's five keys, on the screen's own steps, to find where the column fails
and how long after the move — before the table's grid is planned.

The keys: the furnace (MW), the reflux, the loops' flow (a share of the file's
85 / 95 / 65 kg/s, all three together), their return temperatures (moved
together from the file's 150 / 215 / 270 °C, in K), and the reboiler — free
(a quarter of the runs), or held at a duty [MW] on 5 MW steps from 40 to 215
(the free reboiler's ceiling, 2.5 times the 86.7 MW it settles on, §79.6).

usage: specs_sample.py <count> [seed]   (prints the specs, `;`-separated)
"""
import random
import sys

FURNACE = [5.0 * i for i in range(9)]           # 0..40 MW
REFLUX = [0.5 * i for i in range(1, 31)]        # 0.5..15
FLOW = [0.25 * i for i in range(9)]             # 0..2
RET = [-60.0 + 20.0 * i for i in range(7)]      # -60..+60 K
HELD = [40.0 + 5.0 * i for i in range(36)]      # 40..215 MW


def g(x):
    return '%g' % x


count = int(sys.argv[1])
rng = random.Random(int(sys.argv[2]) if len(sys.argv) > 2 else 7)
specs = []
for _ in range(count):
    keys = 'furnace=%s+reflux=%s+flow=%s+ret=%s' % (
        g(rng.choice(FURNACE)), g(rng.choice(REFLUX)), g(rng.choice(FLOW)), g(rng.choice(RET)))
    reboiler = 'free=1' if rng.random() < 0.25 else 'reb=%s' % g(rng.choice(HELD))
    specs.append(reboiler + '+' + keys)
print(';'.join(specs))
