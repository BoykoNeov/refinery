"""M74.4's cost sample: random single jumps across the five keys (seed 7)."""
import random, sys
random.seed(int(sys.argv[2]) if len(sys.argv) > 2 else 7)
n = int(sys.argv[1]) if len(sys.argv) > 1 else 40
FURNACE = [5.0 * i for i in range(9)]
REFLUX = [0.5 * i for i in range(1, 31)]
FLOW = [0.25 * i for i in range(9)]
RET = [-60.0 + 20.0 * i for i in range(7)]
REB = ['free'] + [50.0 + 10.0 * i for i in range(9)]   # free, or held 50..130 MW
g = lambda x: '%g' % x
specs = []
for _ in range(n):
    reb = random.choice(REB)
    parts = ['furnace=%s' % g(random.choice(FURNACE)), 'reflux=%s' % g(random.choice(REFLUX)),
             'flow=%s' % g(random.choice(FLOW)), 'ret=%s' % g(random.choice(RET)),
             'free=1' if reb == 'free' else 'reb=%s' % g(reb)]
    specs.append('+'.join(parts))
print(';'.join(specs))
