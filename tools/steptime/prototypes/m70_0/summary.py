"""M70.0: each run's ticks after its step — median wall, solves a tick — and
the settled 500 ticks before it, from m70_0_step's CSVs.

python summary.py <out dir> <label>:<hold> ...
"""
import csv, statistics, sys

out = sys.argv[1]
for arg in sys.argv[2:]:
    label, hold = arg.split(':')
    hold = int(hold)
    rows = list(csv.DictReader(open(f'{out}/{label}.csv')))
    after = rows[hold:]
    wall = lambda rs: statistics.median(float(r['wall_us']) for r in rs) / 1e3
    solves = lambda rs: sum(1 + int(r['re_solves']) for r in rs) / len(rs)
    print(f"{label:12} settled {wall(rows[hold-500:hold]):5.2f} ms | after the step: "
          f"60 ticks {wall(after[:60]):5.2f} ms {solves(after[:60]):4.2f} solves; "
          f"600 ticks {wall(after[:600]):5.2f} ms {solves(after[:600]):4.2f}; "
          f"the rest {wall(after[600:]):5.2f} ms {solves(after[600:]):4.2f}; "
          f"over 2.5 ms in 600: {sum(float(r['wall_us']) > 2500 for r in after[:600])}")
