"""M71.2: each edge-check run against M70.2's committed result for the same spec.

python compare.py <label> ...   (reads m70_2/<label>.out and W:/temp/claude/m71-2/<label>.out)
Prints the counts and every spec whose verdict (RUNS, or FAIL and its tick) differs.
"""
import re, sys

OLD = r'W:\Claude_projects\refinery\tools\steptime\prototypes\m70_2'
NEW = r'W:\temp\claude\m71-2'


def verdicts(path):
    out = {}
    for line in open(path, encoding='utf-8', errors='replace'):
        m = re.match(r'RESULT (\S+): (RUNS|FAIL at t=\d+|REFUSED)', line)
        if m:
            out[m.group(1)] = m.group(2)
    return out


for label in sys.argv[1:]:
    old, new = verdicts(rf'{OLD}\{label}.out'), verdicts(rf'{NEW}\{label}.out')
    runs = lambda v: sum(x == 'RUNS' for x in v.values())
    print(f'{label}: old {runs(old)} run of {len(old)}, new {runs(new)} run of {len(new)}')
    for spec in old:
        if spec not in new:
            print(f'   missing on new: {spec}')
        elif old[spec] != new[spec]:
            print(f'   {spec}: {old[spec]} -> {new[spec]}')
