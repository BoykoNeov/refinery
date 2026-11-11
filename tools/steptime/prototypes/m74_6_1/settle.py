import re, sys, collections
sys.stdout.reconfigure(encoding="utf-8")
for path in sys.argv[1:]:
    runs = collections.OrderedDict()
    for line in open(path, encoding="utf-8", errors="replace"):
        m = re.match(r"TRACE (\S+) t=(\d+): (.*)", line)
        if not m: continue
        spec, t, rest = m.group(1), int(m.group(2)), m.group(3)
        p = re.search(r"products ([\d./-]+)", rest)
        T = re.search(r"T \[([^\]]*)\]", rest)
        lv = re.search(r"drum ([\d.]+) base ([\d.]+)", rest)
        runs.setdefault(spec, []).append((t, [float(x) for x in p.group(1).split('/')] if p else None,
                                          [float(x) for x in T.group(1).split(',')] if T else None,
                                          lv.groups() if lv else None))
    print("==", path)
    for spec, rows in runs.items():
        end = rows[-1][1]
        settled = None
        for i, (t, pr, _, _) in enumerate(rows):
            if all(all(abs(r[1][k] - end[k]) <= 0.02 * abs(end[k]) for k in range(4)) for r in rows[i:]):
                settled = t; break
        peak = max(r[2][-1] for r in rows)
        drum10 = next((r[0] for r in rows if r[3] and float(r[3][0]) >= 0.1), None)
        base10 = next((r[0] for r in rows if r[3] and float(r[3][1]) >= 0.1), None)
        side = next((r[0] for r in rows if r[1] and r[1][1] > 0.5), None)
        f = lambda x: "-" if x is None else f"{x/600:.0f} min"
        print(f"{spec}: to {rows[-1][0]/600:.0f} min | top four settled {f(settled)} | base peak {peak:.0f} C | drum 10% {f(drum10)} side products {f(side)} base 10% {f(base10)} | end products {'/'.join(f'{x:.2f}' for x in end)} | end levels {rows[-1][3]}")
