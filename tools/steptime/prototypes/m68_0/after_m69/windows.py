import csv, sys, statistics as st
rows = list(csv.DictReader(open(sys.argv[1])))
def win(name, a, b):
    r = [x for x in rows if a <= int(x['tick']) <= b]
    w = sorted(float(x['wall_us'])/1000 for x in r)
    col = sum(float(x['separate_us']) for x in r) / max(1e-9, sum(float(x['wall_us']) for x in r))
    ev = [int(x['ev_start'])+int(x['ev_jac'])+int(x['ev_search'])+int(x['ev_other']) for x in r]
    sv = [int(x['solves']) for x in r]
    over = sum(1 for v in w if v > 2.5)
    print(f"{name:28s} ticks {a}-{b}: wall median {st.median(w):6.2f} p90 {w[int(.9*len(w))]:6.2f} max {w[-1]:7.2f} ms, over 2.5 ms {over}/{len(w)}; column {100*col:4.1f} %; solves/tick {sum(sv)/len(sv):.2f}; evals/tick {sum(ev)/len(ev):.1f}")
for spec in sys.argv[2:]:
    n, a, b = spec.split(':'); win(n, int(a), int(b))
