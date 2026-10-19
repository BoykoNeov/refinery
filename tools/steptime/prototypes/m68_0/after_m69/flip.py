import csv, sys
rows = list(csv.DictReader(open(sys.argv[1])))
for spec in sys.argv[2:]:
    n, a, b = spec.split(':'); a, b = int(a), int(b)
    r = [x for x in rows if a <= int(x['tick']) <= b]
    over = [x for x in r if float(x['wall_us']) > 2500]
    jac = sum(1 for x in over if float(x['wall_us']) - float(x['jac_us']) <= 2500)
    col = sum(1 for x in over if float(x['wall_us']) - float(x['separate_us']) <= 2500)
    print(f"{n:22s} {a}-{b}: over {len(over)}/{len(r)}; inside without the Jacobian {jac}; inside without the whole column {col}")
