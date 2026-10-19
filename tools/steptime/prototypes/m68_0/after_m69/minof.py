import csv, sys, os
# per-tick minimum wall (and its column time) over repeat runs -> one CSV
base, plant, out = sys.argv[1], sys.argv[2], sys.argv[3]
runs = [list(csv.DictReader(open(os.path.join(base, f"rep{i}", plant + ".csv")))) for i in (1, 2, 3)]
w = csv.DictWriter(open(out, "w", newline=""), fieldnames=runs[0][0].keys())
w.writeheader()
for rows in zip(*runs):
    w.writerow(min(rows, key=lambda r: float(r["wall_us"])))
