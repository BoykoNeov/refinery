import json, sys, os, glob
def walk(a, b, path, out):
    if isinstance(a, dict):
        for k in a:
            if k in b: walk(a[k], b[k], path + '.' + str(k), out)
    elif isinstance(a, list):
        if len(a) != len(b):
            out.append((float('inf'), path + ' (length)', len(a), len(b))); return
        for i, (x, y) in enumerate(zip(a, b)):
            name = x.get('name') if isinstance(x, dict) else None
            walk(x, y, path + '[' + (name or str(i)) + ']', out)
    elif isinstance(a, (int, float)) and isinstance(b, (int, float)) and not isinstance(a, bool):
        scale = max(abs(a), abs(b))
        if scale > 1e-9 and a != b:
            out.append((abs(a - b) / scale, path, a, b))
    elif a != b:
        out.append((float('inf'), path, a, b))
base, other = sys.argv[1], sys.argv[2]
for f in sorted(glob.glob(base + '/*.json')):
    name = os.path.basename(f)
    g = os.path.join(other, name)
    if not os.path.exists(g): continue
    out = []
    walk(json.load(open(f)), json.load(open(g)), '', out)
    out = [o for o in out if not o[1].startswith('.solver')]
    out.sort(key=lambda x: -x[0])
    big = [o for o in out if o[0] > 1e-6]
    worst = out[0] if out else None
    print(f"{name:32} fields differing {len(out):5d}, over 1e-6: {len(big):4d}; worst {worst[0]:.2e} {worst[1]} {worst[2]} vs {worst[3]}" if worst else f"{name:32} identical")
