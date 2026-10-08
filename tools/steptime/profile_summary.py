"""Where a profile's time goes, by function (M56, docs/DESIGN.md §61.0).

samply cannot read this workspace's PDBs, so symbols come from a linker map:

  cargo rustc --release --bin <bin> -- -C link-arg=/MAP:<out>.map
  samply record --save-only -o <out>.json.gz -- <bin> ...
  python profile_summary.py <out>.json.gz <out>.map [focus-substring] [rows]

Prints inclusive and self shares of the busiest thread's samples.
"""
import json, gzip, sys, bisect, re
from collections import Counter
prof, mapf = sys.argv[1], sys.argv[2]
import os
EXE = os.path.basename(mapf).replace('.map', '.exe').lower()
focus = sys.argv[3] if len(sys.argv) > 3 else None
syms = []
for line in open(mapf, errors='replace'):
    m = re.match(r'\s*[0-9a-f]{4}:[0-9a-f]{8}\s+(\S+)\s+([0-9a-f]{16})\s', line)
    if m: syms.append((int(m.group(2), 16) - 0x140000000, m.group(1)))
syms.sort(); rv = [a for a, _ in syms]
p = json.load(gzip.open(prof)); libs = p['libs']
th = max(p['threads'], key=lambda x: x['samples']['length'])
ft, fn, stk, rt, sa = th['frameTable'], th['funcTable'], th['stackTable'], th['resourceTable'], th['stringArray']
def dem(n):
    # crude demangle of MSVC-style Rust symbols isn't needed: rust v0/legacy names in map are mangled; keep path pieces
    m = re.findall(r'(\d+)([A-Za-z_][A-Za-z0-9_]*)', n)
    if n.startswith('_ZN'):
        out=[]; i=3
        while i < len(n) and n[i].isdigit():
            j=i
            while n[j].isdigit(): j+=1
            L=int(n[i:j]); out.append(n[j:j+L]); i=j+L
        out=[o for o in out if not re.fullmatch(r'h[0-9a-f]{16}',o)]
        return '::'.join(out[-3:])
    return n[:100]
cache = {}
def fname(fi):
    if fi in cache: return cache[fi]
    f = ft['func'][fi]; r = fn['resource'][f]; name = sa[fn['name'][f]]
    if r is not None and r >= 0 and libs[rt['lib'][r]]['name'].lower() == EXE:
        a = ft['address'][fi]; i = bisect.bisect_right(rv, a) - 1
        if i >= 0: name = dem(syms[i][1])
    cache[fi] = name; return name
incl, selfc = Counter(), Counter(); total = 0
for si in th['samples']['stack']:
    if si is None: continue
    chain = []; x = si
    while x is not None:
        chain.append(fname(stk['frame'][x])); x = stk['prefix'][x]
    if focus and not any(focus in c for c in chain): continue
    total += 1
    selfc[chain[0]] += 1
    for c in set(chain): incl[c] += 1
print('samples', total)
print('--- inclusive'); [print(f'{v/total*100:5.1f}% {k}') for k, v in incl.most_common(int(sys.argv[4]) if len(sys.argv)>4 else 50)]
print('--- self'); [print(f'{v/total*100:5.1f}% {k}') for k, v in selfc.most_common(25)]
