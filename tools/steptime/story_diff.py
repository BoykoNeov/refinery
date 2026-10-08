"""Every value of two runs' snapshots, tick by tick, nothing cut off (M56.1,
docs/DESIGN.md §61.3): the largest relative move per field, zero-against-nonzero
moves, and every field that appears, disappears or flips.

  python story_diff.py <a.jsonl> <b.jsonl> <label>

The JSON lines come from `steptime` with SNAPOUT (or `refinery run --out`).
Iteration counts and residuals are solver-internal and skipped.
"""
import json, sys, re
from collections import defaultdict
a=[json.loads(l) for l in open(sys.argv[1])]; b=[json.loads(l) for l in open(sys.argv[2])]
rel=defaultdict(lambda:(0.0,None)); disc=[]; absz=defaultdict(lambda:(0.0,None))
def walk(x,y,path,t):
    if isinstance(x,dict) and isinstance(y,dict):
        for k in set(x)|set(y):
            if k not in x or k not in y: disc.append((t,path+'.'+k,'present' if k in x else 'absent', 'present' if k in y else 'absent')); continue
            walk(x[k],y[k],path+'.'+k,t)
    elif isinstance(x,list) and isinstance(y,list):
        if len(x)!=len(y): disc.append((t,path,f'len {len(x)}',f'len {len(y)}'))
        for i,(u,v) in enumerate(zip(x,y)): walk(u,v,path+f'[{i}]',t)
    elif isinstance(x,bool) or isinstance(y,bool) or isinstance(x,str) or x is None or y is None:
        if x!=y: disc.append((t,path,x,y))
    elif isinstance(x,(int,float)) and isinstance(y,(int,float)):
        if x==y: return
        key=re.sub(r'\[\d+\]','[]',path)
        if 'iteration' in path or 're_solves' in path: 
            disc.append((t,path,x,y)) if False else None; return
        if x==0 or y==0:
            d=abs(x-y)
            if d>absz[key][0]: absz[key]=(d,(t,x,y))
        else:
            r=abs(x-y)/max(abs(x),abs(y))
            if r>rel[key][0]: rel[key]=(r,(t,x,y))
    elif x!=y: disc.append((t,path,x,y))
for t,(x,y) in enumerate(zip(a,b),1): walk(x,y,'',t)
print('==',sys.argv[3], 'ticks', len(a), len(b))
for k,(r,w) in sorted(rel.items(), key=lambda kv:-kv[1][0])[:8]: print(f'  rel {r:.2e}  {k}  at tick {w[0]}: {w[1]} vs {w[2]}')
for k,(d,w) in sorted(absz.items(), key=lambda kv:-kv[1][0])[:4]: print(f'  zero-vs-nonzero abs {d:.2e}  {k}  tick {w[0]}: {w[1]} vs {w[2]}')
print('  discrete differences:', len(disc))
for d in disc[:12]: print('   ', d)
