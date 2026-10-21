import json,itertools
r=json.load(open('W:/Claude_projects/refinery/demo/column_region_trays.json'))
G=r['grid'];K=r['keys'];A=r['allowed']
sh=[len(G[k]) for k in K]
def a(s): return A[s[0]][s[1]][s[2]][s[3]]=='1'
out=[]
for fi in (0,8):
    edge=[]
    for s in itertools.product([fi],*[range(n) for n in sh[1:]]):
        if not a(s): continue
        for ax in (1,2,3):
            for d in (-1,1):
                t=list(s);t[ax]+=d
                if 0<=t[ax]<sh[ax] and not a(tuple(t)): edge.append(s);break
            else: continue
            break
    step=max(1,len(edge)//17)
    out+=edge[::step][:17]
    print(len(edge))
out+=[(4,1,8,5),(4,3,8,6)]  # the timeline's f20 r1 p2 t40, and r2 p2 t60 that frees its hold
g=lambda x:'%g'%x
print(';'.join('furnace=%s+reflux=%s+flow=%s+ret=%s'%tuple(g(G[k][s[i]]) for i,k in enumerate(K)) for s in out))
