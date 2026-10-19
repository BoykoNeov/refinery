import csv,glob,statistics
out='W:/temp/claude/m70_1/out'
for p in ['crude_unit_boiling','crude_unit_stripped']:
    for v in ['head','new']:
        runs=[list(csv.DictReader(open(f))) for f in sorted(glob.glob(f'{out}/split_{p}_{v}_[0-9].csv'))]
        wall=[min(float(r[i]['wall_us']) for r in runs)/1e3 for i in range(len(runs[0]))]
        line=f"{p:20} {v:9}"
        for a,b in [(100,1000),(1000,2000)]:
            rs=runs[0][a:b]; w=wall[a:b]
            it=sum(int(r['iterations']) for r in rs)/len(rs); so=sum(1+int(r['re_solves']) for r in rs)/len(rs)
            res=sorted(float(r['residual']) for r in rs)
            line+=f" | {a}-{b}: {statistics.mean(w):5.2f} ms solves {so:4.2f} iter {it:4.2f} residual median {res[len(res)//2]:.1e} >1e-10: {sum(x>1e-10 for x in res):3}"
        print(line)
