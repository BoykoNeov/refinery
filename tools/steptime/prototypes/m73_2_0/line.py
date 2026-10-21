import json,sys,os,glob
OUT=os.environ.get('M73_OUT',r'W:\temp\claude\m73_2_0\out')
modes=sys.argv[1].split(',') if len(sys.argv)>1 else ['off','lag_raw','lag_cap','solve_raw','solve_cap']
def load(mode,r):
    f=os.path.join(OUT,'line_%s_%s.jsonl'%(mode,('%g'%r)))
    return [json.loads(l) for l in open(f)]
refl=sorted({float(os.path.basename(f).split('_')[-1][:-6]) for f in glob.glob(os.path.join(OUT,'line_off_*.jsonl'))})
base={}
for r in refl:
    rows=load('off',r); last=rows[-1]
    base[r]=None if 'error' in last else last
for mode in modes:
    print('==',mode)
    print(' R    | err | dP used/raw/cap mbar | maxFF  maxDC | jet dc weep | Tbot  dTbot | dPur(pts) max | solves jump/mean/max | settle ring')
    for r in refl:
        rows=load(mode,r)
        last=rows[-1]
        if 'error' in last:
            ok=[x for x in rows if 'error' not in x]
            print(' %4.1f | ERR at tick %s: %s'%(r, ok[-1]['tick']+1 if ok else '?', last['error'][:150])); continue
        post=[x for x in rows if x['tick']>=300]
        fin=last['used_total_pa']
        settle=next((i for i,x in enumerate(post) if all(abs(y['used_total_pa']-fin)<1.0 for y in post[i:])),None)
        dev=[x['used_total_pa']-fin for x in post[:60]]
        ring=sum(1 for a,b in zip(dev,dev[1:]) if a*b<0 and abs(a)>1 and abs(b)>1)
        b=base[r]
        dT = last['temps_k'][-1]-b['temps_k'][-1] if b else float('nan')
        dP = max(abs(p-q) for p,q in zip(last['purity'],b['purity']))*100 if b else float('nan')
        s=[x['solves'] for x in post]
        print(' %4.1f |  ok | %6.1f %6.1f %6.1f | %5.2f %5.2f | %2d %2d %2d | %6.1f %+5.2f | %5.2f | %2d %4.2f %2d | %s %d'%(
            r, last['used_total_pa']/100, last['raw_total_pa']/100, last['capped_total_pa']/100,
            max(last['flood']), max(last['backup']), len(last['jet_flooding']), len(last['downcomer_flooding']), len(last['weeping']),
            last['temps_k'][-1]-273.15, dT, dP, s[0], sum(s)/len(s), max(s), settle, ring))
