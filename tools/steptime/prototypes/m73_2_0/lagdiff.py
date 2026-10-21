import json,os,glob
OUT=r'W:\temp\claude\m73_2_0\out'
def load(f): return [json.loads(l) for l in open(os.path.join(OUT,f))]
print('E1 file settings, cold start:')
for m in ['off','lag_raw','lag_cap','solve_raw','solve_cap']:
    rows=load('file_%s.jsonl'%m)
    for r in rows[:3]+[rows[-1]]:
        print('  %-9s t=%3d wall=%6.2fms passes=%d solves=%2d dP=%6.1f Tbot=%.2f Ttop=%.2f feedvf=%.4f pur=%s duty=%s'%(m,r['tick'],r['wall_s']*1e3,r['passes'],r['solves'],r['used_total_pa']/100,r['temps_k'][-1]-273.15,r['temps_k'][0]-273.15,r['feed_vf'],[round(x*100,2) for x in r['purity']],{k:round(v/1e6,3) for k,v in r['duty'].items()}))
    w=[r['wall_s'] for r in rows[1:]]
    print('   warm wall mean %.3f ms'%(sum(w)/len(w)*1e3))
print()
print('Lag against solve, after the jump (tick>=300): max |dP diff| mbar, max |Tbot diff| K, ticks with dP diff > 1 mbar')
for rule in ['raw','cap']:
    for f in sorted(glob.glob(os.path.join(OUT,'line_lag_%s_*.jsonl'%rule)), key=lambda f: float(f.split('_')[-1][:-6])):
        r=f.split('_')[-1][:-6]
        lag=load(os.path.basename(f)); sol=load('line_solve_%s_%s.jsonl'%(rule,r))
        pairs=[(a,b) for a,b in zip(lag,sol) if a['tick']>=300]
        dp=[abs(a['used_total_pa']-b['used_total_pa'])/100 for a,b in pairs]
        dt=[abs(a['temps_k'][-1]-b['temps_k'][-1]) for a,b in pairs]
        n=sum(1 for x in dp if x>1)
        print('  %s R=%5s  dP %6.2f  Tbot %5.2f  ticks>1mbar %d  (first ticks dP: %s)'%(rule,r,max(dp),max(dt),n,[round(x,1) for x in dp[:4]]))
