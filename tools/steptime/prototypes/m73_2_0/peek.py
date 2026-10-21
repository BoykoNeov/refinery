import json,sys
for f in sys.argv[1:]:
    rows=[json.loads(l) for l in open(f)]
    print(f, len(rows))
    for r in rows[:3]+rows[-1:]:
        if 'error' in r: print('  ERR',r['error'][:300]); continue
        print('  t=%d wall=%.4f passes=%d solves=%d used=%.0f raw=%.0f sumS=%.1e sumT=%.1e Tbot=%.2f Ttop=%.2f pur=%s' % (
            r['tick'],r['wall_s'],r['passes'],r['solves'],r['used_total_pa'],r['raw_total_pa'],r['sum_at_stage'],r['sum_at_top'],
            r['temps_k'][-1]-273.15,r['temps_k'][0]-273.15,[round(x,4) for x in r['purity']]))
