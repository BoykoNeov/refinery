# one line per snapshot: tick, charge kg/s, column feed mol-ish, flash, reboiler/condenser MW, coil C, stage 7->8 vapour
import json, sys
for path in sys.argv[1:]:
    for line in open(path):
        d = json.loads(line)
        nodes = {n["name"]: n for n in d["nodes"]}
        names = {n["id"]: n["name"] for n in d["nodes"]}
        charge = sum(e["stream"]["mass_flow"] for e in d["edges"] if e["name"] == "feed_spool")
        tl = [e for e in d["edges"] if e["name"] == "transfer_line"][0]["stream"]
        col = nodes["column"]
        op = col.get("column_operation", {})
        du = col.get("column_duty", {})
        fur = nodes["furnace"]["kind"]
        print(f"{path.split('/')[-1]:>22} t={d['tick']:>5} charge={charge:7.2f} kg/s  TL={tl['temperature']-273.15:6.1f}C vf={tl.get('vapour_fraction',0):.3f}  flash={op.get('feed_vapour_fraction',float('nan')):.3f}  reb={du.get('reboiler_w',0)/1e6:6.1f} cond={du.get('condenser_w',0)/1e6:6.1f} MW  coil={fur['coil']['temperature']-273.15:6.1f}C  bot={op.get('stage_temperatures_k',[0])[-1]-273.15:6.1f}C  V7={op.get('stage_vapour_mol_s',[0]*10)[7]:6.1f} V8={op.get('stage_vapour_mol_s',[0]*10)[8]:6.1f}  pa={sum(p['duty'] for p in col['kind']['cascade']['pumparounds'])/1e6:.1f}")
