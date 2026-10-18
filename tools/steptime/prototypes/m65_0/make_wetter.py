import os
os.makedirs(r"W:/temp/claude/m65_0", exist_ok=True)
import re, sys
src = open(r"W:/Claude_projects/refinery/scenarios/crude_unit_wet.toml").read()
oil = dict(light_naphtha=0.07976, heavy_naphtha=0.13958, kerosene=0.15952, diesel=0.1994, gas_oil=0.13958, residue=0.27916)
s = sum(oil.values())
for w in [float(x) for x in sys.argv[1:]]:
    parts = ", ".join(f"{k} = {v/s*(1-w/100):.6f}" for k, v in oil.items()) + f", water = {w/100:.6f}"
    out = re.sub(r"composition = \{ light_naphtha = 0\.07976[^}]*\}", "composition = { " + parts + " }", src, count=1)
    out = out.replace('name = "crude_unit_wet"', f'name = "wet_{w:g}"')
    open(rf"W:/temp/claude/m65_0/wet_{w:g}.toml", "w").write(out)
