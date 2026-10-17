"""Generate `scenarios/crude_unit_wet.toml` from `scenarios/crude_unit_boiling.toml`
(M62.5, docs/DESIGN.md §67.17).

The boiling crude unit with water in its crude: a free-water slate member, the
crude 0.3 % water by mass (a desalted crude's), and the column's water draw —
the water decanted at its condenser, piped down to a sour-water tank.

Run: python -I make_wet.py <repo root>
"""

import re
import sys

WATER_MASS_FRACTION = 0.003

root = sys.argv[1]
source = open(f"{root}/scenarios/crude_unit_boiling.toml", encoding="utf-8").read()
header = open(f"{root}/tools/steptime/prototypes/m62_5/crude_unit_wet_header.toml", encoding="utf-8").read()

body = source[source.index("[meta]"):]
body = body.replace('name = "crude_unit_boiling"', 'name = "crude_unit_wet"')
body = re.sub(r'description = ".*"', 'description = "The boiling crude unit with water in its crude: the water boils off in the coil, rides up the column as vapour and is decanted at its condenser to a sour-water tank."', body, count=1)

# The slate's water, after the last cut.
water = '''[[components]]
name = "water"
tb_c = 100.0
molar_mass_kg_per_mol = 0.018015
density_kg_per_m3 = 998.0
cp_j_per_kg_k = 4184.0
dh_vap_j_per_mol = 40650.0
cp_vapour_j_per_kg_k = 2000.0
free_water = true

[fidelity]'''
body = body.replace("[fidelity]", water, 1)

# The crude: its oil scaled to leave room for the water.
match = re.search(r'(\[nodes\.crude_source\][^\[]*?composition = \{)([^}]*)\}', body)
oil = {k.strip(): float(v) for k, v in (kv.split("=") for kv in match.group(2).split(","))}
scaled = ", ".join(f"{k} = {round(v * (1.0 - WATER_MASS_FRACTION), 6)}" for k, v in oil.items())
body = body[:match.start(2)] + f" {scaled}, water = {WATER_MASS_FRACTION} " + body[match.end(2):]

# The coil's metal where THIS plant settles it (measured over 40 000 ticks):
# the water boiling at the supply passes less crude through the valve than the
# dry plant, so its coil runs hotter, and a coil started off its own settled
# metal re-solves every tick of the drift (docs/DEFERRED.md A29).
assert body.count("coil_temperature_c = 318.3") == 1
body = body.replace("coil_temperature_c = 318.3", "coil_temperature_c = 322.05")

# The water draw, second, and its tank and pipe.
body = body.replace(
    '    { outlet = "naphtha_pump", stage = 0, draw_ratio = 0.22 },\n',
    '    { outlet = "naphtha_pump", stage = 0, draw_ratio = 0.22 },\n'
    '    { outlet = "sour_water_tank", stage = 0, phase = "water" },\n',
)
tank = '''[nodes.sour_water_tank]
type = "tank"
area_m2 = 10.0
height_m = 10.0
initial_level_m = 0.5
temperature_c = 40.0
composition = { water = 1.0 }

[nodes.naphtha_tank]'''
body = body.replace("[nodes.naphtha_tank]", tank, 1)
pipe = '''[[pipes]]
name = "water_draw"
from = "column"
to = "sour_water_tank"
length_m = 40.0
diameter_m = 0.05
elevation_change_m = -15.0

[[pipes]]
name = "naphtha_suction"'''
body = body.replace('[[pipes]]\nname = "naphtha_suction"', pipe, 1)

open(f"{root}/scenarios/crude_unit_wet.toml", "w", encoding="utf-8", newline="\n").write(
    header + "\n" + body
)
print("written")
