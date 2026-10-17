"""Write scenarios/crude_unit_boiling.toml: the crude unit as a plant that
models boiling (M62.4), from scenarios/crude_unit.toml and the pieces each
sub-slice gated."""
import pathlib
import sys

REPO = pathlib.Path(r"W:/Claude_projects/refinery")
src = (REPO / "scenarios" / "crude_unit.toml").read_text(encoding="utf-8")
out = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else REPO / "scenarios" / "crude_unit_boiling.toml"


def rep(old, new):
    global src
    assert src.count(old) == 1, old[:70]
    src = src.replace(old, new)


# The header comment: replaced whole.
body = src[src.index("[meta]"):]
src = body
rep('name = "crude_unit"', 'name = "crude_unit_boiling"')
rep(
    'description = "A crude unit: a preheated crude, a furnace, and a heat-balance column with three side draws and three pumparounds."',
    'description = "The crude unit as a plant that models boiling: a coil the crude boils in, a two-phase transfer line, and product lines cooled before their tanks."',
)
rep('boiloff = "none"', 'boiloff = "flash"\nline_flash = "equilibrium"')
# The coil starts at its own settled metal, as crude_unit.toml's does at its
# 359.5 °C: started 40 K hot, it settles over some 20 minutes, and every tick of
# that drift re-solves on its own states (DEFERRED A29).
rep("coil_temperature_c = 359.5", "coil_temperature_c = 318.3")
rep(
    "fluid_heating_value_mj_per_kg = 42.686",
    "fluid_heating_value_mj_per_kg = 42.686\ncoil_cells = 8\ncoil_length_m = 800.0\ncoil_diameter_m = 0.24",
)
rep("[nodes.furnace]", '[nodes.feed_valve]\ntype = "valve"\nkv = 135.0\nopening = 1.0\n\n[nodes.furnace]')
rep(
    'name = "crude_line"\nfrom = "crude_source"\nto = "furnace"\nlength_m = 30.0\ndiameter_m = 0.08',
    'name = "crude_line"\nfrom = "crude_source"\nto = "feed_valve"\nlength_m = 30.0\ndiameter_m = 0.30\n\n'
    '[[pipes]]\nname = "feed_spool"\nfrom = "feed_valve"\nto = "furnace"\nlength_m = 5.0\ndiameter_m = 0.30',
)
rep(
    'name = "transfer_line"\nfrom = "furnace"\nto = "column"\nlength_m = 15.0\ndiameter_m = 0.08',
    'name = "transfer_line"\nfrom = "furnace"\nto = "column"\nlength_m = 15.0\ndiameter_m = 0.40',
)
LINES = [
    ("naphtha", 75.0, 150.0),
    ("kerosene", 115.0, 90.0),
    ("diesel", 190.0, 120.0),
    ("gas_oil", 150.0, 70.0),
    ("residue", 460.0, 100.0),
]
for product, water, ua in LINES:
    tank = f"{product}_tank"
    rep(f'{{ outlet = "{tank}",', f'{{ outlet = "{product}_pump",')
    rep(
        f'name = "{product}_draw"\nfrom = "column"\nto = "{tank}"\nlength_m = 20.0\ndiameter_m = 0.15',
        f'name = "{product}_suction"\nfrom = "column"\nto = "{product}_pump"\nlength_m = 20.0\n'
        f'diameter_m = 0.20\nelevation_change_m = -15.0',
    )
    src += f'''
[nodes.{product}_pump]
type = "pump"
h0_m = 40.0
a = 2000.0
on = true

[nodes.{product}_cooler]
type = "water_cooler"
ua_kw_per_k = {ua}
water_inlet_c = 30.0
water_flow_kg_per_s = {water}

[[pipes]]
name = "{product}_discharge"
from = "{product}_pump"
to = "{product}_cooler"
length_m = 30.0
diameter_m = 0.15

[[pipes]]
name = "{product}_rundown"
from = "{product}_cooler"
to = "{tank}"
length_m = 100.0
diameter_m = 0.15
elevation_change_m = 15.0
'''
header = (REPO / "tools" / "steptime" / "prototypes" / "m62_4" / "crude_unit_boiling_header.toml").read_text(encoding="utf-8")
out.write_text(header.rstrip("\n") + "\n\n" + src, encoding="utf-8", newline="\n")
print("wrote", out)
