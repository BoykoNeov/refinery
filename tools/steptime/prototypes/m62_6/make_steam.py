"""Generate `scenarios/crude_unit_steam.toml` from `scenarios/crude_unit_wet.toml`
(M62.6d, docs/DESIGN.md §67.21).

The wet crude unit with stripping steam: a 10 bar, 400 °C supply, a control
valve and a 15 cm line into the column's bottom stage. The valve chokes on the
all-vapour steam (M62.6b; the ISA gas law on its x_t since M63.3), so its
opening sets the rate.

Run: python -I make_steam.py <repo root>
"""

import re
import sys

STEAM_KV = 80.0
STEAM_OPENING = 0.4
# The valve's pressure differential ratio factor (IEC 60534-2-1), a globe
# valve's, as `gas_valve.toml`'s (M63.3, docs/DESIGN.md §68.3); with the
# opening lowered from M62.6d's 0.5 to keep the steam near 0.85 kg/s.
STEAM_X_T = 0.72
STEAM_STAGE = 10

root = sys.argv[1]
source = open(f"{root}/scenarios/crude_unit_wet.toml", encoding="utf-8").read()
header = open(f"{root}/tools/steptime/prototypes/m62_6/crude_unit_steam_header.toml", encoding="utf-8").read()

body = source[source.index("[meta]"):]
body = body.replace('name = "crude_unit_wet"', 'name = "crude_unit_steam"')
body = re.sub(r'description = ".*"', 'description = "The wet crude unit with stripping steam: a 10 bar steam supply through a choking valve into the column\'s bottom stage, its water leaving by the condenser\'s water draw."', body, count=1)

# The steam inlet on the column's cascade.
assert body.count("pumparounds = [{ stage = 4,") == 1
body = body.replace(
    "pumparounds = [{ stage = 4,",
    f'steam = {{ pipe = "stripping_steam", stage = {STEAM_STAGE} }}\npumparounds = [{{ stage = 4,',
)

# The supply and its valve, before the sour-water tank.
supply = f'''[nodes.steam_supply]
type = "source"
pressure_bar = 10.0
temperature_c = 400.0
composition = {{ water = 1.0 }}

[nodes.steam_valve]
type = "valve"
kv = {STEAM_KV}
opening = {STEAM_OPENING}
x_t = {STEAM_X_T}

[nodes.sour_water_tank]'''
assert body.count("[nodes.sour_water_tank]") == 1
body = body.replace("[nodes.sour_water_tank]", supply, 1)

# The steam's pipes, before the transfer line.
pipes = '''[[pipes]]
name = "steam_header"
from = "steam_supply"
to = "steam_valve"
length_m = 30.0
diameter_m = 0.15

[[pipes]]
name = "stripping_steam"
from = "steam_valve"
to = "column"
length_m = 10.0
diameter_m = 0.15

[[pipes]]
name = "transfer_line"'''
assert body.count('[[pipes]]\nname = "transfer_line"') == 1
body = body.replace('[[pipes]]\nname = "transfer_line"', pipes, 1)

open(f"{root}/scenarios/crude_unit_steam.toml", "w", encoding="utf-8", newline="\n").write(
    header + "\n" + body
)
print("written")
