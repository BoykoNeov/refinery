"""Generate `scenarios/crude_unit_stripped.toml` from `scenarios/crude_unit_steam.toml`
(M64.3, docs/DESIGN.md §69.4, §69.9).

The steamed crude unit with its reboiler held at zero, its furnace and coil at
their settled state, its pumparounds off and its charge held on a flow loop.
Every number below was measured on the running plant (§69.9).

Run: python -I make_stripped.py <repo root>
"""

import sys

FURNACE_MW = 58.0
# The coil's settled temperature at 58 MW on the held 112.8 kg/s [°C].
COIL_C = 436.88
FEED_KV = 400.0
# The opening the loop settles at: tick 1 runs at it, before any measurement.
FEED_OPENING = 0.419
CHARGE_KG_PER_S = 112.8
LOOP_GAIN_PER_KG_PER_S = 0.004
LOOP_INTEGRAL_S = 10.0

root = sys.argv[1]
source = open(f"{root}/scenarios/crude_unit_steam.toml", encoding="utf-8").read()
header = open(f"{root}/tools/steptime/prototypes/m64_3/crude_unit_stripped_header.toml", encoding="utf-8").read()

body = source[source.index("[meta]"):]


def replace_once(text, old, new):
    assert text.count(old) == 1, old
    return text.replace(old, new, 1)


body = replace_once(body, 'name = "crude_unit_steam"', 'name = "crude_unit_stripped"')
body = replace_once(
    body,
    "description = \"The wet crude unit with stripping steam: a 10 bar steam supply through a choking valve into the column's bottom stage, its water leaving by the condenser's water draw.\"",
    "description = \"The steamed crude unit stripped by its steam alone: the reboiler held at zero, the reflux a result, the furnace carrying the heat and the charge held on a flow loop.\"",
)
body = replace_once(body, "kv = 135.0\nopening = 1.0\n", f"kv = {FEED_KV}\nopening = {FEED_OPENING}\n")
body = replace_once(body, "duty_mw = 20.0\n", f"duty_mw = {FURNACE_MW}\n")
body = replace_once(body, "coil_temperature_c = 322.05\n", f"coil_temperature_c = {COIL_C}\n")
body = replace_once(body, "reflux_ratio = 4.0\n", "reflux_ratio = 4.0\nreboiler_duty_mw = 0.0\n")
body = replace_once(
    body,
    "pumparounds = [{ stage = 4, duty_mw = 15.0 }, { stage = 6, duty_mw = 15.0 }, { stage = 7, duty_mw = 10.0 }]",
    "pumparounds = [{ stage = 4, duty_mw = 0.0 }, { stage = 6, duty_mw = 0.0 }, { stage = 7, duty_mw = 0.0 }]",
)

body = body.rstrip("\n") + f'''

# The charge loop. `initial_output` is the loop's memory, seeded on tick 2
# against tick 1's measurement (M19's rule), and says the valve's opening.
[[controls]]
name = "charge_flow"
measurement = {{ pipe = "crude_line", variable = "flow" }}
actuator = "feed_valve"
algorithm = "pi"
mode = "auto"
action = "reverse"
setpoint_kg_per_s = {CHARGE_KG_PER_S}
gain_per_kg_per_s = {LOOP_GAIN_PER_KG_PER_S}
integral_time_s = {LOOP_INTEGRAL_S}
initial_output = {FEED_OPENING}
'''

open(f"{root}/scenarios/crude_unit_stripped.toml", "w", encoding="utf-8", newline="\n").write(
    header + "\n" + body
)
print("written")
