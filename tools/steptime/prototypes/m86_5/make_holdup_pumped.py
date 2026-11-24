"""M86.5 (docs/DESIGN.md §84.5): the column that holds liquid with product lines
that need their pumps, generated from `scenarios/crude_unit_holdup.toml`.

    python make_holdup_pumped.py grade  <out.toml> [overrides.json]
    python make_holdup_pumped.py lifted <out.toml> [overrides.json]

Every draw gets the product line of the M86.4 plant of the same name
(`tools/steptime/prototypes/m86_4/make_pumped.py`): a 15 m fall to the pump,
30 m of discharge, a check valve (Kv 400, 0.015 bar band), a metre to a water
cooler, then the rundown — for `grade` 1.5 km to the tank farm, entered 15 m
up (the draws' level), pumps of 100 m head; for `lifted` 100 m climbing 40 m,
pumps of 40 m head. The column, its feed and its 8 cm transfer line stay the
holdup plant's, so its drum and base keep the sizes they were given for its
own flows.

`overrides.json` maps a product to any of its LINE keys, for sizing probes.
"""

import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[4]

# Each product's cooling water [kg/s] and cooler UA [kW/K]
# (`product_line_reference::LINES`).
COOLING = {
    "naphtha": (75.0, 150.0),
    "kerosene": (115.0, 90.0),
    "diesel": (190.0, 120.0),
    "gas_oil": (150.0, 70.0),
    "residue": (460.0, 100.0),
}

# Per product: pump shutoff head h0 [m], curve a [m per (m3/s)^2], the
# rundown's length [m], diameter [m] and climb [m]: the M86.4 plants' lines.
VARIANTS = {
    "lifted": {
        p: {"h0": 40.0, "a": 2000.0, "length": 100.0, "diameter": 0.15, "rise": 40.0}
        for p in COOLING
    },
    "grade": {
        p: {"h0": 100.0, "a": 2000.0, "length": 1500.0, "diameter": d, "rise": 15.0}
        for p, d in {
            "naphtha": 0.154,
            "kerosene": 0.131,
            "diesel": 0.14,
            "gas_oil": 0.119,
            "residue": 0.152,
        }.items()
    },
}

DESCRIPTIONS = {
    "lifted": "The holding tray plant with its product tanks on high ground: a stopped product pump stops its product and the column keeps it, so the reflux drum or the base fills and spills to slop, or a side product floods the trays and leaves with the naphtha and the residue.",
    "grade": "The holding tray plant with its product tanks at grade at the end of long rundowns: a stopped product pump cuts its product to just over half and the column keeps the rest, so the base fills, or a side product's rest leaves with the naphtha and the residue.",
}

COMMON = """#
# Both M86.5 plants (docs/DESIGN.md §84.5) are `crude_unit_holdup.toml` — the
# thirteen-tray column holding liquid on its trays, in an 18.7 m3 reflux drum
# and a 115.4 m3 base, the distillate holding the drum's level and the
# bottoms the base's — with a product line on every draw: the draw falls 15 m
# to its pump, which pushes it through a check valve and a water cooler to its
# tank. The check valve keeps a tank from pushing its product back through a
# stopped pump (§84.3).
#
# A COLUMN THAT HOLDS LIQUID KEEPS WHAT A LINE CANNOT TAKE (§84.2), unlike the
# M86.4 plants' column, which holds none and sends it to the next draw down.
# What the naphtha line cannot take stays in the reflux drum, what the residue
# line cannot take in the base: the level rises, its level loop asks for more
# (at most 2.5 times), and past full the excess goes to slop, off-spec. What a
# side draw's line cannot take stays on its tray and runs down the column; on
# the hotter trays below its lighter part boils back up, so it leaves partly
# with the naphtha and partly with the residue, and trays it overloads read as
# flooding. Nothing halts: every pump may stop.
#
# A stopped centrifugal pump is not a closed valve: product still flows
# through it if something else pushes (Chaudhry 2014, Table A-1). What stops
# the product is what the line asks the pump to lift.
"""

HEADERS = {
    "lifted": """# M86.5 demo plant: a stopped product pump STOPS its product, and the
# column holding liquid keeps it (docs/DESIGN.md §84.5).
"""
    + COMMON
    + """#
# HERE THE TANKS STAND HIGH. Each rundown climbs 40 m from its pump, so every
# tank's inlet is 25 m above the column's draws, out of the reach of the
# column's own push (0.8 bar over the air outside). With the pump stopped
# nothing gets up to the tank: the line stops and the check valve's disc
# shuts. Running (40 m of head each), every line carries 2.1-4.0 times its
# product. Measured, the newton solver, from the pump stopped:
#
# - naphtha: the drum 0.50 -> 0.60 of its volume in 5 min and full in about
#   28 (its loop sends more back as reflux too), then 8 kg/s to slop; the top
#   five trays flood.
# - kerosene, diesel or gas oil: the naphtha up 5-26 %, the residue 35-70 %;
#   up to five trays flood.
# - residue: the base 0.50 -> 0.63 in 5 min and full in 20, then the whole
#   bottoms, 29 kg/s, to slop.
#
# A real refinery's tanks stand at grade; tanks this high are what it takes
# to make a stopped centrifugal pump stop its product completely.
# `crude_unit_holdup_pumps_grade.toml` is the plant as refineries build it.
#
# Generated by `tools/steptime/prototypes/m86_5/make_holdup_pumped.py lifted`.
""",
    "grade": """# M86.5 demo plant: a stopped product pump CUTS its product to just over half,
# and the column holding liquid keeps the rest (docs/DESIGN.md §84.5).
"""
    + COMMON
    + """#
# HERE THE TANKS STAND AT GRADE, as in a refinery, 1.5 km away in the tank
# farm, entered 15 m up (the column's draws' level). Each pump has 100 m of
# head, the long rundown taking most of it. With the pump stopped, the
# column's 0.8 bar over the air outside still pushes product through the
# stopped pump, so the line carries 54-58 % of its product; running, 1.9-2.0
# times it. Measured, the newton solver, from the pump stopped:
#
# - naphtha: its flow to 0.55; the drum barely moves (0.50 -> 0.54, back to
#   0.51 in 40 min): its loop sends the rest back as reflux, and it leaves
#   with the residue, up 36 % in 40 min.
# - kerosene, diesel or gas oil: its flow to 0.55-0.58; the naphtha up
#   3-12 %, the residue 13-25 %.
# - residue: its flow to 0.54; the base 0.50 -> 0.56 in 5 min, 0.98 in 40.
#
# `crude_unit_holdup_pumps_lifted.toml` is the same plant with its tanks on
# high ground, where a stopped pump stops its product completely.
#
# Generated by `tools/steptime/prototypes/m86_5/make_holdup_pumped.py grade`.
""",
}


def make(variant: str, overrides: dict) -> str:
    src = (ROOT / "scenarios" / "crude_unit_holdup.toml").read_text(encoding="utf-8")
    src = src[src.index("[meta]"):]
    lines = {p: dict(v) for p, v in VARIANTS[variant].items()}
    for p, o in overrides.items():
        lines[p].update(o)
    stem = f"crude_unit_holdup_pumps_{variant}"
    old_meta = src[: src.index("[simulation]")]
    src = src.replace(
        old_meta,
        f'[meta]\nname = "{stem}"\ndescription = "{DESCRIPTIONS[variant]}"\n\n',
        1,
    )
    src = HEADERS[variant] + "\n" + src
    for p, (water, ua) in COOLING.items():
        line = lines[p]
        tank = f"{p}_tank"
        draw = f'{{ outlet = "{tank}",'
        assert draw in src, p
        src = src.replace(draw, f'{{ outlet = "{p}_pump",', 1)
        old = (
            f'name = "{p}_draw"\nfrom = "column"\nto = "{tank}"\n'
            "length_m = 20.0\ndiameter_m = 0.15"
        )
        assert old in src, p
        src = src.replace(
            old,
            f'name = "{p}_suction"\nfrom = "column"\nto = "{p}_pump"\n'
            "length_m = 20.0\ndiameter_m = 0.20\nelevation_change_m = -15.0",
            1,
        )
        src += f"""
[nodes.{p}_pump]
type = "pump"
h0_m = {float(line["h0"])!r}
a = {float(line["a"])!r}
on = true

[nodes.{p}_check]
type = "check_valve"
kv = 400.0
full_open_bar = 0.015

[nodes.{p}_cooler]
type = "water_cooler"
ua_kw_per_k = {float(ua)!r}
water_inlet_c = 30.0
water_flow_kg_per_s = {float(water)!r}

[[pipes]]
name = "{p}_discharge"
from = "{p}_pump"
to = "{p}_check"
length_m = 30.0
diameter_m = 0.15

[[pipes]]
name = "{p}_check_outlet"
from = "{p}_check"
to = "{p}_cooler"
length_m = 1.0
diameter_m = 0.15

[[pipes]]
name = "{p}_rundown"
from = "{p}_cooler"
to = "{tank}"
length_m = {float(line["length"])!r}
diameter_m = {float(line["diameter"])!r}
elevation_change_m = {float(line["rise"])!r}
"""
    return src


if __name__ == "__main__":
    variant, out = sys.argv[1], sys.argv[2]
    overrides = json.loads(pathlib.Path(sys.argv[3]).read_text()) if len(sys.argv) > 3 else {}
    pathlib.Path(out).write_text(make(variant, overrides), encoding="utf-8", newline="\n")
