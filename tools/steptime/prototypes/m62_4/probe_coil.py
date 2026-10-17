"""M62.4c probe (docs/DESIGN.md §67.14): the crude unit under the line flash,
its feed through a control valve into a furnace coil in cells. Writes a plant
per case, runs it (release build), and reports the coil.

Run:  python probe_coil.py N8:8:800:0.24:135 ...   (name:cells:length:bore:kv;
cells 0 for the lumped coil)
"""
import json
import pathlib
import subprocess
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[3]
OUT = pathlib.Path(r"W:/temp/claude/m62_4")  # where each case's plant and run land
OUT.mkdir(parents=True, exist_ok=True)
# The crude unit under the line flash (M62.4a's fixture): a 40 cm transfer
# line, tubes that never burst.
BASE = (
    (REPO / "scenarios" / "crude_unit.toml").read_text(encoding="utf-8")
    .replace('boiloff = "none"', 'boiloff = "flash"\nline_flash = "equilibrium"')
    .replace("tube_failure_c = 550.0", "tube_failure_c = 1951.1")
    .replace(
        'name = "transfer_line"\nfrom = "furnace"\nto = "column"\nlength_m = 15.0\ndiameter_m = 0.08',
        'name = "transfer_line"\nfrom = "furnace"\nto = "column"\nlength_m = 15.0\ndiameter_m = 0.40',
    )
)
assert "line_flash" in BASE and "diameter_m = 0.40" in BASE
EXE = str(REPO / "target" / "release" / "refinery.exe")


def plant(cells, length, diameter, kv, crude_d=0.30):
    src = BASE.replace(
        'name = "crude_line"\nfrom = "crude_source"\nto = "furnace"\nlength_m = 30.0\ndiameter_m = 0.08',
        f'name = "crude_line"\nfrom = "crude_source"\nto = "feed_valve"\nlength_m = 30.0\ndiameter_m = {crude_d}\n\n'
        f'[[pipes]]\nname = "feed_spool"\nfrom = "feed_valve"\nto = "furnace"\nlength_m = 5.0\ndiameter_m = {crude_d}',
    )
    src = src.replace("[nodes.furnace]", f'[nodes.feed_valve]\ntype = "valve"\nkv = {kv}\nopening = 1.0\n\n[nodes.furnace]')
    assert "feed_valve" in src
    if cells:
        src = src.replace(
            "fluid_heating_value_mj_per_kg = 42.686",
            f"fluid_heating_value_mj_per_kg = 42.686\ncoil_cells = {cells}\ncoil_length_m = {length}\ncoil_diameter_m = {diameter}",
        )
    return src


def run(name, src, ticks=3000):
    path = str(OUT / f"coil_{name}.toml")
    out = str(OUT / f"coil_{name}.jsonl")
    open(path, "w", encoding="utf-8", newline="").write(src)
    r = subprocess.run([EXE, "run", path, "--ticks", str(ticks), "--snapshot-every", "1", "--out", out], capture_output=True, text=True)
    t = time.perf_counter()
    subprocess.run([EXE, "run", path, "--ticks", str(ticks), "--snapshot-every", "0", "--out", out + ".last"], capture_output=True, text=True)
    wall = (time.perf_counter() - t) / ticks * 1e3
    if r.returncode != 0:
        return f"{name}: FAILED {r.stderr.strip()[-300:]}"
    lines = open(out).read().strip().splitlines()
    worst = max(json.loads(l)["solver"]["iterations"] for l in lines)
    resolves = max(json.loads(l)["solver"].get("re_solves", 0) for l in lines)
    s = json.loads(lines[-1])
    n = {x["name"]: x for x in s["nodes"]}
    e = {x["name"]: x for x in s["edges"]}
    f = n["furnace"]
    valve = round(n["feed_valve"]["pressure_pa"] / 1e5, 2)
    coil = f["kind"]["coil"]["temperature"] - 273.15
    cells = sorted([x for x in n if x.startswith("furnace__cell_")], key=lambda x: int(x.split("_")[-1]))
    profile = [(round(n[c]["pressure_pa"] / 1e5, 2), round(n[c]["temperature_k"] - 273.15, 1), round(n[c].get("vapour_fraction") or 0, 3)) for c in cells]
    outlet = e["transfer_line"]["stream"]
    mass = outlet["mass_flow"]
    # The outlet's mixture velocity: the first segment out of the furnace.
    rho_l = 1.0 / sum(w / d for w, d in zip(outlet["composition"]["mass_fractions"], [680, 750, 800, 850, 900, 980]))
    q = f.get("vapour_fraction") or 0.0
    mv = sum(w / m for w, m in zip(outlet["composition"]["mass_fractions"], [0.095, 0.13, 0.17, 0.22, 0.3, 0.55]))
    return (
        f"{name}: m={mass:.2f} kg/s, outlet {f['temperature_k'] - 273.15:.2f} C at {f['pressure_pa'] / 1e5:.3f} bar, "
        f"q={f.get('vapour_fraction')}, metal {coil:.2f} C, worst iter {worst}, re-solves {resolves}, "
        f"{wall:.3f} ms/tick\n   cells (bar, C, q): {profile}"
    )


if __name__ == "__main__":
    cases = sys.argv[1:]
    for case in cases:
        name, cells, length, diameter, kv = case.split(":")
        print(run(name, plant(int(cells), float(length), float(diameter), float(kv))), flush=True)
