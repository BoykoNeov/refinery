"""M61.0 probe driver: run crude_unit.toml variants through the probe build.

usage: python scan.py <ticks> <furnace_mw> <pa_scale> <steam_kg_s|none> [steam_C] [feed_stage] [stages]
Prints the last PROBE summary line (or the error) for the variant.
"""
import os
import re
import subprocess
import sys

BIN = r"W:/temp/claude/m61_probe/target/release/refinery.exe"
SRC = r"W:/Claude_projects/refinery/scenarios/crude_unit.toml"
OUT = r"W:/temp/claude/m61_probe_tools/variants"


def variant(furnace_mw, pa_scale):
    text = open(SRC, encoding="utf-8").read()
    coil = os.environ.get("COIL_C")
    if coil:
        text = text.replace("coil_temperature_c = 359.5", f"coil_temperature_c = {coil}", 1)
    text = text.replace("duty_mw = 20.0", f"duty_mw = {furnace_mw}", 1)
    def scale(m):
        return f"duty_mw = {float(m.group(1)) * pa_scale}"
    head, sep, tail = text.partition("pumparounds = [")
    line, nl, rest = tail.partition("\n")
    line = re.sub(r"duty_mw = ([0-9.]+)", scale, line)
    text = head + sep + line + nl + rest
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, f"cu_f{furnace_mw}_pa{pa_scale}_c{coil}.toml")
    open(path, "w", encoding="utf-8").write(text)
    return path


def run(ticks, furnace_mw, pa_scale, steam, steam_c=400.0, extra_env=None):
    path = variant(furnace_mw, pa_scale)
    env = dict(os.environ, PROBE_PRINT="1")
    if steam is not None:
        env["PROBE_STEAM_KG_S"] = str(steam)
        env["PROBE_STEAM_C"] = str(steam_c)
    if extra_env:
        env.update(extra_env)
    p = subprocess.run([BIN, "run", path, "--ticks", str(ticks)], env=env,
                       capture_output=True, text=True,
                       creationflags=0x00004000)  # BELOW_NORMAL_PRIORITY_CLASS
    lines = [l for l in p.stderr.splitlines() if l.startswith("PROBE")]
    summaries = [l for l in lines if l.startswith("PROBE clamp")]
    draws = lines[-5:] if lines else []
    err = "" if p.returncode == 0 else p.stderr.strip().splitlines()[-1]
    return summaries, draws, err


if __name__ == "__main__":
    ticks = int(sys.argv[1])
    furnace = float(sys.argv[2])
    pa = float(sys.argv[3])
    steam = None if sys.argv[4] == "none" else float(sys.argv[4])
    steam_c = float(sys.argv[5]) if len(sys.argv) > 5 else 400.0
    summaries, draws, err = run(ticks, furnace, pa, steam, steam_c)
    if summaries:
        print("first:", summaries[0][:400])
        print("last: ", summaries[-1])
        for d in draws:
            print(d)
    print("ERR:", err if err else "none", "| ticks solved:", len(summaries))
