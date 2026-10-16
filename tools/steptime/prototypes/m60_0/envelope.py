import subprocess, os

base = [(4, 15.0), (6, 15.0), (7, 10.0)]
rows = []
for scale in [0.0, 0.5, 1.0, 1.5, 2.0, 3.0]:
    pa = ", ".join(f"{{ stage = {s}, duty_mw = {d * scale} }}" for s, d in base)
    for reflux in [1, 2, 4, 6, 8]:
        r = subprocess.run(["python", r"W:/temp/claude/m60-scripts/real.py", "350"], capture_output=True, text=True,
                           env={**os.environ, "REFLUX": str(reflux), "TICKS": "300", "PA": pa})
        out = r.stdout
        if "FAILED" in out:
            reason = "dry" if "liquid leaving" in out else ("no-reflux-helps" if "more reflux does not" in out else ("bottoms" if "bottoms rate" in out else out.split("error:")[-1][:60].strip()))
            cell = f"FAIL:{reason}"
        else:
            import re
            m = re.search(r"'reflux_ratio': ([0-9.]+)", out)
            cell = f"ok R_run={float(m.group(1)):.2f}"
        rows.append((scale, reflux, cell))
        print(f"PA x{scale:<4} ({40*scale:5.1f} MW)  reflux {reflux}: {cell}", flush=True)
