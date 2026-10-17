# wet_<w>_pa<s>.toml: the wetter crude with every pumparound scaled by s
import re, sys
w = sys.argv[1]
src = open(rf"W:/temp/claude/m65_0/wet_{w}.toml").read()
for s in [float(x) for x in sys.argv[2:]]:
    out = re.sub(r"duty_mw = (\d+\.\d+) \}", lambda m: f"duty_mw = {float(m.group(1))*s:.4f} }}", src)
    out = out.replace(f'name = "wet_{w}"', f'name = "wet_{w}_pa{s:g}"')
    open(rf"W:/temp/claude/m65_0/wet_{w}_pa{s:g}.toml", "w").write(out)
