# wet_<w>_loop_<o>.toml: the wetter crude, its charge held at 112.8 kg/s by the stripped plant's loop on a Kv-400 valve at opening o
import sys
w, o = sys.argv[1], float(sys.argv[2])
src = open(rf"W:/temp/claude/m65_0/wet_{w}.toml").read()
src = src.replace("kv = 135.0\nopening = 1.0", f"kv = 400.0\nopening = {o}")
src = src.replace(f'name = "wet_{w}"', f'name = "wet_{w}_loop"')
src += f'''
[[controls]]
name = "charge_flow"
measurement = {{ pipe = "crude_line", variable = "flow" }}
actuator = "feed_valve"
algorithm = "pi"
mode = "auto"
action = "reverse"
setpoint_kg_per_s = 112.8
gain_per_kg_per_s = 0.004
integral_time_s = 10.0
initial_output = {o}
'''
open(rf"W:/temp/claude/m65_0/wet_{w}_loop_{o:g}.toml", "w").write(src)
