"""The cold crude unit's presses while its reboiler is held (M67.5, DESIGN
§72.10): the screen's start-up (the furnace to 58 MW at 10, the pumparounds off
and the reboiler walked down from 500, `walk=2.5`), and at a tick of the walk
or after it lands one of M66.4's held presses — the steam down one, four or
seven steps, the furnace down one, four or eight, the pumparounds up to 25, 50
or 100 % — each run 2 700 ticks on.

usage: specs_cold_held.py > cold_held.specs   (PLANT=cold WARM=10 RUN=2700)
"""
PRESSES = [
    ('steam=0.35', 1), ('steam=0.35@10>steam=0.3@10>steam=0.25@10>steam=0.2', 4),
    ('steam=0.35@10>steam=0.3@10>steam=0.25@10>steam=0.2@10>steam=0.15@10>steam=0.1@10>steam=0.05', 7),
    ('furnace=55', 1), ('furnace=55@10>furnace=52.5@10>furnace=50@10>furnace=47.5', 4),
    ('furnace=55@10>furnace=52.5@10>furnace=50@10>furnace=47.5@10>furnace=45@10>furnace=42.5@10>'
     'furnace=40@10>furnace=37.5', 8),
    ('pa=0.25', 1), ('pa=0.5', 2), ('pa=1', 4),
]
specs = []
for tick in [550, 650, 750, 850, 1000, 1500, 2000]:
    for moves, _ in PRESSES:
        specs.append('furnace=58@490>pa=0+walk=2.5@%d>%s' % (tick - 500, moves))
print(';'.join(specs))
