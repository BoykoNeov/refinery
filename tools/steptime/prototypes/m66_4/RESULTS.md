# M66.4 — the cold plant's keys, measured from the running plant (2026-10-18)

Throwaway `m66_4_box.rs` in `crates/scenarios/tests/` (registered in `suite.rs`), on
`dcd100d` (M66.3), release, below-normal priority, the machine under other
load (no timings kept). Each run: the start-up story (the furnace to 58 MW at
tick 10, the pumparounds off at 500, the reboiler held at its running duty less
2.5 MW every 10 ticks from 500) plus one move; `NOSTORY` runs the move alone
from the cold plant. `HALT` is a refused tick; `OK` the state at the last tick.

```
== held mid-walk: steam down a step
BOX @t1500 steam:0.35 feed 346.9 reb 10.007 MW
BOX HALT t1500: numerical error: heat-balance cascade: found no step that improves it at iteration 15: the worst equation is the energy balance of stage 10, off by 8.446e-2 (scaled) against a tolerance of 8.910e-8. The trays at stage 7 run 
== held mid-walk: steam to 100%
BOX @t1500 steam:1 feed 346.9 reb 10.007 MW
BOX OK t3000 reb 0.000 MW declined false R 3.339 bottom 345.7 feed 371.4
== held mid-walk: furnace to 65
BOX @t1500 furnace:65 feed 346.9 reb 10.007 MW
BOX OK t3000 reb 0.000 MW declined false R 3.360 bottom 361.0 feed 382.6
== held mid-walk: furnace down to 50
BOX @t1500 furnace:50 feed 346.9 reb 10.007 MW
BOX OK t3000 reb 5.007 MW declined false R 2.941 bottom 420.5 feed 358.7
== held mid-walk: pumparounds back to 50%
BOX @t1500 pa:0.5 feed 346.9 reb 10.007 MW
BOX OK t3000 reb 0.000 MW declined false R 1.421 bottom 354.2 feed 371.4
== held mid-walk: reboiler up a step
BOX @t1500 up:0 feed 346.9 reb 10.007 MW
BOX OK t3000 reb 12.507 MW declined false R 3.475 bottom 501.8 feed 371.4
== free at the coolest point (t600)
BOX @t600 free:0 feed 318.0 reb 55.007 MW
BOX OK t3000 reb 21.450 MW declined false R 4.051 bottom 540.9 feed 371.4
== free mid-walk (t1500)
BOX @t1500 free:0 feed 346.9 reb 10.007 MW
BOX OK t3000 reb 21.450 MW declined false R 4.051 bottom 540.9 feed 371.4
== free just after landing (t2400)
BOX @t2400 free:0 feed 364.1 reb 0.000 MW
BOX OK t3000 reb 21.450 MW declined false R 4.051 bottom 540.9 feed 371.4
== landed: steam to 10%
BOX @t2400 steam:0.1 feed 364.1 reb 0.000 MW
BOX HALT t2400: numerical error: heat-balance cascade: a column holding its reboiler at 0.000 MW starts from the same column with its reboiler (docs/DESIGN.md §69.3), and the walk from its closure of 63.085 MW stopped at step 9 of 10, the 
== landed: steam down a step
BOX @t2400 steam:0.35 feed 364.1 reb 0.000 MW
BOX OK t4000 reb 0.000 MW declined false R 3.265 bottom 360.1 feed 378.9
== free corner furnace 20 steam 0.1 pa 0
BOX @t100 furnace:20 feed 289.5 reb 87.945 MW
BOX @t100 steam:0.1 feed 289.5 reb 87.945 MW
BOX @t100 pa:0 feed 289.5 reb 87.945 MW
BOX OK t3000 reb 48.235 MW declined false R 4.000 bottom 578.8 feed 289.5
== free corner furnace 20 steam 1.0 pa 0
BOX @t100 furnace:20 feed 289.5 reb 87.945 MW
BOX @t100 steam:1 feed 289.5 reb 87.945 MW
BOX @t100 pa:0 feed 289.5 reb 87.945 MW
BOX OK t3000 reb 43.938 MW declined false R 4.000 bottom 551.9 feed 289.5
== free corner furnace 65 steam 0.1 pa 0
BOX @t100 furnace:65 feed 289.5 reb 87.945 MW
BOX @t100 steam:0.1 feed 289.5 reb 87.945 MW
BOX @t100 pa:0 feed 289.5 reb 87.945 MW
BOX OK t3000 reb 22.989 MW declined false R 4.340 bottom 561.9 feed 385.9
== free corner furnace 65 steam 1.0 pa 0
BOX @t100 furnace:65 feed 289.5 reb 87.945 MW
BOX @t100 steam:1 feed 289.5 reb 87.945 MW
BOX @t100 pa:0 feed 289.5 reb 87.945 MW
BOX OK t3000 reb 19.235 MW declined false R 4.501 bottom 511.6 feed 385.9
== free corner furnace 20 steam 0.1 pa 1
BOX @t100 furnace:20 feed 289.5 reb 87.945 MW
BOX @t100 steam:0.1 feed 289.5 reb 87.945 MW
BOX @t100 pa:1 feed 289.5 reb 87.945 MW
BOX OK t3000 reb 89.114 MW declined false R 4.000 bottom 585.1 feed 289.5
== free corner furnace 65 steam 0.1 pa 1
BOX @t100 furnace:65 feed 289.5 reb 87.945 MW
BOX @t100 steam:0.1 feed 289.5 reb 87.945 MW
BOX @t100 pa:1 feed 289.5 reb 87.945 MW
BOX OK t3000 reb 60.781 MW declined false R 4.000 bottom 582.2 feed 385.9
== free corner furnace 65 steam 1.0 pa 1
BOX @t100 furnace:65 feed 289.5 reb 87.945 MW
BOX @t100 steam:1 feed 289.5 reb 87.945 MW
BOX @t100 pa:1 feed 289.5 reb 87.945 MW
BOX OK t3000 reb 57.009 MW declined false R 4.000 bottom 561.8 feed 385.9
== free corner furnace 20 steam 1.0 pa 1
BOX @t100 furnace:20 feed 289.5 reb 87.945 MW
BOX @t100 steam:1 feed 289.5 reb 87.945 MW
BOX @t100 pa:1 feed 289.5 reb 87.945 MW
BOX OK t3000 reb 85.715 MW declined false R 4.000 bottom 571.1 feed 289.5
== free cold plant: reboiler up a step
BOX @t100 up:0 feed 289.5 reb 87.945 MW
BOX OK t1000 reb 90.445 MW declined false R 4.219 bottom 580.5 feed 289.5
== free cold plant: reboiler down a step
BOX @t100 down:0 feed 289.5 reb 87.945 MW
BOX OK t1000 reb 85.445 MW declined false R 3.781 bottom 579.9 feed 289.5
```
