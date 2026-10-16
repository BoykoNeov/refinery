# M62.0 probe — steam stripping on the heat-balance cascade (2026-10-16)

Throwaway. `probe_steam.patch` applies to `cfa270e` (M61 closed); `m62_walk.rs`
goes in `crates/scenarios/tests/`; settings through `refinery_solvers::probe`
(steam kg/s, inlet °C, held reboiler MW). Run (Windows, below normal):

```
PLANT=<plant> [COLD=<steam kg/s>] [PA_SHARE=0.6] [STEAMS=a,b,..] [FURNACE_WALK=..] \
[PA_WALK=i:MW,..] [PROBE_BASE_PA=0,15,15,10 PROBE_BASE_R=4] \
cargo test --release -p refinery-scenarios --test m62_walk -- --nocapture
```

Model in the patch: steam an inert vapour entering below the bottom stage at
400 °C; each stage's K at its oil partial pressure `P·V/(V+S)`; the steam's
sensible heat in every stage energy balance; the bottom stage's energy balance
an equation with the reboiler's duty held (`Specification::Held`), the reflux
ratio an unknown. Steam's condensing heat computed beside the condenser duty,
not in the engine's books. No vapour floor in the held solve — the floor is
MEASURED, not applied.

Plants: `crude_unit_steam_walk.toml` — `crude_unit.toml` with a fourth
pumparound on stage 2 (starts at 0, raised warm), coil UA 6 000 kW/K, tube
limit 550 °C kept. `crude_unit_steam_cold.toml` — the same at the end state:
furnace 65 MW, pumparounds 2.5 / 3.75 / 3.75 / 2.5 MW, declared reflux 3.

## Results

1. **Reboiler to zero, warm** (furnace 65 MW, top pumparound 10 MW, steam
   1 kg/s). Holding the pumparounds at their duty, the reboiler walks 62 → 22 MW
   and stage 1 runs dry (reflux → 0.29): heat comes out of the pumparounds that
   no longer comes in. Taking 60 % or 80 % of each reboiler step off the
   pumparounds, the reboiler reaches 0: reflux 2.95 (pumparounds at 25 %) or
   4.28 (at 1 %).
2. **Bottoms temperature against steam, reboiler at zero** (furnace 65 MW,
   pumparounds 25 %):

   ```
   steam kg/s   0.02   0.05   0.1    0.25   0.5    1      2      4      8
   bottom °C    395.6  393.3  390.6  385.1  378.8  370.5  361.4  349.7  339.0
   oil vapour   1      2      3      4      6      8      10     13     17   mol/s, bottom stage
   reflux       2.76   2.77   2.78   2.83   2.89   2.99   3.29   3.64   4.22
   ```

   The temperature falls BELOW the feed stage (386 → 377 → 370 °C at 1 kg/s),
   the stripping gradient a real column shows. Feed 500 °C liquid-equivalent,
   91 % flashing; coil 500 °C (under 550). Water's dew point at the top 42–83 °C
   against a top tray of 152–168 °C: no water condenses in the column.
   Steam's condensing heat in the condenser: 2.64 MW per kg/s.
3. **The floor, measured** (98 mol/s = 20 % of the feed's 494 mol/s): above the
   feed every stage carries ≥ 208 mol/s in every run, so it never binds there.
   Below the feed the OIL vapour is 1–35 mol/s. Oil + steam ≥ 98 needs steam
   ≥ ~1.6 kg/s. So "steam counts" refuses (or must stop the key) under ~2 kg/s;
   "no floor below the feed" runs to 0.02 kg/s with no stiffness. At steam 0
   with the reboiler at 0 the stripping section has no vapour at all: not
   evaluable.
4. **Cold start** from the file at the end state: the M61 cold warm-up (the
   fixed point, reboiler kept) refuses on tick 1 (a negative vapour at the
   feed). A continuation works: a base column with the reboiler and the shipped
   crude unit's pumparounds (0/15/15/10 MW, reflux 4), then 10 Newton steps
   moving the pumparounds, the feed's enthalpy, the steam and the held duty to
   the plant's. Tick 1 18.9 ms; lands on the warm walk's answer (reflux 3.30).
   A base at the plant's own pumparounds fails (saturated feed or hot).
5. **Range, reboiler at zero, steam 2 kg/s, warm**: furnace 35–80 MW (at 30 the
   gas-oil tray, stage 7, runs dry; at 80 the coil is at 532 °C); the
   pumparounds 0 – ~300 % of the plant's (at ~400 % the reflux reaches 0 and
   the top runs dry). **All pumparounds at ZERO run** (reflux 4.68): with no
   reboiler the trays above the feed stay wet without them — M61's "needs a top
   pumparound" was an artefact of walking the reboiler down with 40 MW of
   pumparounds still on.
6. **Timing**: mean 0.29–0.59 ms a tick, worst 2.23 ms (one step change of the
   steam), 0.5–1.9 elsewhere; cold tick 1 18.9 ms (M61: 5.9).
7. **Books**: the published reboiler closure (which knows no steam) reads
   0.47 MW per kg/s = `S·cp_steam·(400 °C − top)` — the steam's sensible heat in
   minus out, exactly. The energy books close once steam is booked.

# M62.0, after the user's water decisions (2026-10-16)

The user chose ONE boiling water component, free water modelled everywhere,
and steam into the column (reboiler kept) inside M62. Probed:

8. **Unused steam and water components move nothing**: `crude_unit.toml` with
   both added to the slate at zero, 2 000 ticks — every number in the first,
   middle and last snapshots identical.
9. **A pure steam line runs today** (a `phase = "gas"` steam, 10 bar, 400 °C,
   a gas valve, a 1.8 bar destination): both solvers identical. The gas branch
   heats the steam by the valve's `Δp/ρ` (+127 K here) — §64's known model
   term; a line-flash vapour is not heated (§58 fork 6).
10. **`crude_unit.toml` under `line_flash = "equilibrium"`, furnaces and
    columns let through** (`water/crude_unit_flashing.toml`): the 8 cm
    transfer line chokes on its vapour (feed 104 → 76 kg/s) and the column's
    cold start refuses; at 40 cm it runs, both solvers identical, 126 kg/s.
    Three findings: the furnace is not a flashing node (it heats a "liquid" at
    its 341 °C liquid-equivalent where the boiling stream would stand near
    284 °C at its 2.06 bar, so its coil reads the wrong fluid); EVERY column
    draw carries the column node's feed vapour share (q = 0.428) — a draw is a
    liquid at its own stage's temperature, so this is a bug to fix; and the hot
    draws boil into 1-atm tanks (the naphtha tank vents 14.5 of its 27.8 kg/s,
    partly the bug; the residue arrives at 587 °C). Real plants cool products
    before storage; coolers are refused on a flashing plant today.
11. **Free water, by hand** (`water/free_water_check.py`): water's vapour
    pressure from Clausius–Clapeyron with a declared 40.65 kJ/mol reads steam
    tables to 0.1 % at 100–150 °C, +3.9 % at 200, +9.1 % at 250, +14 % at
    300 °C. Free water and oil boil together where `P_oil + P_w = P`: a heavy
    naphtha boils at 174.3 °C dry at 1.8 bar, 110.7 °C wet. With one oil cut
    the three-phase window is one temperature (Gibbs: two components, three
    phases) — the water boils off in a jump, the M53 pure-cut case again; on a
    three-cut naphtha with 5 mol % water it is a window of about 98–100 °C.
    Above it every drop of water is vapour (`y_w·P < P_sat,w`).
