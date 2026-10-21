# M73.0 — trays that flood or weep (C1): the probe, 2026-10-21

Asked after M72 as "what is next"; the user chose 1, flooding and weeping (C1).
Their earlier decisions (DESIGN §77 intro, 2026-10-20): a tray that floods or
weeps is **shown first, then the products worsened**; **real weeping on new
plants only**. No engine code changed.

## The probe

- `probe_loads.patch` (on 211b5f8, `crates/solvers/src/heat_balance.rs`): after
  each converged heat-balance solve, every stage's temperature, liquid and
  vapour mole fractions (`y = K·x`), oil vapour, water vapour, liquid in /
  down / drawn, loop circulation and free water are kept per thread as JSON;
  on only with `PROBE_HYD` set. `probe_last_loads()` reads it.
- `probe_driver.patch` + `m73_0_hyd.rs`: M72.6's grid driver
  (`tools/steptime/prototypes/m72_6/`) printing `LOADS <spec>: <json>` after
  each run's `RESULT`. Put `m73_0_hyd.rs` in `crates/godot-ext/tests/`.
- `specs.py <plant>`: a coarse sample of the column screen's ALLOWED settings
  (`demo/column_region_<plant>.json`): loops 347 (furnace 0–40, reflux
  0.5–15, loop flow 0–200 %, return −60/0/+60 K), crude unit 163 (furnace,
  reflux, pumparounds 0–4×), stripped 135 (furnace 37.5–65, steam 5–100 %,
  pumparounds 0–12 MW). One jump each at tick 1 000, then 4 000 ticks; 14
  threads, about 8 minutes in all.
- `hyd.py`: sieve-tray hydraulics on those loads, each correlation cited in
  the file — Fair's flooding chart as fitted by Lygeros & Magoulas (1986),
  Sinnott's weep point (C&R Vol. 6 eq. 11.84), Francis weir crest, dry-plate
  and residual heads, downcomer backup (eqs. 11.85–11.92). Vapour density
  ideal gas at the stage's T and P (the water vapour included); liquid density
  by volume from the cuts' declared densities, each 0.65 kg/m³ lighter per K
  above 15 °C; surface tension per cut from a critical temperature estimated
  off its boiling point. `analyze.py <plant> [sigma_scale] [expand]` and
  `bykey.py <plant>` read `W:\temp\claude\m73_0\out\` (`base_<plant>.out`
  kept here; the sweeps, 3.3 MB, re-run in 8 minutes).
- Geometry: each tray designed on the plant file's own settings — diameter
  at 80 % of flood, 0.61 m spacing, 50 mm weir, 5 mm holes at 10 % of the
  active area, downcomers 12 % of the column, clearance at 0.40 m/s under the
  downcomer, and the fewest liquid passes (1, 2, 4) holding the downcomer's
  backup at 0.6 of its limit � four on every tray, and still 0.66�0.83 at the
  file's settings. Or the whole column at its widest tray's diameter.
- `lines.py <plant>`: one key moved alone from the file's settings (the
  reflux 0.5�15 on the loop plant and the crude unit; the steam valve 5�100 %
  on the stripped plant), the worst tray's flood, weep and backup.
- The scripts read `$M73_OUT` (default `W:\temp\claude\m73_0\out`).

## Findings

1. **The size.** At the file's settings the 10-tray crude unit needs 6.2–8.7 m
   per tray, the 13-tray loop plant 6.2–9.5 m (one diameter: 8.7 and 9.5 m);
   a 100 kg/s crude (≈ 65 000 bbl/d) — a real column's size, a little wide.
   The widest trays are the ones the loops load and the one above the
   reboiler. The stripped plant's two trays under the feed carry only steam
   (5 kg/s) and want 2 m against 6.5 m above.
2. **The keys move a tray's vapour 0.3 to 3.2–3.9 times what it was designed
   for** (loops, crude unit; liquid 0.1 to 3.8–5.5 times). A real sieve tray
   runs from about 0.6 of its design vapour (weeping, finding 4) to 1.25
   (flooding); a valve tray down to about 0.25. No tray geometry spans what
   the screen allows.
3. **So most of the allowed region floods or weeps.** Each tray its own:
   loops — 207 of 347 sampled settings flood a tray, 162 weep one; crude unit
   116 / 81 of 163; stripped 65 / 91 of 135. One diameter: loops 158 / 214,
   crude 104 / 106, stripped 0 / 135 (its steam trays weep everywhere).
   **The reflux is the key that floods** (every setting at 8 and 15 floods,
   none weeps; at 0.5–2 every one weeps), then the loops' flow and a colder
   return; **the furnace barely matters** on a reboiled column — the
   reboiler makes up whatever the feed does not bring (`bykey.py`).
4. **Weeping comes long before the vapour floor.** Designed at the file's
   settings, a tray weeps below 0.8–1.7 times the feed's molar rate of
   vapour; `VAPOUR_FLOOR_OF_FEED` holds 0.20. The floor never decides where a
   tray weeps, so it can stay as the solve's aid it also is (§65.0: at 0.05
   the solve stalls) with real weeping reported beside it.
5. **The pressure drop**: 100–165 mbar over the column at the file's settings
   (6–9 % of 1.8 bar), 60–630 mbar over the allowed region (past flood the
   correlation means little). Fed back into the stages' pressures it would
   raise the bottom's bubble point about 5–7 K and the middle's about 2 K
   (Trouton estimate, `dT = (R/88)·T·ln(P'/P)`): every product moves, and the
   column's equations gain one pressure per stage that depends on the flows.
6. **Sensitivity**: surface tension ±50 % moves the flooded count not at all
   (the tray is designed with the same property it is checked with) and the
   weeping count 162 → 127 / 175; liquid density at its 15 °C value, 119.
   What a tray is designed for matters far more than the properties.
7. **The downcomers back up** at the file's settings already on single-pass
   trays this wide (backup 1.0–1.3 of the limit): the loops' liquid, 240–290
   kg/s on one weir, is two- or four-pass liquid. Passes are a geometry field.

## What "the products worse" would cost (not built)

- **Entrainment** (flooding): liquid carried up from tray `j` to `j − 1` at
  `x_j` — the shape of a loop's return without its cooler, so every
  component's balance stays tridiagonal (§77.0 finding 4). Its rate from Fair's
  entrainment chart depends on the flows, one more nonlinearity in Newton.
- **Weeping**: liquid falling through the holes skips the contact — liquid
  from `j − 1` reaching `j + 1` breaks the tridiagonal shape; or a tray
  efficiency (Murphree) that falls as the tray weeps, which changes what a
  stage IS (§5's deferred efficiency; `N` stops meaning one thing).
- Either way the shipped plants' numbers stay where they are only if the
  new behaviour is off on them (new plants only, the user's decision).
