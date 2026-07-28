---
name: published-anchor-envelope
description: "When the tabulated source is unreachable, degrade the published anchor from a point match to an envelope — never transcribe numbers from a search summary"
metadata:
  type: feedback
---

M4.2 (2026-07-28) needed the roadmap's required anchor: a reference against
published FCC lump yields. Every source tabulating the five rate constants was
paywalled — ScienceDirect, MDPI, Wiley, ResearchGate, Academia, core.ac.uk and
two university hosts all returned 403, and the Semantic Scholar API 429'd.
Fourteen web calls produced two readable papers and zero constant tables.

The move that worked: **stop searching and re-read what you already fetched.**
One of those two papers reproduced industrial plant data (Ali & Rohani 1997 via
Olufemi et al. 2013, Tables 1–4) at stated conditions. That is a published
anchor, just a coarser one — an ENVELOPE (gasoline 41.8–46.9 wt%, coke 5.3–5.8,
conversion 79 at 795–808 K) rather than a point. Constants were then
**calibrated** to land inside it and labelled as calibrated, not transcribed.

**Why:** an envelope from data you read is honest and falsifiable; a `k` table
transcribed from a search-result summary is the silent-wrong-number failure this
project exists to refuse ([[kv-handcalc-reference]]). The reaction↔value
assignment in such a summary is exactly the part that gets mangled, and nothing
downstream would ever complain.

**How to apply:**
- Filter sources by *transcribability*, not fame: reject any that leaves the
  independent variable, its units, or whether a scale factor (catalyst loading,
  C/O, space velocity) is folded into the constant ambiguous.
- Say in the test file what the envelope CAN catch (decade-scale unit slips) and
  what it cannot (percent-level errors in any one constant), then cover the rest
  with closed forms. Two gate kinds, each labelled
  ([[unfalsifiable-is-a-claim-about-coverage]]).
- **Name the circularity, and do not skip this one.** If you CALIBRATE constants
  against the envelope and then gate on the envelope, that is one anchor used
  twice: the gate is a **regression lock on the calibration, not an independent
  validation of it**. It still earns its place — it catches a fault introduced
  later — but writing "the only anchor with no ceiling" over it is the same
  overclaim as [[well-posed-is-not-correct]] and M1's tautology trap
  ([[kv-handcalc-reference]]). Advisor caught exactly this wording in M4.2.
- Watch for bands spliced across tables. M4.2's light-gas band was a residual
  built from two tables of one paper, one of them internally inconsistent —
  flagged in the test as the weakest of the four rather than presented as equal.
- Record the degradation in ROADMAP as a caveat carried forward, with what would
  upgrade it — the user may have access you don't.
- Fetchable hosts, for next time: iiste.org and rjpbcs.com serve PDFs, and the
  Read tool renders a fetched PDF's pages as images when WebFetch's text
  extraction fails on a compressed stream.
