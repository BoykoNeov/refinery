# Prompt audit — Claude Code configuration loaded in W:\Claude_projects\refinery

Date: 2026-10-05. Proposed only: no file was edited.

## Assumptions (Step 0)

- **Target model:** claude-opus-5-5, the model running this session. None of the audited files names its own model.
- **Scope:** the Claude Code instruction files that load in this project.
  - **Project:**
    - `W:\Claude_projects\refinery\CLAUDE.md` is the only instruction file in the project.
    - `W:\Claude_projects\refinery\.claude\` holds only settings and a lock file, so there are no project rules, skills, commands, agents or output styles.
    - No nested CLAUDE.md, CLAUDE.local.md or AGENTS.md exists below the project root.
  - **Ancestors:** `W:\Claude_projects` and `W:\` have no CLAUDE.md or AGENTS.md. `W:\Claude_projects\.claude\` holds only a settings file.
  - **User level** (an edit here affects every project):
    - `C:\Users\boiko\.claude\CLAUDE.md`
    - The Anthropic skills synced into `C:\Users\boiko\.claude\skills\synced\d2eacee2-…_d66da7d3-…\`: docs, docx, google-workspace, import-memory, pdf, pptx, skill-creator and xlsx. Each SKILL.md was read in full.
    - These skills are in edit scope because they sit under `~\.claude\skills`. A later sync from Anthropic may replace a local edit.
    - There is no `rules\`, `commands\`, `agents\` or `output-styles\` folder.
  - **Managed policy:** none. Neither `C:\Program Files\ClaudeCode\` nor `C:\ProgramData\ClaudeCode\` exists.
  - **Imports:** neither CLAUDE.md imports another file.
  - **Installed plugins** (report only, no edits proposed): `skill-creator@claude-plugins-official`, install folder `C:\Users\boiko\.claude\plugins\cache\claude-plugins-official\skill-creator\d182ca456ca0\`. It holds SKILL.md plus agents\grader.md, comparator.md and analyzer.md.
- **Skipped, unread:**
  - **Settings and credentials** (may hold secrets):
    - `W:\Claude_projects\refinery\.claude\settings.json`
    - `W:\Claude_projects\.claude\settings.local.json`
    - `C:\Users\boiko\.claude\settings.json`
    - `C:\Users\boiko\.claude\.credentials.json`
    - `~\.claude.json`
    - any `.mcp.json`
  - **Auto-memory** (`C:\Users\boiko\.claude\projects\W--Claude-projects-refinery\memory\`): it loads, but the request did not list it.
  - **Plugin files that don't load:** the 7 older cached copies of the skill-creator plugin, and the marketplace's uninstalled plugins.
- **Edit history:**
  - The project CLAUDE.md is in git, with 98 commits touching it.
  - The user-level files are not in git, so history cannot show which side of a conflict is newer.

- **Not read:** the skills' extra reference files (REFERENCE.md, FORMS.md, `references\*.md`, `agents\*.md` in the synced copy). The request scoped skills to SKILL.md. The plugin's agents files were read because the request covers subagent definitions.

## Summary

1. **Your global rules and skill-creator pull in opposite directions** (high, flag).
   - skill-creator tells Claude to research with parallel subagents and to launch two subagents per test case at once.
   - Your global CLAUDE.md forbids launching parallel agents without your say-so.
   - skill-creator also writes temp files to `/tmp`, while your global rule sends them to `W:\temp\claude`.
   - Each time you use skill-creator, Claude has to choose between the two. You decide whether skill-creator's test runs count as pre-approved.
2. **The project's corpus command writes `before.json` into the repo folder** (high, flag). That collides with your rule that temp files never land in a project tree, and the file isn't git-ignored.
3. **skill-creator tries to steer how hard Claude thinks with prose** (medium, edit proposed). The line "billions a year … take your time and really mull things over" does this. On this model the effort setting is the only lever that does that, so the sentence is dead weight.
4. **The synced pdf skill is mostly a tutorial on standard libraries** (medium, edit proposed). The docx and pptx skills next to it already drop that kind of material ("The model knows the API; these are the footguns").

Counts:

| Group | Count | Detail |
|---|---|---|
| 1 (dated prompt text) | 4 | 1 medium edit, 3 low flags |
| 2 (configuration files) | 8 | 3 high conflict flags, 1 medium ambiguity flag, 1 medium edit, 3 low flags |
| **Total** | **12** | |
| 3 (tool descriptions) | not applicable | no tool definitions in scope |
| 4 (request code) | not applicable | no request-building code. The sub-agent roster check also comes out at zero: skill-creator's grader, comparator and analyzer have distinct jobs. |

## Findings, highest confidence first

### F1 — Conflict: parallel subagents (high, flag)
- **Where:**
  - `C:\Users\boiko\.claude\CLAUDE.md:105-119`: "Do NOT launch multiple agents/subagents in parallel on your own judgement…"
  - The same file at `:143-160`: explore inline, not via agent.
  - Versus `…\skills\synced\…\skill-creator\SKILL.md:60`: "research in parallel via subagents if available".
  - And `:169-171`: "For each test case, spawn two subagents in the same turn … Launch everything at once".
- **Pattern:** Group 2, instruction files that contradict each other.
- **Why it's flagged and not edited:**
  - Both files are outside the project, and neither has history to show which is newer.
  - The skill's text is a launch instruction. Your global text is a prohibition, and the guide never weakens a prohibition to settle a conflict.
- **What you decide:** whether invoking skill-creator counts as permission to run its test cases in parallel.
  - If yes: add one line to your global CLAUDE.md naming that exception.
  - If no: expect Claude to stop and ask at every test round.
  - The same question applies to the installed plugin copy of skill-creator.

### F2 — Conflict: where temp files go (high, flag)
- **Where:**
  - `C:\Users\boiko\.claude\CLAUDE.md:7-23`: temp files go under `W:\temp\claude`, never in a project tree.
  - Versus `W:\Claude_projects\refinery\CLAUDE.md:81-82`: `corpus … --out before.json` and `--baseline before.json`. That is a relative path, so it lands in the repo root.
  - `W:\Claude_projects\refinery\.gitignore` does not ignore `*.json`, so the file shows up as untracked. That makes it easy to commit by accident.
- **Pattern:** Group 2, contradiction between a project file and a user-level file.
- **Why it's flagged:** the request says to flag conflicts across that boundary and propose no edit. Your global file also says project files win on conflict, but the project file only shows an example command and never states a location.
- **What you decide:** either point the example at `W:\temp\claude\…\before.json` in the project CLAUDE.md, or accept a repo-root baseline and git-ignore it.

### F3 — Conflict: `/tmp` and macOS `open` in skill-creator (high, flag)
- **Where:**
  - `…\skills\synced\…\skill-creator\SKILL.md:369`: "Write to a temp file (e.g., `/tmp/eval_review_<skill-name>.html`) and open it: `open /tmp/…`"
  - The same file at `:440-441`: "Copy to `/tmp/skill-name/`".
  - Versus your temp rule at `C:\Users\boiko\.claude\CLAUDE.md:7-23`.
- **Pattern:** Group 2, contradiction. The file is also a provider-managed skill.
- **Why it's flagged:** on this Windows machine `open` is not a command, and `/tmp` is not your temp location. Your global rule already overrides it for Claude, so no edit is needed for correct behaviour. Flagged so you know the two disagree.

### F4 — Prose that steers thinking depth (medium) → remove
- **Where:** `C:\Users\boiko\.claude\skills\synced\d2eacee2-a2d7-4a09-b677-411f8c90a828_d66da7d3-6136-45a3-960a-6153418850a6\skill-creator\SKILL.md:306`. Affects all projects.
- **Evidence:** "This task is pretty important (we are trying to create billions a year in economic value here!) and your thinking time is not the blocker; take your time and really mull things over."
- **Pattern:** Group 1b, prose that steers thinking depth. The stakes framing also matches 1a, pressure language.
- **Why it's obsolete:**
  - On claude-opus-5-5, thinking is always on, and the effort setting is the only control over how much it thinks.
  - Prose like this doesn't change thinking depth. The stakes framing only adds pressure.
  - Skills can't set effort themselves; the session's effort setting governs it.
- **Action:** remove the sentence. Keep the "draft, then re-read" advice that follows it. See `hunk1-skill-creator.diff`.
- **Caveat:** a later sync may restore it.

### F5 — SKILL.md explains what the model already knows (medium) → rewrite
- **Where:** `C:\Users\boiko\.claude\skills\synced\d2eacee2-a2d7-4a09-b677-411f8c90a828_d66da7d3-6136-45a3-960a-6153418850a6\pdf\SKILL.md:13-167` and `:189-294`. Affects all projects.
- **Evidence:** worked examples of standard library calls: merging with pypdf, a reportlab "Hello World!" page, and `qpdf`/`pdftk` merge, split and rotate commands.
- **Pattern:** Group 2, a verbose SKILL.md explaining things the model already knows.
- **Why it's obsolete:**
  - These are general library APIs.
  - The sibling docx and pptx skills already state "The model knows the API; these are the footguns" and keep only the traps.
  - The pdf skill's only real trap, Unicode subscripts rendering as black boxes in ReportLab, is kept.
- **Action:** replace both ranges with a one-line pointer to the tool table. Keep the subscript section and the REFERENCE.md and FORMS.md pointers. Every tool choice the removed sections made survives as a row in the Quick Reference table: layout-preserving text, image extraction, watermarking, encrypt/decrypt, and OCR with its install note. See `hunk2-pdf.diff`.
- **Caveat:** a later sync may restore it.

### F6 — Internal ambiguity in the full-paths rule (medium, flag)
- **Where:**
  - `C:\Users\boiko\.claude\CLAUDE.md:196-198`: "give the complete absolute path" in "chat replies, summaries, commit messages, plans, docs".
  - Versus `:212-215`: "Inside files that live in the repo — … paths written into a doc that ships with the project — keep using whatever form that file needs".
- **Pattern:** Group 2 / 1c, the same topic ruled two ways.
- **Why it's flagged:**
  - A plan or doc written into the repo fits both sentences, and Claude has to guess.
  - Commit messages are listed for absolute paths but also live in the repo's history.
  - There is no history to say which wording you meant last.
- **What you decide:** whether in-repo docs and commit messages get absolute paths (`W:\…`), or only what you read in chat and in files handed to you outside the repo. One clarifying clause settles it.

### F7 — Capital-letter emphasis in the global file (low, flag)
- **Where:** `C:\Users\boiko\.claude\CLAUDE.md`: headings and lines 25, 50, 70, 107, 109, 145, 194, 197.
- **Evidence:** "quoted IMMEDIATELY", "do NOT", "ONLY", "FULL paths", "hard gate that overrides…".
- **Pattern:** Group 1a, pressure language.
- **Why it's only a flag:** every rule carries a Why line, and several were evidently strengthened after real misses. That is the scoped kind of emphasis the guide allows, so there is no edit.

### F8 — All-caps reminder in skill-creator (low, flag)
- **Where:** `…\skills\synced\…\skill-creator\SKILL.md:451`.
- **Evidence:** "I'm gonna go all caps here: GENERATE THE EVAL VIEWER *BEFORE* evaluating inputs yourself."
- **Pattern:** Group 1a / 1d.
- **Why it's only a flag:** it targets one observed failure in one environment (Cowork) and gives its reason. That is scoped emphasis.

### F9 — Time-relative padding in skill-creator (low, flag)
- **Where:** `…\skills\synced\…\skill-creator\SKILL.md:34`.
- **Evidence:** "If you haven't heard (and how could you, it's only very recently that it started)…"
- **Pattern:** Group 1d, migration- or time-relative phrasing.
- **Note:** harmless today; it will read oddly as it ages.

### F10 — Two copies of skill-creator that have drifted (low, flag)
- **Where:**
  - Synced copy: `…\skills\synced\…\skill-creator\SKILL.md`.
  - Plugin copy: `C:\Users\boiko\.claude\plugins\cache\claude-plugins-official\skill-creator\d182ca456ca0\skills\skill-creator\SKILL.md`.
- **Evidence:** the two files differ only in the "Package and Present" section, lines 408-416. The plugin copy says `present_files` only; the synced copy also allows `SendUserFile`.
- **Why it's only a flag:** this session lists only `anthropic-skills:skill-creator`, so the plugin copy looks inactive.
- **Possible action:** uninstalling the plugin would leave one source. Plugins get no edits.

### F11 — "Current milestone" names no open milestone (low, flag)
- **Where:** `W:\Claude_projects\refinery\CLAUDE.md:145-151`.
- **Evidence:** "Work only on the current milestone unless asked. **Latest closed: M38**…". `W:\Claude_projects\refinery\docs\ROADMAP.md` has no M39 heading, so nothing is open.
- **Note:** the rule still reads sensibly as "start nothing new unless asked". A future reader may look for a current milestone that doesn't exist.

### F12 — The size warning covers two of four large docs (low, flag)
- **Where:** `W:\Claude_projects\refinery\CLAUDE.md:153-154`, which warns that ROADMAP.md and DESIGN.md are too large to read whole.
- **The other two:** DEFERRED.md (141 KB, 592 lines) and MILESTONES.md (142 KB, 2,071 lines) are also large. Line 22 says to read DEFERRED.md before scoping a slice.
- **Why it's only a flag:** nothing in the repo contradicts the guide, so no edit.

## Plugin files reviewed, no edits proposed

- skill-creator plugin SKILL.md: the same findings as F1, F3, F4, F8 and F9 apply at the same line numbers.
- agents\grader.md, comparator.md and analyzer.md: no dated patterns beyond one "Be thorough" (grader.md:220), which is paired with exactly what to check.

## Checked with no findings

- **Project guide** `W:\Claude_projects\refinery\CLAUDE.md`: every path, command, flag and API name it mentions exists in the repository, and each emphatic rule carries its reason.
- **docs, docx, xlsx, pptx, google-workspace, import-memory.** Their "never" lines each carry a concrete reason (file corruption, glyphs missing, data loss, privacy policy), so they are kept.
- pptx's "NEVER use accent lines…" list names specific design defaults, which is the form the guide says to keep.
- Long trigger descriptions are routing text and may carry urgency.

## Proposed diff

Two hunks, in `W:\temp\claude\prompt-audit-refinery\`:
- `hunk1-skill-creator.diff` (F4)
- `hunk2-pdf.diff` (F5)

Both target synced Anthropic skills under `C:\Users\boiko\.claude\skills\synced\…`. Both affect all projects, and a later sync may overwrite them. The edited copies are in `…\b\`.

No hunk touches `W:\Claude_projects\refinery\CLAUDE.md` or `C:\Users\boiko\.claude\CLAUDE.md`: everything there is a flag that needs your decision.
