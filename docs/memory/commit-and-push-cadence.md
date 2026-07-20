---
name: commit-and-push-cadence
description: "When to commit and push in the refinery project — end of session, docs/memory update, or work batch"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 10843f67-db12-4aaf-a9ed-1a7853cf0d0f
  modified: 2026-07-20T18:37:03.761Z
---

The user wants me to **always commit AND push** — proactively, without being
asked — after any change lands. Restated explicitly by the user on 2026-07-14:
"always commit and push." Do it at (at minimum) each of these boundaries:
- end of a work session,
- after a docs update or a memory update,
- after completing a work batch (a coherent chunk of implementation),
- after any standalone change the user requested (even a small one).

Default is "commit + push now," not "batch it up for later."

**Branch policy — the 2026-07-14 "commit to main" is superseded by observed
practice.** As of M3/M4 the actual workflow is **one feature branch per
milestone**, pushed to origin, NOT direct-to-main: M3 shipped on
`m3-composition` (6 commits ahead of `main`, never merged), and M4 opened on
`m4-reactor` off that HEAD on 2026-07-20. So at a milestone boundary, branch
`m<n>-<topic>` off the previous milestone's branch; within a milestone, keep
committing to the current branch. The "always commit and push" cadence is
unchanged — only the destination is a per-milestone branch, not `main`. Don't
switch to `main` to commit; it is behind and unmerged.

**Why:** The user wants durable, frequent checkpoints on the private GitHub
remote (`git@github.com:BoykoNeov/refinery.git`, private) rather than a large
uncommitted working tree — easy rollback and off-machine backup.

**How to apply:** At each boundary above, stage, commit (Conventional Commits
per [[refinery]] CLAUDE.md — every commit must pass build/test/clippy -D
warnings/fmt --check), and `git push`. Don't wait to be reminded. This is
project-scoped workflow, kept out of CLAUDE.md to avoid bloating it.
