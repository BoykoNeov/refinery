---
name: commit-and-push-cadence
description: "When to commit and push in the refinery project — end of session, docs/memory update, or work batch"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 10843f67-db12-4aaf-a9ed-1a7853cf0d0f
---

The user wants me to **always commit AND push** — proactively, without being
asked — after any change lands. Restated explicitly by the user on 2026-07-14:
"always commit and push." Do it at (at minimum) each of these boundaries:
- end of a work session,
- after a docs update or a memory update,
- after completing a work batch (a coherent chunk of implementation),
- after any standalone change the user requested (even a small one).

Default is "commit + push now," not "batch it up for later."

**Commit directly to `main`, no feature branch.** Reaffirmed 2026-07-14 when
offered a branch-first option: the user chose "always commit and push to main."
This overrides the harness default of branching off the default branch — for
this project, commit and push straight to `main`, and don't ask each time.

**Why:** The user wants durable, frequent checkpoints on the private GitHub
remote (`git@github.com:BoykoNeov/refinery.git`, private) rather than a large
uncommitted working tree — easy rollback and off-machine backup.

**How to apply:** At each boundary above, stage, commit (Conventional Commits
per [[refinery]] CLAUDE.md — every commit must pass build/test/clippy -D
warnings/fmt --check), and `git push`. Don't wait to be reminded. This is
project-scoped workflow, kept out of CLAUDE.md to avoid bloating it.
