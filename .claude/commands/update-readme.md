---
description: Update README.md feature list from commits after a given hash
argument-hint: <commit-hash>
allowed-tools: Bash(git log:*), Bash(git show:*), Read, Edit
---

Update [README.md](README.md) to reflect the user-visible changes in all commits after `$1`.

Commits in range:
!`git log --oneline $1..HEAD`

Do this:

1. Read README.md to understand its current structure and wording.
2. For each meaningful, **user-visible** change in the range, decide whether it
   is already covered by an existing bullet/section in README.md:
   - If a feature is **new**, add a concise bullet in the most fitting existing
     section (usually the `## Features` list), matching the exact tone, phrasing,
     and `(RP2350 only)`-style qualifiers already used.
   - If a feature **changed/expanded**, edit the existing bullet in place rather
     than adding a duplicate.
   - If an existing bullet is now **wrong/outdated**, correct it.
3. Skip non-user-visible work: internal refactors, build-script/CI changes,
   memory-optimization-only commits, version bumps, revert pairs, dev tooling.
   When unsure whether something is user-visible, inspect it with
   `git show <hash>` before deciding.
4. Make all changes via Edit on README.md. Keep edits minimal and surgical —
   do not reflow or restyle untouched lines.
5. When done, output a short plain-text summary of which README lines were
   added / changed / removed (not a code block).
