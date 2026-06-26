---
description: Generate Change History release notes from commits after a given hash
argument-hint: <commit-hash>
allowed-tools: Bash(git log:*)
---

Generate a Change History (release notes) for all commits after `$1`.

Commits in range:
!`git log --oneline $1..HEAD`

Now write the Change History following these rules exactly:

- One line per meaningful, user-visible change. Always in English.
- Group related commits into a single entry (e.g. several tape commits → one tape line).
- Skip: version-bump-only commits, revert+re-revert pairs, build-script-only
  changes, dev-tooling/CI changes, and pure internal refactors with no
  user-visible effect.
- Format each entry exactly as:
  `- **Added/Fixed/Improved <Name>** — short one-sentence description of user-visible behavior only.`
- No commit hashes, no file paths, no sub-bullets, no implementation details.
- Output the whole list wrapped in a ```markdown code block so it is
  copy-pasteable straight into GitHub. Output nothing else.
