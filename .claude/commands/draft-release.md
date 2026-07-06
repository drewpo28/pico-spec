---
description: Generate Change History and create a draft GitHub release via draft-release.sh
argument-hint: [new-version e.g. 1.2.27]
---

Create a draft GitHub release with auto-generated Change History notes.

Previous release tag: !`git tag --sort=-v:refname | grep '^v' | head -1`
Current PORT_VERSION: !`grep -oP 'set \(PORT_VERSION "\K[0-9.]+' CMakeLists.txt`
Commits since previous tag: !`git log --oneline $(git tag --sort=-v:refname | grep '^v' | head -1)..HEAD`

Steps:

1. **Generate the Change History** for the commit range above, following the
   rules in `.claude/commands/getch.md` exactly (grouping, skip-list, entry
   format `- **Added/Fixed/Improved <Name>** — ...`). If there are no
   meaningful commits, stop and tell the user there is nothing to release.

2. **Write the notes** to `build-logs/release-notes-draft.md` — raw markdown
   WITHOUT the ```markdown code fence (the file goes verbatim into the GitHub
   release body). Start the file with this exact header, then the entries:

   ```
   ## Change History
   <br>

   ```

3. **Run the release script**:
   - With a version argument: `./draft-release.sh $1 --notes build-logs/release-notes-draft.md`
     (this bumps PORT_VERSION, commits and pushes the bump — that is expected).
   - Without: `./draft-release.sh --notes build-logs/release-notes-draft.md`.
   - The full build takes several minutes — use a 15-minute timeout or run in
     background and wait for completion.
   - If the working tree is dirty and a version bump was requested, the script
     refuses — report that to the user instead of committing anything yourself.

4. **Report the result**: show the generated Change History and the draft
   release URL, and remind that publishing is
   `gh release edit vX.Y.Z --draft=false` (tag + source archives appear on
   publish).
