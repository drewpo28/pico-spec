---
description: Generate Change History and publish a PUBLIC GitHub pre-release via draft-release.sh --prerelease
argument-hint: [new-version e.g. 1.2.27]
---

Create a **public** GitHub pre-release with auto-generated Change History
notes. Unlike /draft-release, the result is immediately visible to users
(marked "Pre-release"): the git tag and the "Source code (zip/tar.gz)"
archives are created right away, and an unpushed HEAD (e.g. the version-bump
commit) is pushed by the script.

Previous release tag: !`git tag --sort=-v:refname | grep '^v' | head -1`
Current PORT_VERSION: !`grep -oP 'set \(PORT_VERSION "\K[0-9.]+' CMakeLists.txt`
Release for this version: !`VER=$(grep -oP 'set \(PORT_VERSION "\K[0-9.]+' CMakeLists.txt); gh release view "v$VER" -R drewpo28/pico-spec --json name,isDraft,isPrerelease,targetCommitish,assets --template '{{.name}} draft={{.isDraft}} prerelease={{.isPrerelease}} target={{.targetCommitish}} assets={{len .assets}}' 2>&1 || true`
HEAD vs origin: !`git fetch origin --quiet; git rev-parse HEAD; git branch --show-current; git merge-base --is-ancestor HEAD origin/$(git branch --show-current) && echo "HEAD is pushed" || echo "HEAD is NOT pushed"`
Commits since previous tag: !`git log --oneline $(git tag --sort=-v:refname | grep '^v' | head -1)..HEAD`

Steps:

0. **Check the "Release for this version" line above** — never assume state
   from an earlier run:
   - `release not found` → the script will create a fresh public pre-release;
   - existing with `draft=true` → the script updates its assets/notes and
     PROMOTES the draft to a public pre-release;
   - existing with `draft=false prerelease=true` → the script DELETES and
     RECREATES the pre-release (tag included) at HEAD, so the tag, the
     "Source code" archives and the release date all match the new build;
   - existing with `draft=false prerelease=false` → STOP: this version is
     already published as a full release; the user must bump PORT_VERSION
     (or pass a new version argument) first.

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

3. **Confirm before going public**: use AskUserQuestion with a summary
   (version, commit count, whether an existing draft will be promoted, and —
   if HEAD is NOT pushed — that the script will push the branch first) and
   options "Publish pre-release v<VERSION>" / "Cancel". Do NOT run the
   script without an explicit yes: unlike a draft, this is visible to users
   the moment it is created.

4. **Run the release script** (only after confirmation):
   - With a version argument: `./draft-release.sh $1 --prerelease --notes build-logs/release-notes-draft.md`
     (this bumps PORT_VERSION, commits it, and the script pushes the branch
     so the tag can point at the bump commit).
   - Without: `./draft-release.sh --prerelease --notes build-logs/release-notes-draft.md`.
   - The full build takes several minutes — use a 15-minute timeout or run in
     background and wait for completion.
   - If the working tree is dirty and a version bump was requested, the script
     refuses — report that to the user instead of committing anything yourself.

5. **Report the result**: show the generated Change History and the public
   pre-release URL, and remind that promoting it to a full release later is
   `gh release edit vX.Y.Z --prerelease=false --latest`.
