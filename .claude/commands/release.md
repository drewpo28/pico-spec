---
description: Publish the draft GitHub release after safety checks (version, clean tree, assets)
---

Publish the draft release for the current version. The version comes from
`PORT_VERSION` in CMakeLists.txt — the single source of truth.

Current PORT_VERSION (working tree): !`grep -oP 'set \(PORT_VERSION "\K[0-9.]+' CMakeLists.txt`
PORT_VERSION committed at HEAD: !`git show HEAD:CMakeLists.txt | grep -oP 'set \(PORT_VERSION "\K[0-9.]+'`
Working tree status: !`git status --porcelain`
HEAD vs origin: !`git fetch origin --quiet; git rev-parse HEAD; git branch --show-current; git merge-base --is-ancestor HEAD origin/$(git branch --show-current) && echo "HEAD is pushed" || echo "HEAD is NOT pushed"`
Draft release state: !`VER=$(grep -oP 'set \(PORT_VERSION "\K[0-9.]+' CMakeLists.txt); gh release view "v$VER" -R drewpo28/pico-spec --json name,isDraft,targetCommitish,assets --template '{{.name}} draft={{.isDraft}} target={{.targetCommitish}} assets={{len .assets}}: {{range .assets}}{{.name}} {{end}}' 2>&1`

Steps — ALL checks must pass before asking to publish; on any failure STOP and
report what is wrong and how to fix it (do not fix, commit or push anything
yourself):

1. **Version consistency**: working-tree PORT_VERSION == PORT_VERSION at HEAD.
   If they differ, the version bump is uncommitted — tell the user to commit
   and push it first (the published tag's source archive must carry the
   released version).

2. **Clean tree**: `git status --porcelain` must be empty (untracked files in
   `build-logs/` or `firmware/` are acceptable — they are build outputs).
   Anything else uncommitted → stop and list it.

3. **HEAD pushed**: HEAD must be an ancestor of the remote branch.

4. **Draft exists and matches**: release `v<VERSION>` must exist, be a draft,
   and every asset name must end with `-<VERSION>.uf2`. Wrong version in
   assets → tell the user to re-run /draft-release. Expect 13 assets; if the
   count differs, mention it in the confirmation so the user decides.

5. **Are you sure**: use AskUserQuestion with a summary (version, asset count,
   target commit — note if it differs from HEAD) and options
   "Publish v<VERSION>" / "Cancel". Do NOT publish without an explicit yes.

6. **Publish** (only after confirmation):
   - If the draft's targetCommitish differs from HEAD:
     `gh release edit v<VERSION> -R drewpo28/pico-spec --target <HEAD sha>`
   - `gh release edit v<VERSION> -R drewpo28/pico-spec --draft=false`
   - Verify: `gh release view v<VERSION>` shows draft=false, and the tag now
     exists on the remote.

7. **Report**: the public release URL, and note that the git tag and the
   "Source code (zip / tar.gz)" archives were created automatically by GitHub
   at publish time.
