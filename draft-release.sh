#!/bin/bash
# Create (or update) a DRAFT GitHub release with all firmware .uf2 files.
#
# The release is created as a draft: the git tag and the auto-generated
# "Source code (zip/tar.gz)" archives appear only when you publish it
# (gh release edit vX.Y.Z --draft=false, or the web UI). Until then the
# draft is invisible to users and can be freely edited or deleted.
#
# With --prerelease the release is instead published immediately as a PUBLIC
# pre-release (visible to users, marked "Pre-release", tag + source archives
# created at once). An existing draft of the same version is promoted; an
# already-published pre-release is deleted and recreated (tag included) so
# the tag, the source archives and the release date always match the build.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# --- Pico SDK tools bootstrap ---
# The VS Code Pico extension injects cmake/toolchain into its own terminals;
# plain shells (cron, CI, Claude background tasks) don't have them on PATH.
if ! command -v cmake >/dev/null 2>&1; then
    CMAKE_BIN=$(ls -d "$HOME/.pico-sdk/cmake"/*/bin 2>/dev/null | sort -V | tail -1)
    [ -n "$CMAKE_BIN" ] && export PATH="$CMAKE_BIN:$PATH"
fi
if [ -z "$PICO_TOOLCHAIN_PATH" ]; then
    TC_VER=$(grep -oP 'set\(toolchainVersion \K[^)]+' CMakeLists.txt)
    TC_DIR="$HOME/.pico-sdk/toolchain/$TC_VER"
    if [ -d "$TC_DIR" ]; then
        export PICO_TOOLCHAIN_PATH="$TC_DIR"
        export PATH="$TC_DIR/bin:$PATH"
    fi
fi

REPO="drewpo28/pico-spec"
SKIP_BUILD=false
PRERELEASE=false
NOTES_FILE=""
NEW_VERSION=""

usage() {
    cat <<EOF
Usage: $0 [NEW_VERSION] [--skip-build] [--prerelease] [--notes FILE]

  NEW_VERSION   Optional, e.g. 1.2.27. Bumps PORT_VERSION in CMakeLists.txt
                and commits it (no push — pushing is required only to publish).
                Without it the current PORT_VERSION from CMakeLists.txt is used.
  --skip-build  Reuse firmware/*.uf2 from a previous build_all.sh run.
  --prerelease  Publish immediately as a PUBLIC pre-release instead of a
                draft (tag + source archives are created right away; an
                unpushed HEAD — e.g. the version-bump commit — is pushed).
  --notes FILE  Release notes file. Default: RELEASE_NOTES.md if present,
                otherwise GitHub auto-generated notes.

Publish later with:  gh release edit vX.Y.Z --draft=false
EOF
    exit 0
}

while [ $# -gt 0 ]; do
    case "$1" in
        --skip-build) SKIP_BUILD=true; shift ;;
        --prerelease) PRERELEASE=true; shift ;;
        --notes)      NOTES_FILE="$2"; shift 2 ;;
        -h|--help)    usage ;;
        [0-9]*.[0-9]*.[0-9]*) NEW_VERSION="$1"; shift ;;
        *) echo "Unknown argument: $1 (see --help)"; exit 1 ;;
    esac
done

CUR_VERSION=$(grep -oP 'set \(PORT_VERSION "\K[0-9.]+' CMakeLists.txt)
if [ -z "$CUR_VERSION" ]; then
    echo "Error: could not parse PORT_VERSION from CMakeLists.txt"
    exit 1
fi

# --- Optional version bump (commit + push, so the draft can target it) ---
if [ -n "$NEW_VERSION" ] && [ "$NEW_VERSION" != "$CUR_VERSION" ]; then
    if ! git diff --quiet || ! git diff --cached --quiet; then
        echo "Error: working tree has uncommitted changes; commit or stash them"
        echo "       before bumping the version (the bump is its own commit)."
        exit 1
    fi
    echo "Bumping PORT_VERSION: $CUR_VERSION -> $NEW_VERSION"
    sed -i "s/set (PORT_VERSION \"$CUR_VERSION\")/set (PORT_VERSION \"$NEW_VERSION\")/" CMakeLists.txt
    git add CMakeLists.txt
    git commit -m "Bump version to $NEW_VERSION"
    VER="$NEW_VERSION"
else
    VER="$CUR_VERSION"
fi
TAG="v$VER"

# --- Pick the draft's target commit. It must exist on GitHub, so use HEAD
# when it's pushed and the remote branch tip otherwise (a draft is invisible
# and gets retargeted at publish time — /release enforces the final state) ---
git fetch origin --quiet
BRANCH=$(git branch --show-current)
HEAD_SHA=$(git rev-parse HEAD)
if git merge-base --is-ancestor "$HEAD_SHA" "origin/$BRANCH" 2>/dev/null; then
    TARGET_SHA="$HEAD_SHA"
elif $PRERELEASE; then
    # A public pre-release creates the tag at its target immediately, so the
    # commit it points at must exist on GitHub before the release does.
    echo "Pre-release mode: pushing $BRANCH so the tag can point at HEAD"
    git push origin "$BRANCH"
    TARGET_SHA="$HEAD_SHA"
else
    TARGET_SHA=$(git rev-parse "origin/$BRANCH")
    echo "Note: HEAD is not pushed — the draft will target origin/$BRANCH"
    echo "      ($TARGET_SHA). Push and retarget before publishing."
fi

# --- Build ---
if $SKIP_BUILD; then
    echo "Skipping build (reusing firmware/)"
else
    ./build_all.sh
fi

# firmware/ must hold exactly this version's uf2s (build_all.sh cleans it,
# but --skip-build after a version bump would leave stale files)
FILES=(firmware/*-"$VER".uf2)
if [ ! -f "${FILES[0]}" ]; then
    echo "Error: no firmware/*-$VER.uf2 files found — run without --skip-build"
    exit 1
fi
STALE=$(ls firmware/*.uf2 | grep -v -- "-$VER.uf2" || true)
if [ -n "$STALE" ]; then
    echo "Error: firmware/ contains files of another version:"
    echo "$STALE"
    exit 1
fi
echo ""
echo "Firmware files (${#FILES[@]}):"
printf '  %s\n' "${FILES[@]##*/}"
echo ""

# --- Release notes ---
NOTES_ARGS=(--generate-notes)
if [ -z "$NOTES_FILE" ] && [ -f RELEASE_NOTES.md ]; then
    NOTES_FILE="RELEASE_NOTES.md"
fi
if [ -n "$NOTES_FILE" ]; then
    if [ ! -f "$NOTES_FILE" ]; then
        echo "Error: notes file not found: $NOTES_FILE"
        exit 1
    fi
    NOTES_ARGS=(--notes-file "$NOTES_FILE")
    echo "Release notes: $NOTES_FILE"
else
    echo "Release notes: auto-generated by GitHub"
fi

# --- Create draft/pre-release, or update an existing editable one ---
# Editable: a draft always; in --prerelease mode also a published pre-release.
# A published FULL release is never touched — bump PORT_VERSION instead.
EXISTS=false
IS_DRAFT=""
if gh release view "$TAG" -R "$REPO" >/dev/null 2>&1; then
    EXISTS=true
    IS_DRAFT=$(gh release view "$TAG" -R "$REPO" --json isDraft --template '{{.isDraft}}')
    IS_PRE=$(gh release view "$TAG" -R "$REPO" --json isPrerelease --template '{{.isPrerelease}}')
fi

if $EXISTS && [ "$IS_DRAFT" != "true" ]; then
    if ! $PRERELEASE || [ "$IS_PRE" != "true" ]; then
        echo "Error: release $TAG is already PUBLISHED — refusing to overwrite it."
        echo "       Bump PORT_VERSION for a new release."
        exit 1
    fi
    # A published pre-release has an immutable tag: updating assets in place
    # would leave the tag / source archives / release date at the old commit.
    # Recreate the whole release so everything matches this build.
    echo "Pre-release $TAG is already published — recreating it at $TARGET_SHA"
    echo "so the tag, source archives and release date match this build"
    gh release delete "$TAG" -R "$REPO" --cleanup-tag -y
    git tag -d "$TAG" >/dev/null 2>&1 || true
    EXISTS=false
fi

if $EXISTS; then
    echo "Draft $TAG already exists — updating assets (--clobber)"
    gh release upload "$TAG" -R "$REPO" --clobber "${FILES[@]}"
    if [ -n "$NOTES_FILE" ]; then
        gh release edit "$TAG" -R "$REPO" --notes-file "$NOTES_FILE"
    fi
    if $PRERELEASE; then
        echo "Promoting draft $TAG to a public pre-release"
        gh release edit "$TAG" -R "$REPO" --target "$TARGET_SHA" --draft=false --prerelease
    fi
else
    TYPE_ARGS=(--draft)
    if $PRERELEASE; then
        TYPE_ARGS=(--prerelease)
    fi
    gh release create "$TAG" -R "$REPO" \
        "${TYPE_ARGS[@]}" \
        --title "$VER" \
        --target "$TARGET_SHA" \
        "${NOTES_ARGS[@]}" \
        "${FILES[@]}"
fi

# Keep the local tag in sync with the (re)created remote tag
if $PRERELEASE; then
    git fetch origin --tags --force --quiet 2>/dev/null || true
fi

echo ""
gh release view "$TAG" -R "$REPO" --json url,isDraft,isPrerelease \
    --template 'Release: {{.url}}  (draft: {{.isDraft}}, prerelease: {{.isPrerelease}})
'
if $PRERELEASE; then
    echo "Promote to full release with:  gh release edit $TAG -R $REPO --prerelease=false --latest"
else
    echo "Publish with:  gh release edit $TAG -R $REPO --draft=false"
fi
