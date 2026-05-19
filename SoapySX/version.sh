#!/bin/sh
# Print a source file with git tag and commit number.
# $1 is GIT_EXECUTABLE from CMakeLists.txt.
# $2 is the repository root.
GIT="$1"
REPO_DIR="${2:-..}"
TAG_FALLBACK="${3:-unknown}"
COMMIT_FALLBACK="${4:-unknown}"

# For some reason it sometimes shows dirty if changes have just been committed
# but stops doing it after running git status.
# This may not be the right way to fix it but it seems to help.
if [ -d "$REPO_DIR/.git" ] || [ -f "$REPO_DIR/.git" ]; then
    "$GIT" -C "$REPO_DIR" status > /dev/null

    set -e

    printf '%s' 'const char *SoapySX_tag = "'
    "$GIT" -C "$REPO_DIR" describe --tags --always --dirty --broken | tr -d '\n'
    printf '%s\n' '";'

    printf '%s' 'const char *SoapySX_commit = "'
    "$GIT" -C "$REPO_DIR" rev-parse HEAD | tr -d '\n'
    "$GIT" -C "$REPO_DIR" diff-index --quiet HEAD -- || printf '%s' '-dirty'
    printf '%s\n' '";'
else
    printf 'const char *SoapySX_tag = "%s";\n' "$TAG_FALLBACK"
    printf 'const char *SoapySX_commit = "%s";\n' "$COMMIT_FALLBACK"
fi
