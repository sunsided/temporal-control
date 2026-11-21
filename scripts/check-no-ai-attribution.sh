#!/bin/sh
# Refuse commit messages that carry AI attribution, co-author trailers or claude.ai/claude.com links.
#
#   sh scripts/check-no-ai-attribution.sh msg <commit-msg-file>   (commit-msg stage)
#   sh scripts/check-no-ai-attribution.sh push                    (pre-push stage)
#
# The push mode reads the range from PRE_COMMIT_FROM_REF / PRE_COMMIT_TO_REF (set by prek and
# pre-commit); for a new branch it checks every commit not yet on any remote.
set -eu

# Trailers count only at the start of a line and with their `:` or link bracket, so prose such
# as "the table is generated with a script" passes. Case-insensitive (grep -i).
pattern='^[^[:alnum:]]*(co-authored-by|generated with)[[:space:]]*[:[]|claude\.(ai|com)|anthropic'

case "${1:-}" in
  msg)
    if grep -qiE "$pattern" "$2"; then
      echo "commit-msg: remove Co-Authored-By trailers, claude.ai/claude.com links and AI attribution." >&2
      exit 1
    fi
    ;;
  push)
    to="${PRE_COMMIT_TO_REF:-HEAD}"
    from="${PRE_COMMIT_FROM_REF:-}"
    case "$from" in
      '' | *[!0]*) [ -n "$from" ] && range="$from..$to" || range="$to --not --remotes" ;;
      *) range="$to --not --remotes" ;; # all zeros: the remote branch does not exist yet
    esac
    # shellcheck disable=SC2086 # $range holds several git arguments on purpose.
    if git log --format=%B $range | grep -qiE "$pattern"; then
      echo "pre-push: a commit message carries a co-author trailer, claude.ai/claude.com link or AI attribution." >&2
      exit 1
    fi
    ;;
  *)
    echo "usage: $0 msg <file> | push" >&2
    exit 2
    ;;
esac
