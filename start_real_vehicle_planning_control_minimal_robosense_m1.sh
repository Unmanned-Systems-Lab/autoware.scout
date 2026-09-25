#!/usr/bin/env bash
set -e
exec "$(dirname -- "${BASH_SOURCE[0]}")/start_scout.bash" "$@"
