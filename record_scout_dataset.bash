#!/usr/bin/env bash
set -eo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$repo_dir/setup_env.bash"
exec /usr/bin/python3 "$repo_dir/scripts/record_scout_dataset.py" "$@"
