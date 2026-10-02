#!/usr/bin/env bash
# Run every experiment config. Usage: bash experiments/run_all.sh [extra run.py args, e.g. --seeds 1 2]
set -euo pipefail
cd "$(dirname "$0")/.."
PY=${PY:-$HOME/.venvs/netbots/bin/python}
for c in baseline latency loss jitter load robots policy; do
  echo "== $c"
  "$PY" experiments/run.py "experiments/configs/$c.json" "$@"
done
