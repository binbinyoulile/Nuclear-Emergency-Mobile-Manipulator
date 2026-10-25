#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

PYTHON_BIN="${PYTHON_BIN:-python3}"
if ! command -v "$PYTHON_BIN" >/dev/null 2>&1; then
    echo "Python 3.10+ is required, but '$PYTHON_BIN' was not found."
    exit 1
fi

"$PYTHON_BIN" -c 'import sys; raise SystemExit(0 if sys.version_info >= (3, 10) else "Python 3.10+ is required.")'
"$PYTHON_BIN" -m venv .venv
# shellcheck disable=SC1091
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt

echo
echo "Python environment is ready."
if [[ ! -f "models/universal_robots_ur10e/assets/base_0.obj" ]]; then
    echo "Official MuJoCo mesh assets are missing. Download them with:"
    echo "  python scripts/fetch_models.py"
fi
echo "Activate later with: source .venv/bin/activate"
