#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "=== [hesim-3d] Building Linux Wheel ==="
cd "${REPO_ROOT}"

# Ensure output directory
mkdir -p dist

# Build wheel using active python / .venv
PYTHON_BIN="${REPO_ROOT}/.venv/bin/python"
if [ ! -f "${PYTHON_BIN}" ]; then
    PYTHON_BIN="python3"
fi

"${PYTHON_BIN}" -m pip wheel . --no-deps -w dist/

WHEEL_FILE=$(ls -t dist/hesim_3d-*.whl | head -n 1)
echo ""
echo "=== [hesim-3d] Built wheel: ${WHEEL_FILE} ==="
echo "=== [hesim-3d] Inspecting packaged assets ==="
unzip -l "${WHEEL_FILE}" | grep "hesim3d/assets_data" || true

echo ""
echo "=== [hesim-3d] Checking for leaked Eigen headers (should be empty) ==="
unzip -l "${WHEEL_FILE}" | grep "include/eigen3" || echo "✓ No Eigen headers leaked!"

echo ""
echo "=== [hesim-3d] Wheel build complete! ==="
