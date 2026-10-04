#!/usr/bin/env bash
# NexArm one-command setup for macOS and Linux.
#
#   bash setup.sh
#
# Installs uv if it is missing, builds a Python 3.12 environment in .venv,
# installs lerobot and the recording dependencies, then runs the checks.
#
# The Windows equivalent is setup.ps1.

set -euo pipefail
cd "$(dirname "$0")"

echo
echo "========================================="
echo " NexArm setup"
echo "========================================="
echo

# --- uv ---------------------------------------------------------------------
if ! command -v uv >/dev/null 2>&1; then
    export PATH="$HOME/.local/bin:$PATH"
fi
if ! command -v uv >/dev/null 2>&1; then
    echo "[1/4] installing uv..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
    export PATH="$HOME/.local/bin:$PATH"
fi
if ! command -v uv >/dev/null 2>&1; then
    echo "uv did not install. Open a new terminal and run this again."
    exit 1
fi
echo "[1/4] uv ready"

# --- environment ------------------------------------------------------------
# lerobot needs Python 3.12 or newer. uv downloads it if this machine has none.
echo "[2/4] building the Python 3.12 environment..."
uv venv --python 3.12 .venv --allow-existing
PY="$(pwd)/.venv/bin/python"

# Existing on disk is not the same as working: a uv venv records the absolute
# path of one exact Python build, and a uv upgrade or a moved folder leaves an
# interpreter that cannot start. Catch it here, not three steps later.
if ! "$PY" -c "pass" >/dev/null 2>&1; then
    echo "      the environment could not start; rebuilding it..."
    rm -rf .venv
    uv venv --python 3.12 .venv
    if ! "$PY" -c "pass" >/dev/null 2>&1; then
        echo "Python still will not start from .venv. Run: python start.py --doctor"
        exit 1
    fi
fi

# --- dependencies -----------------------------------------------------------
echo "[3/4] installing lerobot and dependencies, this takes a few minutes..."
uv pip install -e . --python "$PY"
uv pip install pyserial opencv-python --python "$PY"

# Recording needs lerobot's "dataset" extra, which `pip install -e .` does NOT
# pull in. Without it teleop works and recording dies at import with a message
# about `datasets` that gives no hint the extra was the problem.
#
# Pinned to the ranges in pyproject.toml rather than installed as `.[dataset]`,
# because that form also drags in torchcodec and can re-resolve torch. Taking the
# latest of each is what breaks it: av 18 removed `av.option`, which lerobot
# imports, and pandas 3 is outside the supported range.
echo "      adding the recording dependencies..."
uv pip install --python "$PY" \
    "av>=15.0.0,<16.0.0" \
    "datasets>=4.7.0,<5.0.0" \
    "pandas>=2.0.0,<3.0.0" \
    "pyarrow>=21.0.0,<30.0.0" \
    "jsonlines>=4.0.0,<5.0.0"

# Motor SDKs for the arms that are not a NexArm: Feetech (SO-100 / SO-101) and
# Dynamixel (Koch, OpenManipulator-X). Same ranges as lerobot's own extras.
echo "      adding support for SO-101, Koch and OpenManipulator arms..."
uv pip install --python "$PY" \
    "feetech-servo-sdk>=1.0.0,<2.0.0" \
    "dynamixel-sdk>=3.7.31,<3.9.0" \
    "deepdiff>=7.0.1,<9.0.0"

"$PY" -c "import lerobot, serial, cv2; from lerobot.scripts import lerobot_record, lerobot_replay; print('imports OK')"
echo "[3/4] dependencies installed"

# --- check ------------------------------------------------------------------
echo "[4/4] checking the install..."
echo
"$PY" station/selftest.py || true
echo

echo "========================================="
echo " Ready."
echo "========================================="
echo
echo "  Plug both arms in, switch them on, then run:"
echo
echo "    ./.venv/bin/python start.py"
echo
echo "  That opens the robot page in your browser and does the rest."
echo
if [[ "$(uname)" == "Darwin" ]]; then
    echo "  macOS only: the first run asks for Camera permission. Allow it, or"
    echo "  the page shows no picture and the arms still work. If it never asks,"
    echo "  System Settings > Privacy & Security > Camera, and tick Terminal."
    echo
fi
