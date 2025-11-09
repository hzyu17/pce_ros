#!/bin/bash
# Helper script to activate GPU environment

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PCE_ROS_DIR="$(dirname "$SCRIPT_DIR")"
VENV_DIR="$PCE_ROS_DIR/venv"

if [ ! -d "$VENV_DIR" ]; then
    echo "ERROR: Virtual environment not found at: $VENV_DIR"
    echo "Please run: ./scripts/setup_gpu_env.sh"
    return 1
fi

source "$VENV_DIR/bin/activate"
echo "✓ GPU environment activated: $VENV_DIR"
echo "  Python: $(which python3)"
