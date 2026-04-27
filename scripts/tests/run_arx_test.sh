#!/bin/bash
# Safe ARX module tests - NO hardware connection
# This script sets up the proper environment and runs the Python tests

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Using shared python_sdk directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"
LEROBOT_DATA_ROOT="$(dirname "$PROJECT_ROOT")"
ARX_ROOT="$(dirname "$LEROBOT_DATA_ROOT")"
ARX_SDK_DIR="${ARX_ROOT}/python_sdk/arx_r5_sdk"

echo "========================================"
echo "ARX VR Teleop Module Tests"
echo "========================================"
echo ""

# Check if SDK directory exists
if [ ! -d "$ARX_SDK_DIR" ]; then
    echo "❌ ERROR: ARX SDK not found at $ARX_SDK_DIR"
    exit 1
fi

# Set LD_LIBRARY_PATH for SDK shared libraries
echo "Setting up LD_LIBRARY_PATH..."
export LD_LIBRARY_PATH="${ARX_SDK_DIR}/bimanual/api/arx_r5_src:${LD_LIBRARY_PATH}"
export LD_LIBRARY_PATH="${ARX_SDK_DIR}/bimanual/api:${LD_LIBRARY_PATH}"
export LD_LIBRARY_PATH="${ARX_SDK_DIR}/bimanual/api/arx_r5_python:${LD_LIBRARY_PATH}"
export LD_LIBRARY_PATH="/opt/ros/jazzy/lib:${LD_LIBRARY_PATH}"

echo "LD_LIBRARY_PATH set."
echo ""

# Run tests
cd "$SCRIPT_DIR"
python3 test_arx_modules.py

exit_code=$?

echo ""
if [ $exit_code -eq 0 ]; then
    echo "========================================"
    echo "✓ All tests passed!"
    echo "========================================"
else
    echo "========================================"
    echo "❌ Some tests failed!"
    echo "========================================"
fi

exit $exit_code
