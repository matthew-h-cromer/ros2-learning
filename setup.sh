#!/bin/bash
# One-time setup script for ros2-learning workspace

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_NAME="$(basename "$SCRIPT_DIR")"

echo "=== ROS2 Learning Workspace Setup ==="
echo ""

# Check prerequisites
echo "Checking prerequisites..."

if ! command -v ros2 &> /dev/null; then
    echo "ERROR: ros2 command not found."
    echo "Please install ROS2 Kilted first: https://docs.ros.org/en/kilted/Installation.html"
    exit 1
fi

if ! command -v colcon &> /dev/null; then
    echo "ERROR: colcon command not found."
    echo "Please install colcon: sudo apt install python3-colcon-common-extensions"
    exit 1
fi

echo "Prerequisites OK"
echo ""

# Install system dependencies
echo "Installing system dependencies..."
sudo apt install -y portaudio19-dev

# Install Python dependencies
echo ""
echo "Installing Python dependencies..."
pip3 install --break-system-packages -r "$SCRIPT_DIR/requirements.txt"

# Build workspace
echo ""
echo "Building workspace..."
cd "$SCRIPT_DIR"
colcon build --symlink-install

echo ""
echo "=== Setup Complete ==="
echo ""
echo "Add this alias to your ~/.bashrc:"
echo ""
echo "  alias $REPO_NAME=\"cd $SCRIPT_DIR && source install/setup.bash\""
echo ""
echo "Then run: source ~/.bashrc"
echo "In new terminals, just type: $REPO_NAME"
