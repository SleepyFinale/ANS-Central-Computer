#!/bin/bash
# TurtleBot3 Workspace Setup Script
# This script clones the required packages and sets up the workspace

set -e  # Exit on error

echo "Setting up TurtleBot3 workspace..."

# Check if git is installed
if ! command -v git &> /dev/null; then
    echo "ERROR: git is not installed. Please install it first:"
    echo "  sudo apt install git"
    exit 1
fi

# Navigate to workspace
cd ~/turtlebot3_ws

# Clone packages if they don't exist
cd src

if [ ! -d "DynamixelSDK" ]; then
    echo "Cloning DynamixelSDK..."
    git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
fi

if [ ! -d "turtlebot3_msgs" ]; then
    echo "Cloning turtlebot3_msgs..."
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
fi

if [ ! -d "turtlebot3" ]; then
    echo "Cloning turtlebot3..."
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
fi

# Remove .git directories to make them part of the single repository
echo "Removing .git directories from cloned packages..."
rm -rf DynamixelSDK/.git
rm -rf turtlebot3_msgs/.git
rm -rf turtlebot3/.git

# Initialize git in workspace root if not already done
cd ~/turtlebot3_ws
if [ ! -d ".git" ]; then
    echo "Initializing git repository..."
    git init
fi

# Stage files
echo "Staging files..."
git add src/ .gitignore README.md setup.sh 2>/dev/null || true

echo ""
echo "Setup complete!"
echo ""
echo "Next steps:"
echo "1. Review the changes: git status"
echo "2. Commit the files: git commit -m 'Initial commit: TurtleBot3 packages from source'"
echo "3. Build the workspace:"
echo "   cd ~/turtlebot3_ws"
echo "   source /opt/ros/humble/setup.bash"
echo "   colcon build --symlink-install"
echo "4. Add to bashrc: echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc"
echo "5. Connect to GitHub (see README.md for instructions)"
