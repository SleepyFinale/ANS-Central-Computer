# TurtleBot3 Workspace - Editable Setup

This workspace contains editable TurtleBot3 packages for ROS 2 Humble, configured for version control in a single GitHub repository.

## Prerequisites

Before proceeding, ensure you have git installed:
```bash
sudo apt install git
```

## Setup Instructions

### 1. Clone the TurtleBot3 Packages

Navigate to the src directory and clone the required packages:

```bash
cd ~/turtlebot3_ws/src
git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
```

### 2. Remove Git Directories from Cloned Packages

To make these packages part of your single repository (instead of submodules), remove their .git directories:

```bash
cd ~/turtlebot3_ws/src
rm -rf DynamixelSDK/.git
rm -rf turtlebot3_msgs/.git
rm -rf turtlebot3/.git
```

### 3. Initialize Git Repository (if not already done)

```bash
cd ~/turtlebot3_ws
git init
```

### 4. Stage and Commit the Source Files

```bash
cd ~/turtlebot3_ws
git add src/ .gitignore README.md
git commit -m "Initial commit: TurtleBot3 packages from source"
```

### 5. Build the Workspace

Install colcon build tools if needed:
```bash
sudo apt install python3-colcon-common-extensions
```

Build the workspace:
```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 6. Configure Your Shell

Add the workspace to your bashrc:
```bash
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## Connecting to GitHub

### Create a New GitHub Repository

1. Go to GitHub and create a new repository (do NOT initialize it with a README, .gitignore, or license)
2. Copy the repository URL (e.g., `https://github.com/yourusername/turtlebot3-workspace.git`)

### Push to GitHub

```bash
cd ~/turtlebot3_ws
git remote add origin <your-github-repo-url>
git branch -M main
git push -u origin main
```

## Making Changes

You can now edit any files in the `src/` directory. After making changes:

```bash
cd ~/turtlebot3_ws
git add .
git commit -m "Description of your changes"
git push
```

## Rebuilding After Changes

After editing source files, rebuild the workspace:

```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Tracking Upstream Changes (Optional)

If you want to keep track of updates from the original ROBOTIS repositories, you can add them as remotes:

```bash
cd ~/turtlebot3_ws/src/DynamixelSDK
git remote add upstream https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git fetch upstream
# To see what's new: git log HEAD..upstream/humble
```

Note: Since we removed the .git directories, you'll need to re-clone if you want to merge upstream changes. Alternatively, you can manually compare and apply changes.

## Package Contents

- **DynamixelSDK**: SDK for DYNAMIXEL servos
- **turtlebot3_msgs**: Message definitions for TurtleBot3
- **turtlebot3**: Main TurtleBot3 packages (bringup, description, etc.)

## Notes

- The `.gitignore` file excludes build artifacts (build/, install/, log/) from version control
- All source code is editable and tracked in this single repository
- Use `colcon build --symlink-install` to create symlinks instead of copying files, making development easier
