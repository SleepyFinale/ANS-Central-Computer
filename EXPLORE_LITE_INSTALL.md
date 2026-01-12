# Explore Lite Installation Guide

## Status

✅ **Cloned**: `m-explore-ros2` has been cloned into `src/m-explore-ros2/`
✅ **Added to Git**: The repository files are now tracked in your git repo (as regular files, not a submodule)

## Important: Complete Installation Required

You need to install the full Navigation2 package before you can build `explore_lite`:

### 1. Install Full Navigation2 Package

You currently have some individual Nav2 packages, but `explore_lite` needs the complete package:

```bash
sudo apt update
sudo apt install -y ros-humble-navigation2
```

This will install all Nav2 packages including `nav2_costmap_2d`, `nav2_voxel_grid`, `nav2_ros_common`, `nav2_lifecycle_manager`, and others that `explore_lite` depends on.

**Current status**: You have these Nav2 packages:
- ros-humble-nav2-common
- ros-humble-nav2-map-server
- ros-humble-nav2-msgs
- ros-humble-nav2-util

**Missing packages needed by explore_lite**:
- nav2_costmap_2d
- nav2_voxel_grid
- nav2_ros_common
- nav2_lifecycle_manager

Installing `ros-humble-navigation2` will install all of these.

### 2. Build Explore Lite

After installing Navigation2, build explore_lite:

```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select explore_lite
source install/setup.bash
```

### 3. Verify Installation

Check that explore_lite is built:

```bash
ros2 pkg executables explore_lite
```

You should see:
```
explore_lite explore
```

## Usage

Once built, you can use explore_lite instead of your custom explorer:

```bash
# Instead of: ros2 run custom_explorer explorer
ros2 run explore_lite explore
```

## Git Status

The `m-explore-ros2` directory is now tracked in your git repository as regular files (not a submodule). This means:
- ✅ Any changes you make will be tracked in your repo
- ✅ Files are part of your repository, not a separate git repo
- ⚠️ You won't automatically get updates from the upstream repository
- ✅ If you want updates from upstream, you'll need to manually merge or re-clone

This is fine if you plan to customize explore_lite for your needs.
