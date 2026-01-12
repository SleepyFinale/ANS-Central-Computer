# Autonomous Exploration Guide for TurtleBot3

**After your robot is running** (`ros2 launch turtlebot3_bringup robot.launch.py` on the robot), run these commands on your **central computer** in **3 separate terminals**:

## Setup (Run once per terminal session)

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
```

---

## Terminal 1: SLAM Toolbox (Mapping)

**Purpose**: Creates the map as the robot explores

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch slam_toolbox online_async_launch.py
```

---

## Terminal 2: Nav2 (Navigation)

**Purpose**: Provides navigation and path planning capabilities

**Option A: Standard Nav2 (Recommended for SLAM)**
```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch nav2_bringup navigation_launch.py
```

**Option B: TurtleBot3 Nav2 (if Option A doesn't work)**
```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py
```

---

## Terminal 3: Explorer (Autonomous Exploration)

**Purpose**: Decides where to explore next and sends goals to Nav2

**Option A: Explore Lite (Recommended - more robust)**
```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run explore_lite explore
```

**Option B: Your Custom Explorer**
```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run custom_explorer explorer
```

---

## Terminal 4: RViz (Visualization - Optional but Recommended)

**Purpose**: Visualize the map, robot position, and exploration progress

```bash
source /opt/ros/humble/setup.bash
ros2 launch nav2_bringup rviz_launch.py
```

---

## Quick Reference: Complete Sequence

1. **Robot**: `ros2 launch turtlebot3_bringup robot.launch.py` (on robot)
2. **Computer Terminal 1**: `ros2 launch slam_toolbox online_async_launch.py`
3. **Computer Terminal 2**: `ros2 launch nav2_bringup navigation_launch.py`
4. **Computer Terminal 3**: `ros2 run explore_lite explore`
5. **Computer Terminal 4**: `ros2 launch nav2_bringup rviz_launch.py` (optional)

---

## What Each Terminal Does

| Terminal | Component | Purpose |
|----------|-----------|---------|
| Robot | Robot Launch | Publishes sensor data (`/scan`), odometry, TF |
| Terminal 1 | SLAM Toolbox | Builds the map from sensor data, publishes `/map` |
| Terminal 2 | Nav2 | Plans paths, avoids obstacles, executes navigation |
| Terminal 3 | Explorer | Detects frontiers, sends exploration goals to Nav2 |
| Terminal 4 | RViz | Visualizes everything in real-time |

---

## Verification Checklist

After starting all terminals, check:

1. **Map is being created**: In RViz, you should see the map growing
2. **Robot is moving**: The robot should start exploring autonomously
3. **Topics are active**: `ros2 topic list | grep -E "(map|scan|cmd_vel)"`
4. **TF is working**: `ros2 run tf2_ros tf2_echo map base_link` (should show transform)

---

## Troubleshooting

- **Robot not moving?** 
  - Wait 10-30 seconds after starting Nav2 (it needs time to initialize)
  - Check Nav2 lifecycle: `ros2 service list | grep lifecycle`
  
- **No map appearing?**
  - Check SLAM Toolbox is receiving `/scan`: `ros2 topic echo /scan --once`
  - Make sure robot launch is running and publishing scan data
  
- **Explorer not finding frontiers?**
  - This is normal - wait for the map to build up first
  - The explorer needs some map data before it can find frontiers

- **TF errors?**
  - Make sure robot launch is running
  - Check: `ros2 topic list | grep tf`
  - Wait a bit - TF tree takes time to build
