# ApexNav Real-world Test Example

This release contains trajectory-generation and MPC control code. Using the supplied scripts, you can perform a real-world test within the Habitat simulator, and the implementation is readily portable to a physical mobile robot.

## Dependencies

Please follow [the main README](../README.md) to set up the ApexNav
environment and the Habitat simulator before running this example.

## Quick Start

> **Note:** Use separate terminals for all 6 processes. ROS2 nodes need the simulation environment below; the two model servers use their own Python environment:
> ```bash
> source /opt/apexnav_ros2_py312/bin/activate
> source /opt/ros/jazzy/setup.bash
> source install/setup.bash
> ```

### ROS2 Build
```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

### Step 1: Run Visual Services
Use the separate visual-service environment described in
[the setup guide](../docs/vision_services.md). Start each server in a separate terminal:
```bash
# Terminal 1
python -m vlm.detector.yoloe --port 12184 --weights data/yoloe-11l-seg.pt

# Terminal 2
python -m vlm.similarity.clip --port 12182 --model-name ViT-B/32
```

The perception node publishes `/yoloe/clouds_with_scores` and `/clip/cosine_score`.
Rebuild the ROS2 workspace to update the C++ subscribers.

### Step 2: Launch RViz Visualization
```bash
# Terminal 3
ros2 launch exploration_manager rviz_traj.launch.py
```

### Step 3: Launch ApexNav Main Algorithm
```bash
# Terminal 4
ros2 launch exploration_manager exploration_traj.launch.py
```

### Step 4: Run Habitat Simulator
```bash
# Terminal 5
python habitat_vel_control.py
```

### Step 5: Run Real-world Test Node
```bash
# Terminal 6
python ./real_world_test_example/real_world_test_habitat.py
```

## Navigation Modes

### Manual Point-to-Point Navigation

Use RViz's *<strong>"2D Pose Estimate"</strong>* tool to set a target pose for direct navigation.

<p align="center" style="font-size: 1.0em;">
  <a href="">
    <img src="../assets/point2point.gif" alt="apexnav_demo1" width="80%">
  </a>
  <br>
</p>

### Autonomous Exploration

Use RViz's *<strong>"2D Goal Pose"</strong>* tool to start autonomous exploration.

<p align="center" style="font-size: 1.0em;">
  <a href="">
    <img src="../assets/auto_search.gif" alt="apexnav_demo2" width="80%">
  </a>
  <br>
</p>

### Object Goal Navigation

Set a target object for the robot to find and navigate to:
```bash
ros2 topic pub /detector/label std_msgs/msg/String "data: 'chair'" --once
```

<p align="center" style="font-size: 1.0em;">
  <a href="">
    <img src="../assets/rviz_show.jpg" alt="rviz_visualization" width="80%">
  </a>
</p>

## Acknowledgment

We would like to acknowledge the contributions of the following projects:
- **[REMANI-Planner](https://github.com/Robotics-STAR-Lab/REMANI-Planner)**: Real-time Whole-body Motion Planning for Mobile Manipulators.
