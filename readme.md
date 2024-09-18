# F1tenth Workspace
A ROS2 workspace for developing and testing control algorithms for F1/10 autonomous vehicles.  
The workspace includes `agent` package, which implements the control logic for steering an F1/10 car.

## Requirements

- Python 3 (tested with Python 3.12)
- ROS 2 (tested with ROS 2 Jazzy) — installation instructions available at: [ROS 2 Jazzy Installation Guide](https://docs.ros.org/en/jazzy/Installation.html)


## Setup
Clone the repository and initialize submodules:
```bash
git clone git@github.com:PetyM/f1tenth_workspace.git
cd f1tenth-workspace
git submodule update --init --recursive
```

The project uses two git submodules:
- [f1tenth_gym_ros](https://github.com/PetyM/f1tenth_gym_ros)
- [f1tenth_racetracks](https://github.com/f1tenth/f1tenth_racetracks/)

## Running Experiments

Before running any experiments, make sure to source the ROS 2 environment and build the workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Next, precompute the necessary data (costmap, position map, and curvature map):

```bash
python3 costmap.py
```

Once preprocessing is done, you can run the experiments using:
- Time Trial
    ```bash
    ros2 launch launch_time_trial.py
    ```
- Head-to-Head
    ```bash
    ros2 launch launch_head_to_head.py
    ```


