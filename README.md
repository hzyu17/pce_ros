# PCE-ROS: Proximal Cross-Entropy Motion Planner for MoveIt

A MoveIt motion planning plugin implementing **Proximal Cross-Entropy (PCE)** and **Natural Gradient Descent (NGD)** trajectory optimization methods for robotic manipulators.

## Overview

PCE-ROS provides sampling-based trajectory optimization planners that integrate seamlessly with MoveIt. The planners use distance field-based collision checking and iterative optimization to find smooth, collision-free trajectories.

### Key Features

- **Two Planning Algorithms**: PCE (Proximal Cross-Entropy) and NGD (Natural Gradient Descent)
- **MoveIt Integration**: Drop-in replacement for standard MoveIt planners
- **Distance Field Collision Checking**: Fast, gradient-aware obstacle avoidance
- **Real-time Visualization**: Collision spheres and trajectory visualization in RViz
- **OpenMP Parallelization**: Multi-threaded collision cost computation
- **Hot-Reloadable Configuration**: Modify parameters without restarting


### Video Demos
#### Optimization process in ROS 1
<img src="figures/pce_1.gif" width="400" alt="Optimization process in ROS 1">

#### Motion Planning results
<img src="figures/pce_2.gif" width="400" alt="Motion Planning results">

## Requirements

### Dependencies

- ROS Noetic
- MoveIt
- Eigen3
- OpenMP
- yaml-cpp

### Robot Support

Tested with:
- Franka Emika Panda (`panda_moveit_config`)

Can be adapted for any robot with a MoveIt configuration.

## Installation

### 1. Clone the Repository

```bash
cd ~/catkin_ws/src
git clone https://github.com/hzyu17/pce_ros.git
```

#### For ROS 1:
```
git checkout master
```

#### For ROS 2:
```
git checkout jazzy
```

### 2. Install Dependencies

```bash
rosdep install --from-paths . --ignore-src -r -y
```

### 3. Build

```bash
cd ~/catkin_ws
catkin build pce_ros
source devel/setup.bash
```

## Usage

### Quick Start with Panda Robot

Launch the demo environment:

```bash
roslaunch pce_ros pce_standalone.launch
```

This launches:
- Robot state publisher
- MoveIt move_group with PCE/NGD planners
- RViz with motion planning visualization

### Adding Collision Objects

Use the provided test script to add a collision box:

```bash
rosrun pce_ros test_box.py
```

### Switching Between Planners

In RViz MotionPlanning panel:
1. Go to **Context** tab
2. Select planning pipeline: `pce` or `ngd`
3. Plan and execute as usual

## Configuration

### Planner Parameters

Configuration is loaded from `config/pce_planning.yaml`:

```yaml
pce:
  planning_groups:
    - panda_arm
  
  panda_arm:
    pce_planner:
      num_samples: 3000        # Samples per iteration
      num_iterations: 20       # Max optimization iterations
      eta: 0.99                # Proximal step size (PCE only)
      temperature: 1.5         # Sampling temperature
      convergence_threshold: 0.001
      num_discretization: 50   # Trajectory waypoints
      total_time: 5.0          # Trajectory duration (seconds)
    
    # Collision parameters
    collision_clearance: 0.05  # Safety margin (meters)
    collision_threshold: 0.05  # Max distance to consider
    sigma_obs: 500.0           # Obstacle cost weight
    sphere_overlap_ratio: 0.05 # Collision sphere density
```

### Visualization Settings

```yaml
pce:
  visualization:
    enable_collision_spheres: true
    enable_trajectory: true
    enable_distance_field: false
    waypoint_size: 0.02
    line_width: 0.01
    marker_lifetime: 50.0
    trajectory_decimation: 10
    collision_spheres_topic: "/pce/collision_spheres"
    trajectory_topic: "/pce/trajectory"
```

### Modifying Parameters at Runtime

Parameters are reloaded from YAML on each planning request. Simply edit `pce_planning.yaml` and re-plan.

## Architecture

```
pce_ros/
├── include/
│   ├── pce_planner.h           # PCE planner context
│   ├── pce_planner_manager.h   # PCE plugin manager
│   ├── ngd_planner.h           # NGD planner context
│   ├── ngd_planner_manager.h   # NGD plugin manager
│   ├── pce_optimization_task.h # Shared optimization task
│   └── visualizer.h            # RViz visualization
├── src/
│   ├── pce_planner.cpp
│   ├── pce_planner_manager.cpp
│   ├── ngd_planner.cpp
│   ├── ngd_planner_manager.cpp
│   ├── pce_optimization_task.cpp
│   └── visualizer.cpp
├── config/
│   ├── pce_planning.yaml       # Planner configuration
│   └── pce_demo.rviz           # RViz configuration
├── launch/
│   ├── pce_standalone.launch   # Full demo launch
│   ├── pce_panda.launch        # Panda-specific launch
│   └── pce_moveit_generic.launch
└── scripts/
    └── test_box.py             # Add test collision object
```

## Algorithm Details

### PCE (Proximal Cross-Entropy Method)

The PCE planner uses importance sampling with a proximal regularization term:

1. **Sample** trajectories from current distribution
2. **Evaluate** collision costs using distance field
3. **Update** distribution using weighted samples with proximal constraint
4. **Repeat** until convergence

Key parameter: `eta` controls the proximal step size (higher = more aggressive updates)

### NGD (Natural Gradient Descent)

The NGD planner uses natural gradient updates on the trajectory distribution:

1. **Initialize** with straight-line trajectory
2. **Compute** collision cost gradient
3. **Update** using natural gradient (Fisher information scaling)
4. **Repeat** until convergence

### Collision Cost Function

Uses CHOMP-style cost based on signed distance field:

```
cost(d) = {
  -d + 0.5ε           if d < 0 (collision)
  0.5(1/ε)(d-ε)²      if 0 ≤ d < ε (safety margin)
  0                   if d ≥ ε (safe)
}
```

Where `ε` is the `collision_clearance` parameter.

## Visualization

### RViz Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/pce/collision_spheres` | MarkerArray | Collision checking spheres on robot |
| `/pce/trajectory` | MarkerArray | Current trajectory path |
| `/pce/distance_field` | MarkerArray | Distance field visualization |
| `/ngd/collision_spheres` | MarkerArray | NGD collision spheres |

## License

MIT.

## Citation

If you use this work, please cite:

```bibtex
Coming soon.
```