## Path Planner

This directory contains the ROS2/Nav2 global planner plugin migrated from the ROS1
`ros_motion_planning` project.

### Package

- `path_planner`: Nav2 `nav2_core::GlobalPlanner` plugin implementation.

Main entry points:

- `path_planner/src/path_planner_node.cpp`: Nav2 plugin adapter.
- `path_planner/src/path_planner.cpp`: shared planning flow, path reuse, smoothing, and debug data.
- `path_planner/src/utils/path_planner_factory.cpp`: planner selection by `planner_name`.

### Supported Algorithms

Graph search planners:

- `GBFS`
- `dijkstra`
- `A*`
- `JPS`
- `D*`
- `D* Lite`
- `LPA*`
- `hybrid A*`

Sampling planners:

- `RRT`
- `RRT*`
- `Informed RRT*`
- `RRT-Connect`

### Configuration

Select the algorithm in:

```yaml
planner_server:
  ros__parameters:
    GridBased:
      plugin: "path_planner/PathPlanner"
      planner_name: "A*"
```

Project default config:

- `src/sim_env/config/nav2_params.yaml`

During development, prefer:

```bash
colcon build --symlink-install
```

so launch/config changes under `src/` are reflected without repeatedly copying files into
`install/`.

### Useful References

- `src/graph_planner_algorithms.md`
- `src/sample_planner_algorithms.md`

Original ROS1 implementation:

- `/home/lhy/projects_files/ros_motion_planning/src/core/path_planner/`
