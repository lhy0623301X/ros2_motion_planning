## ROS2 Workspace Structure

The ROS2 workspace keeps a staged layout aligned with the migration plan.

### Phase 1

- `core/common`
- `core/system_config`
- `sim_env`

### Phase 2

- `core/path_planner/path_planner`
- `core/controller/*`
- `plugins/map_plugins/voronoi_layer`

### Phase 3-4 Reserved

- `plugins/dynamic_rviz_config`
- `plugins/dynamic_xml_config`
- `plugins/gazebo_plugins/*`
- `plugins/rviz_plugins/*`
- `sim_env/launch/app`
- `sim_env/launch/include`
- `sim_env/launch/realworld`
- `sim_env/config/costmap`
- `sim_env/config/robots`

These placeholders are intentionally code-free for now. They exist so Phase 2 can
start from a complete ROS2-side workspace skeleton without losing the ROS1 source
package topology.
