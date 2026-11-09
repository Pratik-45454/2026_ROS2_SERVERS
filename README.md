# bt_servers (2026_ROS2_SERVERS)

PID action servers that execute the motion goals sent by the behavior tree in [`ar_maintreee`](../ar_maintreee). Each server reads feedback (odometry, IMU or SICK), runs a PID loop, and publishes a `Twist` to its own `cmd_vel` topic. The [cmd_multiplex](../2026_ROS2_SERIAL_BRIDGE/README.md) node merges them into `/cmd_vel`.

## Servers

| Executable | Node | Action | Feedback in | Output |
|------------|------|--------|-------------|--------|
| `move_server` | `move_server` | `move_twist` | odometry, front prox | `/move_server/cmd_vel` |
| `odom_server_node` | `odom_pid_server` | `move_twist_` | odometry, front prox | `/move_server/cmd_vel` |
| `rotate_server` | `rotate_exec_server` | `rotate_exec` | IMU yaw | `/rotate_server/cmd_vel` |
| `sick_server_node` | `pid_server_node` | `/movement_sick` | SICK distances | `/cmd_vel` |
| `align_arena_node` | `align_arena_server` | `align_arena_twist` | SICK distances | `/cmd_vel` |
| `brake_server` | `brake_server` | none | odometry | `/brake_server/cmd_vel` |
| `sick_tuning` | | | | PID tuning helper |

Goal types come from `action_pkg` and `ar_interfaces_2026`. All controllers use `robotlibpc` (`include/robotlibpc`, itself a submodule of `Robotics-Club-Pulchowk/robotlibpc`).

## Build

```bash
git submodule update --init --recursive
cd ~/arrobo_ws
colcon build --symlink-install --packages-select bt_servers
```

## Run

```bash
ros2 run bt_servers move_server  --ros-args --params-file src/ar_maintreee/config/master_params.yaml
ros2 run bt_servers rotate_server --ros-args --params-file src/ar_maintreee/config/master_params.yaml
```

## Parameters

Set in [`master_params.yaml`](../ar_maintreee/config/master_params.yaml).

| Node | Parameters |
|------|-----------|
| `move_server` | `x_kp/ki/kd`, `y_kp/ki/kd`, `dist_tol` (mm), `max_vel` (m/s), `sample_time` (ms) |
| `rotate_exec_server` | `yaw_kp/ki/kd`, `yaw_tolerance_rad`, `max_omega_p/n` (rad/s), `sample_time_ms` |

## Layout

```
bt_servers/
├── src/servers/        # one file per server
├── src/TestingCodes/   # PID / SICK tuning, test client
└── include/
    ├── servers/        # server headers
    └── robotlibpc/     # PID + helpers (submodule)
```