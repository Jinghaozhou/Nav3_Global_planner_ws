# PCT Planner 可视化功能包

`pct_planner_viz` 用于在 RViz2 中查看 PCT Planner 当前选择的离线 PCD 地图和全局规划路径。它从规划节点读取 `map`、`map_dir` 参数，将地图采样后发布为 `PointCloud2`；路径由规划节点发布，本功能包只负责显示。地图点云仅供可视化，不作为规划节点的地图输入。

## 构建

本功能包位于工作空间的 `src/pct_planner_viz/`。先按[工作空间 README](../../README.md#环境与构建)准备 ROS 2 Humble、`.venv_pct`、依赖及 PCT Planner 的本机库，然后在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
source .venv_pct/bin/activate
colcon build --base-paths src --packages-up-to pct_planner_viz --symlink-install
```

## 启动

在工作空间根目录打开终端，先启动规划节点：

```bash
source src/pct_planner/scripts/activate_runtime.bash
ros2 launch pct_planner pct_planner.launch.py
```

默认地图由 [`pct_planner.yaml`](../pct_planner/config/pct_planner.yaml) 的 `map` 参数指定，目前是 `building2_9.pcd`，因此无需在启动命令中重复填写 `map:=building2_9.pcd`。选择其他地图时，可以给规划节点传参，例如：

```bash
ros2 launch pct_planner pct_planner.launch.py map:=plaza3_10.pcd
```

在第二个终端启动地图发布节点和 RViz2：

```bash
source src/pct_planner/scripts/activate_runtime.bash
ros2 launch pct_planner_viz pct_visualization.launch.py
```

可视化节点会自动读取 `/pct_planner` 的 `map` 和 `map_dir`，无需单独指定地图。若规划节点尚未启动，可视化节点会等待它的参数服务。

RViz2 的固定坐标系为 `map`。地图启动后即可显示；路径需要成功调用 `/pct/plan` 后才会出现。例如，在第三个终端执行：

```bash
source src/pct_planner/scripts/activate_runtime.bash
ros2 service call /pct/plan pct_planner_interfaces/srv/Plan "{
  start: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 5.0, z: 0.0}}},
  goal:  {header: {frame_id: map}, pose: {position: {x: -6.0, y: -1.0, z: 0.0}}}
}"
```

上述起终点坐标用于默认的 `building2_9.pcd` 示例；使用其他地图时，请填写该地图内有效的起终点。

## 话题与参数

| 话题 | 类型 | 来源与用途 |
| --- | --- | --- |
| `/pct/map_points` | `sensor_msgs/msg/PointCloud2` | 本功能包发布的地图采样点云，`frame_id=map` |
| `/pct/global_path` | `nav_msgs/msg/Path` | 规划节点发布的三维全局路径，RViz2 直接订阅 |

两个话题均使用 `RELIABLE`、`TRANSIENT_LOCAL`，所以稍后打开 RViz2 仍能收到最近发布的地图和路径。

| 启动参数 | 默认值 | 说明 |
| --- | --- | --- |
| `planner_node` | `/pct_planner` | 提供 `map`、`map_dir` 参数的规划节点名称 |
| `max_points` | `200000` | 地图可视化点数上限，必须大于 0；启动时生效 |
| `rviz` | `true` | 是否同时打开 RViz2 |

例如，只启动地图发布节点并将点数上限设为 100000：

```bash
ros2 launch pct_planner_viz pct_visualization.launch.py max_points:=100000 rviz:=false
```

RViz2 配置文件为 [`rviz/pct_visualization.rviz`](rviz/pct_visualization.rviz)，地图订阅 `/pct/map_points`，路径订阅 `/pct/global_path`。

## 切换地图

规划节点运行期间，可以修改其地图参数：

```bash
ros2 param set /pct_planner map plaza3_10.pcd
```

可视化节点约每 2 秒检查一次规划节点的 `map`、`map_dir` 以及地图文件的大小和修改时间。选择变化时，它会清除旧地图显示并发布新地图。切换地图后请再次调用 `/pct/plan`；在重新规划前，RViz2 中的路径仍可能是上一张地图的路径。

PCD 文件应放在工作空间的 `map/` 下，或者通过规划节点的 `map_dir` 参数指定目录。`map` 参数填写该目录中的 `.pcd` 文件名，不能填写路径。地图存放方式见[地图说明](../../map/README.md)。

## 点云采样与排查

- 对二进制 PCD，节点以固定步长读取不超过 `max_points` 个 XYZ 点，避免将整张大型地图发布到 DDS；实际点数可能小于上限。
- 对 ASCII 和 `binary_compressed` PCD，节点使用 Open3D 读取后再采样。这类文件读取时可能需要与完整点云相应的内存。
- 如果地图没有显示，先检查 `ros2 param get /pct_planner map` 和 `ros2 param get /pct_planner map_dir`，确认文件存在；再查看 `pct_map_visualizer` 的日志。
- 如果路径没有显示，先确认 `/pct/plan` 返回 `success: true`；仅启动可视化功能包不会触发规划。
