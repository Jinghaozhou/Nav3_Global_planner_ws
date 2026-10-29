# PCT Planner 可视化功能包

`pct_planner_viz` 用于在 RViz2 中查看降采样的原始 PCD、按代价着色的占据体素和全局路径，并在顶部工具栏点击放置三维起终点。它从规划节点读取 `map`、`map_dir` 参数，使用规划节点发布的完整 Tomogram 最终膨胀代价给 PCD 占据体素着色。原始 PCD 点云仅供显示，不作为规划节点的地图输入。

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

可视化节点会自动读取 `/pct_planner` 的 `map` 和 `map_dir`，无需单独指定地图。若规划节点尚未启动，可视化节点会等待它的参数服务。读取参数后，它调用 `/pct/publish_cost_map`，由规划节点加载或生成 Tomogram 并发布完整层析代价数据；首次生成可能需要较长时间。

RViz2 的固定坐标系为 `map`。着色体素地图准备好后即可显示，原始点云可在 Displays 中开启；路径需要成功调用 `/pct/plan` 后才会出现。例如，在第三个终端执行：

```bash
source src/pct_planner/scripts/activate_runtime.bash
ros2 service call /pct/plan pct_planner_interfaces/srv/Plan "{
  start: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 5.0, z: 0.0}}},
  goal:  {header: {frame_id: map}, pose: {position: {x: -6.0, y: -1.0, z: 0.0}}}
}"
```

上述起终点坐标用于默认的 `building2_9.pcd` 示例；使用其他地图时，请填写该地图内有效的起终点。

## RViz 三维选点

可视化启动文件默认启动 `pct_waypoint_markers` 节点。RViz 顶部有 **Place Start**、**Place Goal** 两个按钮。按下后，对应颜色的小球跟随鼠标在可见三维表面预览；左键点击放置，工具随即退出。再次按同一个按钮会清除旧位置并重新进入放置模式。没有命中可见表面时不会放置。RViz 的 Fixed Frame 应为 `map`。

放置后，蓝色 **START** 和橙色 **GOAL** 球可以继续拖动；X/Y/Z 控制柄可沿单轴调整。左侧 **Waypoint Coordinates** 面板分别显示两点的 `map` 坐标，并可复制单点 XYZ 或完整 `/pct/plan` 命令。拖动时面板坐标同步更新；拖动和放置都不会自动触发规划。运行已复制的命令即可请求路径。地图切换后旧位置会清除，需要重新放置。要改变球和控制柄的大小，可传 `marker_size:=1.2`；`waypoints:=false` 可停用标记节点，此时 RViz 工具仍显示，但不会保存选点。

## 话题与参数

| 话题 | 类型 | 来源与用途 |
| --- | --- | --- |
| `/pct/map_points` | `sensor_msgs/msg/PointCloud2` | 本功能包发布的地图采样点云，`frame_id=map` |
| `/pct/map_cost_voxels` | `visualization_msgs/msg/Marker` (`CUBE_LIST`) | 本功能包发布的占据体素及每体素颜色，包含零代价体素 |
| `/pct/tomogram_layers` | `pct_planner_interfaces/msg/TomogramCost` | 规划节点发布的完整保留层最终代价，含零代价、栅格几何和地图标识 |
| `/pct/global_path` | `nav_msgs/msg/Path` | 规划节点发布的三维全局路径，RViz2 直接订阅 |

这些话题均使用 `RELIABLE`、`TRANSIENT_LOCAL`，所以稍后打开 RViz2 仍能收到最近发布的地图和路径。

可视化节点会调用 `/pct/publish_cost_map`（`pct_planner_interfaces/srv/PublishCostMap`），自动请求完整 Tomogram。也可以手动调用它重新发布当前地图：

```bash
ros2 service call /pct/publish_cost_map pct_planner_interfaces/srv/PublishCostMap "{}"
```

| 启动参数 | 默认值 | 说明 |
| --- | --- | --- |
| `planner_node` | `/pct_planner` | 提供 `map`、`map_dir` 参数的规划节点名称 |
| `display_voxel_size` | `0.10` | 占据体素的边长和原始点云采样密度，单位米，必须大于 0 |
| `max_points` | `200000` | 最多保留的占据体素数；原始点云每体素保留一个代表点 |
| `marker_size` | `0.8` | 起终点交互标记尺寸，单位米，必须大于 0 |
| `waypoints` | `true` | 是否启动三维起终点交互标记节点 |
| `rviz` | `true` | 是否同时打开 RViz2 |

例如，只启动地图发布节点并将点数上限设为 100000：

```bash
ros2 launch pct_planner_viz pct_visualization.launch.py max_points:=100000 rviz:=false
```

RViz2 配置文件为 [`rviz/pct_visualization.rviz`](rviz/pct_visualization.rviz)，默认显示 `/pct/map_cost_voxels`、`/pct/global_path` 和 `/pct/waypoints` 交互标记，并加载顶部选点工具与坐标面板；`/pct/map_points` 原始点云默认关闭，可按需打开对照。

## 代价投影与颜色

节点把原始 PCD 按 `display_voxel_size` 划分为空间体素，只有含 PCD 点的位置才生成立方体。对每个原始点，在相同 XY 栅格中寻找其 Z 上方最近的保留断层，读取该格子的最终膨胀代价；一个体素内取所有有效点代价的最大值。这样地面和墙面都能着色，高代价细节不会被体素内的低代价点平均掉。墙面的颜色表示上方断层的代价投影，并非规划器直接给墙面计算的代价。

所有选中的占据体素都会显示：零代价为低饱和度青绿（`#648F87`），`cost_barrier` 一半为沙金（`#C4AA75`），达到 `cost_barrier` 为砖红（`#B9655D`），中间连续渐变；当前默认 `cost_barrier=50`。没有原始 PCD 点的空气不生成体素。没有上方断层、超出栅格或代价无效的占据体素显示中性灰，表示代价未知，不等同于零代价。

`display_voxel_size` 控制立方体边长和点云采样密度，`max_points` 限制总显示体素数。原始点云从同一批选中的体素各取一个代表点，因此两路显示一起受控。例如：

```bash
ros2 launch pct_planner_viz pct_visualization.launch.py display_voxel_size:=0.05 max_points:=300000
```

体素尺寸越大，显示越粗；窄小的高代价区域可能与其他区域合并。`max_points` 小于占据体素总数时，按稳定的体素哈希保留分布在整张地图中的体素，未选中的体素不会显示。两个参数只影响显示，规划始终使用完整地图。

Tomogram 消息包含每个保留层的完整栅格（包括零代价），使用地图签名防止切换地图时着错颜色。颜色显示的是规划输入代价，实际路径还受搜索距离、跨层约束和轨迹优化影响。

## 切换地图

规划节点运行期间，可以修改其地图参数：

```bash
ros2 param set /pct_planner map plaza3_10.pcd
```

地图发布节点约每 2 秒检查一次规划节点的地图、层析与可通行性参数，以及地图文件的大小和修改时间。地图变化时，它清除旧点云和体素图并读取新 PCD；选点节点收到空地图后清除旧标记和面板坐标。层析或可通行性参数变化时会重新计算体素代价。切换地图后请重新放置起终点并调用 `/pct/plan`；在重新规划前，RViz2 中的路径仍可能是上一张地图的路径。

PCD 文件应放在工作空间的 `map/` 下，或者通过规划节点的 `map_dir` 参数指定目录。`map` 参数填写该目录中的 `.pcd` 文件名，不能填写路径。地图存放方式见[地图说明](../../map/README.md)。

## 点云采样与排查

- 对二进制 PCD，节点分块扫描并按体素聚合代价，最终保留的体素数不超过 `max_points`。地图和 Tomogram 匹配后才发布两路显示数据。
- 对 ASCII 和 `binary_compressed` PCD，节点使用 Open3D 读取后再采样。这类文件读取时可能需要与完整点云相应的内存。
- 如果地图没有显示，先检查 `ros2 param get /pct_planner map` 和 `ros2 param get /pct_planner map_dir`，确认文件存在；再查看 `pct_map_visualizer` 的日志。
- 如果路径没有显示，先确认 `/pct/plan` 返回 `success: true`；仅启动可视化功能包不会触发规划。
