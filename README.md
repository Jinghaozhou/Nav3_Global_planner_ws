# PCT Planner（ROS 2）

基于离线全局 PCD 点云建立 Tomogram，并按服务请求生成 `map` 坐标系中的三维全局路径。模块不负责定位、局部避障或底盘控制；可以独立于 Fast-LIO、KISS-Matcher 和 SCAN Planner 运行。

## 工作空间

```text
Nav3_Global_planner_ws/
├── src/
│   ├── pct_planner_interfaces/       # Plan.srv、PublishCostMap.srv
│   ├── pct_planner_viz/              # 地图点云发布与 RViz 配置
│   └── pct_planner/
│       ├── pct_planner/core/         # 无 ROS 通信依赖的 Tomogram 和规划算法
│       ├── pct_planner/ros/          # rclpy 服务和 Path 发布节点
│       ├── native/                   # C++/pybind11 算法和第三方源码
│       ├── config/
│       ├── launch/
│       └── rviz/
├── map/                              # 放置一个或多个 .pcd 文件
├── build/                            # colcon 构建产物
├── install/
└── log/
```

旧版 ROS 1 脚本、分开的 ROS 2 tomography/planner 节点已经由这个集成节点取代。算法核心仍可作为普通 Python 模块调用，不需要初始化 ROS 2。原代码可从 Git 历史恢复。

## 环境与构建

当前工作空间使用 ROS 2 Humble 和 Python 3.10。需要 C++ 编译器、CMake、Eigen3、Boost、TBB 和 NVIDIA 驱动。Python 依赖安装在工作空间内的 `.venv_pct` 中，不修改系统 Python。Open3D 用于读取 PCD；CuPy 用 GPU 从 PCD 计算 Tomogram。已有有效缓存时，重新规划不需要再次运行 GPU 层析。

先为**当前机器**编译仓库内的 GTSAM 和 OSQP，再构建 ROS 2 功能包。不要复用其他机器或其他 Python 版本的 `.so` 文件。

```bash
cd /path/to/Nav3_Global_planner_ws
source /opt/ros/humble/setup.bash
python3 -m venv --system-site-packages .venv_pct
source .venv_pct/bin/activate
python -m pip install -r src/pct_planner/requirements-ros2.txt
./src/pct_planner/scripts/build_native_deps.sh
colcon build --base-paths src --packages-up-to pct_planner pct_planner_viz \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source src/pct_planner/scripts/activate_runtime.bash
```

若提示缺少 `ensurepip`，先安装 Ubuntu 的 `python3.10-venv` 再创建虚拟环境。运行环境脚本会加载 ROS 2、虚拟环境、工作空间及虚拟环境中的 CUDA 运行库路径。

`src/pct_planner_interfaces` 生成服务类型；`src/pct_planner` 构建本机 Python 对应的 C++ 扩展并安装节点。第三方库的构建结果保存在忽略 Git 的 `install_ros2/` 目录中。

如果是把已有工作空间从 `PCT_planner` 重命名为 `Nav3_Global_planner_ws`，旧的 `.venv_pct` 启动脚本和 `build/`、`install/` 中的生成文件可能仍含旧绝对路径。请在新目录重新创建虚拟环境并重新构建 ROS 2 功能包；PCD 地图无需移动，Tomogram 缓存会因地图绝对路径变化而重新生成。重新构建前可先移走旧的 `build/`、`install/`、`log/`，避免 CMake 使用旧路径的缓存。

## 地图

将离线全局点云放到工作空间根目录的 `map/` 中。可以存放多个 PCD，通过 ROS 参数 `map` 选择其中一个文件名，不再使用 `building`、`spiral`、`plaza` 等场景模式。

```text
map/
├── building2_9.pcd
├── plaza3_10.pcd
└── stairs_standard.pcd
```

已有示例地图另存于 `map/pcd_files.zip`。PCD 和 Tomogram 缓存默认不纳入 Git；新工作空间可从 ZIP 解压示例或放入自己的 PCD。`map_dir` 默认为自动查找工作空间根目录的 `map/`，也可传入绝对路径。`cache_dir` 默认为 `map/cache/`。

同一张地图和相同层析参数会复用缓存。修改 `map` 参数、替换 PCD 文件或修改层析参数后，下次规划请求会加载对应缓存或重新计算 Tomogram，无需重启节点。服务调用期间会同步计算，首次请求可能较慢。

## 启动和接口

```bash
source src/pct_planner/scripts/activate_runtime.bash
ros2 launch pct_planner pct_planner.launch.py
```

不传 `map` 时，使用 `src/pct_planner/config/pct_planner.yaml` 中的默认值（目前为 `building2_9.pcd`）；该文件必须位于 `map/` 中。需要选择其他地图时可加 `map:=stairs_standard.pcd`，也可以修改 YAML 默认值。`map` 只接受地图目录中的 PCD 文件名。

也可以指定地图目录：

```bash
ros2 launch pct_planner pct_planner.launch.py \
  map:=stairs_standard.pcd map_dir:=/path/to/Nav3_Global_planner_ws/map
```

| 接口 | 类型 | 说明 |
| --- | --- | --- |
| `/pct/plan` | `pct_planner_interfaces/srv/Plan` | 请求包含 `start`、`goal` 两个 `PoseStamped`；响应包含 `success`、`path`、`message` |
| `/pct/global_path` | `nav_msgs/msg/Path` | 成功规划后发布，`RELIABLE + TRANSIENT_LOCAL` |
| `/pct/map_points` | `sensor_msgs/msg/PointCloud2` | 辅助功能包发布的地图可视化采样，`RELIABLE + TRANSIENT_LOCAL` |
| `/pct/map_cost_voxels` | `visualization_msgs/msg/Marker` | 辅助功能包发布按代价着色的占据体素，包含零代价体素 |
| `/pct/tomogram_layers` | `pct_planner_interfaces/msg/TomogramCost` | 规划节点发布含零代价和几何信息的完整保留层数据 |
| `/pct/publish_cost_map` | `pct_planner_interfaces/srv/PublishCostMap` | 加载并发布完整层析代价，空请求，返回状态和地图签名 |

请求示例：

```bash
source src/pct_planner/scripts/activate_runtime.bash
ros2 service call /pct/plan pct_planner_interfaces/srv/Plan "{
  start: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 5.0, z: 0.0}}},
  goal:  {header: {frame_id: map}, pose: {position: {x: -6.0, y: -1.0, z: 0.0}}}
}"
```

起终点必须明确填写 `frame_id: map`，坐标单位为米，`x/y/z` 必须为有限数值。服务收到请求才触发规划，没有起终点 Topic 或启动时的自动规划。若地图不存在、坐标不合法或无可行路径，服务返回 `success: false`、空路径和错误原因；节点保持运行，并发布空路径清除先前路径。

运行时可切换已有地图，下一次服务请求将使用新地图：

```bash
source src/pct_planner/scripts/activate_runtime.bash
ros2 param set /pct_planner map plaza3_10.pcd
```
或者
```bash
source src/pct_planner/scripts/activate_runtime.bash
ros2 param set /pct_planner map stairs_standard.pcd
```


路径 `Path.header.frame_id` 和每个 `PoseStamped.header.frame_id` 均为 `map`。输入的 `z` 用于选择该 XY 位置下方最近的可行走地面；输出的路径点是机器人参考点，可能高于地面高度。路径点包含有效的 `x/y/z`。起终点姿态中的方向目前不参与搜索。

## 参数

参数集中在 [`src/pct_planner/config/pct_planner.yaml`](src/pct_planner/config/pct_planner.yaml)：

| 参数 | 含义 |
| --- | --- |
| `map`、`map_dir`、`cache_dir` | PCD 文件名、地图目录和 Tomogram 缓存目录 |
| `resolution`、`ground_h`、`slice_dh` | 栅格分辨率、最低地面高度、断层高度 |
| `kernel_size`、`interval_min`、`interval_free` | 邻域尺寸和机器人所需垂直净空 |
| `slope_max`、`step_max`、`standable_ratio` | 坡度、台阶和可站立区域约束 |
| `safe_margin`、`inflation`、`cost_barrier` | 安全距离、膨胀距离和障碍代价 |
| `use_quintic`、`max_heading_rate` 等 | 路径与轨迹优化参数 |

调整机器人尺寸时，优先核对 `interval_min`、`interval_free`、`safe_margin` 和 `inflation`。典型坡道、楼梯、多层结构仍需结合具体地图和机器人尺寸验证。

当前默认代价参数已恢复第一版：`step_max=0.17 m`、`standable_ratio=0.20`、`safe_margin=0.40 m`、`inflation=0.20 m`。这些值影响台阶高度、可站立邻域和障碍周围代价；实际部署前应按机器人越阶能力和外形尺寸核对。修改 YAML 后重启规划节点；运行中也可用 `ros2 param set /pct_planner <参数名> <数值>` 调整，下一次地图发布或规划请求会重算 Tomogram。

## 地图和路径可视化

完整的独立使用说明见 [`pct_planner_viz/README.md`](src/pct_planner_viz/README.md)。

先启动规划节点，再在另一个终端启动辅助功能包：

```bash
source src/pct_planner/scripts/activate_runtime.bash
ros2 launch pct_planner_viz pct_visualization.launch.py
```

RViz2 的固定坐标系为 `map`，默认显示着色体素 `/pct/map_cost_voxels` 和规划路径 `/pct/global_path`；原始 PCD 点云 `/pct/map_points` 可按需打开。可视化节点从 `/pct_planner` 读取 `map` 和 `map_dir` 参数，订阅完整 Tomogram 数据；启动时无需再填写地图名。首次生成 Tomogram 可能耗时。用 `ros2 param set /pct_planner map plaza3_10.pcd` 切换地图后，可视化节点会在约 2 秒内请求更新；随后重新调用 `/pct/plan` 获取新地图的路径。

RViz 顶部的 **Place Start**、**Place Goal** 工具可在三维地图表面点击放置起终点球；再次点击同一按钮会清除旧位置并重新放置。球可沿 X/Y/Z 调整，左侧 **Waypoint Coordinates** 面板显示并复制两点坐标或完整 `/pct/plan` 命令。放置和拖动本身不会自动调用规划服务。可用 `marker_size:=1.2` 调整标记尺寸，或用 `waypoints:=false` 关闭标记节点。

可通过 `display_voxel_size:=0.10` 调整体素边长，通过 `max_points:=200000` 限制占据体素数；原始点云从同一批体素各取一个代表点，规划不受这些参数影响。原始 PCD 不会通过话题传给规划节点。若只需地图话题而不打开 RViz，可设 `rviz:=false`。例如：

```bash
ros2 launch pct_planner_viz pct_visualization.launch.py display_voxel_size:=0.05 max_points:=100000 rviz:=false
```

着色时，每个原始 PCD 点读取同一 XY 位置上方最近保留断层的最终膨胀代价，每个占据体素取其中的最大有效代价。零代价到不可通行使用低饱和度青绿、沙金、砖红渐变；代价未知显示灰色，空气不生成体素。墙面的颜色是代价投影，不代表规划器直接在墙面计算了代价。完整说明见 [`pct_planner_viz/README.md`](src/pct_planner_viz/README.md)。

地图和路径话题使用 `TRANSIENT_LOCAL`，因此在节点启动后打开 RViz 也能看到最近发布的结果。原来的 `src/pct_planner/rviz/pct_planner.rviz` 仍可单独用于路径查看。
