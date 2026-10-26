# PCT Planner（ROS 2）

基于离线全局 PCD 点云建立 Tomogram，并按服务请求生成 `map` 坐标系中的三维全局路径。模块不负责定位、局部避障或底盘控制；可以独立于 Fast-LIO、KISS-Matcher 和 SCAN Planner 运行。

## 工作空间

```text
PCT_planner/
├── src/
│   ├── pct_planner_interfaces/       # Plan.srv
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

当前工作空间使用 ROS 2 Humble 和 Python 3.10。需要 C++ 编译器、CMake、Eigen3、Boost、TBB 和 NVIDIA 驱动。Python 依赖安装在工作空间内的 `.venv_ros2` 中，不修改系统 Python。Open3D 用于读取 PCD；CuPy 用 GPU 从 PCD 计算 Tomogram。已有有效缓存时，重新规划不需要再次运行 GPU 层析。

先为**当前机器**编译仓库内的 GTSAM 和 OSQP，再构建 ROS 2 功能包。不要复用其他机器或其他 Python 版本的 `.so` 文件。

```bash
cd /path/to/PCT_planner
source /opt/ros/humble/setup.bash
python3 -m venv --system-site-packages .venv_ros2
source .venv_ros2/bin/activate
python -m pip install -r src/pct_planner/requirements-ros2.txt
./src/pct_planner/scripts/build_native_deps.sh
colcon build --base-paths src --packages-up-to pct_planner \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source src/pct_planner/scripts/activate_runtime.bash
```

若提示缺少 `ensurepip`，先安装 Ubuntu 的 `python3.10-venv` 再创建虚拟环境。运行环境脚本会加载 ROS 2、虚拟环境、工作空间及虚拟环境中的 CUDA 运行库路径。

`src/pct_planner_interfaces` 生成服务类型；`src/pct_planner` 构建本机 Python 对应的 C++ 扩展并安装节点。第三方库的构建结果保存在忽略 Git 的 `install_ros2/` 目录中。

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
ros2 launch pct_planner pct_planner.launch.py map:=building2_9.pcd
```

也可以指定地图目录：

```bash
ros2 launch pct_planner pct_planner.launch.py \
  map:=stairs_standard.pcd map_dir:=/path/to/PCT_planner/map
```

| 接口 | 类型 | 说明 |
| --- | --- | --- |
| `/pct/plan` | `pct_planner_interfaces/srv/Plan` | 请求包含 `start`、`goal` 两个 `PoseStamped`；响应包含 `success`、`path`、`message` |
| `/pct/global_path` | `nav_msgs/msg/Path` | 成功规划后发布，`RELIABLE + TRANSIENT_LOCAL` |

请求示例：

```bash
ros2 service call /pct/plan pct_planner_interfaces/srv/Plan "{
  start: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 5.0, z: 0.0}}},
  goal:  {header: {frame_id: map}, pose: {position: {x: -6.0, y: -1.0, z: 0.0}}}
}"
```

起终点必须明确填写 `frame_id: map`，坐标单位为米，`x/y/z` 必须为有限数值。服务收到请求才触发规划，没有起终点 Topic 或启动时的自动规划。若地图不存在、坐标不合法或无可行路径，服务返回 `success: false`、空路径和错误原因；节点保持运行，并发布空路径清除先前路径。

运行时可切换已有地图，下一次服务请求将使用新地图：

```bash
ros2 param set /pct_planner map plaza3_10.pcd
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

RViz2 配置位于 `src/pct_planner/rviz/pct_planner.rviz`；其中路径显示已指向 `/pct/global_path`。
