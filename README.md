# PCT Planner

PCT Planner 是论文 **Efficient Global Navigational Planning in 3-D Structures Based on Point Cloud Tomography** 的实现，用于基于点云 tomogram 在多层三维结构中进行全局导航规划。

## 代码结构

```text
PCT_planner/
├── planner/                     # 规划算法核心
│   ├── config/                  # planner 参数
│   ├── scripts/
│   │   ├── planner_wrapper.py   # Python 规划封装
│   │   ├── plan.py              # 原始示例脚本
│   │   └── utils/               # 坐标转换和可视化辅助
│   └── lib/
│       ├── src/                 # A*、高程规划、轨迹优化 C++ 源码
│       ├── *.so                 # pybind11 Python 扩展
│       └── 3rdparty/            # GTSAM、OSQP、pybind11 等依赖
├── tomography/                  # tomogram 生成核心
│   ├── config/                  # 场景和地图参数
│   └── scripts/
│       ├── tomogram.py          # CuPy tomogram 计算
│       ├── kernels.py           # CUDA/CuPy kernel
│       └── tomography.py        # 原始脚本
├── pct_tomography_ros2/         # ROS2 tomogram 生成包
├── pct_planner_ros2/            # ROS2 planner 包
└── rsc/
    ├── pcd/                     # 输入点云
    ├── tomogram/                # 生成的 pickle 文件
    └── rviz/pct_ros2.rviz       # RViz2 配置
```

当前阶段保留原有算法目录。ROS2 包只负责节点、参数、消息和可视化接口。

## 数据流

```text
PCD 点云
   ↓
pct_tomography_ros2/tomography_node
   ├── 保存 rsc/tomogram/<scene>.pickle
   ├── 发布 /global_points
   ├── 发布 /layer_G_<n>、/layer_C_<n>
   └── 发布 /tomogram
          ↓
pct_planner_ros2/planner_node
   ├── 读取 tomogram
   ├── A* + elevation planner + 轨迹优化
   ├── 提供 /pct_planner/get_plan 服务
   └── 发布 /pct_path
```

tomography 节点只计算一次，完成后继续运行。planner 节点启动时根据参数规划一次，随后可以通过服务请求新路径。地图和路径 publisher 使用 `RELIABLE + TRANSIENT_LOCAL` QoS。

## 环境准备

需要 Ubuntu、ROS2 Jazzy、Python 3.12、CUDA、可用 NVIDIA GPU，以及 NumPy、Open3D 和 CuPy。开发阶段直接运行源代码，不依赖 `ros2 run` 的安装结果。

每个新终端执行：

```bash
cd /home/pc/PCT_planner
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
export PCT_PLANNER_ROOT=$PWD
export PYTHONPATH="$PWD:$PWD/planner:$PWD/planner/scripts:$PWD/planner/lib:$PWD/tomography:$PWD/tomography/scripts:$PWD/pct_tomography_ros2:$PWD/pct_planner_ros2:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$PWD/planner/lib:$PWD/planner/lib/build/src/common/smoothing:$PWD/planner/lib/3rdparty/gtsam-4.1.1/install/lib:${LD_LIBRARY_PATH:-}"
```

## 运行 tomography 节点

确保 `rsc/pcd/building2_9.pcd` 存在：

```bash
cd /home/pc/PCT_planner
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
export PCT_PLANNER_ROOT=$PWD
export PYTHONPATH="$PWD/tomography:$PWD/tomography/scripts:$PWD/pct_tomography_ros2:${PYTHONPATH:-}"

python3 -m pct_tomography_ros2.tomography_node \
  --ros-args \
  -p scene:=building \
  -p pcd_dir:=$PWD/rsc/pcd \
  -p output_dir:=$PWD/rsc/tomogram \
  -p map_frame:=map
```

成功后生成 `rsc/tomogram/building2_9.pickle`，并发布 `/global_points`、`/tomogram`、`/layer_G_<n>`、`/layer_C_<n>`。节点保持运行，不重复计算。

## 运行 planner 节点

在另一个终端执行环境准备后运行：

```bash
cd /home/pc/PCT_planner
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
export PCT_PLANNER_ROOT=$PWD
export PYTHONPATH="$PWD:$PWD/planner:$PWD/planner/scripts:$PWD/planner/lib:$PWD/pct_planner_ros2:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$PWD/planner/lib:$PWD/planner/lib/build/src/common/smoothing:$PWD/planner/lib/3rdparty/gtsam-4.1.1/install/lib:${LD_LIBRARY_PATH:-}"

python3 -m pct_planner_ros2.planner_node \
  --ros-args \
  -p tomogram:=building2_9 \
  -p tomogram_dir:=$PWD/rsc/tomogram \
  -p start_x:=5.0 \
  -p start_y:=5.0 \
  -p start_z:=0.0 \
  -p goal_x:=-6.0 \
  -p goal_y:=-1.0 \
  -p goal_z:=0.0 \
  -p frame_id:=map
```

`start_x/start_y/start_z` 和 `goal_x/goal_y/goal_z` 均为 `map` 坐标系中的真实世界坐标，单位为米。planner 会先把 x/y 转换为 tomogram 栅格索引，再根据 z 在该位置选择目标高度以下最近的有效地面层。

例如某个位置存在 `0.0 m`、`4.5 m`、`9.0 m` 和 `13.5 m` 四个地面高度时：

```text
目标 z = 7.0 m  → 选择 4.5 m 地面
目标 z = 12.0 m → 选择 9.0 m 地面
目标 z = 20.0 m → 选择 13.5 m 地面
```

因此，如果输入目标位于半空中，机器人会规划到目标点正下方最近的可行走地面，而不会选择目标上方的楼层。如果目标高度低于当前位置的最低有效地面，节点会输出错误信息。

### 起终点与路径的坐标语义

planner 的输入点和输出路径采用不同但相互关联的高度语义：

- `start_x/start_y/start_z` 和 `goal_x/goal_y/goal_z` 表示机器人需要到达的位置，用于查询对应位置下方的可行走地面；
- planner 会将输入点投影到目标高度以下最近的有效地面，并使用该地面所在的 tomogram layer 进行搜索；
- `/pct_path` 表示机器人参考点的运动轨迹，例如机器人质心、机体中心或其他规划参考点，不要求与地面高度完全重合。

因此，路径点的 z 坐标通常高于对应的地面高度。例如输入起点地面高度为 `0.0 m` 时，当前实现输出的路径参考点高度约为 `0.6 m`。该偏移主要来自原始 planner 中的参考高度和坐标转换偏移：

```text
路径参考点高度 ≈ 地面高度 + 0.1 m reference height + 0.5 m 坐标偏移
```

跨层规划时也应区分以下三个高度：

```text
目标请求高度：用于从空间点向下查询地面
选中的地面高度：机器人实际需要到达的可行走表面
路径参考点高度：在地面高度基础上增加机器人参考点偏移后的轨迹高度
```

例如：

```text
goal_z = 20.0 m
目标下方最高有效地面 = 13.5 m
路径终点参考点高度 ≈ 14.1 m
```

这里路径终点不需要等于 `20.0 m`，也不需要严格等于 `13.5 m`。当前高度偏移暂时保留原始 planner 的行为；接入具体机器人模型后，可以进一步将其整理为 `robot_reference_height` 等显式参数。

成功时日志会包含类似内容：

```text
Start world XYZ: ... -> index: ..., selected surface height=...
Goal world XYZ: ... -> index: ..., selected surface height=...
path found
Plan: optimization done, success=1
[pct_planner]: Published /pct_path with ... poses
```

节点启动时根据参数规划一次，随后继续运行并等待新的规划服务请求。tomogram 和底层 planner 只初始化一次，重新规划时会直接复用。

### 请求新路径

planner 提供标准的 `nav_msgs/srv/GetPlan` 服务：

```text
/pct_planner/get_plan
```

请求中的 `start` 和 `goal` 坐标语义与启动参数一致，均为真实世界坐标。服务规划成功后会在响应中返回路径，同时更新 `/pct_path`，因此 RViz2 会显示最新路径。

保持 planner 节点运行，在另一个已经加载 ROS2 环境的终端执行：

```bash
ros2 service call /pct_planner/get_plan nav_msgs/srv/GetPlan "{
  start: {
    header: {frame_id: map},
    pose: {position: {x: 5.0, y: 5.0, z: 0.0}}
  },
  goal: {
    header: {frame_id: map},
    pose: {position: {x: -5.0, y: -1.0, z: 5.0}}
  },
  tolerance: 0.0
}"
```

`start.header.frame_id` 和 `goal.header.frame_id` 可以为空；如果填写，则必须与 planner 的 `frame_id` 参数一致。当前底层算法不支持目标容差，因此 `tolerance` 应设置为 `0.0`，非零值会被忽略并输出警告。

规划失败时，服务响应包含一条 poses 为空的 `Path`，失败原因会写入 planner 节点日志。由于规划和轨迹优化是同步执行的，服务会在本次规划完成后返回。

### 跨层规划示例

下面的示例从 Building 场景底层规划到 `(5.0, 5.0)` 位置上方的最高地面。该位置最高有效地面为 `13.5 m`，将 `goal_z` 设置为 `20.0 m` 后，planner 会自动选择其下方的最高地面层：

```bash
python3 -m pct_planner_ros2.planner_node \
  --ros-args \
  -p tomogram:=building2_9 \
  -p tomogram_dir:=$PWD/rsc/tomogram \
  -p start_x:=5.0 \
  -p start_y:=5.0 \
  -p start_z:=0.0 \
  -p goal_x:=5.0 \
  -p goal_y:=5.0 \
  -p goal_z:=20.0 \
  -p frame_id:=map
```

运行新的 planner 节点前，应先停止仍在发布 `/pct_path` 的旧 planner 节点，避免多个 publisher 同时发布不同路径。

## 检查话题

```bash
source /opt/ros/jazzy/setup.bash
ros2 node list
ros2 topic list
ros2 service list
ros2 service type /pct_planner/get_plan
ros2 topic info /tomogram --verbose
ros2 topic info /pct_path --verbose
ros2 topic echo /tomogram --once
ros2 topic echo /pct_path --once
```

## RViz2 可视化

```bash
source /opt/ros/jazzy/setup.bash
rviz2 -d /home/pc/PCT_planner/rsc/rviz/pct_ros2.rviz
```

固定坐标系为 `map`。主要显示项：

- `/global_points`：原始点云；
- `/tomogram`：合并后的 tomogram；
- `/layer_G_<n>`：ground 层；
- `/layer_C_<n>`：ceiling 层；
- `/pct_path`：规划路径。

tomography 和 planner 节点需要保持运行，RViz2 才能通过 transient-local publisher 获取最后一条消息。

## 场景

- `building`：`building2_9.pcd`；
- `spiral`：`spiral0.3_2.pcd`；
- `plaza`：`plaza3_10.pcd`。

## 原始算法构建

需要重新编译 planner C++/pybind 扩展时执行：

```bash
cd /home/pc/PCT_planner/planner
./build_thirdparty.sh
./build.sh
```

日常修改 ROS2 Python 节点时，可以继续使用源码运行方式，不需要每次重新构建。

## 引用与许可证

```bibtex
@ARTICLE{yang2024efficient,
  author={Yang, Bowen and Cheng, Jie and Xue, Bohuan and Jiao, Jianhao and Liu, Ming},
  journal={IEEE/ASME Transactions on Mechatronics},
  title={Efficient Global Navigational Planning in 3-D Structures Based on Point Cloud Tomography},
  year={2024},
  volume={}, number={}, pages={1-12}
}
```

源代码遵循 GPLv2 发布。商业使用请联系 [Bowen Yang](mailto:byangar@connect.ust.hk)。
