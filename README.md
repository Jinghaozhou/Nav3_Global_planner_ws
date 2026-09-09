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
   └── 发布 /pct_path
```

两个节点都只计算一次，完成后继续运行；地图和路径 publisher 使用 `RELIABLE + TRANSIENT_LOCAL` QoS。

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

成功时日志会包含类似内容：

```text
Start world XYZ: ... -> index: ..., selected surface height=...
Goal world XYZ: ... -> index: ..., selected surface height=...
path found
Plan: optimization done, success=1
[pct_planner]: Published /pct_path with ... poses
```

节点只规划一次，随后继续运行。

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
