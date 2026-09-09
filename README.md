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
  -p goal_x:=-6.0 \
  -p goal_y:=-1.0 \
  -p frame_id:=map
```

成功时日志会包含 `path found`、`optimization done, success=1` 和 `Published /pct_path with ... poses`。节点只规划一次，随后继续运行。

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
