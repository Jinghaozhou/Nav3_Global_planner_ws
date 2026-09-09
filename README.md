# PCT Planner

## Overview

This is an implementation of paper **Efficient Global Navigational Planning in 3-D Structures Based on Point Cloud Tomography** (accepted by TMECH).
It provides a highly efficient and extensible global navigation framework based on a tomographic understanding of the environment to navigate ground robots in multi-layer structures.

**Demonstrations**: [pct_planner](https://byangw.github.io/projects/tmech2024/)

![demo](rsc/docs/demo.png)

## Citing

If you use PCT Planner, please cite the following paper:

[Efficient Global Navigational Planning in 3-D Structures Based on Point Cloud Tomography](https://ieeexplore.ieee.org/document/10531813)

```bibtex
@ARTICLE{yang2024efficient,
  author={Yang, Bowen and Cheng, Jie and Xue, Bohuan and Jiao, Jianhao and Liu, Ming},
  journal={IEEE/ASME Transactions on Mechatronics}, 
  title={Efficient Global Navigational Planning in 3-D Structures Based on Point Cloud Tomography}, 
  year={2024},
  volume={},
  number={},
  pages={1-12}
}
```

## Prerequisites

### Environment

- Ubuntu >= 20.04
- ROS >= Noetic with ros-desktop-full installation
- CUDA >= 11.7

### Python

- Python >= 3.8
- [CuPy](https://docs.cupy.dev/en/stable/install.html) with CUDA >= 11.7
- Open3d

## Build & Install

Inside the package, there are two modules: the point cloud tomography module for tomogram reconstruction (in **tomography/**) and the planner module for path planning and optimization (in **planner/**).
You only need to build the planner module before use.
In **planner/**, run **build_thirdparty.sh** first and then run **build.sh**. 

```bash
cd planner/
./build_thirdparty.sh
./build.sh
```

## ROS 2 Jazzy planner node

The ROS 2 package is in `pct_planner_ros2/`. Build it from a standard ROS 2
workspace so that `ros2 pkg` and `ros2 run` can discover it:

```bash
mkdir -p ~/pct_ros2_ws/src
ln -sfn ~/PCT_planner/pct_planner_ros2 ~/pct_ros2_ws/src/pct_planner_ros2
cd ~/pct_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/PCT_planner/.venv/bin/activate
colcon build --symlink-install --packages-select pct_planner_ros2
source install/setup.bash
```

Before running the node, expose the existing planner extension and shared
libraries:

```bash
cd ~/PCT_planner
export PCT_PLANNER_ROOT=$PWD
export LD_LIBRARY_PATH="$PWD/planner/lib/3rdparty/gtsam-4.1.1/install/lib:$PWD/planner/lib/build/src/common/smoothing:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$PWD/planner/lib:$PWD/planner/scripts:${PYTHONPATH:-}"
```

Then verify the package and publish a planned path:

```bash
ros2 pkg prefix pct_planner_ros2
ros2 run pct_planner_ros2 pct_planner_node
```

In another terminal, source Jazzy and the workspace, then inspect the result:

```bash
ros2 topic echo /pct_path --qos-durability transient_local --once
```

## Run Examples

Three example scenarios are provided: **"Spiral"**, **"Building"**, and **"Plaza"**.
- **"Spiral"**: A spiral overpass scenario released in the [3D2M planner](https://github.com/ZJU-FAST-Lab/3D2M-planner).
- **"Building"**: A multi-layer indoor scenario with various stairs, slopes, overhangs and obstacles.
- **"Plaza"**: A complex outdoor plaza for repeated trajectory generation evaluation.

### Tomogram Construction

To plan in a scenario, first you need to construct the scene tomogram using the pcd file.
- Unzip the pcd files in **rsc/pcd/pcd_files.zip** to **rsc/pcd/**.
- For scene **"Spiral"**, you can download the pcd file from [3D2M planner spiral0.3_2.pcd](https://github.com/ZJU-FAST-Lab/3D2M-planner/tree/main/planner/src/read_pcd/PCDFiles).
- Run **roscore**, start **RViz** with the provided config (**rsc/rviz/pct_ros.rviz**). 
- In **tomography/scripts/**, run **tomography.py** with the **--scene** argument:

```bash
cd tomography/scripts/
python3 tomography.py --scene Spiral
```

- The generated tomogram is visualized as ROS PointCloud2 message in RViz and saved in **rsc/tomogram/**.

### Trajectory Generation 

After the tomogram is constructed, you can run the trajectory generation example.
- In **planner/scripts/**, run **plan.py** with the **--scene** argument:

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/YOUR/DIRECTORY/TO/PCT_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib
cd planner/scripts/
python3 plan.py --scene Spiral
```

- The generated trajectory is visualized as ROS Path message in RViz.

## License

The source code is released under [GPLv2](http://www.gnu.org/licenses/) license.

For commercial use, please contact Bowen Yang [byangar@connect.ust.hk](mailto:byangar@connect.ust.hk).
