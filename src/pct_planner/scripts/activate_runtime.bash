#!/usr/bin/env bash
# Source this file in each terminal before running the ROS 2 planner.

_pct_package_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_pct_workspace_dir="$(cd "${_pct_package_dir}/../.." && pwd)"

if [[ ! -f "${_pct_workspace_dir}/.venv_ros2/bin/activate" || ! -f "${_pct_workspace_dir}/install/setup.bash" ]]; then
  echo "Build the workspace and create .venv_ros2 before sourcing this script" >&2
  return 1
fi

source /opt/ros/humble/setup.bash
source "${_pct_workspace_dir}/.venv_ros2/bin/activate"
source "${_pct_workspace_dir}/install/setup.bash"

_pct_site_packages="${_pct_workspace_dir}/.venv_ros2/lib/python3.10/site-packages"
_pct_cuda_libs="${_pct_site_packages}/nvidia/cuda_nvrtc/lib:${_pct_site_packages}/nvidia/cuda_runtime/lib"
export LD_LIBRARY_PATH="${_pct_cuda_libs}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export CUPY_CACHE_DIR="${_pct_workspace_dir}/.venv_ros2/cupy_cache"
export PCT_MAP_DIR="${_pct_workspace_dir}/map"

unset _pct_package_dir _pct_workspace_dir _pct_site_packages _pct_cuda_libs
