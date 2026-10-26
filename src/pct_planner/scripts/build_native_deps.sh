#!/usr/bin/env bash
set -euo pipefail

package_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="$(cd "${package_dir}/../.." && pwd)"
native_dir="${package_dir}/native"

cmake -S "${native_dir}/3rdparty/gtsam-4.1.1" -B "${workspace_dir}/build/pct_gtsam" \
  -DCMAKE_INSTALL_PREFIX="${native_dir}/3rdparty/gtsam-4.1.1/install_ros2" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_RPATH='$ORIGIN' \
  -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_UNSTABLE=OFF \
  -DGTSAM_BUILD_PYTHON=OFF
cmake --build "${workspace_dir}/build/pct_gtsam" -j "${CMAKE_BUILD_PARALLEL_LEVEL:-4}"
cmake --install "${workspace_dir}/build/pct_gtsam"

cmake -S "${native_dir}/3rdparty/osqp" -B "${workspace_dir}/build/pct_osqp" \
  -DCMAKE_INSTALL_PREFIX="${native_dir}/3rdparty/osqp/install_ros2" \
  -DCMAKE_BUILD_TYPE=Release -DUNITTESTS=OFF -DENABLE_MKL_PARDISO=OFF
cmake --build "${workspace_dir}/build/pct_osqp" -j "${CMAKE_BUILD_PARALLEL_LEVEL:-4}"
cmake --install "${workspace_dir}/build/pct_osqp"
