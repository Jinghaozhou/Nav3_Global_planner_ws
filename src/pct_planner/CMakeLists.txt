cmake_minimum_required(VERSION 3.16)
project(pct_planner)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(pct_planner_interfaces REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

ament_python_install_package(${PROJECT_NAME})
ament_get_python_install_dir(PCT_PYTHON_INSTALL_DIR)

add_subdirectory(native)

set(PCT_NATIVE_TARGETS
  a_star ele_planner traj_opt py_map_manager
  a_star_search ele_planner_lib gpmp_optimizer map_manager common_smoothing
)
foreach(target IN LISTS PCT_NATIVE_TARGETS)
  set_target_properties(${target} PROPERTIES
    BUILD_RPATH_USE_ORIGIN TRUE
    BUILD_RPATH "${CMAKE_INSTALL_PREFIX}/${PCT_PYTHON_INSTALL_DIR}/pct_planner/core/lib"
    INSTALL_RPATH "$ORIGIN"
  )
endforeach()
install(TARGETS ${PCT_NATIVE_TARGETS}
  LIBRARY DESTINATION ${PCT_PYTHON_INSTALL_DIR}/pct_planner/core/lib
)

# The vendored GTSAM build is a native runtime dependency of gpmp_optimizer.
install(FILES native/3rdparty/gtsam-4.1.1/install_ros2/lib/libgtsam.so.4.1.1
  DESTINATION ${PCT_PYTHON_INSTALL_DIR}/pct_planner/core/lib
  RENAME libgtsam.so.4
)
install(FILES native/3rdparty/gtsam-4.1.1/install_ros2/lib/libmetis-gtsam.so
  DESTINATION ${PCT_PYTHON_INSTALL_DIR}/pct_planner/core/lib
)

install(PROGRAMS scripts/pct_planner_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
