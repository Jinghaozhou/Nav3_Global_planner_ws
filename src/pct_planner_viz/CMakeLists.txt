cmake_minimum_required(VERSION 3.16)
project(pct_planner_viz)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_library(pct_waypoint_ui SHARED rviz_plugin/waypoint_ui.cpp)
target_compile_features(pct_waypoint_ui PRIVATE cxx_std_17)
ament_target_dependencies(pct_waypoint_ui
  pluginlib rclcpp rviz_common rviz_rendering geometry_msgs std_msgs visualization_msgs
)
target_link_libraries(pct_waypoint_ui Qt5::Widgets)
pluginlib_export_plugin_description_file(rviz_common rviz_plugin/plugin_description.xml)
install(TARGETS pct_waypoint_ui LIBRARY DESTINATION lib)

install(PROGRAMS scripts/pct_map_visualizer scripts/pct_waypoint_markers
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
