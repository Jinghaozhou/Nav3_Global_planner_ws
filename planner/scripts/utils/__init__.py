from .convertion import transTrajGrid2Map

# ROS visualization is optional for the planner core.  Keeping this import
# guarded lets planner_wrapper run in a ROS2 or non-ROS Python environment;
# legacy ROS1 scripts can still use traj2ros when nav_msgs is available.
try:
    from .vis_ros import traj2ros
except ImportError:
    traj2ros = None
