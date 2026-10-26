#include "ele_planner/offline_ele_planner.h"

void OfflineElePlanner::InitMap(
    const double a_start_cost_threshold, const double safe_cost_margin,
    const double resolution, const int num_layers, const double step_cost_weight,
    const Eigen::MatrixXd& cost_map, const Eigen::MatrixXd& height_map,
    const Eigen::MatrixXd& ceiling, const Eigen::MatrixXd& ele_map,
    const Eigen::MatrixXd& grad_x, const Eigen::MatrixXd& grad_y) {
  printf("InitMap: starting Astar\n");
  fflush(stdout);
  path_finder_.Init(a_start_cost_threshold, num_layers, resolution, step_cost_weight, cost_map,
                    height_map, ele_map);
  printf("InitMap: Astar done\n");
  fflush(stdout);
  map_ = std::make_shared<DenseElevationMap>();
  map_->Init(resolution, num_layers, cost_map, ele_map, height_map, ceiling,
             grad_x, grad_y);
  printf("InitMap: elevation map done\n");
  fflush(stdout);
  trajectory_optimizer_ = GPMPOptimizerWnoa(safe_cost_margin, map_);
  printf("InitMap: WNOA optimizer done\n");
  fflush(stdout);
  trajectory_optimizer_wnoj_ =
      GPMPOptimizer(safe_cost_margin, max_heading_rate_, map_);
  printf("InitMap: WNOJ optimizer done\n");
  fflush(stdout);
}

bool OfflineElePlanner::Plan(const Eigen::Vector3i& start,
                             const Eigen::Vector3i& goal, const bool optimize) {
  printf("Plan: starting Astar search (%d,%d,%d) -> (%d,%d,%d)\n",
         start[0], start[1], start[2], goal[0], goal[1], goal[2]);
  fflush(stdout);
  if (!path_finder_.Search(start, goal)) {
    printf("A star Failed!\n");
    return false;
  }
  printf("Plan: Astar search done, optimize=%d\n", optimize ? 1 : 0);
  fflush(stdout);

  if (optimize) {
    path_ = path_finder_.GetPathPoints();
    path_.front().ref_v = 1;
    path_.back().ref_v = 1;

    bool success = false;
    if (use_quintic_) {
      printf("Plan: starting WNOJ optimization\n"); fflush(stdout);
      success = trajectory_optimizer_wnoj_.GenerateTrajectory(path_, 200);
    } else {
      printf("Plan: starting WNOA optimization\n"); fflush(stdout);
      success = trajectory_optimizer_.GenerateTrajectory(path_, 200);
    }

    printf("Plan: optimization done, success=%d\n", success ? 1 : 0);
    fflush(stdout);

    return success;
  }

  return true;
}
