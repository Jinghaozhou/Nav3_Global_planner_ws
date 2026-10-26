#include "ele_planner/offline_ele_planner.h"
#include <cstring>
#include "pybind11/eigen.h"
#include "pybind11/numpy.h"
#include "pybind11/pybind11.h"

namespace py = pybind11;

static Eigen::MatrixXd array_to_matrix(const py::array_t<double,
                                                        py::array::c_style |
                                                            py::array::forcecast>& a) {
  auto info = a.request();
  if (info.ndim != 2) throw std::runtime_error("init_map expects 2D arrays");
  Eigen::MatrixXd out(info.shape[0], info.shape[1]);
  const auto* src = static_cast<const double*>(info.ptr);
  for (ssize_t r = 0; r < info.shape[0]; ++r)
    for (ssize_t c = 0; c < info.shape[1]; ++c)
      out(r, c) = src[r * info.shape[1] + c];
  return out;
}

static Eigen::Vector3i array_to_vec3i(const py::array_t<int,
                                                        py::array::c_style |
                                                            py::array::forcecast>& a) {
  auto info = a.request();
  if (info.ndim != 1 || info.shape[0] != 3)
    throw std::runtime_error("plan expects 3-element index arrays");
  const auto* src = static_cast<const int*>(info.ptr);
  return Eigen::Vector3i(src[0], src[1], src[2]);
}

PYBIND11_MODULE(ele_planner, m) {
  auto pyOfflineElePlanner =
      py::class_<OfflineElePlanner>(m, "OfflineElePlanner");
  pyOfflineElePlanner
      .def(py::init<double, bool>(), py::arg("max_heading_rate"),
           py::arg("use_quintic") = false)
      .def("init_map", [](OfflineElePlanner& self, double a, double s,
                           double res, int layers, double w,
                           py::array_t<double, py::array::c_style | py::array::forcecast> cost,
                           py::array_t<double, py::array::c_style | py::array::forcecast> height,
                           py::array_t<double, py::array::c_style | py::array::forcecast> ceiling,
                           py::array_t<double, py::array::c_style | py::array::forcecast> ele,
                           py::array_t<double, py::array::c_style | py::array::forcecast> gx,
                           py::array_t<double, py::array::c_style | py::array::forcecast> gy) {
        self.InitMap(a, s, res, layers, w, array_to_matrix(cost),
                     array_to_matrix(height), array_to_matrix(ceiling),
                     array_to_matrix(ele), array_to_matrix(gx), array_to_matrix(gy));
      })
      .def("plan", [](OfflineElePlanner& self,
                       py::array_t<int, py::array::c_style | py::array::forcecast> start,
                       py::array_t<int, py::array::c_style | py::array::forcecast> goal,
                       bool optimize) {
        return self.Plan(array_to_vec3i(start), array_to_vec3i(goal), optimize);
      }, py::arg("start"), py::arg("goal"), py::arg("optimize") = true)
      .def("debug", &OfflineElePlanner::Debug)
      .def("set_reference_height", &OfflineElePlanner::SetReferenceHeight)
      .def("set_max_iterations", &OfflineElePlanner::set_max_iterations)
      .def("get_path_finder", &OfflineElePlanner::get_path_finder)
      .def("get_map", &OfflineElePlanner::get_map)
      .def("get_trajectory_optimizer",
           &OfflineElePlanner::get_trajectory_optimizer)
      .def("get_trajectory_optimizer_wnoj",
           &OfflineElePlanner::get_trajectory_optimizer_wnoj)
      .def("get_debug_path", &OfflineElePlanner::GetDebugPath);
}
