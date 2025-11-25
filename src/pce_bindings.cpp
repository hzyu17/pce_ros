// pce_bindings.cpp
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "carla_pce_wrapper.h"

namespace py = pybind11;

PYBIND11_MODULE(pce_planner, m) {
    m.doc() = "PCE Motion Planner for CARLA";
    
    // Obstacle2D
    py::class_<Obstacle2D>(m, "Obstacle2D")
        .def(py::init<float, float, float, float, float>(),
             py::arg("x"), py::arg("y"), 
             py::arg("width"), py::arg("length"),
             py::arg("rotation") = 0.0f)
        .def_readwrite("x", &Obstacle2D::x)
        .def_readwrite("y", &Obstacle2D::y)
        .def_readwrite("width", &Obstacle2D::width)
        .def_readwrite("length", &Obstacle2D::length)
        .def_readwrite("rotation", &Obstacle2D::rotation);
    
    // CarlaPCEPlanner
    py::class_<CarlaPCEPlanner>(m, "PCEPlanner")
        .def(py::init<>())
        .def("initialize", &CarlaPCEPlanner::initialize,
             "Initialize planner with parameters",
             py::arg("num_dimensions") = 3,
             py::arg("num_waypoints") = 50,
             py::arg("total_time") = 10.0f,
             py::arg("num_samples") = 3000,
             py::arg("num_iterations") = 30)
        .def("set_obstacles", &CarlaPCEPlanner::setObstacles,
             "Set obstacles in the scene (list of [x, y, width, length, rotation])",
             py::arg("obstacles"))
        .def("plan", &CarlaPCEPlanner::plan,
             "Plan trajectory from start to goal",
             py::arg("start"),
             py::arg("goal"))
        .def("set_parameters", &CarlaPCEPlanner::setParameters,
             "Set planner parameters",
             py::arg("num_iterations"),
             py::arg("gamma"),
             py::arg("temperature"))
        .def("set_safety_radius", &CarlaPCEPlanner::setSafetyRadius,
             "Set safety radius around obstacles",
             py::arg("radius"))
        .def("get_trajectory", &CarlaPCEPlanner::getTrajectory,
             "Get the final planned trajectory")
        .def("get_trajectory_history", &CarlaPCEPlanner::getTrajectoryHistory,
             "Get trajectory history from all iterations");
}
