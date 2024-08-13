#include <iostream>
#include <string>

#include <Eigen/Dense>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "framework/cccb_trajopt/test_interface.hpp"
#include "framework/cccb_trajopt/test_interface_nocol.hpp"
#include "framework/user_command.hpp"

class PyInterface : public EnvInterface {
  using EnvInterface::EnvInterface;
  void getCommand(SensorData *_sensor_data, RobotCommand *_command) override {
    PYBIND11_OVERLOAD_PURE(void, EnvInterface, getCommand, _sensor_data, _command);
  }
  bool doPlanning(void* user_cmd) override {
    PYBIND11_OVERLOAD_PURE(bool, EnvInterface, doPlanning, user_cmd);
  }
  void updateState(SensorData *_sensor_data) override {
    PYBIND11_OVERLOAD_PURE(void, EnvInterface, updateState, _sensor_data);
  }
};

namespace py = pybind11;

PYBIND11_MODULE(trajopt_planner, m) {
  py::class_<TRAJ_DATA>(m, "TRAJ_DATA")
      .def(py::init<>())
      .def_readwrite("tdata", &TRAJ_DATA::tdata)
      .def_readwrite("qdata", &TRAJ_DATA::qdata)
      .def_readwrite("dqdata", &TRAJ_DATA::dqdata)
      .def_readwrite("xdata", &TRAJ_DATA::xdata)
      .def_readwrite("dxdata", &TRAJ_DATA::dxdata)
      .def_readwrite("period", &TRAJ_DATA::period);

  py::class_<PLANNING_COMMAND>(m, "PLANNING_COMMAND")
      .def(py::init<>())
      .def_readwrite("joint_path", &PLANNING_COMMAND::joint_path)
      .def_readwrite("cartesian_path", &PLANNING_COMMAND::cartesian_path)
      .def_readwrite("max_joint_jerk", &PLANNING_COMMAND::max_joint_jerk)
      .def_readwrite("max_joint_acceleration", &PLANNING_COMMAND::max_joint_acceleration)
      .def_readwrite("max_joint_speed", &PLANNING_COMMAND::max_joint_speed)
      .def_readwrite("obstacles", &PLANNING_COMMAND::obstacles);

  py::class_<OBSTACLE>(m, "OBSTACLE")
      .def(py::init<>())
      .def_readwrite("pose", &OBSTACLE::pose)
      .def_readwrite("dimension", &OBSTACLE::dimension);

  py::class_<SOLUTION>(m, "SOLUTION")
      .def(py::init<>())
      .def_readwrite("h", &SOLUTION::h)
      .def_readwrite("path", &SOLUTION::path)
      .def_readwrite("velocity", &SOLUTION::velocity)
      .def_readwrite("acceleration", &SOLUTION::acceleration)
      .def_readwrite("jerk", &SOLUTION::jerk);

  py::class_<WPT_DATA>(m, "WPT_DATA")
      .def(py::init<>())
      .def("getsize", &WPT_DATA::getsize)
      .def("getdata", &WPT_DATA::getdata)
      .def_readwrite("data", &WPT_DATA::data)
      .def_readwrite("b_cartesian", &WPT_DATA::b_cartesian);

  py::class_<VEC_DATA>(m, "VEC_DATA")
      .def(py::init<>())
      .def_readwrite("data", &VEC_DATA::data);  

  py::class_<EnvInterface, PyInterface>(m, "Interface")
      .def(py::init<>())
      .def("getCommand", &EnvInterface::getCommand)
      .def("doPlanning", &EnvInterface::doPlanning)
      .def("updateState", &EnvInterface::updateState);      

  py::class_<TestInterface, EnvInterface>(m, "TestInterface")
      .def(py::init<std::string, std::string>())
      .def("getPlannedTrajectory", &TestInterface::getPlannedTrajectory)
      .def("getPlannedResult", &TestInterface::getPlannedResult)
      .def("updateVelLimit", &TestInterface::updateVelLimit)
      .def("updateAccLimit", &TestInterface::updateAccLimit)
      .def("updateJerkLimit", &TestInterface::updateJerkLimit)
      .def("updateAlpha", &TestInterface::updateAlpha);

  py::class_<NoColTestInterface, EnvInterface>(m, "NoColTestInterface")
      .def(py::init<std::string>())
      .def("getPlannedTrajectory", &NoColTestInterface::getPlannedTrajectory)
      .def("getPlannedResult", &NoColTestInterface::getPlannedResult)
      .def("updateVelLimit", &NoColTestInterface::updateVelLimit)
      .def("updateAccLimit", &NoColTestInterface::updateAccLimit)
      .def("updateJerkLimit", &NoColTestInterface::updateJerkLimit)
      .def("updateAlpha", &NoColTestInterface::updateAlpha);

  py::class_<SensorData>(m, "SensorData")
      .def(py::init<int>())
      .def_readwrite("elapsed_time", &SensorData::elapsedtime)
      .def_readwrite("joint_positions", &SensorData::q)
      .def_readwrite("joint_velocities", &SensorData::qdot);

  py::class_<RobotCommand>(m, "RobotCommand")
      .def(py::init<int>())
      .def_readwrite("joint_positions", &RobotCommand::q)
      .def_readwrite("joint_velocities", &RobotCommand::qdot)
      .def_readwrite("joint_acclerations", &RobotCommand::qddot)
      .def_readwrite("joint_torques", &RobotCommand::jtrq);
}
