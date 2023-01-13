/**
 * @file py_esp32imu.cpp
 * @brief Python bindings for esp32imu
 * @author Parker Lusk <plusk@mit.edu>
 * @date 22 Nov 2020
 */

#include <cstdint>

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>

#include <protocol/esp32imu.h>
#include <esp32imu/serial_driver.h>
#include <esp32imu/udp_driver.h>

namespace py = pybind11;

PYBIND11_MODULE(esp32imu, m)
{
  m.doc() = "Communication driver and tools for esp32imu";
  m.attr("__version__") = PROJECT_VERSION;

  py::class_<esp32imu_imu_msg_t>(m, "IMUMsg")
    .def_readwrite("t_us", &esp32imu_imu_msg_t::t_us)
    .def_readwrite("accel_x", &esp32imu_imu_msg_t::accel_x)
    .def_readwrite("accel_y", &esp32imu_imu_msg_t::accel_y)
    .def_readwrite("accel_z", &esp32imu_imu_msg_t::accel_z)
    .def_readwrite("gyro_x", &esp32imu_imu_msg_t::gyro_x)
    .def_readwrite("gyro_y", &esp32imu_imu_msg_t::gyro_y)
    .def_readwrite("gyro_z", &esp32imu_imu_msg_t::gyro_z);

  py::class_<esp32imu_rate_msg_t>(m, "RateMsg")
    .def(py::init<uint16_t>(),
          py::arg("frequency")=500)
    .def_readwrite("frequency", &esp32imu_rate_msg_t::frequency);

  py::class_<esp32imu_rgbled_msg_t>(m, "RGBLedCmdMsg")
    .def(py::init<>())
    .def_readwrite("r", &esp32imu_rgbled_msg_t::r)
    .def_readwrite("g", &esp32imu_rgbled_msg_t::g)
    .def_readwrite("b", &esp32imu_rgbled_msg_t::b)
    .def_readwrite("brightness", &esp32imu_rgbled_msg_t::brightness);

  py::class_<esp32imu::SerialDriver>(m, "SerialDriver")
    .def(py::init<const std::string&, uint32_t>(),
          py::arg("port")="/dev/ttyUSB0", py::arg("baud")=115200)
    .def("sendRate", &esp32imu::SerialDriver::sendRate)
    .def("sendRGBLedCmd", &esp32imu::SerialDriver::sendRGBLedCmd)
    .def("registerCallbackIMU", &esp32imu::SerialDriver::registerCallbackIMU)
    .def("registerCallbackRate", &esp32imu::SerialDriver::registerCallbackRate)
    .def("unregisterCallbacks", &esp32imu::SerialDriver::unregisterCallbacks, py::call_guard<py::gil_scoped_release>());

  py::class_<esp32imu::UDPDriver>(m, "UDPDriver")
    .def(py::init<const std::string&, uint32_t>(),
          py::arg("bind_host")="0.0.0.0", py::arg("bind_port")=3333)
    .def("sendRate", &esp32imu::UDPDriver::sendRate)
    .def("sendRGBLedCmd", &esp32imu::UDPDriver::sendRGBLedCmd)
    .def("registerCallbackIMU", &esp32imu::UDPDriver::registerCallbackIMU)
    .def("registerCallbackRate", &esp32imu::UDPDriver::registerCallbackRate)
    .def("unregisterCallbacks", &esp32imu::UDPDriver::unregisterCallbacks, py::call_guard<py::gil_scoped_release>());
}