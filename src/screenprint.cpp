/**
 * @file screenprint.cpp
 * @brief Example application that dumps esp32imu IMU data to terminal
 * @author Parker Lusk <plusk@mit.edu>
 * @date 21 Nov 2020
 */

#include <chrono>
#include <future>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>
#include <sstream>

#include <argparse/argparse.hpp>

#include <esp32imu/serial_driver.h>
#include <esp32imu/udp_driver.h>

/// \brief Global variables
uint32_t last_t_us = 0;

/**
 * @brief      Handles IMU messages when received via serial
 *
 * @param[in]  msg   The unpacked IMU message
 */
void callback(const esp32imu_imu_msg_t& msg)
{
  const double dt = (msg.t_us - last_t_us) * 1e-6; // us to s
  // const double hz = 1. / dt;
  last_t_us = msg.t_us;

  std::stringstream hz;
  hz << std::fixed << std::setprecision(0) << 1. / dt;

  static constexpr int w = 5;
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2)
     << std::setw(w) << std::setfill(' ')
     << msg.accel_x << ", "
     << std::setw(w) << std::setfill(' ')
     << msg.accel_y << ", "
     << std::setw(w) << std::setfill(' ')
     << msg.accel_z
     << '\t'
     << std::setw(w) << std::setfill(' ')
     << msg.gyro_x << ", "
     << std::setw(w) << std::setfill(' ')
     << msg.gyro_y << ", "
     << std::setw(w) << std::setfill(' ')
     << msg.gyro_z;

  std::cout << "Got IMU " << msg.id << " at " << msg.t_us << " us (" << hz.str() << " Hz): "
            << ss.str() << std::endl;
}

void rateCb(const esp32imu_rate_msg_t& msg)
{
  std::cout << std::endl << "***************************" << std::endl;
  std::cout << "Sample Rate: " << msg.frequency << " Hz" << std::endl;
  std::cout << "***************************" << std::endl << std::endl;
}

int main(int argc, char const *argv[])
{
  argparse::ArgumentParser program("screenprint", PROJECT_VERSION);

  program.add_argument("--serial")
    .help("Use Serial communication")
    .default_value(false)
    .implicit_value(true);

  program.add_argument("--port")
    .help("Serial port to use for communication")
    .default_value(std::string({"/dev/ttyUSB0"}));

  program.add_argument("--baud")
    .help("Baud rate for serial communication")
    .default_value(2000000)
    .scan<'i', int>();

  program.add_argument("--udp")
    .help("Use UDP communication")
    .default_value(false)
    .implicit_value(true);

  program.add_argument("--bind-host")
    .help("UDP bind host to use for communication")
    .default_value(std::string({"0.0.0.0"}));

  program.add_argument("--bind-port")
    .help("UDP bind port to use for communication")
    .default_value(3333)
    .scan<'i', int>();

  try {
    program.parse_args(argc, argv);
  } catch (const std::runtime_error& err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    std::exit(1);
  }

  std::unique_ptr<esp32imu::AbstractDriver> driver;
  if (program["--udp"] == true) {
    const std::string host = program.get<std::string>("--bind-host");
    const int port = program.get<int>("--bind-port");
    driver.reset(new esp32imu::UDPDriver(host, port));
  } else/* if (program["--serial"] == true)*/ {
    const std::string port = program.get<std::string>("--port");
    const int baud = program.get<int>("--baud");
    driver.reset(new esp32imu::SerialDriver(port, baud));
  }

  driver->registerCallbackIMU(callback);
  driver->registerCallbackRate(rateCb);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // driver->sendRate(500);

  esp32imu_rgbled_msg_t msg;
  msg.r = 0;
  msg.g = 0;
  msg.b = 255;
  msg.brightness = 5;
  driver->sendRGBLedCmd(msg);

  // spin forever and let CPU do other things (no busy waiting)
  std::promise<void>().get_future().wait();
  return 0;
}
