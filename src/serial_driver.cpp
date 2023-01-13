/**
 * @file serial_driver.cpp
 * @brief Serial communication driver for esp32imu
 * @author Parker Lusk <plusk@mit.edu>
 * @date 11 Jan 2023
 */

#include <iostream>
#include <stdexcept>

#include <async_comm/serial.h>

#include "esp32imu/serial_driver.h"

namespace esp32imu {

SerialDriver::SerialDriver(const std::string& port, uint32_t baud)
{
  using namespace std::placeholders;

  serial_.reset(new async_comm::Serial(port, baud));
  serial_->register_receive_callback(
            std::bind(&SerialDriver::callback, this, _1, _2));

  if (!serial_->init()) {
    throw std::runtime_error("Could not open serial port '" + port + "'");
  }
}

// ----------------------------------------------------------------------------

SerialDriver::~SerialDriver()
{
  serial_->close();
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void SerialDriver::sendBytes(const uint8_t * src, size_t len)
{
  serial_->send_bytes(src, len);
}

} // ns esp32imu
