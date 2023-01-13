/**
 * @file udp_driver.cpp
 * @brief UDP communication driver for esp32imu
 * @author Parker Lusk <plusk@mit.edu>
 * @date 11 Jan 2023
 */

#include <iostream>
#include <stdexcept>

#include <async_comm/udp.h>

#include "esp32imu/udp_driver.h"

namespace esp32imu {

UDPDriver::UDPDriver(const std::string& bind_host, uint32_t bind_port)
{
  using namespace std::placeholders;

  udp_.reset(new async_comm::UDP(bind_host, bind_port));
  udp_->register_receive_callback(
            std::bind(&UDPDriver::callback, this, _1, _2));

  if (!udp_->init()) {
    throw std::runtime_error("UDP could not bind to '" + bind_host + ":" + std::to_string(bind_port) + "'");
  }
}

// ----------------------------------------------------------------------------

UDPDriver::~UDPDriver()
{
  udp_->close();
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void UDPDriver::sendBytes(const uint8_t * src, size_t len)
{
  udp_->send_bytes(src, len);
}

} // ns esp32imu
