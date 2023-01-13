/**
 * @file serial_driver.h
 * @brief Serial communication driver for esp32imu
 * @author Parker Lusk <plusk@mit.edu>
 * @date 11 Jan 2023
 */

#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "esp32imu/abstract_driver.h"

namespace async_comm { class Serial; }

namespace esp32imu {

  class SerialDriver : public AbstractDriver
  {
  public:
    SerialDriver(const std::string& port = "/dev/ttyUSB0", uint32_t baud = 115200);
    ~SerialDriver();

  private:
    std::unique_ptr<async_comm::Serial> serial_;
    void sendBytes(const uint8_t * src, size_t len) override;
  };

} // ns esp32imu
