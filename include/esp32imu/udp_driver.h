/**
 * @file udp_driver.h
 * @brief UDP communication driver for esp32imu
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

namespace async_comm { class UDP; }

namespace esp32imu {

  class UDPDriver : public AbstractDriver
  {
  public:
    UDPDriver(const std::string& bind_host = "0.0.0.0", uint32_t bind_port = 3333,
          const std::string& rem_host = "192.168.4.1", uint32_t rem_port = 3333);
    ~UDPDriver();

  private:
    std::unique_ptr<async_comm::UDP> udp_;
    void sendBytes(const uint8_t * src, size_t len) override;
  };

} // ns esp32imu
