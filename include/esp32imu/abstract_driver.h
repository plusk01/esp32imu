/**
 * @file abstract_driver.h
 * @brief Abstract communication driver for esp32imu
 * @author Parker Lusk <plusk@mit.edu>
 * @date 11 Jan 2023
 */

#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "protocol/esp32imu.h"

namespace esp32imu {

  using CallbackIMU = std::function<void(const esp32imu_imu_msg_t&)>;
  using CallbackRate = std::function<void(const esp32imu_rate_msg_t&)>;
  using CallbackConfig = std::function<void(const esp32imu_config_msg_t&)>;

  class AbstractDriver
  {
  public:
    virtual ~AbstractDriver() {};

    void sendRate(uint16_t frequency);
    void sendRGBLedCmd(const esp32imu_rgbled_msg_t& msg);
    void sendConfig(const esp32imu_config_msg_t& msg);

    void registerCallbackIMU(CallbackIMU cb);
    void registerCallbackRate(CallbackRate cb);
    void registerCallbackConfig(CallbackConfig cb);
    void unregisterCallbacks();
    
  protected:
    void callback(const uint8_t * data, size_t len);

  private:
    CallbackIMU cb_imu_;
    CallbackRate cb_rate_;
    CallbackConfig cb_config_;
    std::mutex mtx_; ///< synchronize callback resource reg/unreg


    void handleIMUMsg(const esp32imu_message_t& msg);
    void handleRateMsg(const esp32imu_message_t& msg);
    void handleConfigMsg(const esp32imu_message_t& msg);

    virtual void sendBytes(const uint8_t * src, size_t len) = 0;
  };

} // ns esp32imu
