/**
 * @file abstract_driver.cpp
 * @brief Abstract communication driver for esp32imu
 * @author Parker Lusk <plusk@mit.edu>
 * @date 11 Jan 2023
 */

#include <iostream>
#include <stdexcept>

#include "esp32imu/abstract_driver.h"

namespace esp32imu {

// ----------------------------------------------------------------------------
// Send message methods
// ----------------------------------------------------------------------------

void AbstractDriver::sendRate(uint16_t frequency)
{
  // put into standard message
  esp32imu_rate_msg_t msg;
  msg.frequency = frequency;

  uint8_t buf[ESP32IMU_MAX_MESSAGE_LEN];
  const size_t len = esp32imu_rate_msg_send_to_buffer(buf, &msg);
  sendBytes(buf, len);
}

// ----------------------------------------------------------------------------

void AbstractDriver::sendRGBLedCmd(const esp32imu_rgbled_msg_t& msg)
{
  uint8_t buf[ESP32IMU_MAX_MESSAGE_LEN];
  const size_t len = esp32imu_rgbled_msg_send_to_buffer(buf, &msg);
  sendBytes(buf, len);
}

// ----------------------------------------------------------------------------

void AbstractDriver::sendConfig(const esp32imu_config_msg_t& msg)
{
  uint8_t buf[ESP32IMU_MAX_MESSAGE_LEN];
  const size_t len = esp32imu_config_msg_send_to_buffer(buf, &msg);
  sendBytes(buf, len);
}

// ----------------------------------------------------------------------------
// Callback stuff
// ----------------------------------------------------------------------------

void AbstractDriver::registerCallbackIMU(CallbackIMU cb)
{
  std::lock_guard<std::mutex> lock(mtx_);
  cb_imu_ = cb;
}

// ----------------------------------------------------------------------------

void AbstractDriver::registerCallbackRate(CallbackRate cb)
{
  std::lock_guard<std::mutex> lock(mtx_);
  cb_rate_ = cb;
}

// ----------------------------------------------------------------------------

void AbstractDriver::registerCallbackConfig(CallbackConfig cb)
{
  std::lock_guard<std::mutex> lock(mtx_);
  cb_config_ = cb;
}

// ----------------------------------------------------------------------------

void AbstractDriver::unregisterCallbacks()
{
  std::lock_guard<std::mutex> lock(mtx_);
  cb_imu_ = nullptr;
  cb_rate_ = nullptr;
  cb_config_ = nullptr;
}


// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void AbstractDriver::callback(const uint8_t * data, size_t len)
{
  esp32imu_message_t msg;
  for (size_t i=0; i<len; ++i) {
    if (esp32imu_parse_byte(data[i], &msg)) {

      switch (msg.type) {
        case ESP32IMU_MSG_IMU:
          handleIMUMsg(msg);
          break;
        case ESP32IMU_MSG_RATE:
          handleRateMsg(msg);
          break;
        case ESP32IMU_MSG_CONFIG:
          handleConfigMsg(msg);
          break;
      }

    }
  }
}

// ----------------------------------------------------------------------------

void AbstractDriver::handleIMUMsg(const esp32imu_message_t& msg)
{
  esp32imu_imu_msg_t imu;
  esp32imu_imu_msg_unpack(&imu, &msg);

  std::lock_guard<std::mutex> lock(mtx_);
  if (cb_imu_) cb_imu_(imu);
}

// ----------------------------------------------------------------------------

void AbstractDriver::handleRateMsg(const esp32imu_message_t& msg)
{
  esp32imu_rate_msg_t rate;
  esp32imu_rate_msg_unpack(&rate, &msg);

  std::lock_guard<std::mutex> lock(mtx_);
  if (cb_rate_) cb_rate_(rate);
}

// ----------------------------------------------------------------------------

void AbstractDriver::handleConfigMsg(const esp32imu_message_t& msg)
{
  esp32imu_config_msg_t config;
  esp32imu_config_msg_unpack(&config, &msg);

  std::lock_guard<std::mutex> lock(mtx_);
  if (cb_config_) cb_config_(config);
}

} // ns esp32imu
