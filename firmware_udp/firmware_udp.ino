#include <memory>

#include <WiFi.h>
#include <WiFiAP.h>
#include <AsyncUDP.h>
#include <SPI.h>

#include <ICM42688.h>   // Click here to get the library: http://librarymanager/All#ICM42688
#include <RGBLed.h>     // http://librarymanager/All#RGB_WILMOUTH

#include "esp32imu.h"

SPIClass SPI2(HSPI);
SPIClass SPI3(VSPI);

RGBLed rgbled(0, 2, 4, RGBLed::COMMON_ANODE);
std::unique_ptr<ICM42688> imu;

AsyncUDP udp;

//=============================================================================
// configuration options
//=============================================================================

// sensor polling interval (micros)
uint32_t SENSOR_POLL_INTERVAL_US = 10000; // default, can be changed online

static constexpr float g = 9.80665f;

// WIFI AP credentials
const char *ssid = "esp32fcu";
const char *password = "esp32fcu";

static constexpr int UDP_PORT = 3333;

//=============================================================================
// global variables
//=============================================================================

// serial stuff
uint8_t out_buf[ESP32IMU_MAX_MESSAGE_LEN];
esp32imu_message_t msg_buf;

// timing
uint32_t sensor_poll_previous_us = 0;
uint32_t start_time_us = 0;

// computed constants
static constexpr double DEG2RAD = M_PI/180.;

//=============================================================================
// Helper functions
//=============================================================================

void update_sample_rate(uint16_t rate)
{
  SENSOR_POLL_INTERVAL_US = static_cast<uint32_t>(1e6 / rate);
  
  // pack and ship rate info
  esp32imu_rate_msg_t rate_msg;
  rate_msg.frequency = rate;
 
  const size_t len = esp32imu_rate_msg_send_to_buffer(out_buf, &rate_msg);
  Serial.write(out_buf, len);
}

void init_imu() {
  SPI2.begin();

  bool initialized = false;
  while (!initialized) {
    imu.reset(new ICM42688(SPI2, SPI2.pinSS(), 12000000));
    int status = imu->begin();
    if (status < 0) {
      Serial.println("IMU initialization unsuccessful");
      Serial.println("Check IMU wiring or try cycling power");
      Serial.print("Status: ");
      Serial.println(status);
      delay(500);
    } else {
      initialized = true;
    }
  }

  imu->setAccelODR(ICM42688::odr8k);
  imu->setGyroODR(ICM42688::odr8k);

  Serial.println("IMU config'd");
}

//=============================================================================
// initialize
//=============================================================================

void setup()
{
  rgbled.brightness(5); // 5% brightness
  rgbled.setColor(RGBLed::WHITE);

  // set up serial communication
  Serial.begin(115200);
  Serial.println();
  Serial.println("Configuring access point...");

  // You can remove the password parameter if you want the AP to be open.
  WiFi.softAP(ssid, password);
  // IPAddress myIP = WiFi.softAPIP();
  // Serial.print("AP IP address: ");
  // Serial.println(myIP);

  if (udp.listen(UDP_PORT)) {
    udp.onPacket([](AsyncUDPPacket packet) {
      char buf[10];
      for (size_t i=0; i<packet.length(); ++i) {
        sprintf(buf, "0x%02x ", packet.data()[i]); Serial.print(buf);
        message_parser(packet.data()[i]);
      }
      Serial.println();
    });
  }

  init_imu();
}

//=============================================================================
// loop
//=============================================================================

void loop()
{
  uint32_t current_time_us = micros() - start_time_us;
 
  if (current_time_us >= sensor_poll_previous_us + SENSOR_POLL_INTERVAL_US) {

    imu->getAGT();

    // pack and ship IMU data
    esp32imu_imu_msg_t imu_msg;
    imu_msg.t_us = current_time_us;
    imu_msg.accel_x = imu->accX() * g;
    imu_msg.accel_y = imu->accY() * g;
    imu_msg.accel_z = imu->accZ() * g;
    imu_msg.gyro_x = imu->gyrX() * DEG2RAD;
    imu_msg.gyro_y = imu->gyrY() * DEG2RAD;
    imu_msg.gyro_z = imu->gyrZ() * DEG2RAD;
   
    const size_t len = esp32imu_imu_msg_send_to_buffer(out_buf, &imu_msg);
    // Serial.write(out_buf, len);
    udp.broadcastTo(out_buf, len, UDP_PORT);

    sensor_poll_previous_us = current_time_us;
  }
}

//=============================================================================
// handle received serial data
//=============================================================================

// void serialEvent()
// {
//   while (Serial.available()) {
//     uint8_t in_byte = (uint8_t) Serial.read();
//     message_parser(in_byte);
//   }
// }

void message_parser(uint8_t in_byte)
{
  if (esp32imu_parse_byte(in_byte, &msg_buf)) {
    switch (msg_buf.type) {
      case ESP32IMU_MSG_RATE:
      {
        esp32imu_rate_msg_t msg;
        esp32imu_rate_msg_unpack(&msg, &msg_buf);
        handle_rate_msg(msg);
        break;
      }
      case ESP32IMU_MSG_RGBLED:
      {
        esp32imu_rgbled_msg_t msg;
        esp32imu_rgbled_msg_unpack(&msg, &msg_buf);
        handle_rgbled_msg(msg);
        break;
      }
    }
  }
}

//=============================================================================
// handle received messages
//=============================================================================

void handle_rate_msg(const esp32imu_rate_msg_t& msg)
{
  start_time_us = micros(); // reset start time
  sensor_poll_previous_us = 0;
  update_sample_rate(msg.frequency);
}

void handle_rgbled_msg(const esp32imu_rgbled_msg_t& msg)
{
  rgbled.brightness(msg.brightness);
  rgbled.setColor(msg.r, msg.g, msg.b);
}
