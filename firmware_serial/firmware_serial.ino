#include <memory>

#include <FS.h>
#include <SD.h>
#include <SPI.h>

#include <ICM42688.h>   // Click here to get the library: http://librarymanager/All#ICM42688
#include <RGBLed.h>     // http://librarymanager/All#RGB_WILMOUTH

#include "esp32imu.h"

SPIClass SPI2(HSPI);
SPIClass SPI3(VSPI);

RGBLed rgbled(0, 2, 4, RGBLed::COMMON_ANODE);
std::unique_ptr<ICM42688> imu;

// IMU ring buffer
static constexpr int IMUBUF_SIZE = 1000;
esp32imu_imu_msg_t imubuf_[IMUBUF_SIZE];
uint16_t imubuf_head_ = 0;
uint16_t imubuf_tail_ = 0;
uint32_t seq = 0;

volatile uint32_t t0_ = 0; ///< starting time

// logger state machine
enum State { IDLE, INIT, LOG, LOG_DONE, READ, DONE };
State state;

//=============================================================================
// configuration options
//=============================================================================

// sensor polling interval (micros)
uint32_t SENSOR_POLL_INTERVAL_US = 2000; // default, can be changed online

static constexpr float g = 9.80665f;

bool stream_imu_ = true;

char DATABIN[] = "/data.bin";

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

File file_;

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

// --------------------------------------------------------------------

bool init_fs() {
  SPI3.begin();
  if (!SD.begin(SPI3.pinSS(), SPI3, 25000000)) {
    return false;
  }
  return true;
}

// --------------------------------------------------------------------

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
  rgbled.setColor(RGBLed::BLUE);

  // set up serial communication
  Serial.begin(2000000);

  init_fs();
  init_imu();

  //
  // RTOS Tasks
  //

  xTaskCreatePinnedToCore(vLoop, "vSdLogger", 4096, NULL, 1, NULL, 1);
}

//=============================================================================
// loop
//=============================================================================

void loop()
{
  // vTaskDelete(NULL);
  uint32_t current_time_us = micros() - start_time_us;

  if (current_time_us >= sensor_poll_previous_us + SENSOR_POLL_INTERVAL_US) {

    imu->getAGT();

    // pack IMU data
    esp32imu_imu_msg_t imu_msg;
    imu_msg.t_us = current_time_us;
    imu_msg.id = seq++;
    imu_msg.accel_x = imu->accX() * g;
    imu_msg.accel_y = imu->accY() * g;
    imu_msg.accel_z = imu->accZ() * g;
    imu_msg.gyro_x = imu->gyrX() * DEG2RAD;
    imu_msg.gyro_y = imu->gyrY() * DEG2RAD;
    imu_msg.gyro_z = imu->gyrZ() * DEG2RAD;

    // put IMU messsage into circular buffer
    imubuf_[imubuf_head_] = imu_msg;
    // move the head of the buffer, wrapping around if neccessary
    imubuf_head_ = (imubuf_head_ + 1) % IMUBUF_SIZE;  

    if (stream_imu_) {
      const size_t len = esp32imu_imu_msg_send_to_buffer(out_buf, &imu_msg);
      Serial.write(out_buf, len);
    }

    sensor_poll_previous_us = current_time_us;
  }
}

// -------------------------------------------------------------------------

void vSdLogger(void * pvParameters) {
  static float t0 = 0;
  static float tf = 0;
  static uint32_t seqnow = 0;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {

    if (state == State::IDLE) {
      // state = State::INIT;
    } else if (state == State::INIT) {
      t0 = millis() * 1e-3;
      file_ = SD.open(DATABIN, FILE_WRITE);
      // Serial.print(DATABIN); Serial.print(": "); Serial.print(file_.size()); Serial.println(" bytes");
      state = State::LOG;

      rgbled.brightness(100);
      rgbled.setColor(RGBLed::RED);
    } else if (state == State::LOG) {
    
      // timer may put additional data in buffer while this loop executes,
      // but we will only read up to what we can see right now
      const uint16_t head = imubuf_head_;
      seqnow = seq;
  
      // if there is new data to write
      if (head != imubuf_tail_) {
  
  
        // if head ptr has looped back around, first get the data from here to the end
        if (head < imubuf_tail_) {
            const uint16_t len = IMUBUF_SIZE - imubuf_tail_;
            file_.write(reinterpret_cast<uint8_t*>(&imubuf_[imubuf_tail_]), len * sizeof(esp32imu_imu_msg_t));
            imubuf_tail_ = 0;
        }
  
        
        // getting data is easy, from tail to head
        if (head > imubuf_tail_) {
          const uint16_t len = head - imubuf_tail_;
          file_.write(reinterpret_cast<uint8_t*>(&imubuf_[imubuf_tail_]), len * sizeof(esp32imu_imu_msg_t));
          imubuf_tail_ = head;
        }
        
      }

    } else if (state == State::LOG_DONE) {
      rgbled.brightness(100);
      rgbled.setColor(RGBLed::GREEN);
      file_.close();

      state = State::IDLE;
    } else if (state == State::READ) {
      rgbled.brightness(100);
      rgbled.setColor(RGBLed::YELLOW);

      stream_imu_ = false;
  
      file_ = SD.open(DATABIN);
      // Serial.print(DATABIN); Serial.print(": "); Serial.print(file_.size()); Serial.println(" bytes");
  
      if (file_) {
        esp32imu_imu_msg_t imu_msg;
        while (file_.position() < file_.size()) {
          file_.read(reinterpret_cast<uint8_t*>(&imu_msg), sizeof(esp32imu_imu_msg_t));
  
          const size_t len = esp32imu_imu_msg_send_to_buffer(out_buf, &imu_msg);
          Serial.write(out_buf, len);
        }
        
      } else {
        rgbled.brightness(5);
        rgbled.setColor(RGBLed::RED);
      }
  
      rgbled.brightness(100);
      rgbled.setColor(RGBLed::GREEN);
      state = State::IDLE;
    }
    
    xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
  }
}

//=============================================================================
// handle received serial data
//=============================================================================

void serialEvent()
{
  while (Serial.available()) {
    uint8_t in_byte = (uint8_t) Serial.read();
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
        case ESP32IMU_MSG_CONFIG:
        {
          esp32imu_config_msg_t msg;
          esp32imu_config_msg_unpack(&msg, &msg_buf);
          handle_config_msg(msg);
          break;   
        }
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
  // set_imu_sample_rate(msg.frequency);
}

void handle_rgbled_msg(const esp32imu_rgbled_msg_t& msg)
{
  rgbled.brightness(msg.brightness);
  rgbled.setColor(msg.r, msg.g, msg.b);
}

void handle_config_msg(const esp32imu_config_msg_t& msg)
{
  stream_imu_ = msg.stream;
  if (state == State::IDLE && msg.logging) state = State::INIT;
  if (state == State::LOG && !msg.logging) state = State::LOG_DONE;
  if (state == State::IDLE && msg.readlog) state = State::READ;
}