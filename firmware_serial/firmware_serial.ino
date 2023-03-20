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

union imudata_t {
 uint8_t bytes[32];
 struct {
   float t_sec;
   uint32_t idx;
   float a[3];
   float g[3];
 } data;
};

// IMU ring buffer
static constexpr int IMUBUF_SIZE = 1000;
imudata_t imubuf_[IMUBUF_SIZE];
uint16_t imubuf_head_ = 0;
uint16_t imubuf_tail_ = 0;
uint32_t seq = 0;

volatile uint32_t t0_ = 0; ///< starting time

// logger state machine
enum State { IDLE, INIT, LOG, READ, DONE };
State state;

//=============================================================================
// configuration options
//=============================================================================

// sensor polling interval (micros)
// uint32_t SENSOR_POLL_INTERVAL_US = 1000; // default, can be changed online
// note that ICM20948 has max Fs,accel = 4500 Hz; Fs,gyro = 9000 Hz

static constexpr float g = 9.80665f;

// sample rate, controlled by hardware timer
hw_timer_t * timer = nullptr;
static constexpr int TICKS_PER_SEC = 1000000;
static constexpr int SAMPLE_PER_SEC = 500;
// static constexpr int PERIOD_TICKS = TICKS_PER_SEC / SAMPLE_PER_SEC;

// how many samples to collect?
static constexpr int NUM_SAMPLES = 2 * SAMPLE_PER_SEC;

TaskHandle_t xTaskToNotify = NULL;

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

char DATABIN[] = "/data.bin";
File file_;

//=============================================================================
// Helper functions
//=============================================================================

void update_sample_rate(uint16_t rate)
{
  // SENSOR_POLL_INTERVAL_US = static_cast<uint32_t>(1e6 / rate);
  
  // // pack and ship rate info
  // esp32imu_rate_msg_t rate_msg;
  // rate_msg.frequency = rate;
 
  // const size_t len = esp32imu_rate_msg_send_to_buffer(out_buf, &rate_msg);
  // Serial.write(out_buf, len);
}

// --------------------------------------------------------------------

void set_imu_sample_rate(uint32_t hz)
{
  const uint32_t ticks = TICKS_PER_SEC / hz;
  timerAlarmDisable(timer);
  timerAlarmWrite(timer, ticks, true);
  timerAlarmEnable(timer);
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

// -------------------------------------------------------------------------

void IRAM_ATTR timer_isr() {
  vTaskNotifyGiveFromISR(xTaskToNotify, NULL);
  // TODO: if higher priority woken, do context switch?
}

// -------------------------------------------------------------------------

void vTaskGetData(void * pvParameters) {
  (void) pvParameters;

  // begin the hardware timer / sampling
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, timer_isr, true);
  set_imu_sample_rate(SAMPLE_PER_SEC);
  t0_ = micros();
  
  for (;;) {
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == pdTRUE) {

      // const float t0 = micros(); // use this to measure SPI time
      // (i.e., change t0_ to t0)

      // complete a SPI transaction with IMU to get data
      // takes ~55usec @ 4MHz, ~41usec @ 7MHz, ~32usec @ 12MHz,
      //       ~29usec @ 16MHz, ~26usec @ 24MHz
      imu->getAGT();

      imubuf_[imubuf_head_].data.t_sec = (micros() - t0_) * 1e-6;
      imubuf_[imubuf_head_].data.idx = seq++;
      imubuf_[imubuf_head_].data.a[0] = imu->accX();
      imubuf_[imubuf_head_].data.a[1] = imu->accY();
      imubuf_[imubuf_head_].data.a[2] = imu->accZ();
      imubuf_[imubuf_head_].data.g[0] = imu->gyrX();
      imubuf_[imubuf_head_].data.g[1] = imu->gyrY();
      imubuf_[imubuf_head_].data.g[2] = imu->gyrZ();

      // stream_imu(imubuf_[imubuf_head_]);

      // move the head of the buffer, wrapping around if neccessary
      imubuf_head_ = (imubuf_head_ + 1) % IMUBUF_SIZE;      
    }
  }
}

//=============================================================================
// initialize
//=============================================================================

void setup()
{
  rgbled.brightness(5); // 5% brightness
  rgbled.setColor(RGBLed::WHITE);

  // set up serial communication
  Serial.begin(2000000);

  init_imu();

  //
  // Transmit configuration info
  //

  // this seems to break serial / not work anyways (see safe_serial_write)
//  update_sample_rate(static_cast<uint16_t>(1e6/SENSOR_POLL_INTERVAL_US));

  xTaskCreatePinnedToCore(vTaskGetData, "vTaskGetData", 1024, NULL, 2, &xTaskToNotify, 1);
  xTaskCreatePinnedToCore(vLoop, "vLoop", 4096, NULL, 1, NULL, 0);
}

//=============================================================================
// loop
//=============================================================================

void stream_imu(const imudata_t& data)
{
  // pack and ship IMU data
    esp32imu_imu_msg_t imu_msg;
    imu_msg.t_us = data.data.t_sec * 1e6;
    imu_msg.accel_x = data.data.a[0] * g;
    imu_msg.accel_y = data.data.a[1] * g;
    imu_msg.accel_z = data.data.a[2] * g;
    imu_msg.gyro_x = data.data.g[0] * DEG2RAD;
    imu_msg.gyro_y = data.data.g[1] * DEG2RAD;
    imu_msg.gyro_z = data.data.g[2] * DEG2RAD;

    const size_t len = esp32imu_imu_msg_send_to_buffer(out_buf, &imu_msg);
    Serial.write(out_buf, len);
}

void loop()
{
  uint32_t current_time_us = micros() - start_time_us;
 
  if (current_time_us >= sensor_poll_previous_us + 10000) {
  // if (current_time_us >= sensor_poll_previous_us + SENSOR_POLL_INTERVAL_US) {

  //   imu->getAGT();

  //   // pack and ship IMU data
  //   esp32imu_imu_msg_t imu_msg;
  //   imu_msg.t_us = current_time_us;
  //   imu_msg.accel_x = imu->accX() * g;
  //   imu_msg.accel_y = imu->accY() * g;
  //   imu_msg.accel_z = imu->accZ() * g;
  //   imu_msg.gyro_x = imu->gyrX() * DEG2RAD;
  //   imu_msg.gyro_y = imu->gyrY() * DEG2RAD;
  //   imu_msg.gyro_z = imu->gyrZ() * DEG2RAD;
   
  //   const size_t len = esp32imu_imu_msg_send_to_buffer(out_buf, &imu_msg);
  //   Serial.write(out_buf, len);

  //   sensor_poll_previous_us = current_time_us;

    // stream_imu(imubuf_[imubuf_head_]);
  }
}

// -------------------------------------------------------------------------

void vLoop(void * pvParameters) {
  static float t0 = 0;
  static float tf = 0;
  static uint32_t seqnow = 0;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {

    if (state == State::IDLE) {
      state = State::LOG;
    } else if (state == State::INIT) {
      t0 = millis() * 1e-3;
      // file_ = fs_.open(DATABIN, FILE_WRITE);
      // Serial.print(DATABIN); Serial.print(": "); Serial.print(file_.size()); Serial.println(" bytes");
      state = State::LOG;
    } else if (state == State::LOG) {
    
      // timer may put additional data in buffer while this loop executes,
      // but we will only read up to what we can see right now
      const uint16_t head = imubuf_head_;
      seqnow = seq;
  
      // if there is new data to write
      if (head != imubuf_tail_) {
  
  
        // if head ptr has looped back around, first get the data from here to the end
        if (head < imubuf_tail_) {

            for (; imubuf_tail_ < IMUBUF_SIZE; imubuf_tail_++) {
              stream_imu(imubuf_[imubuf_tail_]);
            }

            // const uint16_t len = IMUBUF_SIZE - imubuf_tail_;
            // file_.write(reinterpret_cast<uint8_t*>(&imubuf_[imubuf_tail_]), len * sizeof(imudata_t));
            imubuf_tail_ = 0;
        }
  
        
        // getting data is easy, from tail to head
        if (head > imubuf_tail_) {
          for (; imubuf_tail_ < head; imubuf_tail_++) {
              stream_imu(imubuf_[imubuf_tail_]);
            }
          // const uint16_t len = head - imubuf_tail_;
          // file_.write(reinterpret_cast<uint8_t*>(&imubuf_[imubuf_tail_]), len * sizeof(imudata_t));
          imubuf_tail_ = head;
        }
        
      }
  
      // if (seqnow >= NUM_SAMPLES) {
      //   tf = (millis() * 1e-3) - t0;
      //   Serial.print(seqnow); Serial.print(" samples in "); Serial.print(tf); Serial.println(" seconds");
      //   state = State::READ;
      // }
    // } else if (state == State::READ) {
  
    //   file_.close();
  
    //   Serial.println("");
    //   Serial.println("Reading...");
    //   Serial.println("");
  
    //   file_ = fs_.open(DATABIN);
    //   Serial.print(DATABIN); Serial.print(": "); Serial.print(file_.size()); Serial.println(" bytes");
  
    //   if (file_) {
    //     imudata_t imu;
    //     while (file_.position() < file_.size()) {
    //       file_.read(imu.bytes, sizeof(imudata_t));
  
    //       static constexpr int txtlen = 200;
    //       char txt[txtlen];
    //       snprintf(txt, txtlen, "%d, %3.6f, %3.6f, %7.4f, %7.4f, %9.5f, %9.5f, %9.5f\n",
    //           imu.data.idx, imu.data.t_sec,
    //           imu.data.a[0], imu.data.a[1], imu.data.a[2],
    //           imu.data.g[0], imu.data.g[1], imu.data.g[2]);
    //       Serial.print(txt);
    //     }
        
    //   } else {
    //     Serial.println("Failed to open data file for reading");
    //   }
  
    //   state = State::DONE;
    // } else if (state == State::DONE) {
  
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
