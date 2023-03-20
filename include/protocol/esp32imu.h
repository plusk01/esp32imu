/**
 * @file esp32imu.h
 * @brief Serial protocol for ACL esp32fcu
 * @author Parker Lusk <plusk@mit.edu>
 * @date 11 Jan 2023
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifdef __GNUC__
#define PACKED_STRUCT(name) struct __attribute__((__packed__)) name
#else
#define PACKED_STRUCT(name) __pragma(pack(push, 1)) struct name __pragma(pack(pop))
#endif

//=============================================================================
// message types
//=============================================================================

typedef enum {
  ESP32IMU_MSG_IMU,
  ESP32IMU_MSG_RATE,
  ESP32IMU_MSG_RGBLED,
  ESP32IMU_MSG_CONFIG,
  ESP32IMU_NUM_MSGS
} ti_msg_type_t;

//=============================================================================
// message definitions
//=============================================================================

typedef struct {
  uint32_t t_us;
  uint32_t id;
  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
} esp32imu_imu_msg_t;

typedef struct {
  uint16_t frequency;
} esp32imu_rate_msg_t;

typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t brightness;
} esp32imu_rgbled_msg_t;

typedef struct {
  bool stream;
  bool logging;
  bool readlog;
} esp32imu_config_msg_t;

// payload lengths
static constexpr float ESP32IMU_PAYLOAD_LEN[] = {
  sizeof(esp32imu_imu_msg_t),
  sizeof(esp32imu_rate_msg_t),
  sizeof(esp32imu_rgbled_msg_t),
  sizeof(esp32imu_config_msg_t)
};

// TODO: Manually indicate the largest msg payload
static constexpr size_t ESP32IMU_MAX_PAYLOAD_LEN = sizeof(esp32imu_imu_msg_t);

//=============================================================================
// generic message type
//=============================================================================

static constexpr uint8_t ESP32IMU_MAGIC = 0xA5;

PACKED_STRUCT(esp32imu_message_t) {
  uint8_t magic;
  uint8_t type;
  uint8_t payload[ESP32IMU_MAX_PAYLOAD_LEN];
  uint8_t crc;
};

static constexpr size_t ESP32IMU_MAX_MESSAGE_LEN = sizeof(esp32imu_message_t);

//=============================================================================
// utility functions
//=============================================================================

// source: http://www.nongnu.org/avr-libc/user-manual/group__util__crc.html#gab27eaaef6d7fd096bd7d57bf3f9ba083
inline uint8_t esp32imu_update_crc(uint8_t inCrc, uint8_t inData)
{
  uint8_t i;
  uint8_t data;

  data = inCrc ^ inData;

  for ( i = 0; i < 8; i++ )
  {
    if (( data & 0x80 ) != 0 )
    {
      data <<= 1;
      data ^= 0x07;
    }
    else
    {
      data <<= 1;
    }
  }
  return data;
}

inline void esp32imu_finalize_message(esp32imu_message_t *msg)
{
  msg->magic = ESP32IMU_MAGIC;

  uint8_t crc = 0;
  crc = esp32imu_update_crc(crc, msg->magic);
  crc = esp32imu_update_crc(crc, msg->type);
  for (size_t i=0; i<ESP32IMU_PAYLOAD_LEN[msg->type]; ++i)
  {
    crc = esp32imu_update_crc(crc, msg->payload[i]);
  }

  msg->crc = crc;
}

inline size_t esp32imu_send_to_buffer(uint8_t *dst, const esp32imu_message_t *src)
{
  size_t offset = 0;
  memcpy(dst + offset, &src->magic,  sizeof(src->magic)); offset += sizeof(src->magic);
  memcpy(dst + offset, &src->type,   sizeof(src->type));  offset += sizeof(src->type);
  memcpy(dst + offset, src->payload, ESP32IMU_PAYLOAD_LEN[src->type]); offset += ESP32IMU_PAYLOAD_LEN[src->type];
  memcpy(dst + offset, &src->crc,    sizeof(src->crc)); offset += sizeof(src->crc);
  return offset;
}

//=============================================================================
// IMU message
//=============================================================================

inline void esp32imu_imu_msg_pack(esp32imu_message_t *dst, const esp32imu_imu_msg_t *src)
{
  dst->type = ESP32IMU_MSG_IMU;
  size_t offset = 0;
  memcpy(dst->payload + offset, &src->t_us, sizeof(src->t_us)); offset += sizeof(src->t_us);
  memcpy(dst->payload + offset, &src->id, sizeof(src->id)); offset += sizeof(src->id);
  memcpy(dst->payload + offset, &src->accel_x, sizeof(src->accel_x)); offset += sizeof(src->accel_x);
  memcpy(dst->payload + offset, &src->accel_y, sizeof(src->accel_y)); offset += sizeof(src->accel_y);
  memcpy(dst->payload + offset, &src->accel_z, sizeof(src->accel_z)); offset += sizeof(src->accel_z);
  memcpy(dst->payload + offset, &src->gyro_x,  sizeof(src->gyro_x));  offset += sizeof(src->gyro_x);
  memcpy(dst->payload + offset, &src->gyro_y,  sizeof(src->gyro_y));  offset += sizeof(src->gyro_y);
  memcpy(dst->payload + offset, &src->gyro_z,  sizeof(src->gyro_z));  offset += sizeof(src->gyro_z);
  esp32imu_finalize_message(dst);
}

inline void esp32imu_imu_msg_unpack(esp32imu_imu_msg_t *dst, const esp32imu_message_t *src)
{
  size_t offset = 0;
  memcpy(&dst->t_us, src->payload + offset, sizeof(dst->t_us)); offset += sizeof(dst->t_us);
  memcpy(&dst->id, src->payload + offset, sizeof(dst->id)); offset += sizeof(dst->id);
  memcpy(&dst->accel_x, src->payload + offset, sizeof(dst->accel_x)); offset += sizeof(dst->accel_x);
  memcpy(&dst->accel_y, src->payload + offset, sizeof(dst->accel_y)); offset += sizeof(dst->accel_y);
  memcpy(&dst->accel_z, src->payload + offset, sizeof(dst->accel_z)); offset += sizeof(dst->accel_z);
  memcpy(&dst->gyro_x,  src->payload + offset, sizeof(dst->gyro_x));  offset += sizeof(dst->gyro_x);
  memcpy(&dst->gyro_y,  src->payload + offset, sizeof(dst->gyro_y));  offset += sizeof(dst->gyro_y);
  memcpy(&dst->gyro_z,  src->payload + offset, sizeof(dst->gyro_z));  offset += sizeof(dst->gyro_z);
}

inline size_t esp32imu_imu_msg_send_to_buffer(uint8_t *dst, const esp32imu_imu_msg_t *src)
{
  esp32imu_message_t msg;
  esp32imu_imu_msg_pack(&msg, src);
  return esp32imu_send_to_buffer(dst, &msg);
}

//=============================================================================
// Rate message
//=============================================================================

inline void esp32imu_rate_msg_pack(esp32imu_message_t *dst, const esp32imu_rate_msg_t *src)
{
  dst->type = ESP32IMU_MSG_RATE;
  size_t offset = 0;
  memcpy(dst->payload + offset, &src->frequency, sizeof(src->frequency)); offset += sizeof(src->frequency);
  esp32imu_finalize_message(dst);
}

inline void esp32imu_rate_msg_unpack(esp32imu_rate_msg_t *dst, const esp32imu_message_t *src)
{
  size_t offset = 0;
  memcpy(&dst->frequency, src->payload + offset, sizeof(dst->frequency)); offset += sizeof(dst->frequency);
}

inline size_t esp32imu_rate_msg_send_to_buffer(uint8_t *dst, const esp32imu_rate_msg_t *src)
{
  esp32imu_message_t msg;
  esp32imu_rate_msg_pack(&msg, src);
  return esp32imu_send_to_buffer(dst, &msg);
}

//=============================================================================
// RGBLed command message
//=============================================================================

inline void esp32imu_rgbled_msg_pack(esp32imu_message_t *dst, const esp32imu_rgbled_msg_t *src)
{
  dst->type = ESP32IMU_MSG_RGBLED;
  size_t offset = 0;
  memcpy(dst->payload + offset, &src->r, sizeof(src->r)); offset += sizeof(src->r);
  memcpy(dst->payload + offset, &src->g, sizeof(src->g)); offset += sizeof(src->g);
  memcpy(dst->payload + offset, &src->b, sizeof(src->b)); offset += sizeof(src->b);
  memcpy(dst->payload + offset, &src->brightness, sizeof(src->brightness)); offset += sizeof(src->brightness);
  esp32imu_finalize_message(dst);
}

inline void esp32imu_rgbled_msg_unpack(esp32imu_rgbled_msg_t *dst, const esp32imu_message_t *src)
{
  size_t offset = 0;
  memcpy(&dst->r, src->payload + offset, sizeof(dst->r)); offset += sizeof(dst->r);
  memcpy(&dst->g, src->payload + offset, sizeof(dst->g)); offset += sizeof(dst->g);
  memcpy(&dst->b, src->payload + offset, sizeof(dst->b)); offset += sizeof(dst->b);
  memcpy(&dst->brightness, src->payload + offset, sizeof(dst->brightness)); offset += sizeof(dst->brightness);
}

inline size_t esp32imu_rgbled_msg_send_to_buffer(uint8_t *dst, const esp32imu_rgbled_msg_t *src)
{
  esp32imu_message_t msg;
  esp32imu_rgbled_msg_pack(&msg, src);
  return esp32imu_send_to_buffer(dst, &msg);
}

//=============================================================================
// Config command message
//=============================================================================

inline void esp32imu_config_msg_pack(esp32imu_message_t *dst, const esp32imu_config_msg_t *src)
{
  dst->type = ESP32IMU_MSG_CONFIG;
  size_t offset = 0;
  memcpy(dst->payload + offset, &src->stream, sizeof(src->stream)); offset += sizeof(src->stream);
  memcpy(dst->payload + offset, &src->logging, sizeof(src->logging)); offset += sizeof(src->logging);
  memcpy(dst->payload + offset, &src->readlog, sizeof(src->readlog)); offset += sizeof(src->readlog);
  esp32imu_finalize_message(dst);
}

inline void esp32imu_config_msg_unpack(esp32imu_config_msg_t *dst, const esp32imu_message_t *src)
{
  size_t offset = 0;
  memcpy(&dst->stream, src->payload + offset, sizeof(dst->stream)); offset += sizeof(dst->stream);
  memcpy(&dst->logging, src->payload + offset, sizeof(dst->logging)); offset += sizeof(dst->logging);
  memcpy(&dst->readlog, src->payload + offset, sizeof(dst->readlog)); offset += sizeof(dst->readlog);
}

inline size_t esp32imu_config_msg_send_to_buffer(uint8_t *dst, const esp32imu_config_msg_t *src)
{
  esp32imu_message_t msg;
  esp32imu_config_msg_pack(&msg, src);
  return esp32imu_send_to_buffer(dst, &msg);
}

//==============================================================================
// parser
//==============================================================================

typedef enum
{
  ESP32IMU_PARSE_STATE_IDLE,
  ESP32IMU_PARSE_STATE_GOT_MAGIC,
  ESP32IMU_PARSE_STATE_GOT_TYPE,
  ESP32IMU_PARSE_STATE_GOT_PAYLOAD
} esp32imu_parse_state_t;

inline bool esp32imu_parse_byte(uint8_t byte, esp32imu_message_t *msg)
{
  static esp32imu_parse_state_t parse_state = ESP32IMU_PARSE_STATE_IDLE;
  static uint8_t crc_value = 0;
  static size_t payload_index = 0;
  static esp32imu_message_t msg_buffer;

  bool got_message = false;
  switch (parse_state)
  {
  case ESP32IMU_PARSE_STATE_IDLE:
    if (byte == ESP32IMU_MAGIC)
    {
      crc_value = 0;
      payload_index = 0;

      msg_buffer.magic = byte;
      crc_value = esp32imu_update_crc(crc_value, byte);

      parse_state = ESP32IMU_PARSE_STATE_GOT_MAGIC;
    }
    break;

  case ESP32IMU_PARSE_STATE_GOT_MAGIC:
    msg_buffer.type = byte;
    crc_value = esp32imu_update_crc(crc_value, byte);
    parse_state = ESP32IMU_PARSE_STATE_GOT_TYPE;
    break;

  case ESP32IMU_PARSE_STATE_GOT_TYPE:
    msg_buffer.payload[payload_index++] = byte;
    crc_value = esp32imu_update_crc(crc_value, byte);
    if (payload_index == ESP32IMU_PAYLOAD_LEN[msg_buffer.type])
    {
      parse_state = ESP32IMU_PARSE_STATE_GOT_PAYLOAD;
    }
    break;

  case ESP32IMU_PARSE_STATE_GOT_PAYLOAD:
    msg_buffer.crc = byte;
    if (msg_buffer.crc == crc_value)
    {
      got_message = true;
      memcpy(msg, &msg_buffer, sizeof(msg_buffer));
    }
    parse_state = ESP32IMU_PARSE_STATE_IDLE;
    break;
  }

  return got_message;
}
