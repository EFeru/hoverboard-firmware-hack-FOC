/**
  * This file is part of the hoverboard-sideboard-hack project (sensor variant).
  * Based on hoverboard-sideboard-hack-GD by Emanuel FERU (GPLv3).
  *
  * SBSP v1 - SideBoard Sensor Protocol.
  * The sideboard becomes a standalone UART sensor node:
  *   - streams roll/pitch (deg x 100) at 1..200 Hz, decimated from the 200 Hz DMP,
  *   - takes LED commands from the host (ESP32),
  *   - reports the two optical inputs,
  *   - keeps its configuration in the last 1 KB flash page (linker: FLASH = 31K).
  *
  * Wire format:  A5 5A | type | len | payload[len] | CRC-8 (poly 0x07, init 0x00)
  */

#ifndef SENSOR_PROTO_H
#define SENSOR_PROTO_H

#ifdef SERIAL_SENSOR

#include <stdint.h>

/* ------------------------------ wire protocol ------------------------------ */
#define SBSP_SOF0               0xA5U
#define SBSP_SOF1               0x5AU
#define SBSP_MAX_PAYLOAD        24U
#define SBSP_MAX_FRAME          (SBSP_MAX_PAYLOAD + 5U)

/* board -> host */
#define SBSP_MSG_DATA           0x01U   /* len 10 */
#define SBSP_MSG_DATA_EXT       0x02U   /* len 22 */
#define SBSP_MSG_INFO           0xE0U   /* len  7 */
#define SBSP_MSG_ACK            0xE1U   /* len  2 */

/* host -> board */
#define SBSP_MSG_SET_RATE       0x10U   /* len 3: rate_hz(u16), save(u8) */
#define SBSP_MSG_SET_MODE       0x11U   /* len 1: mode bits */
#define SBSP_MSG_LED            0x12U   /* len 2: mask(u8), blink(u8) */
#define SBSP_MSG_SET_ID         0x13U   /* len 1: id(u8) */
#define SBSP_MSG_SAVE           0x14U   /* len 0 */
#define SBSP_MSG_GET_INFO       0x15U   /* len 0 */

/* ACK results */
#define SBSP_OK                 0U
#define SBSP_ERR_UNKNOWN_CMD    2U
#define SBSP_ERR_BAD_VALUE      3U
#define SBSP_ERR_FLASH          4U

/* mode bits (SET_MODE / INFO) */
#define SBSP_MODE_EXT           0x01U   /* stream DATA_EXT instead of DATA */
#define SBSP_MODE_LED_SLAVE     0x02U   /* host owns the LEDs (default: local diagnostic) */
#define SBSP_MODE_HEARTBEAT     0x04U   /* repeat the last sample every 500 ms at <=5 Hz */

/* status bits in DATA / DATA_EXT */
#define SBSP_ST_SENSOR1         0x01U
#define SBSP_ST_SENSOR2         0x02U
#define SBSP_ST_MPU_OK          0x04U
#define SBSP_ST_SETTINGS        0x08U   /* configuration was loaded from flash */

/* defaults (used when the settings page is empty/corrupt) */
#define SBSP_DEFAULT_RATE_HZ    200U
#define SBSP_DEFAULT_ID         0U
#define SBSP_DEFAULT_MODE       0U

/* ------------------------------ settings page ------------------------------ */
/* Last 1 KB page of the 32 KB flash; the linker script limits the code to 31K. */
#define SBSP_SETTINGS_ADDR      0x08007C00U
#define SBSP_SETTINGS_MAGIC     0x31534253U     /* "SBS1" */
#define SBSP_SETTINGS_VERSION   1U
#define SBSP_CRC_LEN            (sizeof(sbsp_settings_t) - 4U)

typedef struct {
    uint32_t magic;
    uint8_t  version;
    uint8_t  id;
    uint8_t  mode;
    uint8_t  reserved0;
    uint16_t rate_hz;
    uint8_t  led_mask;
    uint8_t  reserved1;
    uint16_t crc16;                             /* CRC-16/CCITT-FALSE over the first SBSP_CRC_LEN bytes */
    uint16_t reserved2;                         /* padding: the page is written as 4 words */
} sbsp_settings_t;

/* ------------------------------ API ------------------------------ */
void     sensor_proto_init(void);               /* call at the end of input_init() */
void     sensor_proto_poll(void);               /* call from the main loop */
void     sensor_proto_rx_bytes(const uint8_t *data, uint32_t len);  /* call from handle_usart() */
void     sensor_proto_leds(void);               /* call from handle_leds() */
uint8_t *sensor_proto_tx_buffer(void);          /* for usart_Tx_DMA_config() */
uint16_t sensor_proto_tx_buffer_size(void);

/* Set in Src/mpu6050.c on every fresh DMP sample (see mpu_get_data()). */
extern volatile uint8_t mpu_new_sample;

/* Implemented in Src/util.c: optical inputs + MPU status (SBSP_ST_* bits). */
uint8_t  sensor_proto_status_bits(void);

#endif /* SERIAL_SENSOR */
#endif /* SENSOR_PROTO_H */
