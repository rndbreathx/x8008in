/*******************************************************************************
 * spi_flash_log.h - persistent event & alarm log on SPI NOR flash (W25Qxx)
 *
 * Include this from the sketch; implementation in spi_flash_log.ino.
 * (Struct in a .h - Arduino auto-prototype rule, same as loop_displays.h)
 ******************************************************************************/

#ifndef SPI_FLASH_LOG_H
#define SPI_FLASH_LOG_H

#include <Arduino.h>
#include <SPI.h>

/* ---- hardware ---- */
#define FLOG_CS_PIN 49            /* chip-select for the W25Qxx - set to your PCB */
#define FLOG_SPI_HZ 4000000UL

/* ---- log region (circular). 64 x 4KB = 16384 records of 16 bytes ---- */
#define FLOG_FIRST_SECTOR 0       /* first 4KB sector used by the log       */
#define FLOG_SECTORS 64
#define FLOG_REC_SIZE 16
#define FLOG_RECS_PER_SECTOR (4096 / FLOG_REC_SIZE)

/* ---- RAM queue (events captured any time; flash touched only when gated) */
#define FLOG_QUEUE_LEN 16

/* ---- record type ---- */
#define FLOG_T_ALARM_ON 0x01
#define FLOG_T_ALARM_OFF 0x02
#define FLOG_T_EVENT 0x03

/* ---- alarm codes (match your alarm() conditions) ---- */
#define FLOG_A_HIGH_PAW 0x01
#define FLOG_A_LOW_PAW 0x02
#define FLOG_A_HIGH_BPM 0x03
#define FLOG_A_LOW_BPM 0x04
#define FLOG_A_HIGH_VT 0x05
#define FLOG_A_LOW_VT 0x06
#define FLOG_A_HIGH_MV 0x07
#define FLOG_A_LOW_MV 0x08
#define FLOG_A_HIGH_FIO2 0x09
#define FLOG_A_LOW_FIO2 0x0A
#define FLOG_A_SENSOR_FAIL 0x0B
#define FLOG_A_PR1_FAIL 0x0C
#define FLOG_A_O2_DISCONNECT 0x0D
#define FLOG_A_LOW_BATTERY 0x0E
#define FLOG_A_POWER_FAIL 0x0F
#define FLOG_A_APNEA 0x10

/* ---- event codes ---- */
#define FLOG_E_POWER_ON 0x40
#define FLOG_E_STANDBY_ENTER 0x41
#define FLOG_E_STANDBY_EXIT 0x42
#define FLOG_E_MODE_CHANGE 0x43      /* value = setmode                    */
#define FLOG_E_SETTING_CONFIRM 0x44  /* value = settingbutton_count        */
#define FLOG_E_MUTE 0x45
#define FLOG_E_NEW_PATIENT 0x46
#define FLOG_E_O2_CAL 0x47
#define FLOG_E_NEBULIZER 0x48        /* value = 1 on / 0 off               */

/* ---- one record: 16 bytes ----
   [0] status  0xA5 = valid (0xFF = empty slot)
   [1] type    FLOG_T_*
   [2] code    FLOG_A_* / FLOG_E_*
   [3..4] value (big-endian)
   [5..10] timestamp: yy mm dd hh mi ss
   [11..12] sequence number (big-endian, wraps)
   [13] crc8 of bytes 1..12
   [14..15] reserved 0xFF                                                */
typedef struct {
  uint8_t type;
  uint8_t code;
  uint16_t value;
  uint8_t yy, mo, dd, hh, mi, ss;
  uint16_t seq;
} FlogRecord;

/* ---- API ---- */
uint8_t flog_init(void);                       /* call in setup(); 1 = flash OK */
void flog_event(uint8_t type, uint8_t code, uint16_t value);  /* queue, any time */
void flog_service(void);                       /* call every loop() pass        */
uint8_t flog_readLast(FlogRecord *out, uint8_t maxN);  /* newest-first          */
/* random access, index 0 = newest. Returns 1 = ok, 0 = empty/invalid,
   2 = flash busy (erase/program in progress) - retry next pass.        */
uint8_t flog_readAt(uint32_t idxFromNewest, FlogRecord *out);
uint16_t flog_droppedCount(void);              /* queue overflows (diagnostics) */

/* edge-logging helper for alarms: call every pass per alarm; logs ON/OFF
   transitions only */
void flog_alarmEdge(uint8_t code, uint8_t active, uint8_t *prevState);

#endif /* SPI_FLASH_LOG_H */
