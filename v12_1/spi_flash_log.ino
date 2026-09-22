/*******************************************************************************
 * spi_flash_log.ino - implementation (requires spi_flash_log.h)
 *
 * DESIGN RULES:
 *   1. NEVER WAIT ON THE FLASH. flog_event() only touches a RAM queue -
 *      callable from any phase, ~2 us. flog_service() issues at most ONE
 *      flash operation per loop() pass and polls the busy bit instead of
 *      blocking. Sector erase (up to 400 ms chip-internal) costs the MCU
 *      nothing - the chip is simply "busy" and the writer waits it out
 *      across passes.
 *   2. FLASH TOUCHED ONLY WHEN GATED: during expiration after 300 ms
 *      (past the PEEP-transient window), or any time in standby.
 *   3. APPEND-ONLY CIRCULAR LOG with per-record seq + CRC8. Power loss
 *      mid-write -> one bad-CRC record, skipped on read; never a
 *      corrupted log. Erase-ahead: a sector is erased in the same gated,
 *      non-blocking way just before its first record.
 *
 * INTEGRATION:
 *   setup():  if (!flog_init()) { / * flash absent - log to serial only * / }
 *             flog_event(FLOG_T_EVENT, FLOG_E_POWER_ON, 0);
 *   loop():   flog_service();          // unconditional, self-gating
 *
 *   Alarm edges (in/near alarm()): one static per alarm -
 *     static uint8_t prevHighPaw = 0;
 *     flog_alarmEdge(FLOG_A_HIGH_PAW, hpstate == 1, &prevHighPaw);
 *
 *   Events at the action sites, e.g. in serialread_data:
 *     case TOUCH_MODE_PCV: ... flog_event(FLOG_T_EVENT, FLOG_E_MODE_CHANGE, 5);
 *     standby handler:     ... flog_event(FLOG_T_EVENT, FLOG_E_STANDBY_ENTER, 0);
 *
 * Timestamps use the RTC globals (years..minutes) your DWIN RTC readback
 * maintains; seconds fall back to a millis-derived counter for ordering
 * within the same minute (records also carry a strict sequence number).
 ******************************************************************************/

/* Whole module compiles ONLY when ENABLE_FLASH_LOG is 1 in the main
   sketch tab - the files can stay in the sketch folder permanently;
   the flag alone enables/disables the feature.                       */
#if ENABLE_FLASH_LOG


#include "spi_flash_log.h"

/* ---- W25Qxx opcodes ---- */
#define W25_WREN 0x06
#define W25_PP 0x02
#define W25_READ 0x03
#define W25_SE4K 0x20
#define W25_RDSR 0x05
#define W25_JEDEC 0x9F
#define W25_WIP_BIT 0x01

/* ---- state ---- */
static FlogRecord flogQ[FLOG_QUEUE_LEN];
static volatile uint8_t flogQHead = 0, flogQTail = 0;
static uint16_t flogDropped = 0;

static uint32_t flogWriteSlot = 0;    /* next record index in log region  */
static uint16_t flogSeq = 0;
static uint8_t flogReady = 0;
static uint8_t flogEraseArmed = 0;    /* erase issued for the next sector */

/* ---------- SPI primitives ---------- */
static void w25_select(void) {
  SPI.beginTransaction(SPISettings(FLOG_SPI_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(FLOG_CS_PIN, LOW);
}
static void w25_deselect(void) {
  digitalWrite(FLOG_CS_PIN, HIGH);
  SPI.endTransaction();
}
static uint8_t w25_busy(void) {
  w25_select();
  SPI.transfer(W25_RDSR);
  uint8_t s = SPI.transfer(0);
  w25_deselect();
  return (s & W25_WIP_BIT);
}
static void w25_wren(void) {
  w25_select();
  SPI.transfer(W25_WREN);
  w25_deselect();
}
static void w25_addr(uint32_t a) {
  SPI.transfer((a >> 16) & 0xFF);
  SPI.transfer((a >> 8) & 0xFF);
  SPI.transfer(a & 0xFF);
}
static void w25_read(uint32_t addr, uint8_t *buf, uint16_t n) {
  w25_select();
  SPI.transfer(W25_READ);
  w25_addr(addr);
  for (uint16_t i = 0; i < n; i++) { buf[i] = SPI.transfer(0); }
  w25_deselect();
}
static void w25_program(uint32_t addr, uint8_t *buf, uint16_t n) {
  w25_wren();
  w25_select();
  SPI.transfer(W25_PP);
  w25_addr(addr);
  for (uint16_t i = 0; i < n; i++) { SPI.transfer(buf[i]); }
  w25_deselect();                 /* chip programs internally; poll WIP  */
}
static void w25_erase4k(uint32_t addr) {
  w25_wren();
  w25_select();
  SPI.transfer(W25_SE4K);
  w25_addr(addr);
  w25_deselect();                 /* chip erases internally; poll WIP    */
}

/* ---------- helpers ---------- */
static uint32_t flog_slotAddr(uint32_t slot) {
  uint32_t s = slot % ((uint32_t)FLOG_SECTORS * FLOG_RECS_PER_SECTOR);
  return ((uint32_t)FLOG_FIRST_SECTOR * 4096UL) + (s * FLOG_REC_SIZE);
}

static uint8_t flog_crc8(uint8_t *d, uint8_t n) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static void flog_pack(FlogRecord *r, uint8_t *b) {
  b[0] = 0xA5;
  b[1] = r->type;
  b[2] = r->code;
  b[3] = (r->value >> 8) & 0xFF;
  b[4] = r->value & 0xFF;
  b[5] = r->yy; b[6] = r->mo; b[7] = r->dd;
  b[8] = r->hh; b[9] = r->mi; b[10] = r->ss;
  b[11] = (r->seq >> 8) & 0xFF;
  b[12] = r->seq & 0xFF;
  b[13] = flog_crc8(&b[1], 12);
  b[14] = 0xFF; b[15] = 0xFF;
}

static uint8_t flog_unpack(uint8_t *b, FlogRecord *r) {
  if (b[0] != 0xA5) { return 0; }
  if (flog_crc8(&b[1], 12) != b[13]) { return 0; }
  r->type = b[1]; r->code = b[2];
  r->value = ((uint16_t)b[3] << 8) | b[4];
  r->yy = b[5]; r->mo = b[6]; r->dd = b[7];
  r->hh = b[8]; r->mi = b[9]; r->ss = b[10];
  r->seq = ((uint16_t)b[11] << 8) | b[12];
  return 1;
}

/* ---------- init: verify chip, find write head ---------- */
uint8_t flog_init(void) {
  pinMode(FLOG_CS_PIN, OUTPUT);
  digitalWrite(FLOG_CS_PIN, HIGH);
  SPI.begin();

  w25_select();
  SPI.transfer(W25_JEDEC);
  uint8_t mf = SPI.transfer(0);
  SPI.transfer(0); SPI.transfer(0);
  w25_deselect();
  if (mf == 0x00 || mf == 0xFF) { return 0; }   /* no chip responding    */

  /* find head: newest sector by first-record seq, then first empty slot */
  uint8_t b[FLOG_REC_SIZE];
  FlogRecord r;
  int32_t bestSector = -1;
  uint16_t bestSeq = 0;
  uint8_t any = 0;

  for (uint16_t s = 0; s < FLOG_SECTORS; s++) {
    w25_read(flog_slotAddr((uint32_t)s * FLOG_RECS_PER_SECTOR), b, FLOG_REC_SIZE);
    if (flog_unpack(b, &r)) {
      /* seq comparison robust to wrap: signed 16-bit distance */
      if (!any || (int16_t)(r.seq - bestSeq) > 0) {
        any = 1;
        bestSeq = r.seq;
        bestSector = s;
      }
    }
  }

  if (!any) {
    flogWriteSlot = 0;
    flogSeq = 1;
  } else {
    uint32_t base = (uint32_t)bestSector * FLOG_RECS_PER_SECTOR;
    uint32_t i = 0;
    for (i = 0; i < FLOG_RECS_PER_SECTOR; i++) {
      w25_read(flog_slotAddr(base + i), b, FLOG_REC_SIZE);
      if (b[0] == 0xFF) { break; }
      if (flog_unpack(b, &r)) { flogSeq = r.seq + 1; }
    }
    flogWriteSlot = base + i;     /* may be the next sector's slot 0     */
  }

  flogReady = 1;
  return 1;
}

/* ---------- queue an event (any phase, RAM only) ---------- */
/* ===== TIMESTAMP ADAPTER ====================================================
   Map these to YOUR firmware's RTC globals (names differ per version).
   The compiler confirmed years/months/dayss exist; find your hour and
   minute globals by searching the main .ino for VP_RTC_HOUR - the
   variable passed to display_write_variable(VP_RTC_HOUR, X) is it.

   BETTER OPTION (RTClib is in this build): if you have a global RTC
   object (e.g. RTC_DS3231 rtc;), comment the macro block out, set
   FLOG_USE_RTCLIB 1 and set FLOG_RTC_OBJ - you get real seconds too. */
#define FLOG_USE_RTCLIB 0
#define FLOG_RTC_OBJ rtc

#if !FLOG_USE_RTCLIB
/* names taken from the SYS_RTC_RESPONSE_ADDR handler in serialread_data:
   years/months/dayss/hh1/mm1/ss1 <- DWIN built-in RTC readback.
   NOTE: match the extern types to the actual declarations in the main
   .ino (change int -> uint8_t etc. if the linker complains).            */
extern uint8_t years;
extern uint8_t months;
extern uint8_t dayss;
extern uint8_t hh1;
extern uint8_t mm1;
extern uint8_t ss1;
#define FLOG_TS_YY ((uint8_t)years)
#define FLOG_TS_MO ((uint8_t)months)
#define FLOG_TS_DD ((uint8_t)dayss)
#define FLOG_TS_HH ((uint8_t)hh1)
#define FLOG_TS_MI ((uint8_t)mm1)
#define FLOG_TS_SS ((uint8_t)ss1)   /* DWIN provides real seconds */
#endif
/* ========================================================================== */

void flog_event(uint8_t type, uint8_t code, uint16_t value) {
  uint8_t next = (flogQHead + 1) % FLOG_QUEUE_LEN;
  if (next == flogQTail) {        /* queue full: drop, count it          */
    flogDropped++;
    return;
  }
  FlogRecord *r = &flogQ[flogQHead];
  r->type = type;
  r->code = code;
  r->value = value;
#if FLOG_USE_RTCLIB
  DateTime _n = FLOG_RTC_OBJ.now();
  r->yy = (uint8_t)(_n.year() % 100); r->mo = _n.month(); r->dd = _n.day();
  r->hh = _n.hour(); r->mi = _n.minute(); r->ss = _n.second();
#else
  r->yy = FLOG_TS_YY; r->mo = FLOG_TS_MO; r->dd = FLOG_TS_DD;
  r->hh = FLOG_TS_HH; r->mi = FLOG_TS_MI;
  r->ss = FLOG_TS_SS;               /* real seconds from the DWIN RTC   */
#endif
  r->seq = 0;                     /* assigned at write time              */
  flogQHead = next;
}

void flog_alarmEdge(uint8_t code, uint8_t active, uint8_t *prevState) {
  if (active && !(*prevState)) {
    flog_event(FLOG_T_ALARM_ON, code, 0);
  } else if (!active && (*prevState)) {
    flog_event(FLOG_T_ALARM_OFF, code, 0);
  }
  *prevState = active ? 1 : 0;
}

uint16_t flog_droppedCount(void) { return flogDropped; }

/* ---------- gated, non-blocking writer: call every loop() pass ---------- */
void flog_service(void) {

  if (!flogReady) { return; }
  if (flogQHead == flogQTail) { return; }        /* nothing pending      */

  /* write gate: expiration past the PEEP-transient window, or standby */
  uint8_t gated = (standbybutton_count == 1)
               || ((mainrelaypinstate == LOW) && ((TCMillis - TPMillis) > 300));
  if (!gated) { return; }

  if (w25_busy()) { return; }     /* previous program/erase still running */

  /* erase-ahead: first slot of a sector -> erase it before writing      */
  if ((flogWriteSlot % FLOG_RECS_PER_SECTOR) == 0 && !flogEraseArmed) {
    w25_erase4k(flog_slotAddr(flogWriteSlot));
    flogEraseArmed = 1;
    return;                       /* one flash op per pass               */
  }

  /* program one record */
  FlogRecord *r = &flogQ[flogQTail];
  r->seq = flogSeq;
  uint8_t b[FLOG_REC_SIZE];
  flog_pack(r, b);
  w25_program(flog_slotAddr(flogWriteSlot), b, FLOG_REC_SIZE);

  flogSeq++;
  flogWriteSlot = (flogWriteSlot + 1)
                % ((uint32_t)FLOG_SECTORS * FLOG_RECS_PER_SECTOR);
  flogEraseArmed = 0;
  flogQTail = (flogQTail + 1) % FLOG_QUEUE_LEN;
}

/* ---------- random access for the log-view page ---------- */
uint8_t flog_readAt(uint32_t idxFromNewest, FlogRecord *out) {
  if (!flogReady) { return 0; }
  if (w25_busy()) { return 2; }   /* reads are invalid during erase/program */
  uint32_t total = (uint32_t)FLOG_SECTORS * FLOG_RECS_PER_SECTOR;
  if (idxFromNewest >= total) { return 0; }
  uint32_t slot = (flogWriteSlot + total - 1UL - idxFromNewest) % total;
  uint8_t b[FLOG_REC_SIZE];
  w25_read(flog_slotAddr(slot), b, FLOG_REC_SIZE);
  if (b[0] == 0xFF) { return 0; }
  return flog_unpack(b, out) ? 1 : 0;
}

/* ---------- read newest-first for the alarm-log HMI page ---------- */
uint8_t flog_readLast(FlogRecord *out, uint8_t maxN) {
  if (!flogReady) { return 0; }
  uint8_t b[FLOG_REC_SIZE];
  uint8_t n = 0;
  uint32_t total = (uint32_t)FLOG_SECTORS * FLOG_RECS_PER_SECTOR;
  uint32_t slot = flogWriteSlot;

  for (uint32_t i = 0; i < total && n < maxN; i++) {
    slot = (slot == 0) ? (total - 1) : (slot - 1);
    w25_read(flog_slotAddr(slot), b, FLOG_REC_SIZE);
    if (b[0] == 0xFF) { break; }              /* reached the empty region */
    if (flog_unpack(b, &out[n])) { n++; }     /* bad CRC: skip silently   */
  }
  return n;
}

#endif /* ENABLE_FLASH_LOG */
