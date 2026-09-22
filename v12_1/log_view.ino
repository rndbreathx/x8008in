/*******************************************************************************
 * log_view.ino v2 - TWO filtered log pages from one flash store
 *   page 43 "Alarms and Fault Logs"  -> FLOG_T_ALARM_ON / FLOG_T_ALARM_OFF
 *   page 41 "System and Event Logs"  -> FLOG_T_EVENT
 * (requires spi_flash_log.h/.ino with flog_readAt)
 *
 * WHY A SCAN PHASE: with filtering, screen rows no longer map 1:1 onto
 * record indices - the viewer must skip non-matching records. Doing that
 * search inside a touch handler would block for up to LOGVIEW_SCAN_MAX
 * flash reads, so it runs in the same paced service as rendering:
 *   SCAN   - one flog_readAt per loop() pass, collecting the indices of
 *            matching records past the scroll offset
 *   RENDER - one row written per pass from the collected indices
 * Neither phase ever stalls the loop; a full page appears in ~0.3-1 s
 * depending on how sparse the matching records are.
 *
 * DGUS: BOTH pages use the SAME row VPs (VP_LOG_LINE_BASE, step
 * VP_LOG_LINE_STEP) - only one page is visible at a time. Ten monospaced
 * text controls per page, plus the shared keys: TP_LOG_UP / TP_LOG_DOWN /
 * TP_LOG_HOME / TP_LOG_BACK on both pages; entry via TP_LOG_OPEN (43)
 * and TP_SYSLOG_OPEN (41) from the standby screen. Data auto-upload,
 * NO DGUS-side page jumps - firmware navigates.
 *
 * loop() integration unchanged: LOGVIEW_SERVICE() each pass.
 ******************************************************************************/

/* Whole module compiles ONLY when ENABLE_FLASH_LOG is 1 in the main
   sketch tab - the files can stay in the sketch folder permanently;
   the flag alone enables/disables the feature.                       */
#if ENABLE_FLASH_LOG


#define LOGVIEW_ROWS 10
#define LOGVIEW_CHARS 40
#define LOGVIEW_SCROLL_STEP 5
#define LOGVIEW_MAX_OFFSET 500     /* scroll depth, in MATCHING records     */
#define LOGVIEW_SCAN_MAX 800       /* raw records examined per scan at most */

static uint16_t logViewOffset = 0;
static int8_t logViewRow = -1;         /* RENDER: next row; -1 idle         */
static uint8_t logScanning = 0;        /* 1 = SCAN phase active             */
static uint32_t logScanIdx = 0;        /* raw index being examined          */
static uint16_t logScanSkipped = 0;    /* matching records skipped (offset) */
static uint8_t logScanFound = 0;       /* matches collected                 */
static uint8_t logEmptyRun = 0;        /* consecutive empty reads = history end */
static uint32_t logRowIdx[LOGVIEW_ROWS];
static uint8_t logRowValid[LOGVIEW_ROWS];

/* ---- code -> short name (PROGMEM) ---- */
const char flogN_HPAW[] PROGMEM = "HIGH PAW";
const char flogN_LPAW[] PROGMEM = "LOW PAW";
const char flogN_HBPM[] PROGMEM = "HIGH RATE";
const char flogN_LBPM[] PROGMEM = "LOW RATE";
const char flogN_HVT[] PROGMEM = "HIGH VT";
const char flogN_LVT[] PROGMEM = "LOW VT";
const char flogN_HMV[] PROGMEM = "HIGH MV";
const char flogN_LMV[] PROGMEM = "LOW MV";
const char flogN_HFI[] PROGMEM = "HIGH FIO2";
const char flogN_LFI[] PROGMEM = "LOW FIO2";
const char flogN_SEN[] PROGMEM = "SENSOR FAIL";
const char flogN_PR1[] PROGMEM = "PR1 FAIL";
const char flogN_O2D[] PROGMEM = "O2 DISCONN";
const char flogN_BAT[] PROGMEM = "LOW BATTERY";
const char flogN_PWR[] PROGMEM = "POWER FAIL";
const char flogN_APN[] PROGMEM = "APNEA";
const char flogN_PON[] PROGMEM = "POWER ON";
const char flogN_SBI[] PROGMEM = "STANDBY";
const char flogN_SBO[] PROGMEM = "VENT START";
const char flogN_MOD[] PROGMEM = "MODE CHANGE";
const char flogN_SET[] PROGMEM = "SETTING OK";
const char flogN_MUT[] PROGMEM = "MUTE";
const char flogN_NEW[] PROGMEM = "NEW PATIENT";
const char flogN_OCA[] PROGMEM = "O2 CAL";
const char flogN_NEB[] PROGMEM = "NEBULIZER";
const char flogN_UNK[] PROGMEM = "EVENT";

static const char *flog_codeName(uint8_t code) {
  switch (code) {
    case FLOG_A_HIGH_PAW: return flogN_HPAW;
    case FLOG_A_LOW_PAW: return flogN_LPAW;
    case FLOG_A_HIGH_BPM: return flogN_HBPM;
    case FLOG_A_LOW_BPM: return flogN_LBPM;
    case FLOG_A_HIGH_VT: return flogN_HVT;
    case FLOG_A_LOW_VT: return flogN_LVT;
    case FLOG_A_HIGH_MV: return flogN_HMV;
    case FLOG_A_LOW_MV: return flogN_LMV;
    case FLOG_A_HIGH_FIO2: return flogN_HFI;
    case FLOG_A_LOW_FIO2: return flogN_LFI;
    case FLOG_A_SENSOR_FAIL: return flogN_SEN;
    case FLOG_A_PR1_FAIL: return flogN_PR1;
    case FLOG_A_O2_DISCONNECT: return flogN_O2D;
    case FLOG_A_LOW_BATTERY: return flogN_BAT;
    case FLOG_A_POWER_FAIL: return flogN_PWR;
    case FLOG_A_APNEA: return flogN_APN;
    case FLOG_E_POWER_ON: return flogN_PON;
    case FLOG_E_STANDBY_ENTER: return flogN_SBI;
    case FLOG_E_STANDBY_EXIT: return flogN_SBO;
    case FLOG_E_MODE_CHANGE: return flogN_MOD;
    case FLOG_E_SETTING_CONFIRM: return flogN_SET;
    case FLOG_E_MUTE: return flogN_MUT;
    case FLOG_E_NEW_PATIENT: return flogN_NEW;
    case FLOG_E_O2_CAL: return flogN_OCA;
    case FLOG_E_NEBULIZER: return flogN_NEB;
    default: return flogN_UNK;
  }
}

/* which record types belong on the currently displayed page */
static uint8_t logview_match(uint8_t type) {
  if (HMI_Page == PAGE_LOG_VIEW) {
    return (type == FLOG_T_ALARM_ON) || (type == FLOG_T_ALARM_OFF);
  }
  return (type == FLOG_T_EVENT);
}

/* ---- API (called from serialread_data cases) ---- */
void logview_show(uint16_t offset) {
  logViewOffset = offset;
  logScanIdx = 0;
  logScanSkipped = 0;
  logScanFound = 0;
  logEmptyRun = 0;
  for (uint8_t i = 0; i < LOGVIEW_ROWS; i++) { logRowValid[i] = 0; }
  logScanning = 1;
  logViewRow = -1;
}

void logview_scroll(int8_t dir) {
  int32_t o = (int32_t)logViewOffset + (int32_t)dir * LOGVIEW_SCROLL_STEP;
  if (o < 0) { o = 0; }
  if (o > LOGVIEW_MAX_OFFSET) { o = LOGVIEW_MAX_OFFSET; }
  logview_show((uint16_t)o);
}

/* ---- paced service: call every loop() pass ---- */
void logview_service(void) {

  uint8_t onLogPage = (HMI_Page == PAGE_LOG_VIEW) || (HMI_Page == PAGE_SYSLOG_VIEW);
  if (!onLogPage) {
    logScanning = 0;
    logViewRow = -1;
    return;
  }

  /* AUTO-EXIT: ventilation started while browsing -> monitoring screen */
  if (standbybutton_count == 2) {
    HMI_Page = PAGE_MAIN;
    dwin_page_Set(PAGE_MAIN);
    logScanning = 0;
    logViewRow = -1;
    return;
  }

  /* ---- SCAN phase: one flash read per pass ---- */
  if (logScanning) {
    FlogRecord r;
    uint8_t st = flog_readAt(logScanIdx, &r);
    if (st == 2) { return; }                 /* flash busy - retry     */

    if (st == 1) {
      logEmptyRun = 0;
      if (logview_match(r.type)) {
        if (logScanSkipped < logViewOffset) {
          logScanSkipped++;
        } else if (logScanFound < LOGVIEW_ROWS) {
          logRowIdx[logScanFound] = logScanIdx;
          logRowValid[logScanFound] = 1;
          logScanFound++;
        }
      }
    } else {
      /* st == 0: empty or invalid slot. A run of empties = end of the
         stored history - stop scanning early.                          */
      logEmptyRun++;
    }
    logScanIdx++;

    if ((logScanFound >= LOGVIEW_ROWS)
        || (logScanIdx >= LOGVIEW_SCAN_MAX)
        || (logEmptyRun >= 8)) {
      logScanning = 0;
      logViewRow = 0;                        /* -> RENDER phase        */
    }
    return;                                  /* one read per pass      */
  }

  /* ---- RENDER phase: one row per pass ---- */
  if (logViewRow < 0 || logViewRow >= LOGVIEW_ROWS) { return; }

  char line[LOGVIEW_CHARS + 1];
  uint8_t rowOk = 0;

  if (logRowValid[logViewRow]) {
    FlogRecord r;
    uint8_t st = flog_readAt(logRowIdx[logViewRow], &r);
    if (st == 2) { return; }                 /* busy - retry this row  */
    if (st == 1) {
      char name[14];
      strncpy_P(name, flog_codeName(r.code), sizeof(name) - 1);
      name[sizeof(name) - 1] = 0;
      if (r.type == FLOG_T_ALARM_ON) {
        snprintf(line, sizeof(line), "%02u/%02u %02u:%02u:%02u %-12s ON",
                 r.dd, r.mo, r.hh, r.mi, r.ss, name);
      } else if (r.type == FLOG_T_ALARM_OFF) {
        snprintf(line, sizeof(line), "%02u/%02u %02u:%02u:%02u %-12s OFF",
                 r.dd, r.mo, r.hh, r.mi, r.ss, name);
      } else {
        snprintf(line, sizeof(line), "%02u/%02u %02u:%02u:%02u %-12s %u",
                 r.dd, r.mo, r.hh, r.mi, r.ss, name, r.value);
      }
      rowOk = 1;
    }
  }

  if (!rowOk) {
    memset(line, ' ', LOGVIEW_CHARS);        /* blank stale rows       */
    line[LOGVIEW_CHARS] = 0;
  }

  display_write_text(VP_LOG_LINE_BASE + (logViewRow * VP_LOG_LINE_STEP), line);
  logViewRow++;
}

#endif /* ENABLE_FLASH_LOG */
