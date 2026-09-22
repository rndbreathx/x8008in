/*******************************************************************************
 * selftest.ino - service self-test suite (Manual / Auto calibration pages)
 *
 * 8 tests, each a paced state machine step - loop() never blocks:
 *   1 TURBINE    spin at test duty -> pressure must develop
 *   2 SENSOR     all pressure sensors at rest within zero-offset band
 *   3 PNEUMATIC  pressurize, stop, measure decay -> leak check
 *   4 FIO2       O2 cell plausibility at ambient (~21%)
 *   5 PRESSURE   PR1 vs PR3 cross-agreement while pressurized
 *   6 O2SRC      supply presence input
 *   7 BATTERY    battery measurement within operating range
 *   8 VALVE      PEEP valve authority: closed-vs-open pressure delta
 *
 * REPORT: one flash-log record per test (FLOG_T_EVENT, code
 * FLOG_E_TEST_BASE+id, value = pass flag + measured value packed) plus
 * a summary record - persistent, reviewable on the System Logs page.
 * Result lines also render live on the Testing Result page.
 *
 * SAFETY: every entry point is standby-gated (standbybutton_count == 1).
 * Tests drive the turbine and valve - NEVER run connected to a patient.
 * The service aborts (actuators safed) if standby is left mid-test.
 *
 * ALL THRESHOLDS ARE BENCH-CONFIRM: calibrate against known-good units
 * before treating FAIL as truth. Marked [BC] below.
 *
 * ---- ADD TO dwin_addresses.h (VERIFY page numbers + free ranges) ----
 *   #define PAGE_CAL_SELECT        44      // Manual/Auto selection    TODO
 *   #define PAGE_SELF_TEST         45      // Testing Result page      TODO
 *   #define TP_CAL_MANUAL          0xA4A0
 *   #define TP_CAL_AUTO            0xA4A1
 *   #define TP_TEST_TURBINE        0xA4A2
 *   #define TP_TEST_SENSOR         0xA4A3
 *   #define TP_TEST_PNEUMATIC      0xA4A4
 *   #define TP_TEST_FIO2           0xA4A5
 *   #define TP_TEST_PRESSURE       0xA4A6
 *   #define TP_TEST_O2SRC          0xA4A7
 *   #define TP_TEST_BATTERY        0xA4A8
 *   #define TP_TEST_VALVE          0xA4A9
 *   #define VP_TEST_LINE_BASE      0xD560  // 8 result rows, step 0x20
 *   #define VP_TEST_LINE_STEP      0x0020  // (verify 0xD560-0xD65F free)
 *
 * ---- ADD TO spi_flash_log.h event codes ----
 *   #define FLOG_E_TEST_BASE 0x30          // 0x30..0x37 = tests 1..8
 *   #define FLOG_E_TEST_SUMMARY 0x38       // value = pass bitmask
 *
 * ---- serialread_data cases (standby-gated like the log pages) ----
 *   case TP_CAL_MANUAL:  if (HMI_Page == PAGE_CAL_SELECT) { selftest_openManual(); } break;
 *   case TP_CAL_AUTO:    if (HMI_Page == PAGE_CAL_SELECT) { selftest_startAuto(); } break;
 *   case TP_TEST_TURBINE:   selftest_start(1); break;
 *   case TP_TEST_SENSOR:    selftest_start(2); break;
 *   case TP_TEST_PNEUMATIC: selftest_start(3); break;
 *   case TP_TEST_FIO2:      selftest_start(4); break;
 *   case TP_TEST_PRESSURE:  selftest_start(5); break;
 *   case TP_TEST_O2SRC:     selftest_start(6); break;
 *   case TP_TEST_BATTERY:   selftest_start(7); break;
 *   case TP_TEST_VALVE:     selftest_start(8); break;
 *
 * ---- loop() ----   selftest_service();   (self-gating, one step/pass)
 ******************************************************************************/

/* ===================== [BC] BENCH-CONFIRM THRESHOLDS ===================== */
#define ST_TURBINE_DUTY 120        /* test spin duty                         */
#define ST_TURBINE_MIN_P 8.0f      /* cmH2O that must develop [BC]           */
#define ST_TURBINE_TIMEOUT 4000UL  /* ms                                     */
#define ST_SENSOR_ZERO_BAND 3.0f   /* cmH2O rest tolerance [BC]              */
#define ST_PNEU_TARGET_P 20.0f     /* pressurize to [BC]                     */
#define ST_PNEU_HOLD_MS 5000UL     /* decay observation window               */
#define ST_PNEU_MAX_DROP 5.0f      /* cmH2O allowed drop in window [BC]      */
#define ST_FIO2_AMBIENT_LO 17      /* % [BC]                                 */
#define ST_FIO2_AMBIENT_HI 25
#define ST_PRESS_AGREE 3.0f        /* PR1 vs PR3 max disagreement [BC]       */
#define ST_BATT_MIN_PCT 10         /* [BC] - uses your battery percent var   */
#define ST_VALVE_MIN_DELTA 6.0f    /* closed-vs-open pressure delta [BC]     */
#define ST_STEP_SETTLE_MS 1500UL

#define ST_IDLE 0
#define ST_RUNNING 1

uint8_t stActive = 0;              /* 0 idle, else test id 1..8              */
uint8_t stAuto = 0;                /* auto mode: chain 1..8                  */
uint8_t stPhase = 0;
uint32_t stT0 = 0;
float stM1 = 0, stM2 = 0;
uint8_t stResults = 0;             /* pass bitmask, bit(id-1)                */
uint8_t stDone = 0;                /* completion bitmask                     */

/* battery percent source - map to your actual variable if named otherwise */
extern int batteryPercent;         /* TODO-VERIFY name                       */
extern int o2AvailabilityStatus;
           
static void st_safeActuators(void) {
  analogWrite(turbinePin, 0);
  analogWrite(peepPumpPin, 0);
}

static void st_report(uint8_t id, uint8_t pass, int measured) {
  char line[41];
  const char *names[9] = { "", "TURBINE", "SENSOR", "PNEUMATIC", "FIO2",
                           "PRESSURE", "O2 SOURCE", "BATTERY", "VALVE" };
  snprintf(line, sizeof(line), "%-10s %-4s  %d",
           names[id], pass ? "PASS" : "FAIL", measured);
  display_write_text(VP_TEST_LINE_BASE + ((id - 1) * VP_TEST_LINE_STEP), line);

  if (pass) { stResults |= (uint8_t)(1 << (id - 1)); }
  stDone |= (uint8_t)(1 << (id - 1));
  /* flash report: value packs pass flag (bit 15) + |measured| (14..0)  */
  uint16_t v = (uint16_t)((pass ? 0x8000 : 0) | ((uint16_t)abs(measured) & 0x7FFF));
  FLOG_HOOK(FLOG_T_EVENT, (uint8_t)(FLOG_E_TEST_BASE + id - 1), v);
}

void selftest_openManual(void) {
  if (standbybutton_count != 1) { return; }
  stAuto = 0; stActive = 0; stResults = 0; stDone = 0;
  HMI_Page = PAGE_SELF_TEST;
  dwin_page_Set(PAGE_SELF_TEST);
}

void selftest_startAuto(void) {
  if (standbybutton_count != 1) { return; }
  stAuto = 1; stResults = 0; stDone = 0;
  HMI_Page = PAGE_SELF_TEST;
  dwin_page_Set(PAGE_SELF_TEST);
  selftest_start(1);
}

void selftest_start(uint8_t id) {
  if (standbybutton_count != 1) { return; }
  if (HMI_Page != PAGE_SELF_TEST) { return; }
  if (stActive != 0) { return; }             /* one test at a time      */
  stActive = id;
  stPhase = 0;
  stT0 = millis();
  stM1 = 0; stM2 = 0;
}

static void st_finish(uint8_t pass, int measured) {
  st_safeActuators();
  st_report(stActive, pass, measured);
  uint8_t last = stActive;
  stActive = 0;
  if (stAuto) {
    if (last < 8) {
      selftest_start((uint8_t)(last + 1));
    } else {
      stAuto = 0;
      FLOG_HOOK(FLOG_T_EVENT, FLOG_E_TEST_SUMMARY, stResults);
    }
  } else if (stDone == 0xFF) {
    FLOG_HOOK(FLOG_T_EVENT, FLOG_E_TEST_SUMMARY, stResults);
  }
}

void selftest_service(void) {
  if (stActive == 0) { return; }

  /* abort hard if standby is left mid-test */
  if (standbybutton_count != 1 || HMI_Page != PAGE_SELF_TEST) {
    st_safeActuators();
    stActive = 0; stAuto = 0;
    return;
  }

  uint32_t el = millis() - stT0;
  float p1 = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);
  float p3 = PRESSURE_FACTOR * (analogRead(Analog_pr3) - ZFP_pr3);

  switch (stActive) {

    case 1:                                  /* TURBINE                 */
      if (stPhase == 0) {
        analogWrite(peepPumpPin, 255);     /* valve closed            */
        analogWrite(turbinePin, ST_TURBINE_DUTY);
        stPhase = 1; stT0 = millis();
      } else if (p1 >= ST_TURBINE_MIN_P) {
        st_finish(1, (int)p1);
      } else if (el > ST_TURBINE_TIMEOUT) {
        st_finish(0, (int)p1);
      }
      break;

    case 2:                                  /* SENSOR zero offsets     */
      if (stPhase == 0) {
        st_safeActuators();                  /* everything off, settle  */
        stPhase = 1; stT0 = millis();
      } else if (el > ST_STEP_SETTLE_MS) {
        uint8_t ok = (p1 > -ST_SENSOR_ZERO_BAND) && (p1 < ST_SENSOR_ZERO_BAND)
                  && (p3 > -ST_SENSOR_ZERO_BAND) && (p3 < ST_SENSOR_ZERO_BAND);
        st_finish(ok, (int)((p1 > p3 ? p1 : p3) * 10));
      }
      break;

    case 3:                                  /* PNEUMATIC leak          */
      if (stPhase == 0) {
        analogWrite(peepPumpPin, 255);
        analogWrite(turbinePin, ST_TURBINE_DUTY);
        stPhase = 1; stT0 = millis();
      } else if (stPhase == 1) {
        if (p1 >= ST_PNEU_TARGET_P) {
          analogWrite(turbinePin, 0);         /* seal & watch decay      */
          stM1 = p1;
          stPhase = 2; stT0 = millis();
        } else if (el > ST_TURBINE_TIMEOUT) {
          st_finish(0, (int)p1);             /* could not pressurize    */
        }
      } else if (el > ST_PNEU_HOLD_MS) {
        float drop = stM1 - p1;
        st_finish(drop <= ST_PNEU_MAX_DROP, (int)(drop * 10));
      }
      break;

    case 4:                                  /* FIO2 ambient            */
      if (stPhase == 0) { stPhase = 1; stT0 = millis(); }
      else if (el > ST_STEP_SETTLE_MS) {
        uint8_t ok = (fio2 >= ST_FIO2_AMBIENT_LO) && (fio2 <= ST_FIO2_AMBIENT_HI);
        st_finish(ok, (int)fio2);
      }
      break;

    case 5:                                  /* PRESSURE cross-check    */
      if (stPhase == 0) {
        analogWrite(peepPumpPin, 255);
        analogWrite(turbinePin, ST_TURBINE_DUTY);
        stPhase = 1; stT0 = millis();
      } else if (el > ST_STEP_SETTLE_MS) {
        float d = p1 - p3; if (d < 0) { d = -d; }
        st_finish(d <= ST_PRESS_AGREE, (int)(d * 10));
      }
      break;

    case 6:                                  /* O2 SOURCE               */
      st_finish(o2AvailabilityStatus == HIGH, o2AvailabilityStatus);
      break;

    case 7:                                  /* BATTERY                 */
      st_finish(batteryPercent >= ST_BATT_MIN_PCT, batteryPercent);
      break;

    case 8:                                  /* VALVE authority         */
      if (stPhase == 0) {
        analogWrite(peepPumpPin, 255);     /* closed                  */
        analogWrite(turbinePin, ST_TURBINE_DUTY);
        stPhase = 1; stT0 = millis();
      } else if (stPhase == 1 && el > ST_STEP_SETTLE_MS) {
        stM1 = p1;                           /* closed pressure         */
        analogWrite(peepPumpPin, 0);       /* open                    */
        stPhase = 2; stT0 = millis();
      } else if (stPhase == 2 && el > ST_STEP_SETTLE_MS) {
        float delta = stM1 - p1;
        st_finish(delta >= ST_VALVE_MIN_DELTA, (int)(delta * 10));
      }
      break;

    default:
      st_safeActuators();
      stActive = 0;
      break;
  }
}
