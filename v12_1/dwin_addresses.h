/*******************************************************************************
 * dwin_addresses.h - named VP / touch-code map for the ventilator DGUS UI
 *
 * Every hex address that appears in the sketch, grouped by function.
 * Naming convention:
 *   VP_*    variable pointer the MCU WRITES to (0x82 frames)
 *   TP_*    touch return code RECEIVED from the display (0x83 frames)
 *   ICON_*  VP of a variable-icon control
 *   SYS_*   DGUS system registers (page, RTC, curve buffer)
 *   EE_*    (bonus) MCU-internal EEPROM map, since it lives beside these
 *
 * Usage: #include "dwin_addresses.h" at the top of the sketch (or paste the
 * block below the existing #defines), then migrate call sites, e.g.:
 *   display_write_variable(0xD020, tidalvolume);
 *     -> display_write_variable(VP_SET_TIDAL_VOLUME, tidalvolume);
 ******************************************************************************/
#ifndef DWIN_ADDRESSES_H
#define DWIN_ADDRESSES_H

/* ---------------- DGUS page IDs / HMI logical states ---------------- */
/* Values passed to dwin_page_Set() and held in HMI_Page. Note:
 * HMI_STATE_SETTINGS (50) is a LOGICAL state only - while in it, the
 * physical page shown is one of PAGE_SET_* (17-24) per active mode.  */
#define PAGE_NEBULIZER         1
#define PAGE_MONITOR_HFT       2
#define PAGE_MAIN              3
#define PAGE_DEMO              5
#define PAGE_START             12
#define PAGE_PATIENT_TYPE      13
#define PAGE_PATIENT_GENDER    14
#define PAGE_PATIENT_WEIGHT    15
#define PAGE_SETTINGS_ENTRY    16
#define PAGE_SET_CPAP          17
#define PAGE_SET_VCV           18
#define PAGE_SET_ACV           19
#define PAGE_SET_PSIMV         20
#define PAGE_SET_SIMV          21
#define PAGE_SET_APNEA_BK      22
#define PAGE_SET_PCV           23
#define PAGE_SET_HFT           24
#define PAGE_CLOCK             26
#define PAGE_MONITOR_2         27
#define PAGE_TEST_SCREEN       28
#define PAGE_ALARM_LIMITS      29
#define PAGE_O2CAL_START       30
#define PAGE_O2CAL_STEP1       31
#define PAGE_O2CAL_STEP2       32
#define PAGE_O2CAL_STEP3       33
#define PAGE_O2CAL_STEP4       34
#define PAGE_O2CAL_NO_O2       35
#define PAGE_STANDBY           36
#define HMI_STATE_SETTINGS     50
/* ---------------- DGUS system registers ---------------- */
#define SYS_PAGE_SWITCH        0x0084  /* dwin_page_Set() target             */
#define SYS_RTC                0x009C  /* clock write (5A A5 frame)          */
#define SYS_RTC_READ           0x0010  /* clockread[]                        */
#define SYS_CURVE_CLEAR        0x0301  /* line1[] - curve buffer reset       */
#define SYS_CURVE_BUFFER       0x0310  /* CURVE_TREND[] target               */

/* ---------------- waveform page ---------------- */
#define VP_CURVE_COLOR_SP      0x8007  /* colour_update[] (SP+7 of widget)   */
#define VP_PRESSURE_AXIS_MAX   0xD391
#define VP_PRESSURE_AXIS_MID   0xD282
#define VP_FLOW_AXIS_MAX       0xD383
#define VP_FLOW_AXIS_MID       0xD284
#define VP_VOLUME_AXIS_MAX     0xD371
#define VP_VOLUME_AXIS_MID     0xD286

/* ---------------- set-parameter VPs (settings pages 17-24) ---------------- */
#define VP_SET_FLOWRATE        0xD000  /* HFT flow setting                   */
#define VP_HFT_MEASURED_FLOW   0xD005  /* pressure2 live readout (HFT)       */
#define VP_SET_BODYWEIGHT      0xD010
#define VP_SET_TIDAL_VOLUME    0xD020
#define VP_SET_PINSP           0xD030
#define VP_SET_FREQUENCY       0xD040
#define VP_SET_IE_I            0xD050  /* i  x10                             */
#define VP_SET_TINSP_SIMV      0xD060  /* Tinsp_simv x10/1000                */
#define VP_SET_EXSENCE_PCT     0xD070
#define VP_SET_APNEA_TIME      0xD080
#define VP_SET_PS              0xD090
#define VP_SET_PEEP            0xD100
#define VP_SET_PTR             0xD110  /* trigger x10                        */
#define VP_SET_FIO2            0xD120
#define VP_SET_IE_E            0xD130  /* e  x10                             */
#define VP_SET_FREQ_SIMV       0xD140
#define VP_PATIENT_HEIGHT      0xD150

/* ---------------- alarm-limit VPs (page 29) ---------------- */
#define VP_ALM_PAW_HIGH        0xD180
#define VP_ALM_PAW_LOW         0xD190
#define VP_ALM_BPM_HIGH        0xD200
#define VP_ALM_BPM_LOW         0xD210
#define VP_ALM_VT_HIGH         0xD220
#define VP_ALM_VT_LOW          0xD230
#define VP_ALM_MV_HIGH         0xD240  /* x10                                */
#define VP_ALM_MV_LOW          0xD250  /* x10                                */
#define VP_ALM_FIO2_HIGH       0xD260
#define VP_ALM_FIO2_LOW        0xD270

/* ---------------- countdowns / misc numeric VPs ---------------- */
#define VP_MUTE_COUNTDOWN      0xD280  /* 90 s mute timer                    */
#define VP_MODEL_ID            0xD289  /* written 11 at boot                 */
#define VP_FW_VERSION          0xD290  /* pcb_VR x100                        */
#define VP_NEB_TIME            0xD297
#define VP_UNMOUNT_COUNTDOWN   0xD307
#define VP_CLOCK_HH            0xD314
#define VP_CLOCK_MM            0xD324
#define VP_O2CAL_PROGRESS      0xD325
#define VP_O2CAL_RAW           0x8325  /* raw fio2 ADC shown during cal -
                                          NOTE: 0x8325 vs 0xD325; verify this
                                          is intentional, not a typo         */

/* ---------------- test screen (page 28) ---------------- */
#define VP_CAL_HFT             0xC02B
#define VP_TEST_PR4_RAW        0xC02C
#define VP_CAL_FLOW            0xD317
#define VP_TEST_PR3_PRESSURE   0xD330
#define VP_TEST_BATT_VOLT      0xD335  /* x10                                */
#define VP_TEST_SMPS_VOLT      0xD340  /* x10                                */

/* ---- O2 cell diagnostics on the service page (verify 0xD660-0xD68F free) */
#define VP_TEST_FIO2_RAW       0xD660  /* raw ADC count from the O2 cell     */
#define VP_TEST_FIO2_INST      0xD670  /* instantaneous FiO2, x10            */
#define VP_TEST_FIO2_OK        0xD680  /* 1 = cell plausible, 0 = fault      */

/* ---------------- block-write base addresses ---------------- */
#define VP_MONITOR_BLOCK_1     0xC000  /* monitor_variable_1_write[] (7 words)*/
#define VP_MONITOR_BLOCK_2     0xC010  /* monitor_variable_2_write[] (6 words)*/
#define VP_TESTSCREEN_BLOCK    0xC020  /* test_1_variable_write[] (11 words) */
#define VP_SETPARAM_BLOCK      0x8000  /* Setparameter_variable_write[]      */
#define VP_FLOAT_DEMO          0x8050  /* writefloatvariable[]               */
#define VP_ALARM_TEXT          0x9000  /* alarm_buf_clear[]                  */

/* ---------------- hour meter (already defined in sketch) ---------------- */
/* DWIN_SEC_MTR_ADDR 0xBA07, DWIN_HR_MTR_ADDR 0xBA09, DWIN_MIN_MTR_ADDR
   0xBA11, DWIN_HR_TXT_ADDR 0xBA14, DWIN_LOAD_ADDR 0xBA50,
   DWIN_CT_TXT_ADDR 0xBA60, DWIN_TT_TXT_ADDR 0xBA80                          */

/* ---------------- icon VPs (variable-icon controls) ---------------- */
#define ICON_CURSOR            0x6000  /* Icon1  - settings cursor highlight */
#define ICON_AUX_1             0x6001  /* Icon2                              */
#define ICON_GENDER            0x6003  /* Icon3                              */
#define ICON_ALARM_MAIN        0x6008  /* Icon4  - paw/vt/bpm/mv alarms      */
#define ICON_ALARM_AUX_1       0x6004  /* Icon5                              */
#define ICON_ALARM_AUX_2       0x6005  /* Icon6                              */
#define ICON_ALARM_AUX_3       0x6006  /* Icon7                              */
#define ICON_ALARM_AUX_4       0x6007  /* Icon8                              */
#define ICON_ALARM_AUX_5       0x6009  /* Icon9                              */
#define ICON_FIO2_ALARM        0x6010  /* Icon10                             */
#define ICON_SENSOR_ALARM      0x6011  /* Icon11                             */
#define ICON_APNEA_BACKUP      0x6012  /* Icon12                             */
#define ICON_POWER_ALARM       0x6013  /* Icon13                             */
#define ICON_MUTE              0x6014  /* Icon14                             */
#define ICON_MODE_BADGE        0x6015  /* Icon15 - active mode label         */
#define ICON_AUX_2             0x6016  /* Icon16                             */
#define ICON_LIMIT_CURSOR      0x6017  /* Icon17 - alarm-limit page cursor   */
#define ICON_STANDBY_CURSOR    0x6018  /* Icon18                             */
#define ICON_BATTERY           0x6019  /* Icon19 - battery gauge             */
#define ICON_AUX_3             0x6020  /* Icon20                             */
#define ICON_NEBULIZER         0x6021  /* Icon21                             */
#define ICON_O2_STATUS         0x6022  /* Icon22                             */
#define ICON_TRIGGER           0x6023  /* Icon23 - patient trigger flag      */
#define ICON_LOCK              0x6026  /* Icon26 - screen lock               */

/* ---------------- touch return codes (0x83 frames from display) --------- */
/* mode selection (opens page in dwin_page_Set argument)                     */
#define TP_MODE_PCV            0xA409  /* mode1=5  -> page 17                */
#define TP_MODE_VCV            0xA410  /* mode1=1  -> page 18                */
#define TP_MODE_ACV            0xA411  /* mode1=3  -> page 19                */
#define TP_MODE_PSIMV          0xA412  /* mode1=8  -> page 20                */
#define TP_MODE_SIMV           0xA413  /* mode1=2  -> page 21                */
#define TP_MODE_APNEA_BK       0xA414  /* mode1=6  -> page 22                */
#define TP_MODE_CPAP            0xA415  /* mode1=7  -> page 23                */
#define TP_MODE_HFT            0xA416  /* mode1=4  -> page 24                */
/* action buttons                                                            */
#define TP_CONFIRM             0xA417

/* ============================================================
 * ALARM & EVENT LOG VIEWER (page 43) - keys 0xA493-0xA497 and
 * VP window 0xD400-0xD533 verified unused. Access: entry only
 * from the standby screen; auto-exit when ventilation starts.
 * ============================================================ */
#define PAGE_LOG_VIEW          43      /* Alarms and Fault Logs page        */
#define PAGE_SYSLOG_VIEW       41      /* System and Event Logs page        */
#define TP_SYSLOG_OPEN         0xA498  /* -> page 41 (from standby screen)  */
#define TP_LOG_OPEN            0xA493  /* -> page 43 (from standby screen)  */
#define TP_LOG_UP              0xA494  /* scroll to OLDER records (+5)      */
#define TP_LOG_DOWN            0xA495  /* scroll to NEWER records (-5)      */
#define TP_LOG_HOME            0xA496  /* jump to newest + refresh          */
#define TP_LOG_BACK            0xA497  /* page 43 -> standby page 36        */
#define VP_LOG_LINE_BASE       0xD400  /* 10 text rows, step 0x20           */
#define VP_LOG_LINE_STEP       0x0020
#define TP_NEWPATIENT_SETUP    0xA401  /* new-patient flow entry             */
#define TP_PATIENT_PEDIATRIC   0xA403
#define TP_PATIENT_ADULT       0xA404
#define TP_PATIENT_MALE        0xA405
#define TP_PATIENT_FEMALE      0xA406
#define TP_PATIENT_PROCEED     0xA408  /* weight page -> settings            */
#define TP_PARAM_SEL_A         0xA418
#define TP_PARAM_SEL_B         0xA419
#define TP_PARAM_SEL_C         0xA420
#define TP_PARAM_SEL_D         0xA421
#define TP_PARAM_SEL_E         0xA422
#define TP_PARAM_SEL_F         0xA424
#define TP_PARAM_SEL_G         0xA425
#define TP_PARAM_SEL_H         0xA426
#define TP_PARAM_SEL_I         0xA427
#define TP_PARAM_SEL_J         0xA428
#define TP_PARAM_SEL_K         0xA430
#define TP_FIO2_SELECT         0xA432
#define TP_INCREMENT           0xA433
#define TP_DECREMENT           0xA434
#define TP_FLOW_SELECT         0xA435
#define TP_NAV_TO_P14          0xA436
#define TP_NAV_TO_P13          0xA437
#define TP_NAV_TO_MAIN         0xA438
#define TP_MUTE_TOGGLE         0xA439
#define TP_NAV_ALT             0xA441
#define TP_TESTSCREEN_OPEN     0xA442
#define TP_MAIN_FROM_TEST      0xA443
#define TP_TEST_OVDUTY_SEL     0xA444
#define TP_LIMIT_CUR_PAWHI     0xA448
#define TP_LIMIT_CUR_PAWLO     0xA447
#define TP_O2CAL_OPEN          0xA449
#define TP_O2CAL_CONFIRM       0xA450
#define TP_O2CAL_CANCEL        0xA451
#define TP_LIMIT_CUR_BPMHI     0xA452
#define TP_LIMIT_CUR_BPMLO     0xA453
#define TP_LIMIT_CUR_VTHI      0xA454
#define TP_LIMIT_CUR_VTLO      0xA455
#define TP_LIMIT_CUR_MVHI      0xA456
#define TP_LIMIT_CUR_MVLO      0xA457
#define TP_LIMIT_CUR_FIO2HI    0xA458
#define TP_LIMIT_CUR_FIO2LO    0xA459
#define TP_LIMITS_OPEN         0xA460
#define TP_BACK_CANCEL         0xA461
#define TP_NEB_OPEN            0xA462
#define TP_NEB_CANCEL          0xA463
#define TP_UNMOUNT             0xA464
#define TP_VENTCHK_1           0xA465
#define TP_VENTCHK_2           0xA466
#define TP_VENTCHK_3           0xA467
#define TP_VENTCHK_4           0xA468
#define TP_VENTCHK_5           0xA469
#define TP_VENTCHK_6           0xA470
#define TP_VENTCHK_7           0xA471
#define TP_VENTCHK_8           0xA472
#define TP_SETTINGS_REOPEN     0xA473
#define TP_VENT_MENU           0xA474
#define TP_NEB_START           0xA475
#define TP_BACK_P14_TO_P13     0xA476
#define TP_BACK_P15_TO_P14     0xA477
#define TP_CONFIRM_FROM_P29    0xA479
#define TP_TEST_HFTCAL_SEL     0xA480
#define TP_TEST_FLOWCAL_SEL    0xA481
#define TP_LOCK_TOGGLE         0xA488
#define TP_STANDBY_SCREEN      0xA491
#define TP_MONITOR_OPEN        0xC903
#define TP_NEWPATIENT_OPEN     0xC904
#define TP_SETTINGS_OPEN       0xC905
#define TP_ALARMLIMIT_OPEN     0xC906
#define TP_CALB_TRIGGER        0xCAAB
#define TP_CLOCK_HH_SEL        0xD301
#define TP_CLOCK_MM_SEL        0xD302
#define TP_CLOCK_SAVE          0xD303
#define TP_CLOCK_EXIT          0xD304
#define TP_CLOCK_OPEN          0xD305
#define TP_RTC_RESPONSE        0x1004  /* parsed from Buffer1[5..6]          */

#define VP_CLOCK_DD            0xD344  /* day   edit field, page 26 */
#define VP_CLOCK_MON           0xD354  /* month edit field          */
#define VP_CLOCK_YY            0xD364  /* year  edit field (2-digit)*/
#define TP_CLOCK_DD_SEL        0xD306  /* touch: select day         */
#define TP_CLOCK_MON_SEL       0xD308  /* touch: select month       */
#define TP_CLOCK_YY_SEL        0xD309  /* touch: select year        */


#define PAGE_CAL_SELECT        44      // Manual/Auto selection    TODO
#define PAGE_SELF_TEST         45      // Testing Result page      TODO
#define TP_CAL_MANUAL          0xA4A0
#define TP_CAL_AUTO            0xA4A1
#define TP_TEST_TURBINE        0xA4A2
#define TP_TEST_SENSOR         0xA4A3
#define TP_TEST_PNEUMATIC      0xA4A4
#define TP_TEST_FIO2           0xA4A5
#define TP_TEST_PRESSURE       0xA4A6
#define TP_TEST_O2SRC          0xA4A7
#define TP_TEST_BATTERY        0xA4A8
#define TP_TEST_VALVE          0xA4A9
#define VP_TEST_LINE_BASE      0xD560  // 8 result rows, step 0x20
#define VP_TEST_LINE_STEP      0x0020  // (verify 0xD560-0xD65F free)

#endif /* DWIN_ADDRESSES_H */
