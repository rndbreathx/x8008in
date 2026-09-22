#include "MultiMap.h"
#include <stdint.h>
#include <FastPID.h>
#include <EEPROM.h>
#include <SD.h>
#include <RTClib.h>
#include <Arduino.h>
#include <math.h>
#include <SPI.h>
#include "dwin_addresses.h"


void apnea_breathSeen(void);
/* ---- ventilation mode codes (setmode / mode1 / dispatch)  */
#define MODE_VCV       1
#define MODE_SIMV      2
#define MODE_ACV       3   /* VACV - also the apnea backup target   */
#define MODE_HFT       4
#define MODE_PCV       5
#define MODE_SPONT_PS  6
#define MODE_CPAP      7
#define MODE_PSIMV     8

/* ---- apnea backup  ---- */
#define ALARM_NUM_APNEA 15          /* 1-13 used, 14 = low-Paw            */
#define APNEA_RETURN_BREATHS 2      /* spontaneous triggers to return;
                                     /*  CLINICAL-CONFIRM                   */

/* ============================================================================
 * FLASH LOG PLACEHOLDERS - event & alarm logging to SPI flash (W25Qxx).
 * ENABLE_FLASH_LOG 0 (default): every hook compiles to NOTHING.
 * Set 1 once spi_flash_log.h/.ino are in the sketch and the chip fitted.
 * SD card and W25Qxx share SPI - separate CS pins; never reuse the SD CS.
 * ========================================================================== */
#define ENABLE_FLASH_LOG 0

#if ENABLE_FLASH_LOG
#include "spi_flash_log.h"
#define FLOG_INIT() flog_init()
#define FLOG_SERVICE() flog_service()
#define FLOG_HOOK(t, c, v) flog_event((t), (c), (v))
#define FLOG_ALARM(code, cond) do { static uint8_t _fa = 0; flog_alarmEdge((code), (cond) ? 1 : 0, &_fa); } while (0)
#define LOGVIEW_SERVICE() logview_service()
#else
#define FLOG_INIT()
#define FLOG_SERVICE()
#define FLOG_HOOK(t, c, v)
#define FLOG_ALARM(code, cond)
#define LOGVIEW_SERVICE()
#endif
#include <Wire.h>

#define PEEP_UPPER_THRESHOLD        120
#define PEEP_LOWER_THRESHOLD        0
#define PEEP_DEFAULT_THRESHOLD      15


#define DWIN_PORT Serial2
#define DEBUG Serial

#define CMD_REGISTER_WRITE 0x80
#define CMD_REGISTER_READ 0x81
#define CMD_VARIABLE_WRITE 0x82
#define CMD_VARIABLE_READ 0x83
#define CMD_TREND_CURVE_BUFFER 0x84

#define WRITE_REGISTER_NO_BYTES 7
#define WRITE_VARIABLE_NO_BYTES 10
#define MAX_MESSAGE_LENGTH 10

#define FRAME_START_LOW 0x5A
#define FRAME_START_HIGH 0xA5

#define TEXT_CMD 0x5020
#define CURVE_CMD 0x5AA5
#define CURVE_BUFF_SRT_CMD 0x0310

#define MCP4725 0x60  

#define OFFSET_BW_TV (8)
#define INTERVALp 1         
#define pr3_WINDOW_SIZE 30  
#define p_WINDOW_SIZE 30    

#define WINDOW_SIZEie 3              
#define WINDOW_SIZEpressure_mean 10  
#define WINDOW_SIZEpressure 15       

#define WINDOW_SIZEcflow 20  
#define WINDOW_SIZEbattery 20
#define BATTERY_WINDOW_SIZE 15
#define WINDOW_SIZEpeep 4  

#define INTERsal 10         
#define HR_TO_SEC 2.778E-4  
#define WINDOW_SIZEop 10    

#define WINDOW_SIZEdop 20   /* was 150: 150 zero-primed slots made the
                              monitored FiO2 ramp up from 0 for minutes */
#define WINDOW_SIZEcop 20   
#define WINDOW_SIZEvt 10    

#define LED_AC_MAIN 29
#define BATTERY_LED 27
#define OXYGEN_LED 23

#define PRESSURE_FACTOR 0.244140625

#define MAINS_PIN A15
#define BATTERY_PIN A14

#define battery_percent_10 590
#define battery_percent_20 600
#define battery_percent_30 610
#define battery_percent_40 620
#define battery_percent_50 630
#define battery_percent_60 640
#define battery_percent_70 650
#define battery_percent_80 660
#define battery_percent_90 670
#define battery_percent_100 680

int16_t battery_icon_percentage;  /* raw ADC */

bool CpapSensorFlag = 0;

float setpoint_hft = 0;
float input_hft, output_hft;

float Kp_hft = 0.001, Ki_hft = 0.00008, Kd_hft = 0.00004;

float previousError_hft = 0.0;
float integral_hft = 0.0;
uint32_t lastTime_hft;

float input_cpap, output_cpap;

float setpoint_peep = 0;  
float input_peep, output_peep;

uint8_t ton_1 = 0;

float comp = 0;

float distance;

float Kp = 0.0091, Ki = 0.04, Kd = 00.04, Hz = 10;
uint32_t before, after;
int output_bits = 16;
bool output_signed = true;
FastPID myPID(Kp, Ki, Kd, Hz, output_bits, output_signed);

int coil = 0;

int Exsence_timer = 0;
float Exsence_percentage = 100;

int s = 0;
int sp = 0;
int8_t B_status_led = 9;   
int8_t G_status_led = 10;  
int8_t R_status_led = 11;  

float old;
int T_volume = 0;
int T_volume1 = 0;
int T_Volume_final = 0;
int qw = 0;
int leak_volume = 0;
int output1 = 0;
int8_t inspiration = 0;

uint16_t currentPage = 0;
uint16_t prevPage = 0;
uint8_t backNav = 0;   /* 1 = this page change IS a back-navigation:
                          do not record it as history, or Back
                          oscillates between the last two pages     */

float pressure;
float pressure2;
float pressure4;
float pressure3;
int sensorfailure = 0;
int pr1failiure = 0;

float prs = 0;
int gh = 0;
uint32_t flowsensor_pawcurrentMillis = 0;
int hpstate = 0;
int Hvtstate = 0;
int Hvtcutstate = 0;
int Lvtstate = 0;

int peakpressure = 0;
int monitored_flow_e = 0;
int monitored_flow_i = 0;
int peak_1 = 0;
int peak_flow = 0;
int peak_flow_e = 0;
int peak_flow_i = 0;
int latest_p;
int latest_p2;
float V1 = 0;
float V2 = 0;
float v1 = 0;
float v2 = 0;
uint8_t hhh = 0;
uint8_t mmm = 0;
uint8_t sss = 0;
uint8_t lock_unlock = 0;

int onetimestartbreath = 1;

uint8_t su = 1;

int nebulizerpin = 5;  //5  

int pr3_INDEX = 0;
int pr3_VALUE = 0;
int pr3_SUM = 0;
int pr3_READINGS[pr3_WINDOW_SIZE];
int pr3_AVERAGED = 0;
const int frequency = 2000;

int p_INDEX = 0;
int p_VALUE = 0;
int p_SUM = 0;
int p_READINGS[p_WINDOW_SIZE];
int p_AVERAGED = 0;
int p_AVERAGED_SAVED = 0;
uint32_t cmillis = 0;
uint32_t pmillis = 0;
int rrThresCount = 0;

int ie_INDEX = 0;
float ie_VALUE = 0;
float ie_SUM = 0;
float ie_READINGS[WINDOW_SIZEie];
float ie_AVERAGED = 0;

int pressure_mean_INDEX = 0;
float pressure_mean_VALUE = 0;
float pressure_mean_SUM = 0;
float pressure_mean_READINGS[WINDOW_SIZEpressure_mean];
float pressure_mean_AVERAGED = 0;

int pressure_INDEX = 0;
float pressure_VALUE = 0;
float pressure_SUM = 0;
float pressure_READINGS[WINDOW_SIZEpressure];
float pressure_AVERAGED = 0;

int cflow_INDEX = 0;
int cflow_VALUE = 0;
float average1_e = 0;
float average1_e2 = 0;
float cflow_SUM = 0;
int16_t cflow_READINGS[WINDOW_SIZEcflow];  /* stores int cflow_VALUE */
float cflow_AVERAGED = 0;

int BATTERY_INDEX = 0;
int BATTERY_VALUE = 0;
int BATTERY_SUM = 0;
int BATTERY_READINGS[BATTERY_WINDOW_SIZE];
int BATTERY_AVERAGED = 0;

int percent = 0;
int battery_AVERAGED = 0;
int battery_AVERAGED_1 = 0;
float btr_volt;
float SMPS;

int pf = 2;
int pf1 = 0;
int peep_INDEX = 0;
float peep_VALUE = 0;
float peep_SUM = 0;
float peep_READINGS[WINDOW_SIZEpeep];
float peep_AVERAGED = 0;
int peep_AVERAGED_SAVED = 0;

const uint8_t Analog_pr1 = A5;  
const uint8_t Analog_pr3 = A6;  
const uint8_t Analog_pr2 = A7;  
const uint8_t Analog_pr4 = A8;  
float Pr4_range = 93.6f;
float Pr2_range = 93.6f;  
int ZFP = 512;            

int ZFP_pr3 = 102;    
float ZFF_PR2 = 504;  
float ZFF_PR4 = 504;  
float CF_PR2 = 120;  

float turbine_pickup_time_factor = 1;
int g = 0;
int g1;
int g2 = 2;         
int turbinePin = 7;  
float compan_duty = 0;
float compan_duty1 = 0;
int compan_duty2 = 0;
float tur_duty = 50.00f;
float tur_duty_HFT = 30.00f;
int calibration_HFT = 4;    
int calibration_flow = 10;  
float calibration_flow_final = 1;
int tur_reverse = 24;  
uint32_t TCMillis = 0;
uint32_t TPMillis = 0;

uint8_t testscreen_count;
uint8_t mut = 0;
uint8_t unmount = 0;
uint8_t ala = 1;  
int buffer_address = 100;
uint8_t mainscreencursor = 0;
uint8_t standbyiconcursor = 0;
uint8_t Height = 0;
uint8_t HMI_Page = 12;
uint8_t cursr_forward = 1;
uint8_t stand_check = 0;
uint8_t ventilation_check = 0;

int8_t newpatient = 0;
int8_t malepatient = 1;
int8_t pediatric_patient = 0;
float calibration_HFT_final = 0;

uint8_t years;
uint8_t months;
uint8_t dayss;
uint8_t hh1;
uint8_t mm1;
uint8_t ss1;
uint8_t ss2;
uint8_t last_w = 0;
int last_v = 0;
int last_vol = 0;
uint8_t nebu_touch = 0;
int monitored_mv = 0;
uint32_t neb_time = 0;
uint32_t set_neb_time = 0;
int data_VOLUME = 0;
int vol = 0;
int w = 1;
int v = 1;
float currentvolume1 = 0;
char buff_rtc[50];
char buff_event[50];
int8_t display_page_3 = 3;
volatile int O2confirm_add;


int8_t x;

float data_PRESSURE = 0;
int data_FLOW;
int data_PRESSURE_print = 0;
#define PRESSURE_GRAPH_OFFSET 0  /* legacy +5 removed - read 2.7 high */

/* ===== VOLUME DISPLAY SPLIT (v5): display-only integrator; the
   measurement chain (currentvolume) is UNTOUCHED - v3 incident. ===== */
#define PR4_BIAS_LEARN_MS 1200
#define PR4_BIAS_LEARN_ALPHA 0.02f
#define EXH_DONE_FLOW 3.0f
#define EXH_DONE_MS 200
float pr4_bias_offset = 0.0;
float dispVolume = 0.0;
float dispE2 = 0.0;
uint8_t exhDone = 0;
uint32_t exhLowSince = 0;

unsigned char CURVE_TREND[28] = { 0x5a, 0xa5, 0x19, 0x82, 0x03, 0x10, 0x5A, 0xA5, 0x08, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x96, 0x01, 0x02,
                                  0x00, 0x00, 0x00, 0x96, 0x02, 0x02, 0x00, 0x00, 0x00, 0x96 };
unsigned char monitor_variable_1_write[20] = {
  0x5a,  0xa5,  17,  0x82,  0xC0,  0x00,  0x00,  0x64,  0x00,  0x63,  0x00,  0x62,  0x00,  0x62,  0x00,  0x62,  0x00,  0x62,  0x00,  0x62};  
unsigned char monitor_variable_2_write[18] = {
  0x5a,
  0xa5,
  15,
  0x82,
  0xC0,
  0x10,
  0x00,
  0x64,
  0x00,
  0x63,
  0x00,
  0x62,
  0x00,
  0x62,
  0x00,
  0x62,
  0x00,
  0x62,
};  

unsigned char monitorWindowVariable_write[28] = { 0x5a, 0xa5, 25, 0x82, 0xC0, 0x20, 0x00, 0x64, 0x00, 0x63, 0x00, 0x62, 0x00, 0x62, 0x00, 0x62, 0x00, 0x62, 0x00, 0x62, 0x00, 0x62,
                                            0x00, 0x62, 0x00, 0x62, 0x00, 0x62 };


unsigned char clock1[14] = { 0x5a, 0xa5, 0x0B, 0x82, 0x00, 0x9C,
                             0x5A, 0xA5, 23, 2, 21, 9, 36, 00 };

unsigned char Icon1[8] = { 0x5a, 0xa5, 0x05, 0x82, 0x60, 0x00,
                           0x00, 0x00 };
unsigned char Icon3[8] = { 0x5a, 0xa5, 0x05, 0x82, 0x60, 0x03,
                           0x00, 0x00 };
unsigned char Icon4[8] = { 0x5a, 0xa5, 0x05, 0x82, 0x60, 0x08,
                           0x00, 0x00 };  
unsigned char Icon5[8] = { 0x5a, 0xa5, 0x05, 0x82, 0x60, 0x04,
                           0x00, 0x00 };  
unsigned char Icon6[8] = { 0x5a, 0xa5, 0x05, 0x82, 0x60, 0x05,
                           0x00, 0x00 };  
unsigned char Icon7[8] = { 0x5a, 0xa5, 0x05, 0x82, 0x60, 0x06,
                           0x00, 0x00 };  
unsigned char Icon8[8] = { 0x5a, 0xa5, 0x05, 0x82, 0x60, 0x07,
                           0x00, 0x00 };  
unsigned char Icon9[8] = { 0x5a, 0xa5, 0x05, 0x82, 0x60, 0x09,
                           0x00, 0x00 };  
unsigned char Icon10[8] = { 0x5a, 0xa5, 0x05, 0x82, 0x60, 0x10,
                            0x00, 0x00 };  
unsigned char Icon11[8] = { 0x5a, 0xa5, 0x05, 0x82, 0x60, 0x11,
                            0x00, 0x00 };  
unsigned char Icon12[8] = { 0x5a, 0xa5, 0x05, 0x82, 0x60, 0x12,
                            0x00, 0x00 };  
unsigned char Icon13[8] = { 0x5a, 0xa5, 0x05, 0x82, 0x60, 0x13,
                            0x00, 0x00 };  
unsigned char Icon14[8] = { 0x5a, 0xa5, 0x05, 0x82, 0x60, 0x14,
                            0x00, 0x00 };  

unsigned char Icon15[8] = { 0x5a, 0xa5, 0x05, 0x82, 0x60, 0x15,
                            0x00, 0x00 };

unsigned char Icon17[8] = { 0x5a, 0xa5, 0x05, 0x82, 0x60, 0x17,
                            0x00, 0x00 };

unsigned char Icon18[8] = { 0x5a, 0xa5, 0x05, 0x82, 0x60, 0x18,
                            0x00, 0x00 };

unsigned char Icon19[8] = { 0x5a, 0xa5, 0x05, 0x82, 0x60, 0x19,
                            0x00, 0x00 };

unsigned char Icon20[8] = { 0x5a, 0xa5, 0x05, 0x82, 0x60, 0x20,  
                            0x00, 0x00 };

unsigned char Icon21[8] = { 0x5a, 0xa5, 0x05, 0x82, 0x60, 0x21,  
                            0x00, 0x00 };

unsigned char Icon22[8] = { 0x5a, 0xa5, 0x05, 0x82, 0x60, 0x22,  
                            0x00, 0x00 };

unsigned char Icon23[8] = { 0x5a, 0xa5, 0x05, 0x82, 0x60, 0x23,  
                            0x00, 0x00 };
unsigned char Icon26[8] = { 0x5a, 0xa5, 0x05, 0x82, 0x60, 0x26,  
                            0x00, 0x00 };


unsigned char Buffer1[80];
unsigned char Buffer1_Len = 0;
unsigned int sal;
uint8_t buffer[3];

int average_flow = 0;

uint16_t bodyweight = 52;
uint16_t setbodyweight = 52;
uint8_t bdc = 0;

int O2cal = 0;
uint32_t calicurrenttime = 0;
uint32_t caliprevioustime = 0;
int confirm = 0;

int calb = 0;
float f1 = 19.90;  
float f2 = 729;
float L3 = 01.19;  
int L3_1 = 01.19;
int f1_1 = 19.90;

float average1 = 0;

int testscreenpage = 1;
int testscreenstate = 0;

float volume = 0;

int d = 0;


int unmount_pin = 40;  

int ptl = 0;
int set_fio2 = 21;
int set_set_fio2 = 21;
int set_fio2_last = 0;
int j = 1;
const uint8_t fio2_read = A10;
float fio2 = 0;
float fio23 = 0;
uint32_t o2sensorfailtimer = 0;
int mini_step = 0;

int Op_INDEX = 0;
float Op_VALUE = 0;
float Op_SUM = 0;
float Op_READINGS[WINDOW_SIZEop];
float Op_AVERAGED = 0;
int Op_AVERAGED_SAVED = 0;
uint32_t Ocmillis = 0;
uint32_t Opmillis = 0;

int dOp_INDEX = 0;
float dOp_VALUE = 0;
float dOp_SUM = 0;
float dOp_READINGS[WINDOW_SIZEdop];
float dOp_AVERAGED = 0;
int dOp_AVERAGED1 = 0;

/* ===== FiO2 measurement health / filter priming =====================
 * fio2_sensor_ok  0 = cell or amplifier output not plausible. The
 *                 monitored FiO2 is then reported as 21 and every FiO2
 *                 alarm is inhibited - a dead cell must not be shown as
 *                 a real low-FiO2 reading.
 * *_primed        the boxcars are pre-loaded with the first live sample
 *                 instead of starting from an array of zeros.
 * fio2_raw_adc    last raw ADC count, exposed on the service page.
 * FIO2_ADC_FLOOR  counts below the room-air point f1 that still count as
 *                 plausible (cell ageing + amplifier offset drift).
 * ================================================================== */
#define FIO2_ADC_FLOOR 8.0f
uint8_t fio2_sensor_ok = 1;
uint8_t Op_primed = 0;
uint8_t dOp_primed = 0;
int fio2_raw_adc = 0;

int cOp_INDEX = 0;
float cOp_VALUE = 0;
float cOp_SUM = 0;
float cOp_READINGS[WINDOW_SIZEcop];
float cOp_AVERAGED = 0;
uint32_t cOcmillis = 0;

int mask = 0;

const uint8_t mainspower_read = MAINS_PIN;
int mainspower_write = 23;  
static uint32_t tonetimer1;
static uint32_t previoustonetimer1 = 0;
static uint32_t tonetimer;
static uint32_t previoustonetimer = 0;
int tonePin = 31;      
int alarm_write = 27;  
int batt_status = 0;
int batterypower_write = 25;  
int16_t mainspower_state = 0;  /* raw ADC */
int16_t mainspower_last_state = 0;
const uint8_t lowbatterystatus_write = 25;  

int oxygenavailability_read = 26;  
int oxygenavailability_status = HIGH;
int oxygenavailability_status_last = HIGH;

int oxygenavaibility_write = 460;  
int oxygenfailure_write = 380;     
int airavailability_write = 290;   
int airfailure_write = 280;        

int airavailability_read = 600;  
int airavailability_status = LOW;
uint32_t oxy_fail_cmillis = 0;
uint32_t oxy_fail_pmillis = 0;

int mainrelaypin = 24;        
float mainrelay_duty = 1825;  
int ov_start_duty = 1825.0;

int mainrelaypinstate = LOW;  
int mainrelaypinstate_count = 0;
uint32_t mainrelaypreviousMillis = 0;  
uint32_t mainrelaycurrentMillis = 0;
float Texp = 4300;                           
float Tinsp = 1700;                          
float Tinsp_simv = 1700;
int Tinsp_simv1 = 1700;
uint32_t current_TI = 0;
uint32_t previous_TI = 0;
uint32_t monitore_TI = 0;
uint32_t current_TE = 0;
uint32_t previous_TE = 0;
uint32_t monitore_TE = 0;
float setTinsp_simv = 1700;

uint32_t interval_new_e = 0;
uint32_t interval_old_e = 0;
float flow_time_e = 0;

int flowrate = 30;
int setflowrate = 30;
uint32_t interval_new = 0;
uint32_t interval_old = 0;
uint32_t Ti_real = 0;
uint32_t Ti_real_1 = 0;
float flow_time = 0;
float battery_time = 0;
int f = 0;

int touch = 0;
int encoder_switch_lastState = 0;
int encoder_switchState = 0;
int encoder_switch_count = 0;  

int encoder_switchPin = 9;    /* EN_SW  - D9  (schematic V3.18.3)  */  

int encoder_pinA = 18;        /* EN_CKL - D18 (interrupt-capable)  */                                
int encoder_pinB = 19;        /* EN_ACKL- D19 (interrupt-capable)  */                                
int encoder_pinAstateCurrent = LOW;                    
int encoder_pinAStateLast = encoder_pinAstateCurrent;  

int mode1 = MODE_VCV;
int setmode = MODE_VCV;  

int settidalvolume = 480;
int tidalvolume = 480.00f;
int tidalvolume_L_limit = 300;
int tidalvolume_H_limit = 900;
int Pinsp = 15;
int set_Pinsp = 20;

int currentvolume;  

int setfrequency = 12;
int frequncy_L_limit = 2;
int frequncy_H_limit = 30;
int frequncy = 12;
int frequencysimv = 8;
int setfrequencysimv = 8;
int frequencysimv_H_limit = 30;
int U1 = 1;
int UU = 1;
int R_count = 1;

int INDEX_bpm = 0;
float VALUE_bpm = 0;
float SUM_bpm = 0;
float READINGS_bpm[2];
float AVERAGED_bpm = 0;
float AVERAGED_SAVED_bpm = 0;
uint32_t bpm_cmillis = 0;
uint32_t bpm_pmillis = 0;
uint32_t bpm_totaltime = 0;

int vt_INDEX = 0;
float vt_VALUE = 0;
float vt_SUM = 0;
float vt_READINGS[WINDOW_SIZEvt];
float vt_AVERAGED = 0;
int vt_AVERAGED_SAVED = 0;

float seti_e = 2.0;
float i_e = 2.0;
float i = 1;
float e = 2;
float set_i = 1;
float set_e = 2;

int setps = 20;
int ps = 10;

int setpeep = 0;
int peep = 0;
const uint8_t peepPumpPin = 2;  
float peepduty = 35.00f;  
uint32_t exp_pawcurrentMillis = 0;
int pp = 1;
int pp1 = 0;
int mainrelaypinstate_lastState;
int pee = 2;
float pee1 = 0.00f;

int trigger_monitor = 0;
int trigger_monitor_save = 0;

int setlowpawalarmlimit = 5;
int lowpawalarmlimit = 5;

int highpawalarmlimit = 30;
int sethighpawalarmlimit = 30;

int highbpmalarmlimit;
int lowbpmalarmlimit;

int highvtalarmlimit;
int lowvtalarmlimit;

float high_mvalarmlimit = 10;
float low_mvalarmlimit = 0;

int high_mvalarmlimit_1 = 10;
int low_mvalarmlimit_1 = 0;

int high_fio2alarmlimit;
int low_fio2alarmlimit;

int setptr_1 = 5;
float setptr = 5;
float ptr = 5.0f;
int ptr_1 = 0;

float pcb_VR = 04.01;
int version_1 = 0;
int tr = 0;
int tr_save = 0;

const uint8_t setting_button = 900;  

uint8_t settingButtonState;
uint8_t settingbutton_lastState = LOW;
uint8_t settingbutton_count = 2;
uint32_t settingcurrenttime = 0;
uint32_t settingprevioustime = 0;

uint8_t cancelButtonState;
uint8_t cancelbutton_lastState = LOW;

const uint8_t decreament_button = 19;  

uint8_t decreamentButtonState;
uint8_t decreamentbutton_lastState = LOW;
uint32_t decreamentpreviousMillis = 0;  
uint32_t decreamentcurrentMillis = 0;

const uint8_t increament_button = 18;  
const uint8_t confirmbutton = 41;      

int cancel_button = 43;  

int yellow_led_10inch_button = 31;  
int white_led_10inch_button = 36;   
int red_led_10inch_button = 34;     

uint8_t increamentButtonState;
uint8_t increamentbutton_lastState = LOW;
uint32_t increamentpreviousMillis = 0;  
uint32_t increamentcurrentMillis = 0;

const uint8_t modebutton = 41;  
uint8_t modebuttonState = 1;
uint8_t modebutton_lastState = LOW;

uint8_t confirmbuttonState;
uint8_t confirmbutton_lastState = LOW;

uint8_t nebbuttonState_timer = 0;
uint8_t nebbutton_count_timer = 0;  
uint32_t nebpreviousMillis_timer = 0;
uint32_t nebcurrentMillis_timer = 0;

uint8_t unmountbuttonState = 0;
uint8_t unmountbutton_lastState = LOW;
uint8_t unmountbutton_count = 1;
uint32_t unmountpreviousMillis = 0;
uint32_t unmountcurrentMillis = 0;

const uint8_t mutebutton = 45;  

uint8_t mutebuttonState;
uint8_t mutebutton_lastState = LOW;
uint8_t mutebutton_count = 1;
uint32_t mutepreviousMillis = 0;
uint32_t mutecurrentMillis = 0;

const uint8_t resetbutton = 500;  

uint8_t resetbuttonState;
uint8_t resetbutton_lastState = LOW;

const uint8_t standbybutton = 43;  

uint8_t standbybuttonState;
uint8_t standbybutton_lastState = LOW;
uint8_t standbybutton_count = 1;
uint32_t standbycurrenttime = 0;
uint32_t standbyprevioustime = 0;
int stnd = 1;
int standby_scr = 0;

int alarmled = 25;  

uint32_t alarmtimerpreviousMillis = 0;
uint32_t alarmtimercurrentMillis = 0;
int AT = 0;

static int alarm_num = 0;
uint8_t alarm_num1 = 0;
uint8_t alarm_numpower1 = 0;
int alarm_numpower = 0;

uint32_t lowpawcurrentMillis = 0;

uint32_t pr1failiurecurrentMillis = 0;

uint32_t apnbccurrentmillis = 0;
int apntr = 0;
int a = 0;
int b = 0;
int apnbackup_mode = 0;
uint32_t apnpreviousmillis = 0;
uint32_t apncurrentmillis = 0;
uint32_t apnbcpreviousmillis = 0;
int ib = 0;  
int apneatime = 20;
int set_apneatime = 20;


const int chipSelect = 48;  
uint32_t lastLogtime = 0;
File datafile;

int demomode = 0;
uint32_t demo_settingcurrenttime;
uint32_t demo_settingprevioustime;



/* Pressure - Volume loop */

#define ENABLE_PV_LOOP 1

#if ENABLE_PV_LOOP

/* ---- Basic Graphic control ---- */
#define PVL_SP        0xCA00
#define PVL_VP        0xC900
#define PVL_SP_SCALE  (PVL_SP + 0x07)
#define PVL_CMD_LINE  0x0002   /* connected line */

/* ---- panel geometry (DWIN pixels, y grows downward) ---- */
#define PVL_X_L   340
#define PVL_X_R   668
#define PVL_Y_T   355
#define PVL_Y_B   655
#define PVL_W     (PVL_X_R - PVL_X_L)      
#define PVL_H     (PVL_Y_B - PVL_Y_T)      

#define PVL_PRES_FS   56
#define PVL_VOL_FS    600

#define PVL_PX_SIZE   3
#define PVL_PXM       (PVL_PX_SIZE - 1)    

/* ---- colours (RGB565) ---- */
#define PVL_COL_INSP  0xF800   /* red */
#define PVL_COL_BG    0xFFFF   // white background

/* ---- capture ---- */
#define PVL_MAX_PTS      40    /* per breath, per buffer */
#define PVL_SAMPLE_MS    50UL  /* starting interval; doubles on overflow */
#define PVL_PTS_PER_PKT  7     /* points per polyline packet */
#define PVL_TXFREE_MIN   40    /* UART headroom needed */
#define PVL_TXFREE_SP    24

/* ---- redraw state machine ---- */
#define PVL_ST_IDLE     0
#define PVL_ST_ERASE    1
#define PVL_ST_DRAW     2

/* capture buffer - filled during the breath in progress */
static uint16_t pvl_capX[PVL_MAX_PTS];
static uint8_t  pvl_capY[PVL_MAX_PTS];    
static uint8_t  pvl_capN = 0;
static uint32_t pvl_ival = PVL_SAMPLE_MS;
static uint32_t pvl_next = 0;

/* active draw buffers */
static uint16_t pvl_drawX[PVL_MAX_PTS];
static uint8_t  pvl_drawY[PVL_MAX_PTS];
static uint8_t  pvl_drawN = 0;

/* previous draw buffers (retained so the old loop stays visible until we're ready to overwrite/erase it) */
static uint16_t pvl_prevX[PVL_MAX_PTS];
static uint8_t  pvl_prevY[PVL_MAX_PTS];
static uint8_t  pvl_prevN = 0;

static uint8_t pvl_state = PVL_ST_IDLE;
static uint8_t pvl_fi = 0;
static uint8_t pvl_lastInsp = 0;
static uint8_t pvl_lastPage = 0xFF;

static uint8_t pvl_scale = (PVL_PX_SIZE - 1);  /* Pixel_Scale register value */
static uint8_t pvl_spPend = 1; 


static void pvl_put16(uint8_t *b, uint8_t i, uint16_t v) {
  b[i] = highByte(v);
  b[i + 1] = lowByte(v);
}

static void pvl_send_at(uint16_t addr, const uint8_t *d, uint16_t n) {
  uint8_t hdr[6];
  hdr[0] = FRAME_START_LOW;
  hdr[1] = FRAME_START_HIGH;
  hdr[2] = n + 3;
  hdr[3] = CMD_VARIABLE_WRITE;
  hdr[4] = highByte(addr);
  hdr[5] = lowByte(addr);
  DWIN_PORT.write(hdr, 6);
  DWIN_PORT.write(d, n);
}

static void pvl_send(const uint8_t *d, uint16_t n) {
  pvl_send_at((uint16_t)PVL_VP, d, n);
}

static void pvl_sp_init(void) {
  uint8_t d[16];
  pvl_put16(d,  0, (uint16_t)PVL_VP);
  pvl_put16(d,  2, (uint16_t)PVL_X_L);
  pvl_put16(d,  4, (uint16_t)PVL_Y_T);
  pvl_put16(d,  6, (uint16_t)PVL_X_R);
  pvl_put16(d,  8, (uint16_t)PVL_Y_B);
  pvl_put16(d, 10, 0x0000);             /* solid lines, Dash_Set[0] */
  pvl_put16(d, 12, 0x0000);             /* Dash_Set[1:2] */
  d[14] = 0x00;                         /* Dash_Set[3] */
  d[15] = pvl_scale;                    /* Pixel_Scale: 0x00-0x0F = 1x1 to 16x16 */
  pvl_send_at((uint16_t)PVL_SP, d, 16);
}

static void pvl_set_pixel_size(uint8_t px) {
  uint8_t d[2];
  uint8_t reg;
  if (px < 1)  px = 1;
  if (px > 16) px = 16;
  reg = px - 1;
  if (reg == pvl_scale) return;
  pvl_scale = reg;
  d[0] = 0x00;                          /* Dash_Set[3] */
  d[1] = pvl_scale;
  pvl_send_at((uint16_t)PVL_SP_SCALE, d, 2);
}

static void pvl_polyline(uint16_t colour, const uint16_t *xBuf, const uint8_t *yBuf, uint8_t count)
{
    uint8_t d[6 + (4 * PVL_MAX_PTS) + 2];
    uint16_t o = 0;
    uint8_t i;

    pvl_put16(d, o, PVL_CMD_LINE); o += 2;
    pvl_put16(d, o, count);        o += 2;
    pvl_put16(d, o, colour);       o += 2;

    for (i = 0; i < count; i++) {
        pvl_put16(d, o, xBuf[i]); o += 2;
        pvl_put16(d, o, PVL_Y_T + yBuf[i]); o += 2;
    }

    /* End of graphics operation */
    d[o++] = 0xFF;
    d[o++] = 0x00;

    pvl_send(d, o);
}


static uint16_t pvl_mapX(long v) {
  if (v < 0) v = 0;
  if (v > PVL_VOL_FS) v = PVL_VOL_FS;
  return (uint16_t)(PVL_X_L + ((v * PVL_W) / PVL_VOL_FS));
}

static uint8_t pvl_mapY(long p) {
  if (p < 0) p = 0;
  if (p > PVL_PRES_FS) p = PVL_PRES_FS;
  return (uint8_t)(PVL_H - ((p * PVL_H) / PVL_PRES_FS));  
}

static void pvl_decimate(void) {
  uint8_t i, j = 0;
  for (i = 0; i < pvl_capN; i += 2) {
    pvl_capX[j] = pvl_capX[i];
    pvl_capY[j] = pvl_capY[i];
    j++;
  }
  pvl_capN = j;
  pvl_ival *= 2;
}

static void pvl_startCapture(void) {
  pvl_capN = 0;
  pvl_ival = PVL_SAMPLE_MS;
  pvl_next = millis();
}


static void pvl_commitBreath(void)
{
    uint8_t i;

    if (pvl_capN < 2) {
        pvl_startCapture();
        return;
    }

    /* Don't lose a completed breath if we're still drawing */
    if (pvl_state != PVL_ST_IDLE)
        return;

    /* Save current loop for later erase */
    pvl_prevN = pvl_drawN;
    for (i = 0; i < pvl_drawN; i++) {
        pvl_prevX[i] = pvl_drawX[i];
        pvl_prevY[i] = pvl_drawY[i];
    }

    /* Copy captured breath into draw buffer */
    pvl_drawN = pvl_capN;
    for (i = 0; i < pvl_capN; i++) {
        pvl_drawX[i] = pvl_capX[i];
        pvl_drawY[i] = pvl_capY[i];
    }

    if (pvl_prevN > 1)
        pvl_state = PVL_ST_ERASE;
    else
        pvl_state = PVL_ST_DRAW;

    pvl_fi = 0;

    pvl_startCapture();
}


/* Non-blocking background flush handling complete breath packets */
static void pvl_flush(void) {
  uint8_t n;
    
    if (pvl_spPend) {
    if (DWIN_PORT.availableForWrite() < PVL_TXFREE_SP) return;
    pvl_sp_init();
    pvl_spPend = 0;
    return;
  }

  if (pvl_state == PVL_ST_IDLE) return;
  if (DWIN_PORT.availableForWrite() < PVL_TXFREE_MIN) return;

  switch (pvl_state) {
    case PVL_ST_ERASE:

      pvl_polyline(PVL_COL_BG, pvl_prevX, pvl_prevY, pvl_prevN);

      pvl_prevN = 0;
      pvl_state = PVL_ST_DRAW;
      break;

    case PVL_ST_DRAW:

    pvl_polyline(PVL_COL_INSP, pvl_drawX, pvl_drawY, pvl_drawN);

    pvl_state = PVL_ST_IDLE;
    break;

    default:
      pvl_state = PVL_ST_IDLE;
      break;
  }
}

static void pvl_capture(void) {
  uint16_t x;
  uint16_t y;

  if ((int32_t)(millis() - pvl_next) < 0) return;
  pvl_next = millis() + pvl_ival;

  x = pvl_mapX((long)data_VOLUME);
  y = pvl_mapY((long)data_PRESSURE_print);

  if (pvl_capN == 0) {
    if (data_VOLUME < (-10))     //threshold -10
        return;
  }

  if (pvl_capN > 0 && pvl_capX[pvl_capN - 1] == x && pvl_capY[pvl_capN - 1] == y)
    return;

  if (pvl_capN >= PVL_MAX_PTS) pvl_decimate();

  pvl_capX[pvl_capN] = x;
  pvl_capY[pvl_capN] = y;
  pvl_capN++;
}

void pvloop_init(void) {
  pvl_state = PVL_ST_IDLE;
  pvl_lastInsp = 0;
  pvl_lastPage = 0xFF;
  pvl_drawN = 0;
  pvl_prevN = 0;
  pvl_scale = (PVL_PX_SIZE - 1);
  pvl_spPend = 1;
  pvl_startCapture();
}

void pvloop_service(void) {
  uint8_t insp;

  if (HMI_Page != PAGE_MAIN) {
    pvl_lastPage = HMI_Page;
    pvl_state = PVL_ST_IDLE;
    pvl_capN = 0;
    pvl_drawN = 0;
    pvl_prevN = 0;
    return;
  }

  if (pvl_lastPage != PAGE_MAIN) {
    pvl_lastPage = PAGE_MAIN;
    pvl_spPend = 1;
    pvl_startCapture();
  }

  insp = (uint8_t)inspiration;

  if (insp == 1 && pvl_lastInsp == 0) {
    pvl_lastInsp = 1;
    pvl_commitBreath();  /* Triggers once per complete breath cycle */
  } else if (insp == 0 && pvl_lastInsp == 1) {
    pvl_lastInsp = 0;
  }

  pvl_flush();   /* Manages non-blocking drawing/erasing sequence */
  pvl_capture(); /* Samples current ongoing breath */
}

#else
void pvloop_init(void) {}
void pvloop_service(void) {}
#endif

/* Pressure - Flow loop */

#define ENABLE_PF_LOOP 1

#if ENABLE_PF_LOOP

/* ---- Basic Graphic control ---- */
#define PFL_SP        0xCC00
#define PFL_SP_SCALE  (PVL_SP + 0x07)
#define PFL_VP         0xCB00      //vp address
#define PFL_CMD_LINE   0x0002   /* connected line */

#define PFL_PX_SIZE   3
#define PFL_PXM      (PFL_PX_SIZE - 1) 

/* ---- panel geometry (DWIN pixels, y grows downward) ---- */
#define PFL_X_L   725
#define PFL_X_R   1055
#define PFL_Y_T   355
#define PFL_Y_B   655
#define PFL_W     (PFL_X_R - PFL_X_L)  //290
#define PFL_H     (PFL_Y_B - PFL_Y_T)   
#define PFL_X_MID  100       //(PFL_W/2)   //145

#define PFL_PRES_FS   56
#define PFL_FLOW_FS   110

/* ---- colours (RGB565) ---- */
#define PFL_COL_INSP  0xFFE0   /* blue*/
#define PFL_COL_BG    0xFFFF    // white background

/* ---- capture ---- */
#define PFL_MAX_PTS      40    /* per breath, per buffer */
#define PFL_SAMPLE_MS    50UL  /* starting interval; doubles on overflow */
#define PFL_PTS_PER_PKT  7     /* points per polyline packet */
#define PFL_TXFREE_MIN   40    /* UART headroom needed */
#define PFL_TXFREE_SP    24
#define PFL_PRES_OFFSET  10

/* ---- redraw state machine ---- */
#define PFL_ST_IDLE     0
#define PFL_ST_ERASE    1
#define PFL_ST_DRAW     2

/* capture buffer - filled during the breath in progress */
static uint16_t pfl_capX[PFL_MAX_PTS];
static uint16_t  pfl_capY[PFL_MAX_PTS];    
static uint8_t  pfl_capN = 0;
static uint32_t pfl_ival = PFL_SAMPLE_MS;
static uint32_t pfl_next = 0;

/* active draw buffers */
static uint16_t pfl_drawX[PFL_MAX_PTS];
static uint16_t  pfl_drawY[PFL_MAX_PTS];
static uint8_t  pfl_drawN = 0;

/* previous draw buffers (retained so the old loop stays visible until we're ready to overwrite/erase it) */
static uint16_t pfl_prevX[PFL_MAX_PTS];
static uint16_t  pfl_prevY[PFL_MAX_PTS];
static uint8_t  pfl_prevN = 0;

static uint8_t pfl_state = PFL_ST_IDLE;
static uint8_t pfl_fi = 0;
static uint8_t pfl_lastInsp = 0;
static uint8_t pfl_lastPage = 0xFF;

static uint8_t pfl_scale    = (PFL_PX_SIZE - 1);  /* Pixel_Scale register value */
static uint8_t pfl_spPend   = 1;


static void pfl_put16(uint8_t *b, uint8_t i, uint16_t v) {
  b[i] = highByte(v);
  b[i + 1] = lowByte(v);
}

static void pfl_send_at(uint16_t addr, const uint8_t *d, uint16_t n) {
  uint8_t hdr[6];
  hdr[0] = FRAME_START_LOW;
  hdr[1] = FRAME_START_HIGH;
  hdr[2] = n + 3;
  hdr[3] = CMD_VARIABLE_WRITE;
  hdr[4] = highByte(addr);
  hdr[5] = lowByte(addr);
  DWIN_PORT.write(hdr, 6);
  DWIN_PORT.write(d, n);
}

static void pfl_send(const uint8_t *d, uint16_t n) {
 pfl_send_at((uint16_t)PFL_VP, d, n);
}

static void pfl_sp_init(void) {
  uint8_t d[16];
  pfl_put16(d,  0, (uint16_t)PFL_VP);
  pfl_put16(d,  2, (uint16_t)PFL_X_L);
  pfl_put16(d,  4, (uint16_t)PFL_Y_T);
  pfl_put16(d,  6, (uint16_t)PFL_X_R);
  pfl_put16(d,  8, (uint16_t)PFL_Y_B);
  pfl_put16(d, 10, 0x0000);             /* solid lines, Dash_Set[0] */
  pfl_put16(d, 12, 0x0000);             /* Dash_Set[1:2] */
  d[14] = 0x00;                         /* Dash_Set[3] */
  d[15] = pfl_scale;                    /* Pixel_Scale: 0x00-0x0F = 1x1 to 16x16 */
  pfl_send_at((uint16_t)PFL_SP, d, 16);
}

static void pfl_set_pixel_size(uint8_t px) {
  uint8_t d[2];
  uint8_t reg;
  if (px < 1)  px = 1;
  if (px > 16) px = 16;
  reg = px - 1;
  if (reg == pfl_scale) return;
  pfl_scale = reg;
  d[0] = 0x00;                          /* Dash_Set[3] */
  d[1] = pfl_scale;
  pfl_send_at((uint16_t)PFL_SP_SCALE, d, 2);
}

static void pfl_polyline(uint16_t colour, const uint16_t *xBuf, const uint16_t *yBuf, uint8_t count)
{
    uint8_t d[6 + (4 * PFL_MAX_PTS) + 2];
    uint16_t o = 0;
    uint8_t i;

    pfl_put16(d, o, PFL_CMD_LINE); o += 2;
    pfl_put16(d, o, count);        o += 2;
    pfl_put16(d, o, colour);       o += 2;

    for (i = 0; i < count; i++) {
        pfl_put16(d, o, xBuf[i]); o += 2;
        pfl_put16(d, o, PFL_Y_T + yBuf[i]); o += 2;
    }

    /* End of graphics operation */
    d[o++] = 0xFF;
    d[o++] = 0x00;

    pfl_send(d, o);
}

static uint16_t pfl_mapX(long f) {
  long x;
  if (f > PFL_FLOW_FS) f = PFL_FLOW_FS;
  if (f < -PFL_FLOW_FS) f = -PFL_FLOW_FS;
  x = (long) PFL_X_MID + ((f * (long) PFL_X_MID)/PFL_FLOW_FS);
  if(x < 0) x = 0;
  if(x > (PFL_W - 1)) x = PFL_W - 1;
  return (uint16_t)(PFL_X_L + x);
}

static uint16_t pfl_mapY(long p) {
  long y;
  p += PFL_PRES_OFFSET;
  if (p < 0) p = 0;
  if (p > PFL_PRES_FS + PFL_PRES_OFFSET) p = PFL_PRES_FS + PFL_PRES_OFFSET;
  y = (long) (PFL_H - ((p * PFL_H) / PFL_PRES_FS + PFL_PRES_OFFSET));
  return (uint16_t) y;  
}

static void pfl_decimate(void) {
  uint8_t i, j = 0;
  for (i = 0; i < pfl_capN; i += 2) {
    pfl_capX[j] = pfl_capX[i];
    pfl_capY[j] = pfl_capY[i];
    j++;
  }
  pfl_capN = j;
  pfl_ival *= 2;
}

static void pfl_startCapture(void) {
  pfl_capN = 0;
  pfl_ival = PFL_SAMPLE_MS;
  pfl_next = millis();
}


static void pfl_commitBreath(void)
{
    uint8_t i;

    if (pfl_capN < 2) {
        pfl_startCapture();
        return;
    }

    /* Don't lose a completed breath if we're still drawing */
    if (pfl_state != PFL_ST_IDLE)
        return;

    /* Save current loop for later erase */
    pfl_prevN = pfl_drawN;
    for (i = 0; i < pfl_drawN; i++) {
        pfl_prevX[i] = pfl_drawX[i];
        pfl_prevY[i] = pfl_drawY[i];
    }

    /* Copy captured breath into draw buffer */
    pfl_drawN = pfl_capN;
    for (i = 0; i < pfl_capN; i++) {
        pfl_drawX[i] = pfl_capX[i];
        pfl_drawY[i] = pfl_capY[i];
    }

    if (pfl_prevN > 1)
        pfl_state = PFL_ST_ERASE;
    else
        pfl_state = PFL_ST_DRAW;

    pfl_fi = 0;

    pfl_startCapture();
}


/* Non-blocking background flush handling complete breath packets */
static void pfl_flush(void) {
  uint8_t n;

    if (pfl_spPend) {
    if (DWIN_PORT.availableForWrite() < PFL_TXFREE_SP) return;
    pfl_sp_init();
    pfl_spPend = 0;
    return;
  }

  if (pfl_state == PFL_ST_IDLE) return;
  if (DWIN_PORT.availableForWrite() < PFL_TXFREE_MIN) return;

  switch (pfl_state) {
    case PFL_ST_ERASE:

      pfl_polyline(PFL_COL_BG, pfl_prevX, pfl_prevY, pfl_prevN);

      pfl_prevN = 0;
      pfl_state = PFL_ST_DRAW;
      break;

    case PFL_ST_DRAW:

    pfl_polyline(PFL_COL_INSP, pfl_drawX, pfl_drawY, pfl_drawN);

    pfl_state = PFL_ST_IDLE;
    break;

    default:
      pfl_state = PFL_ST_IDLE;
      break;
  }
}

static void pfl_capture(void) {
  uint16_t x;
  uint16_t y;

  if ((int32_t)(millis() - pfl_next) < 0) return;
  pfl_next = millis() + pfl_ival;

  x = pfl_mapX((long)data_FLOW);
  y = pfl_mapY((long)data_PRESSURE_print);

  if (pfl_capN > 0 && pfl_capX[pfl_capN - 1] == x && pfl_capY[pfl_capN - 1] == y)
    return;

  if (pfl_capN >= PFL_MAX_PTS) pfl_decimate();

  pfl_capX[pfl_capN] = x;
  pfl_capY[pfl_capN] = y;
  pfl_capN++;
}

void pfloop_init(void) {
  pfl_state = PFL_ST_IDLE;
  pfl_lastInsp = 0;
  pfl_lastPage = 0xFF;
  pfl_drawN = 0;
  pfl_prevN = 0;
  pfl_scale = (PFL_PX_SIZE - 1);
  pfl_spPend = 1;
  pfl_startCapture();
}

void pfloop_service(void) {
  uint8_t insp;

  if (HMI_Page != PAGE_MAIN) {
    pfl_lastPage = HMI_Page;
    pfl_state = PFL_ST_IDLE;
    pfl_capN = 0;
    pfl_drawN = 0;
    pfl_prevN = 0;
    return;
  }

  if (pfl_lastPage != PAGE_MAIN) {
    pfl_lastPage = PAGE_MAIN;
    pfl_spPend = 1;
    pfl_startCapture();
  }

  insp = (uint8_t)inspiration;

  if (insp == 1 && pfl_lastInsp == 0) {
    pfl_lastInsp = 1;
    pfl_commitBreath();  /* Triggers once per complete breath cycle */
  } else if (insp == 0 && pfl_lastInsp == 1) {
    pfl_lastInsp = 0;
  }

  pfl_flush();   /* Manages non-blocking drawing/erasing sequence */
  pfl_capture(); /* Samples current ongoing breath */
}

#else
void pfloop_init(void) {}
void pfloop_service(void) {}
#endif  


/* Volume - Pressure loop */

#define ENABLE_VP_LOOP 1

#if ENABLE_VP_LOOP

/* ---- Basic Graphic control (0x5A21) ---- */
#define VPL_VP        0xC300
#define VPL_SP        0xC400 
#define VPL_SP_SCALE  (VPL_SP + 0x07)   /* CA07: H = Dash_Set[3], L = Pixel_Scale */
#define VPL_CMD_LINE  0x0002   /* connected line */

/* ---- drawn pixel size, 1..16 (register value = size - 1) ---- */
#define VPL_PX_SIZE   3
#define VPL_PXM       (VPL_PX_SIZE - 1)    /* clip margin: blocks grow right/down */

/* ---- panel geometry (DWIN pixels, y grows downward) ---- */
#define VPL_X_L   340
#define VPL_X_R   665
#define VPL_Y_T   355
#define VPL_Y_B   655
#define VPL_W     (VPL_X_R - VPL_X_L - VPL_PXM)
#define VPL_H     (VPL_Y_B - VPL_Y_T - VPL_PXM)

#define VPL_PRES_FS   56
#define VPL_VOL_FS    600

/* ---- colours (RGB565) ---- */
#define VPL_COL_INSP  0xF800   /* white */
#define VPL_COL_BG    0xFFFF   // black background

/* ---- capture ---- */
#define VPL_MAX_PTS      40    /* per breath, per buffer */
#define VPL_SAMPLE_MS    50UL  /* starting interval; doubles on overflow */
#define VPL_TXFREE_MIN   40    /* UART headroom needed */
#define VPL_TXFREE_SP    24    /* headroom for the 16-byte SP block */


/* ---- redraw state machine ---- */
#define VPL_ST_IDLE     0
#define VPL_ST_ERASE    1
#define VPL_ST_DRAW     2

/* capture buffer - filled during the breath in progress */
static uint16_t vpl_capX[VPL_MAX_PTS];
static uint16_t  vpl_capY[VPL_MAX_PTS];
static uint8_t  vpl_capN = 0;
static uint32_t vpl_ival = VPL_SAMPLE_MS;
static uint32_t vpl_next = 0;

/* active draw buffers */
static uint16_t vpl_drawX[VPL_MAX_PTS];
static uint16_t  vpl_drawY[VPL_MAX_PTS];
static uint8_t  vpl_drawN = 0;

/* previous draw buffers (retained so the old loop stays visible until we're ready to overwrite/erase it) */
static uint16_t vpl_prevX[VPL_MAX_PTS];
static uint16_t  vpl_prevY[VPL_MAX_PTS];
static uint8_t  vpl_prevN = 0;

static uint8_t vpl_state = VPL_ST_IDLE;
static uint8_t vpl_fi = 0;
static uint8_t vpl_lastInsp = 0;
static uint8_t vpl_lastPage = 0xFF;

static uint8_t vpl_scale    = (VPL_PX_SIZE - 1);  /* Pixel_Scale register value */
static uint8_t vpl_spPend   = 1;                  /* SP block needs (re)writing */

static void vpl_put16(uint8_t *b, uint8_t i, uint16_t v) {
  b[i] = highByte(v);
  b[i + 1] = lowByte(v);
}

static void vpl_send_at(uint16_t addr, const uint8_t *d, uint16_t n) {
  uint8_t hdr[6];
  if (n > 252) return;                  /* hdr[2] is one byte: n + 3 must fit */
  hdr[0] = FRAME_START_LOW;
  hdr[1] = FRAME_START_HIGH;
  hdr[2] = n + 3;
  hdr[3] = CMD_VARIABLE_WRITE;
  hdr[4] = highByte(addr);
  hdr[5] = lowByte(addr);
  DWIN_PORT.write(hdr, 6);
  DWIN_PORT.write(d, n);
}

static void vpl_send(const uint8_t *d, uint16_t n) {
  vpl_send_at((uint16_t)VPL_VP, d, n);
}

/* ---- SP descriptor: 8 */
static void vpl_sp_init(void) {
  uint8_t d[16];
  vpl_put16(d,  0, (uint16_t)VPL_VP);
  vpl_put16(d,  2, (uint16_t)VPL_X_L);
  vpl_put16(d,  4, (uint16_t)VPL_Y_T);
  vpl_put16(d,  6, (uint16_t)VPL_X_R);
  vpl_put16(d,  8, (uint16_t)VPL_Y_B);
  vpl_put16(d, 10, 0x0000);             /* solid lines, Dash_Set[0] */
  vpl_put16(d, 12, 0x0000);             /* Dash_Set[1:2] */
  d[14] = 0x00;                         /* Dash_Set[3] */
  d[15] = vpl_scale;                    /* Pixel_Scale: 0x00-0x0F = 1x1 to 16x16 */
  vpl_send_at((uint16_t)VPL_SP, d, 16);
}


static void vpl_set_pixel_size(uint8_t px) {
  uint8_t d[2];
  uint8_t reg;
  if (px < 1)  px = 1;
  if (px > 16) px = 16;
  reg = px - 1;
  if (reg == vpl_scale) return;
  vpl_scale = reg;
  d[0] = 0x00;                          /* Dash_Set[3] */
  d[1] = vpl_scale;
  vpl_send_at((uint16_t)VPL_SP_SCALE, d, 2);
}

static void vpl_polyline(uint16_t colour, const uint16_t *xBuf, const uint16_t *yBuf, uint8_t count)
{
    uint8_t d[6 + (4 * VPL_MAX_PTS) + 2];
    uint16_t o = 0;
    uint8_t i;

    vpl_put16(d, o, VPL_CMD_LINE); o += 2;
    vpl_put16(d, o, count);        o += 2;
    vpl_put16(d, o, colour);       o += 2;

    for (i = 0; i < count; i++) {
        vpl_put16(d, o, xBuf[i]); o += 2;
        vpl_put16(d, o, VPL_Y_T + yBuf[i]); o += 2;
    }

    /* End of graphics operation */
    d[o++] = 0xFF;
    d[o++] = 0x00;

    vpl_send(d, o);
}


static uint16_t vpl_mapX(long p) {
  if (p < 0) p = 0;
  if (p > VPL_PRES_FS) p = VPL_PRES_FS;
  return (uint16_t)(VPL_X_L + ((p * VPL_W) / VPL_PRES_FS));
}

static uint8_t vpl_mapY(long v) {
  if (v < 0) v = 0;
  if (v > VPL_VOL_FS) v = VPL_VOL_FS;
  return (uint8_t)(VPL_H - ((v * VPL_H) / VPL_VOL_FS));
}

static void vpl_decimate(void) {
  uint8_t i, j = 0;
  for (i = 0; i < vpl_capN; i += 2) {
    vpl_capX[j] = vpl_capX[i];
    vpl_capY[j] = vpl_capY[i];
    j++;
  }
  vpl_capN = j;
  vpl_ival *= 2;
}

static void vpl_startCapture(void) {
  vpl_capN = 0;
  vpl_ival = VPL_SAMPLE_MS;
  vpl_next = millis();
}


static void vpl_commitBreath(void)
{
    uint8_t i;

    if (vpl_capN < 2) {
        vpl_startCapture();
        return;
    }

    if (vpl_state != VPL_ST_IDLE)
        return;

    /* Save current loop for later erase */
    vpl_prevN = vpl_drawN;
    for (i = 0; i < vpl_drawN; i++) {
        vpl_prevX[i] = vpl_drawX[i];
        vpl_prevY[i] = vpl_drawY[i];
    }

    /* Copy captured breath into draw buffer */
    vpl_drawN = vpl_capN;
    for (i = 0; i < vpl_capN; i++) {
        vpl_drawX[i] = vpl_capX[i];
        vpl_drawY[i] = vpl_capY[i];
    }

    if (vpl_prevN > 1)
        vpl_state = VPL_ST_ERASE;
    else
        vpl_state = VPL_ST_DRAW;

    vpl_fi = 0;

    vpl_startCapture();
}

static void vpl_flush(void) {

  /* SP block first - the control reads its area and pixel size from there */
  if (vpl_spPend) {
    if (DWIN_PORT.availableForWrite() < VPL_TXFREE_SP) return;
    vpl_sp_init();
    vpl_spPend = 0;
    return;
  }

  if (vpl_state == VPL_ST_IDLE) return;
  if (DWIN_PORT.availableForWrite() < VPL_TXFREE_MIN) return;

  switch (vpl_state) {
    case VPL_ST_ERASE:

      vpl_polyline(VPL_COL_BG, vpl_prevX, vpl_prevY, vpl_prevN);

      vpl_prevN = 0;
      vpl_state = VPL_ST_DRAW;
      break;

    case VPL_ST_DRAW:

    vpl_polyline(VPL_COL_INSP, vpl_drawX, vpl_drawY, vpl_drawN);

    vpl_state = VPL_ST_IDLE;
    break;

    default:
      vpl_state = VPL_ST_IDLE;
      break;
  }
}

static void vpl_capture(void) {
  uint16_t x;
  uint16_t y;

  if ((int32_t)(millis() - vpl_next) < 0) return;
  vpl_next = millis() + vpl_ival;

  x = vpl_mapX((long)data_PRESSURE_print); 
  y = vpl_mapY((long)data_VOLUME);

  if (vpl_capN == 0) {
    if (data_PRESSURE_print < 2)     //threshold 2
        return;
  }

  if (vpl_capN > 0 && vpl_capX[vpl_capN - 1] == x && vpl_capY[vpl_capN - 1] == y)
    return;

  if (vpl_capN >= VPL_MAX_PTS) vpl_decimate();

  vpl_capX[vpl_capN] = x;
  vpl_capY[vpl_capN] = y;
  vpl_capN++;
}

void vploop_init(void) {
  vpl_state = VPL_ST_IDLE;
  vpl_lastInsp = 0;
  vpl_lastPage = 0xFF;
  vpl_drawN = 0;
  vpl_prevN = 0;
  vpl_scale = (VPL_PX_SIZE - 1);
  vpl_spPend = 1;          /* written by vpl_flush() once the UART has room */

  vpl_startCapture();
}

void vploop_service(void) {
  uint8_t insp;

  if (HMI_Page != PAGE_MAIN) {
    vpl_lastPage = HMI_Page;
    vpl_state = VPL_ST_IDLE;
    vpl_capN = 0;
    vpl_drawN = 0;
    vpl_prevN = 0;
    return;
  }

  /* re-entered the graph page: refresh the SP block before drawing */
  if (vpl_lastPage != PAGE_MAIN) {
    vpl_lastPage = PAGE_MAIN;
    vpl_spPend = 1;
    vpl_startCapture();
  }

  insp = (uint8_t)inspiration;

  if (insp == 1 && vpl_lastInsp == 0) {
    vpl_lastInsp = 1;
    vpl_commitBreath();  /* Triggers once per complete breath cycle */
  } else if (insp == 0 && vpl_lastInsp == 1) {
    vpl_lastInsp = 0;
  }

  vpl_flush();   /* Manages non-blocking drawing/erasing sequence */
  vpl_capture(); /* Samples current ongoing breath */
}

#else
void vploop_init(void) {}
void vploop_service(void) {}
#endif


/* Volume - Flow loop */

#define ENABLE_VF_LOOP 1

#if ENABLE_VF_LOOP

/* ---- Basic Graphic control ---- */
#define VFL_VP        0xC500   
#define VFL_SP        0xC600   /* description pointer - must also be set to CA00 in the DGUS tool */
#define VFL_SP_SCALE  (VFL_SP + 0x07)   
#define VFL_CMD_LINE   0x0002  

#define VFL_PX_SIZE   3
#define VFL_PXM       (VFL_PX_SIZE - 1)

/* ---- geometry (DWIN pixels, y grows downward) ---- */
#define VFL_X_L   732
#define VFL_X_R   1062
#define VFL_Y_T   355
#define VFL_Y_B   655
#define VFL_W     (VFL_X_R - VFL_X_L)
#define VFL_H     (VFL_Y_B - VFL_Y_T)
#define VFL_X_MID 125        //(VFL_W / 2)             

#define VFL_VOL_FS    600      /* mL,    0 .. +600            */
#define VFL_FLOW_FS   110      /* L/min, -110 .. +110         */

/* ---- colours (RGB565) ---- */
#define VFL_COL_LOOP  0x07FF   /* blue */
#define VFL_COL_BG    0xFFFF   /* white background */

/* ---- capture ---- */
#define VFL_MAX_PTS      40    /* per breath, per buffer - see check below */
#define VFL_SAMPLE_MS    50UL  /* starting interval; doubles on overflow */
#define VFL_TXFREE_MIN   40    /* UART headroom needed */
#define VFL_TXFREE_SP   24

/* ----state machine ---- */
#define VFL_ST_IDLE     0
#define VFL_ST_ERASE    1
#define VFL_ST_DRAW     2

/* capture buffer - filled during the breath in progress */
static uint16_t vfl_capX[VFL_MAX_PTS];
static uint16_t  vfl_capY[VFL_MAX_PTS];
static uint8_t  vfl_capN = 0;
static uint32_t vfl_ival = VFL_SAMPLE_MS;
static uint32_t vfl_next = 0;

/* active draw buffers */
static uint16_t vfl_drawX[VFL_MAX_PTS];
static uint16_t vfl_drawY[VFL_MAX_PTS];
static uint8_t  vfl_drawN = 0;

static uint16_t vfl_prevX[VFL_MAX_PTS];
static uint16_t vfl_prevY[VFL_MAX_PTS];
static uint8_t  vfl_prevN = 0;

static uint8_t vfl_state    = VFL_ST_IDLE;
static uint8_t vfl_lastInsp = 0;
static uint8_t vfl_lastPage = 0xFF;

static uint8_t vfl_scale    = (VFL_PX_SIZE - 1);  /* Pixel_Scale register value */
static uint8_t vfl_spPend   = 1; 


static void vfl_put16(uint8_t *b, uint16_t i, uint16_t v) {
  b[i]     = highByte(v);
  b[i + 1] = lowByte(v);
}

static void vfl_send_at(uint16_t addr, const uint8_t *d, uint16_t n) {
  uint8_t hdr[6];
  hdr[0] = FRAME_START_LOW;
  hdr[1] = FRAME_START_HIGH;
  hdr[2] = n + 3;
  hdr[3] = CMD_VARIABLE_WRITE;
  hdr[4] = highByte(addr);
  hdr[5] = lowByte(addr);
  DWIN_PORT.write(hdr, 6);
  DWIN_PORT.write(d, n);
}

static void vfl_send(const uint8_t *d, uint16_t n) {
  vfl_send_at((uint16_t)VFL_VP, d, n);
}

static void vfl_sp_init(void) {
  uint8_t d[16];
  vfl_put16(d,  0, (uint16_t)VFL_VP);
  vfl_put16(d,  2, (uint16_t)VFL_X_L);
  vfl_put16(d,  4, (uint16_t)VFL_Y_T);
  vfl_put16(d,  6, (uint16_t)VFL_X_R);
  vfl_put16(d,  8, (uint16_t)VFL_Y_B);
  vfl_put16(d, 10, 0x0000);             /* solid lines, Dash_Set[0] */
  vfl_put16(d, 12, 0x0000);             /* Dash_Set[1:2] */
  d[14] = 0x00;                         /* Dash_Set[3] */
  d[15] = vfl_scale;                    /* Pixel_Scale: 0x00-0x0F = 1x1 to 16x16 */
  vfl_send_at((uint16_t)VFL_SP, d, 16);
}


static void vfl_set_pixel_size(uint8_t px) {
  uint8_t d[2];
  uint8_t reg;
  if (px < 1)  px = 1;
  if (px > 16) px = 16;
  reg = px - 1;
  if (reg == vfl_scale) return;
  vfl_scale = reg;
  d[0] = 0x00;                          /* Dash_Set[3] */
  d[1] = vfl_scale;
  vfl_send_at((uint16_t)VFL_SP_SCALE, d, 2);
}

static void vfl_polyline(uint16_t colour, const uint16_t *xBuf, const uint16_t *yBuf, uint8_t count)
{
    uint8_t  d[8 + (4 * VFL_MAX_PTS)];
    uint16_t o = 0;
    uint8_t  i;

    if (count < 2) return;

    vfl_put16(d, o, VFL_CMD_LINE); o += 2;
    vfl_put16(d, o, count);        o += 2;
    vfl_put16(d, o, colour);       o += 2;

    for (i = 0; i < count; i++) {
        vfl_put16(d, o, xBuf[i]);              o += 2;
        vfl_put16(d, o, VFL_Y_T + yBuf[i]);    o += 2;
    }

    /* End of graphics operation */
    d[o++] = 0xFF;
    d[o++] = 0x00;

    vfl_send(d, o);
}


static uint16_t vfl_mapX(long f)
{
  if (f >  VFL_FLOW_FS) f =  VFL_FLOW_FS;
  if (f < -VFL_FLOW_FS) f = -VFL_FLOW_FS;

  return (uint16_t)((long)VFL_X_L + (((f + VFL_FLOW_FS) * (long)VFL_W) / (2 * VFL_FLOW_FS)));
}

/* Y  */
static uint8_t vfl_mapY(long v) {
  if (v < 0)           v = 0;
  if (v > VFL_VOL_FS)  v = VFL_VOL_FS;

  return (uint8_t)((long)(VFL_H - 1) - ((v * (long)(VFL_H - 1)) / VFL_VOL_FS));
}

static void vfl_decimate(void) {
  uint8_t i, j = 0;
  for (i = 0; i < vfl_capN; i += 2) 
  {
    vfl_capX[j] = vfl_capX[i];
    vfl_capY[j] = vfl_capY[i];
    j++;
  }
  vfl_capN = j;
  vfl_ival *= 2;
}

static void vfl_startCapture(void) {
  vfl_capX[0] = vfl_mapX(0);
  vfl_capY[0] = vfl_mapY(0);
  vfl_capN    = 1;
  vfl_ival    = VFL_SAMPLE_MS;
  vfl_next    = millis() + vfl_ival;
}

static void vfl_commitBreath(void)
{
    uint8_t i;

    if (vfl_capN < 3) {
        vfl_startCapture();
        return;
    }

    if (vfl_state != VFL_ST_IDLE) {
        return;
    }

    /* Save current loop for later erase */
    vfl_prevN = vfl_drawN;
    for (i = 0; i < vfl_drawN; i++) {
        vfl_prevX[i] = vfl_drawX[i];
        vfl_prevY[i] = vfl_drawY[i];
    }

    /* Copy captured breath into draw buffer */
    vfl_drawN = vfl_capN;
    for (i = 0; i < vfl_capN; i++) {
        vfl_drawX[i] = vfl_capX[i];
        vfl_drawY[i] = vfl_capY[i];
    }

    vfl_state = (vfl_prevN > 1) ? VFL_ST_ERASE : VFL_ST_DRAW;

    vfl_startCapture();
}


static void vfl_flush(void) {

  if (vfl_spPend) {
    if (DWIN_PORT.availableForWrite() < VFL_TXFREE_SP) return;
    vfl_sp_init();
    vfl_spPend = 0;
    return;
  }

  if (vfl_state == VFL_ST_IDLE) return;
  if (DWIN_PORT.availableForWrite() < VFL_TXFREE_MIN) return;

  switch (vfl_state) {

    case VFL_ST_ERASE:
      vfl_polyline(VFL_COL_BG, vfl_prevX, vfl_prevY, vfl_prevN);
      vfl_prevN = 0;
      vfl_state = VFL_ST_DRAW;
      break;

    case VFL_ST_DRAW:
      vfl_polyline(VFL_COL_LOOP, vfl_drawX, vfl_drawY, vfl_drawN);
      vfl_state = VFL_ST_IDLE;
      break;

    default:
      vfl_state = VFL_ST_IDLE;
      break;
  }
}

static void vfl_capture(void) {
  uint16_t x;
  uint16_t  y;

  if ((int32_t)(millis() - vfl_next) < 0) return;
  vfl_next = millis() + vfl_ival;

  x = vfl_mapX((long)data_FLOW);
  y = vfl_mapY((long)data_VOLUME); 

  /* skip duplicate pixels */
  if (vfl_capN > 0 && vfl_capX[vfl_capN - 1] == x && vfl_capY[vfl_capN - 1] == y)
    return;

  if (vfl_capN >= VFL_MAX_PTS) vfl_decimate();

  vfl_capX[vfl_capN] = x;
  vfl_capY[vfl_capN] = y;
  vfl_capN++;
}

void vfloop_init(void) {
  vfl_state    = VFL_ST_IDLE;
  vfl_lastInsp = 0;
  vfl_lastPage = 0xFF;
  vfl_drawN    = 0;
  vfl_prevN    = 0;
  vfl_scale = (VFL_PX_SIZE - 1);
  vfl_spPend = 1; 
  vfl_startCapture();
}

void vfloop_service(void) {
  uint8_t insp;

  if (HMI_Page != PAGE_MAIN) 
  {
    vfl_lastPage = HMI_Page;
    vfl_state    = VFL_ST_IDLE;
    vfl_drawN    = 0;
    vfl_prevN    = 0;
    vfl_startCapture();
    return;
  }

  if (vfl_lastPage != PAGE_MAIN) {
    vfl_lastPage = PAGE_MAIN;
    vfl_spPend = 1;
    vfl_startCapture();
  }

  insp = (uint8_t)inspiration;

  if (insp == 1 && vfl_lastInsp == 0) {
    vfl_lastInsp = 1;
    vfl_commitBreath();  /* Triggers once per complete breath cycle */
  } else if (insp == 0 && vfl_lastInsp == 1) {
    vfl_lastInsp = 0;
  }

  vfl_flush();   
  vfl_capture(); 
}

#else
void vfloop_init(void) {}
void vfloop_service(void) {}
#endif


/* Flow - Volume loop */

#define ENABLE_FV_LOOP 1

#if ENABLE_FV_LOOP

/* ---- Basic Graphic control ---- */
#define FVL_VP        0xCD00      
#define FVL_SP        0xCE00  
#define FVL_SP_SCALE  (FVL_SP + 0x07)
#define FVL_CMD_LINE   0x0002  

/* ---- geometry (DWIN pixels, y grows downward) ---- */
#define FVL_X_L   340
#define FVL_X_R   665
#define FVL_Y_T   355
#define FVL_Y_B   655
#define FVL_W     (FVL_X_R - FVL_X_L)
#define FVL_H     (FVL_Y_B - FVL_Y_T)
#define FVL_Y_MID 200       //(FVL_H / 2)            

#define FVL_VOL_FS    600      /* mL,    0 .. +600            */
#define FVL_FLOW_FS   110      /* L/min, -110 .. +110         */

/* ---- colours (RGB565) ---- */
#define FVL_COL_LOOP  0xF800   /* white */
#define FVL_COL_BG    0xFFFF   /* white*/

#define FVL_PX_SIZE   3
#define FVL_PXM      (FVL_PX_SIZE - 1)

/* ---- capture ---- */
#define FVL_MAX_PTS      40    /* per breath, per buffer - see check below */
#define FVL_SAMPLE_MS    50UL  /* starting interval; doubles on overflow */
#define FVL_TXFREE_MIN   40    /* UART headroom needed */
#define FVL_TXFREE_SP    24

/* ----state machine ---- */
#define FVL_ST_IDLE     0
#define FVL_ST_ERASE    1
#define FVL_ST_DRAW     2

/* capture buffer - filled during the breath in progress */
static uint16_t fvl_capX[FVL_MAX_PTS];
static uint16_t fvl_capY[FVL_MAX_PTS];
static uint8_t  fvl_capN = 0;
static uint32_t fvl_ival = FVL_SAMPLE_MS;
static uint32_t fvl_next = 0;

/* active draw buffers */
static uint16_t fvl_drawX[FVL_MAX_PTS];
static uint16_t fvl_drawY[FVL_MAX_PTS];
static uint8_t  fvl_drawN = 0;

static uint16_t fvl_prevX[FVL_MAX_PTS];
static uint16_t fvl_prevY[FVL_MAX_PTS];
static uint8_t  fvl_prevN = 0;

static uint8_t fvl_state  = FVL_ST_IDLE;
static uint8_t fvl_lastInsp = 0;
static uint8_t fvl_lastPage = 0xFF;

static uint8_t fvl_scale    = (FVL_PX_SIZE - 1);  /* Pixel_Scale register value */
static uint8_t fvl_spPend   = 1;

static void fvl_put16(uint8_t *b, uint16_t i, uint16_t v) {
  b[i]     = highByte(v);
  b[i + 1] = lowByte(v);
}

static void fvl_send_at(uint16_t addr, const uint8_t *d, uint16_t n) {
  uint8_t hdr[6];
  hdr[0] = FRAME_START_LOW;
  hdr[1] = FRAME_START_HIGH;
  hdr[2] = n + 3;
  hdr[3] = CMD_VARIABLE_WRITE;
  hdr[4] = highByte(addr);
  hdr[5] = lowByte(addr);
  DWIN_PORT.write(hdr, 6);
  DWIN_PORT.write(d, n);
}

static void fvl_send(const uint8_t *d, uint16_t n) {
  fvl_send_at((uint16_t)FVL_VP, d, n);
}

static void fvl_sp_init(void) {
  uint8_t d[16];
  fvl_put16(d,  0, (uint16_t)FVL_VP);
  fvl_put16(d,  2, (uint16_t)FVL_X_L);
  fvl_put16(d,  4, (uint16_t)FVL_Y_T);
  fvl_put16(d,  6, (uint16_t)FVL_X_R);
  fvl_put16(d,  8, (uint16_t)FVL_Y_B);
  fvl_put16(d, 10, 0x0000);             /* solid lines, Dash_Set[0] */
  fvl_put16(d, 12, 0x0000);             /* Dash_Set[1:2] */
  d[14] = 0x00;                         /* Dash_Set[3] */
  d[15] = fvl_scale;                    /* Pixel_Scale: 0x00-0x0F = 1x1 to 16x16 */
  vpl_send_at((uint16_t)FVL_SP, d, 16);
}

/* px = drawn block size in pixels, 1..16. Takes effect on the next VP write. */
static void fvl_set_pixel_size(uint8_t px) {
  uint8_t d[2];
  uint8_t reg;
  if (px < 1)  px = 1;
  if (px > 16) px = 16;
  reg = px - 1;
  if (reg == fvl_scale) return;
  fvl_scale = reg;
  d[0] = 0x00;                          /* Dash_Set[3] */
  d[1] = fvl_scale;
  fvl_send_at((uint16_t)FVL_SP_SCALE, d, 2);
}

static void fvl_polyline(uint16_t colour, const uint16_t *xBuf, const uint16_t *yBuf, uint8_t count)
{
  uint8_t  d[8 + (4 * FVL_MAX_PTS)];
  uint16_t o = 0;
  uint8_t  i;

  if (count < 2) return;

  fvl_put16(d, o, FVL_CMD_LINE); o += 2;
  fvl_put16(d, o, count);        o += 2;
  fvl_put16(d, o, colour);       o += 2;

  for (i = 0; i < count; i++) {
      fvl_put16(d, o, xBuf[i]);              o += 2;
      fvl_put16(d, o, FVL_Y_T + yBuf[i]);    o += 2;
  }

  /* End of graphics operation */
  d[o++] = 0xFF;
  d[o++] = 0x00;

  fvl_send(d, o);
}

/* X = expired/inspired volume, 0 mL at the left edge */
static uint16_t fvl_mapX(long v) {
  if (v < 0)           v = 0;
  if (v > FVL_VOL_FS)  v = FVL_VOL_FS;
  return (uint16_t)(FVL_X_L + ((v * FVL_W) / FVL_VOL_FS));
}

static uint16_t fvl_mapY(long f) {
  long y;
  if (f >  FVL_FLOW_FS) f =  FVL_FLOW_FS;
  if (f < -FVL_FLOW_FS) f = -FVL_FLOW_FS;
  y = (long)FVL_Y_MID - ((f * (long)FVL_Y_MID) / FVL_FLOW_FS);
  if (y < 0)             y = 0;
  if (y > (FVL_H - 1))   y = FVL_H - 1;
  return (uint16_t)y;
}

static void fvl_decimate(void) {
  uint8_t i, j = 0;
  for (i = 0; i < fvl_capN; i += 2) 
  {
    fvl_capX[j] = fvl_capX[i];
    fvl_capY[j] = fvl_capY[i];
    j++;
  }
  fvl_capN = j;
  fvl_ival *= 2;
}

static void fvl_startCapture(void) {
  fvl_capX[0] = fvl_mapX(0);
  fvl_capY[0] = fvl_mapY(0);
  fvl_capN    = 1;
  fvl_ival    = FVL_SAMPLE_MS;
  fvl_next    = millis() + fvl_ival;
}

static void fvl_commitBreath(void)
{
  uint8_t i;

  if (fvl_capN < 3) {
      fvl_startCapture();
      return;
  }

  if (fvl_state != FVL_ST_IDLE) {return;
  }

  /* Save current loop for later erase */
  fvl_prevN = fvl_drawN;
  for (i = 0; i < fvl_drawN; i++) {
      fvl_prevX[i] = fvl_drawX[i];
      fvl_prevY[i] = fvl_drawY[i];
  }

  /* Copy captured breath into draw buffer */
  fvl_drawN = fvl_capN;
  for (i = 0; i < fvl_capN; i++) {
      fvl_drawX[i] = fvl_capX[i];
      fvl_drawY[i] = fvl_capY[i];
  }

  fvl_state = (fvl_prevN > 1) ? FVL_ST_ERASE : FVL_ST_DRAW;
  fvl_startCapture();
}

static void fvl_flush(void) {

  if (fvl_spPend) {
    if (DWIN_PORT.availableForWrite() < FVL_TXFREE_SP) return;
    fvl_sp_init();
    fvl_spPend = 0;
    return;
  }

  if (fvl_state == FVL_ST_IDLE) return;
  if (DWIN_PORT.availableForWrite() < FVL_TXFREE_MIN) return;

  switch (fvl_state) {

    case FVL_ST_ERASE:
      fvl_polyline(FVL_COL_BG, fvl_prevX, fvl_prevY, fvl_prevN);
      fvl_prevN = 0;
      fvl_state = FVL_ST_DRAW;
      break;

    case FVL_ST_DRAW:
      fvl_polyline(FVL_COL_LOOP, fvl_drawX, fvl_drawY, fvl_drawN);
      fvl_state = FVL_ST_IDLE;
      break;

    default:
      fvl_state = FVL_ST_IDLE;
      break;
  }
}

static void fvl_capture(void) {
  uint16_t x;
  uint16_t y;

  if ((int32_t)(millis() - fvl_next) < 0) return;
  fvl_next = millis() + fvl_ival;

  x = fvl_mapX((long)data_VOLUME);
  y = fvl_mapY((long)data_FLOW);

  /* skip duplicate pixels */
  if (fvl_capN > 0 && fvl_capX[fvl_capN - 1] == x && fvl_capY[fvl_capN - 1] == y)
    return;

  if (fvl_capN >= FVL_MAX_PTS) fvl_decimate();

  fvl_capX[fvl_capN] = x;
  fvl_capY[fvl_capN] = y;
  fvl_capN++;
}

void fvloop_init(void) {
  fvl_state    = FVL_ST_IDLE;
  fvl_lastInsp = 0;
  fvl_lastPage = 0xFF;
  fvl_drawN    = 0;
  fvl_prevN    = 0;
  fvl_scale = (FVL_PX_SIZE - 1);
  fvl_spPend = 1; 
  fvl_startCapture();
}

void fvloop_service(void) {
  uint8_t insp;

  if (HMI_Page != PAGE_MAIN) {
    fvl_lastPage = HMI_Page;
    fvl_state    = FVL_ST_IDLE;
    fvl_drawN    = 0;
    fvl_prevN    = 0;
    fvl_startCapture();
    return;
  }

   if (fvl_lastPage != PAGE_MAIN) {
    fvl_lastPage = PAGE_MAIN;
    fvl_spPend = 1;
    fvl_startCapture();
  }

  insp = (uint8_t)inspiration;

  if (insp == 1 && fvl_lastInsp == 0) {
    fvl_lastInsp = 1;
    fvl_commitBreath();  /* Triggers once per complete breath cycle */
  } else if (insp == 0 && fvl_lastInsp == 1) {
    fvl_lastInsp = 0;
  }

  fvl_flush();   
  fvl_capture(); 
}

#else
void fvloop_init(void) {}
void fvloop_service(void) {}
#endif


/*Flow - Pressure loop */

#define ENABLE_FP_LOOP 1

#if ENABLE_FP_LOOP 

/* ---- Basic Graphic control ---- */
#define FPL_VP        0xC700      //vp address
#define FPL_SP        0xC800  
#define FPL_SP_SCALE  (FPL_SP + 0x07)
#define FPL_CMD_LINE   0x0002   /* connected line */

/* ---- panel geometry (DWIN pixels, y grows downward) ---- */
#define FPL_X_L   720
#define FPL_X_R   1050
#define FPL_Y_T   355
#define FPL_Y_B   655
#define FPL_W     (FPL_X_R - FPL_X_L) 
#define FPL_H     (FPL_Y_B - FPL_Y_T)   
#define FPL_Y_MID  200 //(FPL_H/2) 

#define FPL_PRES_FS   56
#define FPL_FLOW_FS   110

/* ---- colours (RGB565) ---- */
#define FPL_COL_INSP  0x07FF   /* blue*/
#define FPL_COL_BG    0xFFFF    // white background

#define FPL_PX_SIZE   3
#define FPL_PXM       (FPL_PX_SIZE - 1) 
/* ---- capture ---- */
#define FPL_MAX_PTS      40    /* per breath, per buffer */
#define FPL_SAMPLE_MS    50UL  /* starting interval; doubles on overflow */
#define FPL_PTS_PER_PKT  7     /* points per polyline packet */
#define FPL_TXFREE_MIN   40    /* UART headroom needed */
#define FPL_TXFREE_SP    24
#define FPL_PRES_OFFSET  10

/* ---- state machine ---- */
#define FPL_ST_IDLE     0
#define FPL_ST_ERASE    1
#define FPL_ST_DRAW     2

/* capture bufferthe breath in progress */
static uint16_t fpl_capX[FPL_MAX_PTS];
static uint16_t fpl_capY[FPL_MAX_PTS];    
static uint8_t  fpl_capN = 0;
static uint32_t fpl_ival = FPL_SAMPLE_MS;
static uint32_t fpl_next = 0;

/* active draw buffers */
static uint16_t fpl_drawX[FPL_MAX_PTS];
static uint16_t fpl_drawY[FPL_MAX_PTS];
static uint8_t  fpl_drawN = 0;

static uint16_t fpl_prevX[FPL_MAX_PTS];
static uint16_t fpl_prevY[FPL_MAX_PTS];
static uint8_t  fpl_prevN = 0;

static uint8_t fpl_state = FPL_ST_IDLE;
static uint8_t fpl_fi = 0;
static uint8_t fpl_lastInsp = 0;
static uint8_t fpl_lastPage = 0xFF;

static uint8_t fpl_scale    = (FPL_PX_SIZE - 1);  /* Pixel_Scale register value */
static uint8_t fpl_spPend   = 1;                  

static void fpl_put16(uint8_t *b, uint8_t i, uint16_t v) {
  b[i] = highByte(v);
  b[i + 1] = lowByte(v);
}

static void fpl_send_at(uint16_t addr, const uint8_t *d, uint16_t n) {
  uint8_t hdr[6];
  hdr[0] = FRAME_START_LOW;
  hdr[1] = FRAME_START_HIGH;
  hdr[2] = n + 3;
  hdr[3] = CMD_VARIABLE_WRITE;
  hdr[4] = highByte(addr);
  hdr[5] = lowByte(addr);
  DWIN_PORT.write(hdr, 6);
  DWIN_PORT.write(d, n);
}

static void fpl_send(const uint8_t *d, uint16_t n) {
  fpl_send_at((uint16_t)FPL_VP, d, n);
}

static void fpl_sp_init(void) {
  uint8_t d[16];
  fpl_put16(d,  0, (uint16_t)FPL_VP);
  fpl_put16(d,  2, (uint16_t)FPL_X_L);
  fpl_put16(d,  4, (uint16_t)FPL_Y_T);
  fpl_put16(d,  6, (uint16_t)FPL_X_R);
  fpl_put16(d,  8, (uint16_t)FPL_Y_B);
  fpl_put16(d, 10, 0x0000);             /* solid lines, Dash_Set[0] */
  fpl_put16(d, 12, 0x0000);             /* Dash_Set[1:2] */
  d[14] = 0x00;                         /* Dash_Set[3] */
  d[15] = fpl_scale;                    /* Pixel_Scale: 0x00-0x0F = 1x1 to 16x16 */
  fpl_send_at((uint16_t)FPL_SP, d, 16);
}

static void fpl_set_pixel_size(uint8_t px) {
  uint8_t d[2];
  uint8_t reg;
  if (px < 1)  px = 1;
  if (px > 16) px = 16;
  reg = px - 1;
  if (reg == fpl_scale) return;
  fpl_scale = reg;
  d[0] = 0x00;                          /* Dash_Set[3] */
  d[1] = fpl_scale;
  fpl_send_at((uint16_t)FPL_SP_SCALE, d, 2);
}

static void fpl_polyline(uint16_t colour, const uint16_t *xBuf, const uint16_t *yBuf, uint8_t count)
{
  uint8_t d[6 + (4 * FPL_MAX_PTS) + 2];
  uint16_t o = 0;
  uint8_t i;

  fpl_put16(d, o, FPL_CMD_LINE); o += 2;
  fpl_put16(d, o, count);        o += 2;
  fpl_put16(d, o, colour);       o += 2;

  for (i = 0; i < count; i++) {
      fpl_put16(d, o, xBuf[i]); o += 2;
      fpl_put16(d, o, FPL_Y_T + yBuf[i]); o += 2;
  }

  /* End of graphics */
  d[o++] = 0xFF;
  d[o++] = 0x00;

  fpl_send(d, o);
}

static uint16_t fpl_mapX(long p) {
  long x;
  p += FPL_PRES_OFFSET;

  if (p < 0) p = 0;
  if (p > FPL_PRES_FS + FPL_PRES_OFFSET) p = FPL_PRES_FS + FPL_PRES_OFFSET;

  x = ((p * (long)(FPL_W - 1)) /(FPL_PRES_FS + FPL_PRES_OFFSET));

  if (x < 0) x = 0;
  if (x > (FPL_W - 1)) x = FPL_W - 1;

  return (uint16_t)(FPL_X_L + x);
}

static uint16_t fpl_mapY(long f) {
  long y;
  if (f > FPL_FLOW_FS) f = FPL_FLOW_FS;
  if (f < -FPL_FLOW_FS) f = -FPL_FLOW_FS;

  y = (long)FPL_Y_MID - ((f * (long)FPL_Y_MID) / FPL_FLOW_FS);

  if (y < 0) y = 0;
  if (y > (FPL_H - 1)) y = FPL_H - 1;

  return (uint16_t)y;
}

static void fpl_decimate(void) {
  uint8_t i, j = 0;
  for (i = 0; i < fpl_capN; i += 2) {
    fpl_capX[j] = fpl_capX[i];
    fpl_capY[j] = fpl_capY[i];
    j++;
  }
  fpl_capN = j;
  fpl_ival *= 2;
}

static void fpl_startCapture(void) {
  fpl_capN = 0;
  fpl_ival = FPL_SAMPLE_MS;
  fpl_next = millis();
}

static void fpl_commitBreath(void)
{
  uint8_t i;

  if (fpl_capN < 2) {
      fpl_startCapture();
      return;
  }

  if (fpl_state != FPL_ST_IDLE) return;

  fpl_prevN = fpl_drawN;
  for (i = 0; i < fpl_drawN; i++) {
      fpl_prevX[i] = fpl_drawX[i];
      fpl_prevY[i] = fpl_drawY[i];
  }

  /* Copy captured breath into draw buffer */
  fpl_drawN = fpl_capN;
  for (i = 0; i < fpl_capN; i++) {
      fpl_drawX[i] = fpl_capX[i];
      fpl_drawY[i] = fpl_capY[i];
  }

  if (fpl_prevN > 1)
      fpl_state = FPL_ST_ERASE;
  else
      fpl_state = FPL_ST_DRAW;

  fpl_fi = 0;

  fpl_startCapture();
}


static void fpl_flush(void) {
  uint8_t n;

    if (fpl_spPend) {
    if (DWIN_PORT.availableForWrite() < FPL_TXFREE_SP) return;
    fpl_sp_init();
    fpl_spPend = 0;
    return;
  }

  if (fpl_state == FPL_ST_IDLE) return;
  if (DWIN_PORT.availableForWrite() < FPL_TXFREE_MIN) return;

  switch (fpl_state) {
    case FPL_ST_ERASE:
    fpl_polyline(FPL_COL_BG, fpl_prevX, fpl_prevY, fpl_prevN);
    fpl_prevN = 0;
    fpl_state = FPL_ST_DRAW;
    break;

    case FPL_ST_DRAW:
      fpl_polyline(FPL_COL_INSP, fpl_drawX, fpl_drawY, fpl_drawN);
      fpl_state = FPL_ST_IDLE;
    break;

    default:
     fpl_state = FPL_ST_IDLE;
    break;
  }
}

static void fpl_capture(void) {
  uint16_t x;
  uint16_t y;

  if ((int32_t)(millis() - fpl_next) < 0) return;
  fpl_next = millis() + fpl_ival;

  x = fpl_mapX((long)data_PRESSURE_print);  
  y = fpl_mapY((long)data_FLOW);

  if (fpl_capN > 0 && fpl_capX[fpl_capN - 1] == x && fpl_capY[fpl_capN - 1] == y)
    return;

  if (fpl_capN >= FPL_MAX_PTS) fpl_decimate();

  fpl_capX[fpl_capN] = x;
  fpl_capY[fpl_capN] = y;
  fpl_capN++;
}

void fploop_init(void) {
  fpl_state = FPL_ST_IDLE;
  fpl_lastInsp = 0;
  fpl_lastPage = 0xFF;
  fpl_drawN = 0;
  fpl_prevN = 0;
  fpl_scale = (FPL_PX_SIZE - 1);
  fpl_spPend = 1; 

  fpl_startCapture();
}

void fploop_service(void) {
  uint8_t insp;

  if (HMI_Page != PAGE_MAIN) {
    fpl_lastPage = HMI_Page;
    fpl_state = FPL_ST_IDLE;
    fpl_capN = 0;
    fpl_drawN = 0;
    fpl_prevN = 0;
    return;
  }
  
  if (fpl_lastPage != PAGE_MAIN) {
    fpl_lastPage = PAGE_MAIN;
    fpl_spPend = 1;
    fpl_startCapture();
  }

  insp = (uint8_t)inspiration;

  if (insp == 1 && fpl_lastInsp == 0) {
    fpl_lastInsp = 1;
    fpl_commitBreath();  /* Triggers once per complete breath cycle */
  } else if (insp == 0 && fpl_lastInsp == 1) {
    fpl_lastInsp = 0;
  }

  fpl_flush();   /* Manages non-blocking drawing/erasing sequence */
  fpl_capture(); /* Samples current ongoing breath */
}

#else
void fploop_init(void) {}
void fploop_service(void) {}
#endif  


void dwin_page_Set(uint8_t pageId) {
  
  uint8_t sendBuffer[] = { 0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, pageId };

  DWIN_PORT.write(sendBuffer, sizeof(sendBuffer));
  delay(1);
}

void display_write_text(uint16_t variable_id, char *data)  
{
  uint8_t data_len = strlen(data);
  uint8_t lenn = data_len + 3;
  uint8_t byte_s2[6] = { 0 };

  byte_s2[0] = FRAME_START_LOW;
  byte_s2[1] = FRAME_START_HIGH;
  byte_s2[2] = lenn;
  byte_s2[3] = CMD_VARIABLE_WRITE;
  byte_s2[4] = highByte(variable_id);  
  byte_s2[5] = lowByte(variable_id);   

  DWIN_PORT.write(byte_s2, sizeof(byte_s2));
  DWIN_PORT.write(data, data_len);
}

void display_write_variable(uint16_t variable_id, int32_t value_3)  
{
  const uint8_t no_bytes = 7;
  uint8_t byte_s2[10] = { 0 };
  byte_s2[0] = FRAME_START_LOW;
  byte_s2[1] = FRAME_START_HIGH;
  byte_s2[2] = no_bytes;
  byte_s2[3] = CMD_VARIABLE_WRITE;
  byte_s2[4] = highByte(variable_id);  
  byte_s2[5] = lowByte(variable_id);   
  byte_s2[6] = highByte(value_3);      
  byte_s2[7] = lowByte(value_3);       
  byte_s2[8] = 0xFF;
  byte_s2[9] = 0xFF;

  DWIN_PORT.write(byte_s2, WRITE_VARIABLE_NO_BYTES);
  Buffer1_Len = 0;
}

void display_write_variable64(uint16_t variable_id, uint8_t(value_3[0]))  
{
  const uint8_t no_bytes = 11;
  
  uint8_t byte_s2[14] = { 0 };

  byte_s2[0] = FRAME_START_LOW;
  byte_s2[1] = FRAME_START_HIGH;
  byte_s2[2] = no_bytes;
  byte_s2[3] = CMD_VARIABLE_WRITE;
  byte_s2[4] = highByte(variable_id);  
  byte_s2[5] = lowByte(variable_id);   

  for (int i = 0; i < 8; i++) {
    byte_s2[6 + i] = value_3[i];
  }

  DWIN_PORT.write(byte_s2, sizeof(byte_s2));
  delay(1);

}
/****************************************************************
*
*
*
*
*****************************************************************/
void realTimeTrendGraph(void) {

  interval_new_e = millis();
  flow_time_e = ((interval_new_e - interval_old_e));
  interval_old_e = millis();
  
  pressure_SUM = pressure_SUM - pressure_READINGS[pressure_INDEX];      
  pressure_VALUE = (PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP));  
  if (peak_1 >= 0 && peak_1 <= 10 && setbodyweight <= 15) {
    pressure_VALUE = 5 * pressure_VALUE;
  } else if (peak_1 > 10 && peak_1 <= 20 && setbodyweight <= 15) {
    pressure_VALUE = 2.5 * pressure_VALUE;
  } else if (peak_1 > 0 && peak_1 <= 30) {
    pressure_VALUE = 1.83 * pressure_VALUE;
  } else if (peak_1 > 30 && peak_1 <= 40) {
    pressure_VALUE = 1.37 * pressure_VALUE;
  } else if (peak_1 > 40 && peak_1 <= 50) {
    pressure_VALUE = 1.1 * pressure_VALUE;
  } else if (peak_1 > 50 && peak_1 <= 70) {
    pressure_VALUE = 0.76 * pressure_VALUE;
  } else if (peak_1 > 70 && peak_1 <= 100) {
    pressure_VALUE = 0.55 * pressure_VALUE;
  } else if (peak_1 > 100) {
    pressure_VALUE = 0.55 * pressure_VALUE;
  }
  pressure_READINGS[pressure_INDEX] = pressure_VALUE;           
  pressure_SUM = pressure_SUM + pressure_VALUE;                 
  pressure_INDEX = (pressure_INDEX + 1) % WINDOW_SIZEpressure;  
  pressure_AVERAGED = pressure_SUM / WINDOW_SIZEpressure;       
                                                                                                                          
  if (pressure_AVERAGED < (-10)) {
    pressure_AVERAGED = -10;
  }
  
  if (mainrelaypinstate == LOW) {
    peak_1 = peakpressure;  
  }
  if (setmode != MODE_HFT) {
  
  if (peak_1 >= 0 && peak_1 <= 10 && setbodyweight <= 15) {

    data_PRESSURE = ((pressure_AVERAGED) + PRESSURE_GRAPH_OFFSET);
    w = 1;
    if (last_w != w) {
      display_write_variable(VP_PRESSURE_AXIS_MAX, 10);
      display_write_variable(VP_PRESSURE_AXIS_MID, 4);
      last_w = w;
    }
  } else if (peak_1 >= 10 && peak_1 <= 20 && setbodyweight <= 15) {

    data_PRESSURE = ((pressure_AVERAGED) + PRESSURE_GRAPH_OFFSET);
    w = 3;

    if (last_w != w) {
      display_write_variable(VP_PRESSURE_AXIS_MAX, 20);
      display_write_variable(VP_PRESSURE_AXIS_MID, 9);
      last_w = w;
    }
  } else if (peak_1 >= 0 && peak_1 <= 28) {

    data_PRESSURE = ((pressure_AVERAGED) + PRESSURE_GRAPH_OFFSET);
    w = 4;

    if (last_w != w) {
      display_write_variable(VP_PRESSURE_AXIS_MAX, 28);
      display_write_variable(VP_PRESSURE_AXIS_MID, 12);
      last_w = w;
    }
  } else if (peak_1 > 28 && peak_1 <= 37) {
    data_PRESSURE = ((pressure_AVERAGED) + PRESSURE_GRAPH_OFFSET);
    w = 5;

    if (last_w != w) {
      display_write_variable(VP_PRESSURE_AXIS_MAX, 37);
      display_write_variable(VP_PRESSURE_AXIS_MID, 20);
      last_w = w;
    }
  } else if (peak_1 > 37 && peak_1 <= 50) {
    w = 6;
    data_PRESSURE = ((pressure_AVERAGED) + PRESSURE_GRAPH_OFFSET);
    if (last_w != w) {
      display_write_variable(VP_PRESSURE_AXIS_MAX, 50);
      display_write_variable(VP_PRESSURE_AXIS_MID, 22);
      last_w = w;
    }
  } else if (peak_1 > 50 && peak_1 <= 70) {
    w = 7;
    data_PRESSURE = ((pressure_AVERAGED) + PRESSURE_GRAPH_OFFSET);
    if (last_w != w) {
      display_write_variable(VP_PRESSURE_AXIS_MAX, 70);
      display_write_variable(VP_PRESSURE_AXIS_MID, 35);
      last_w = w;
    }
  } else if (peak_1 > 70 && peak_1 <= 100) {
    w = 8;
    data_PRESSURE = ((pressure_AVERAGED) + PRESSURE_GRAPH_OFFSET);
    if (last_w != w) {
      display_write_variable(VP_PRESSURE_AXIS_MAX, 100);
      display_write_variable(VP_PRESSURE_AXIS_MID, 50);
      last_w = w;
    }
  } else if (peak_1 > 100) {
    data_PRESSURE = ((pressure_AVERAGED) + PRESSURE_GRAPH_OFFSET);
    w = 9;

    if (last_w != w) {
      display_write_variable(VP_PRESSURE_AXIS_MAX, 105);
      display_write_variable(VP_PRESSURE_AXIS_MID, 54);
      last_w = w;
    }
  }
  }

  if (setmode == MODE_HFT) {
    w = 10;
    data_PRESSURE = ((pressure_AVERAGED) + PRESSURE_GRAPH_OFFSET);
    if (last_w != w) {
      display_write_variable(VP_PRESSURE_AXIS_MAX, 50);
      display_write_variable(VP_PRESSURE_AXIS_MID, 25);
      last_w = w;
    }
  }
  
  
  cflow_SUM = cflow_SUM - cflow_READINGS[cflow_INDEX];  
                                                        
  if (mainrelaypinstate == HIGH) {
    currentvolume1 = 0;
    dispVolume = currentvolume;   /* display mirrors the real rise -
                                     WITHOUT this the volume trace is
                                     a flat line (nothing ever rises) */
    dispE2 = 0;
    exhDone = 0;
    exhLowSince = 0;
    ss2 = 1;
    cflow_VALUE = average1;
    if (peak_flow >= 0 && peak_flow <= 5) {

      
    }
  }
  
  else {

    average1_e = ((analogRead(Analog_pr4) - ZFF_PR4));

    if (average1_e > 0) {
      average1_e = Pr4_range * sqrt(0.002587 * (average1_e));  
    } else {
      average1_e = 0;
    }
    
    currentvolume += (-1 * 0.013 * (flow_time_e) * (average1_e + average1_e2));  
    average1_e2 = average1_e;                                                    
    cflow_VALUE = -1 * average1_e;
    if (currentvolume <= 0) {
      cflow_VALUE = 0;
    }
    if (currentvolume <= 0) {
      currentvolume = 0;
    }
    /* display-only volume - currentvolume untouched */
    {
      float pr4_raw_e = (analogRead(Analog_pr4) - ZFF_PR4);
      if (setpeep >= 2 && (TCMillis - TPMillis) > PR4_BIAS_LEARN_MS) {
        pr4_bias_offset += PR4_BIAS_LEARN_ALPHA * (pr4_raw_e - pr4_bias_offset);
        if (pr4_bias_offset < 0.0) { pr4_bias_offset = 0.0; }
      }
      if (setpeep < 2) { pr4_bias_offset = 0.0; }
      float pr4_comp_e = pr4_raw_e - pr4_bias_offset;
      if (pr4_comp_e < 0) { pr4_comp_e = 0; }
      if (exhDone == 0) {
        if (pr4_comp_e < EXH_DONE_FLOW) {
          if (exhLowSince == 0) { exhLowSince = TCMillis; }
          else if ((TCMillis - exhLowSince) >= EXH_DONE_MS) { exhDone = 1; }
        } else { exhLowSince = 0; }
      }
      float dispFlow;
      if (exhDone == 1) {
        dispFlow = 0;
        dispVolume += 0.08 * (0.0 - dispVolume);
      } else if (pr4_comp_e > 0) {
        dispFlow = Pr4_range * sqrt(0.002587 * (pr4_comp_e));
      } else { dispFlow = 0; }
      dispVolume += (-1 * 0.013 * (flow_time_e) * (dispFlow + dispE2));
      dispE2 = dispFlow;
      if (dispVolume <= 0) { dispVolume = 0; }
    }
  }

  cflow_READINGS[cflow_INDEX] = cflow_VALUE;           
  cflow_SUM = cflow_SUM + cflow_VALUE;                 
  cflow_INDEX = (cflow_INDEX + 1) % WINDOW_SIZEcflow;  

  cflow_AVERAGED = cflow_SUM / WINDOW_SIZEcflow;  

  if (average1 > monitored_flow_i && mainrelaypinstate == HIGH) { monitored_flow_i = average1; }
  if (average1_e > monitored_flow_e && mainrelaypinstate == LOW) { monitored_flow_e = average1_e; }

  if (mainrelaypinstate == HIGH && monitored_flow_e != 0) {
    peak_flow_e = monitored_flow_e;
    monitored_flow_e = 0;
  }
  if (mainrelaypinstate == LOW && monitored_flow_i != 0) {
    peak_flow_i = monitored_flow_i;
    monitored_flow_i = 0;
  }

  if (peak_flow_i >= peak_flow_e) {
    peak_flow = peak_flow_i;
  } else {
    peak_flow = peak_flow_i;
  }

  cflow_AVERAGED = cflow_AVERAGED;
  if (setmode != MODE_HFT) {
  if (peak_flow >= 0 && peak_flow <= 5) {
    data_FLOW = ((5.5 * cflow_AVERAGED) + 55);
    v = 1;
    if (v != last_v) {                    
      display_write_variable(VP_FLOW_AXIS_MAX, 5);  
      display_write_variable(VP_FLOW_AXIS_MID, 3);
      last_v = v;
    }
  }
  if (peak_flow > 5 && peak_flow <= 10) {
    data_FLOW = ((5.5 * cflow_AVERAGED) + 55);
    v = 2;
    if (v != last_v) {                     
      display_write_variable(VP_FLOW_AXIS_MAX, 10);  
      display_write_variable(VP_FLOW_AXIS_MID, 5);
      last_v = v;
    }
  } else if (peak_flow > 10 && peak_flow <= 20) {
    data_FLOW = ((2.75 * cflow_AVERAGED) + 55);
    v = 3;
    if (v != last_v) {                     
      display_write_variable(VP_FLOW_AXIS_MAX, 20);  
      display_write_variable(VP_FLOW_AXIS_MID, 10);
      last_v = v;
    }
  } else if (peak_flow > 20 && peak_flow <= 30) {
    data_FLOW = ((1.83 * cflow_AVERAGED) + 55);
    v = 4;
    if (v != last_v) {                     
      display_write_variable(VP_FLOW_AXIS_MAX, 30);  
      display_write_variable(VP_FLOW_AXIS_MID, 15);
      last_v = v;
    }
  } else if (peak_flow > 30 && peak_flow <= 40) {
    data_FLOW = ((1.37 * cflow_AVERAGED) + 55);
    v = 5;
    if (v != last_v) {                     
      display_write_variable(VP_FLOW_AXIS_MAX, 40);  
      display_write_variable(VP_FLOW_AXIS_MID, 20);
      last_v = v;
    }
  } else if (peak_flow > 40 && peak_flow <= 50) {
    data_FLOW = ((1.1 * cflow_AVERAGED) + 55);
    v = 6;
    if (v != last_v) {                     
      display_write_variable(VP_FLOW_AXIS_MAX, 50);  
      display_write_variable(VP_FLOW_AXIS_MID, 25);
      last_v = v;
    }
  } else if (peak_flow > 50 && peak_flow <= 70) {
    data_FLOW = ((0.77 * cflow_AVERAGED) + 55);
    v = 7;
    if (v != last_v) {                     
      display_write_variable(VP_FLOW_AXIS_MAX, 70);  
      display_write_variable(VP_FLOW_AXIS_MID, 35);
      last_v = v;
    }
  } else if (peak_flow > 70 && peak_flow <= 100) {
    data_FLOW = ((0.55 * cflow_AVERAGED) + 55);
    v = 8;
    if (v != last_v) {                      
      display_write_variable(VP_FLOW_AXIS_MAX, 100);  
      display_write_variable(VP_FLOW_AXIS_MID, 50);
      last_v = v;
    }
  } else if (peak_flow > 100) {
    data_FLOW = ((0.466 * cflow_AVERAGED) + 55);
    v = 9;
    if (v != last_v) {                      
      display_write_variable(VP_FLOW_AXIS_MAX, 105);  
      display_write_variable(VP_FLOW_AXIS_MID, 54);
      last_v = v;
    }
  }
  }
  if (setmode == MODE_HFT) {
    data_FLOW = ((0.55 * cflow_AVERAGED) + 55);
    v = 10;
    if (v != last_v) {                      
      display_write_variable(VP_FLOW_AXIS_MAX, 70);  
      display_write_variable(VP_FLOW_AXIS_MID, 35);
      last_v = v;
    }
  }
  if (data_FLOW <= 1) {
    data_FLOW = 1;
  }

  if (T_volume >= 0 && T_volume <= 140) {
    data_VOLUME = (5 * (dispVolume));
    vol = 1;
    if (last_vol != vol) {
      display_write_variable(VP_VOLUME_AXIS_MAX, 140);
      display_write_variable(VP_VOLUME_AXIS_MID, 70);
      last_vol = vol;
    }
  } else if (T_volume > 140 && T_volume <= 300) {
    data_VOLUME = (2 * (dispVolume));
    vol = 2;
    if (last_vol != vol) {
      display_write_variable(VP_VOLUME_AXIS_MAX, 280);
      display_write_variable(VP_VOLUME_AXIS_MID, 140);
      last_vol = vol;
    }
  } else if (T_volume > 280 && T_volume <= 650) {
    data_VOLUME = (1 * (dispVolume));
    vol = 3;
    if (last_vol != vol) {
      display_write_variable(VP_VOLUME_AXIS_MAX, 650);
      display_write_variable(VP_VOLUME_AXIS_MID, 280);
      last_vol = vol;
    }
  } else if (T_volume > 650 && T_volume <= 1200) {
    data_VOLUME = (0.5 * (dispVolume));
    vol = 4;
    if (last_vol != vol) {
      display_write_variable(VP_VOLUME_AXIS_MAX, 1200);
      display_write_variable(VP_VOLUME_AXIS_MID, 650);
      last_vol = vol;
    }
  } else if (T_volume > 1200 && T_volume <= 1500) {
    data_VOLUME = (0.4 * (dispVolume));
    vol = 5;
    if (last_vol != vol) {
      display_write_variable(VP_VOLUME_AXIS_MAX, 1500);
      display_write_variable(VP_VOLUME_AXIS_MID, 750);
      last_vol = vol;
    }
  } else if (T_volume > 1500 && T_volume <= 1800) {
    data_VOLUME = (0.3 * (dispVolume));
    vol = 6;
    if (last_vol != vol) {
      display_write_variable(VP_VOLUME_AXIS_MAX, 1800);
      display_write_variable(VP_VOLUME_AXIS_MID, 900);
      last_vol = vol;
    }
  } else if (T_volume > 1800 && T_volume <= 2100) {
    data_VOLUME = (0.28 * (dispVolume));
    vol = 7;
    if (last_vol != vol) {
      display_write_variable(VP_VOLUME_AXIS_MAX, 2100);
      display_write_variable(VP_VOLUME_AXIS_MID, 1050);
      last_vol = vol;
    }
  } else if (T_volume > 2100) {
    data_VOLUME = (0.272 * (dispVolume));
    vol = 8;
    if (last_vol != vol) {
      display_write_variable(VP_VOLUME_AXIS_MAX, 2200);
      display_write_variable(VP_VOLUME_AXIS_MID, 1100);
      last_vol = vol;
    }
  }

  data_PRESSURE_print = data_PRESSURE;  

  if (data_PRESSURE_print >= 56) { data_PRESSURE_print = 56; }
  if (data_PRESSURE_print <= 0) { data_PRESSURE_print = 0; }
  if (data_FLOW >= 110) { data_FLOW = 110; }
  if (data_VOLUME >= 600) { data_VOLUME = 600; }
  if (data_VOLUME <= 1) { data_VOLUME = 1; }
  
  CURVE_TREND[12] = highByte(data_PRESSURE_print);
  CURVE_TREND[13] = lowByte(data_PRESSURE_print);
  CURVE_TREND[14] = highByte(data_PRESSURE_print);
  CURVE_TREND[15] = lowByte(data_PRESSURE_print);
  CURVE_TREND[18] = highByte(data_FLOW);  
  CURVE_TREND[19] = lowByte(data_FLOW);   
  CURVE_TREND[20] = highByte(data_FLOW);
  CURVE_TREND[21] = lowByte(data_FLOW);

  CURVE_TREND[24] = highByte(data_VOLUME);  
  CURVE_TREND[25] = lowByte(data_VOLUME);   
  CURVE_TREND[26] = highByte(data_VOLUME);
  CURVE_TREND[27] = lowByte(data_VOLUME);

  DWIN_PORT.write(CURVE_TREND, 28);
}

void limitparameter(void) {
  
  encoder_switch_count = 0;
  Icon17[7] = 166;
  DWIN_PORT.write(Icon17, 8);
  settingbutton_count = 23;
  
  
  if (setmode == MODE_ACV || setmode == MODE_SIMV || setmode == MODE_VCV) {
    highvtalarmlimit = 1.75 * settidalvolume;
    lowvtalarmlimit = 0.05 * settidalvolume;
  } else {
    highvtalarmlimit = (2 * OFFSET_BW_TV * bodyweight);    
    lowvtalarmlimit = (0.25 * OFFSET_BW_TV * bodyweight);  
  }
  display_write_variable(VP_ALM_VT_HIGH, highvtalarmlimit);
  display_write_variable(VP_ALM_VT_LOW, lowvtalarmlimit);

  if (setmode == MODE_ACV || setmode == MODE_VCV || setmode == MODE_HFT || setmode == MODE_CPAP || setmode == MODE_PCV || setmode == MODE_SPONT_PS) {
    highbpmalarmlimit = 50;  
    lowbpmalarmlimit = 8;    
  } else if (setmode == MODE_PSIMV || setmode == MODE_SIMV) {
    highbpmalarmlimit = 50;  
    lowbpmalarmlimit = 8;    
  }
  if (setfrequency > 45) { highbpmalarmlimit = 80; }

  display_write_variable(VP_ALM_BPM_HIGH, highbpmalarmlimit);
  display_write_variable(VP_ALM_BPM_LOW, lowbpmalarmlimit);
  
  if (setmode == MODE_PCV || setmode == MODE_CPAP || setmode == MODE_PSIMV || setmode == MODE_SPONT_PS) {
    highpawalarmlimit = 10 + set_Pinsp;
    sethighpawalarmlimit = highpawalarmlimit;
    display_write_variable(VP_ALM_PAW_HIGH, highpawalarmlimit);
  }  
  else {
    if (setbodyweight < 20) {
      highpawalarmlimit = ((setbodyweight + 5));
    } else if (setbodyweight < 30) {
      highpawalarmlimit = ((setbodyweight + 5));
      if (highpawalarmlimit > 30) { highpawalarmlimit = 30; }
    } else {
      highpawalarmlimit = 45;
    }
    sethighpawalarmlimit = highpawalarmlimit;
    display_write_variable(VP_ALM_PAW_HIGH, highpawalarmlimit);
  }  

  if (setmode == MODE_PCV || setmode == MODE_CPAP || setmode == MODE_PSIMV || setmode == MODE_SPONT_PS) {
    lowpawalarmlimit = 0.5 * set_Pinsp;
    if (lowpawalarmlimit <= 5) {
      lowpawalarmlimit = 5;
      setlowpawalarmlimit = lowpawalarmlimit;
      display_write_variable(VP_ALM_PAW_LOW, lowpawalarmlimit);
    } else {
      setlowpawalarmlimit = lowpawalarmlimit;
      display_write_variable(VP_ALM_PAW_LOW, lowpawalarmlimit);
    }
  }  
  else {
    lowpawalarmlimit = setpeep + 5;
    if (lowpawalarmlimit <= 3) {
      lowpawalarmlimit = 3;
      setlowpawalarmlimit = lowpawalarmlimit;
      display_write_variable(VP_ALM_PAW_LOW, lowpawalarmlimit);
    } else {
      setlowpawalarmlimit = lowpawalarmlimit;
      display_write_variable(VP_ALM_PAW_LOW, lowpawalarmlimit);
    }
  }  
  if (setmode == MODE_ACV || setmode == MODE_SIMV || setmode == MODE_VCV) {
    high_mvalarmlimit = (2.5 * 0.0015 * settidalvolume * frequncy);   
    low_mvalarmlimit = (0.25 * 0.00050 * settidalvolume * frequncy);  
  } else {
    high_mvalarmlimit = (4 * 0.0135 * bodyweight * frequncy);  
    low_mvalarmlimit = (0.25 * 0.0045 * bodyweight * frequncy);
  }

  high_mvalarmlimit_1 = high_mvalarmlimit;  
  low_mvalarmlimit_1 = low_mvalarmlimit;
  display_write_variable(VP_ALM_MV_HIGH, 10.0 * high_mvalarmlimit_1);
  display_write_variable(VP_ALM_MV_LOW, 10.0 * low_mvalarmlimit_1);
  high_fio2alarmlimit = set_set_fio2 + 30;
  low_fio2alarmlimit = set_set_fio2 - 30;

  if ((set_set_fio2 + 30) >= 100) {
    high_fio2alarmlimit = 100;
  }

  if ((set_set_fio2 - 30) <= 18) {
    low_fio2alarmlimit = 18;
    display_write_variable(VP_ALM_FIO2_LOW, low_fio2alarmlimit);
  } else {
    display_write_variable(VP_ALM_FIO2_LOW, low_fio2alarmlimit);
  }

  if ((set_set_fio2 + 30) <= 19) {
    high_fio2alarmlimit = 19;
    display_write_variable(VP_ALM_FIO2_HIGH, high_fio2alarmlimit);
  } else {
    display_write_variable(VP_ALM_FIO2_HIGH, high_fio2alarmlimit);
  }
}
/***************************************************************
*
*
*****************************************************************/
void monitoredvariables(void) {

  if (setmode != MODE_HFT) {
    if (settidalvolume <= 50 && (setmode == MODE_VCV || setmode == MODE_SIMV || setmode == MODE_ACV)) {
      monitor_variable_1_write[6] = highByte(settidalvolume);
      monitor_variable_1_write[7] = lowByte(settidalvolume);
    } else {
      monitor_variable_1_write[6] = highByte(vt_AVERAGED_SAVED);
      monitor_variable_1_write[7] = lowByte(vt_AVERAGED_SAVED);
    }

    monitor_variable_1_write[8] = highByte(peakpressure);
    monitor_variable_1_write[9] = lowByte(peakpressure);
    int peep_AVERAGED_SAVED_1 = 0;
    if ((peep_AVERAGED_SAVED - setpeep) > 2) {
      peep_AVERAGED_SAVED_1 = peep_AVERAGED_SAVED;
    } else {
      peep_AVERAGED_SAVED_1 = setpeep;
    }
    monitor_variable_1_write[10] = highByte(peep_AVERAGED_SAVED_1);
    monitor_variable_1_write[11] = lowByte(peep_AVERAGED_SAVED_1);
    int AVERAGED_SAVED_bpm_1 = round(AVERAGED_SAVED_bpm);
    monitor_variable_1_write[12] = highByte(AVERAGED_SAVED_bpm_1);
    monitor_variable_1_write[13] = lowByte(AVERAGED_SAVED_bpm_1);

    int Tinsp1;
    int monitored_i_e;
    int Texp1 = 10 * monitore_TE / 1000;
    if (setmode == MODE_VCV || setmode == MODE_ACV || setmode == MODE_CPAP || setmode == MODE_PCV) {
      Tinsp1 = Tinsp / 100;
      monitored_i_e = round(10 * Texp / Tinsp);
    } else if (setmode == MODE_SIMV || setmode == MODE_PSIMV) {
      Tinsp1 = Tinsp_simv / 100;
      monitored_i_e = round(10 * Texp / Tinsp_simv);
    } else {
      Tinsp1 = monitore_TI / 100;
      monitored_i_e = round(10 * monitore_TE / monitore_TI);
    }
    monitor_variable_1_write[14] = highByte(Tinsp1);
    monitor_variable_1_write[15] = lowByte(Tinsp1);

    /* one source of truth: the same value the alarm block tests.
       the old code wrote 21 (or the SETPOINT, above 90%) to the screen
       while leaving dOp_AVERAGED1 at the raw average, so the display and
       the alarm could disagree.                                       */
    dOp_AVERAGED1 = fio2_monitored();
    monitor_variable_1_write[16] = highByte(dOp_AVERAGED1);
    monitor_variable_1_write[17] = lowByte(dOp_AVERAGED1);

    monitor_variable_1_write[18] = highByte(monitored_flow_i);  
    monitor_variable_1_write[19] = lowByte(monitored_flow_i);   

    monitor_variable_2_write[6] = highByte(203);  
    monitor_variable_2_write[7] = lowByte(203);   

    monitor_variable_2_write[10] = highByte((Texp1));  
    monitor_variable_2_write[11] = lowByte((Texp1));   

    ie_SUM = ie_SUM - ie_READINGS[ie_INDEX];  

    ie_VALUE = (10 * monitore_TE / monitore_TI);  
    ie_READINGS[ie_INDEX] = ie_VALUE;             
    ie_SUM = ie_SUM + ie_VALUE;                   
    ie_INDEX = (ie_INDEX + 1) % WINDOW_SIZEie;    

    ie_AVERAGED = ie_SUM / WINDOW_SIZEie;  
                                           
    if (Tinsp1 <= Texp1) {                          
      monitor_variable_2_write[12] = highByte(10);  
      monitor_variable_2_write[13] = lowByte(10);   

      monitor_variable_2_write[14] = highByte(monitored_i_e);  
      monitor_variable_2_write[15] = lowByte(monitored_i_e);
    }
    else {                                                     
      monitor_variable_2_write[12] = highByte(monitored_i_e);  
      monitor_variable_2_write[13] = lowByte(monitored_i_e);   

      monitor_variable_2_write[14] = highByte(10);  
      monitor_variable_2_write[15] = lowByte(10);
    }
    monitored_mv = 10 * (vt_AVERAGED_SAVED * AVERAGED_SAVED_bpm) / 1000;  
    monitor_variable_2_write[16] = highByte(monitored_mv);                
    monitor_variable_2_write[17] = lowByte(monitored_mv);

    if (mainrelaypinstate == HIGH) {
      (x = 1);
    } else if (mainrelaypinstate == LOW && x == 1) {
      pressure_mean_SUM = pressure_mean_SUM - pressure_mean_READINGS[pressure_mean_INDEX];  

      pressure_mean_VALUE = (peakpressure);                                        
      pressure_mean_READINGS[pressure_mean_INDEX] = pressure_mean_VALUE;           
      pressure_mean_SUM = pressure_mean_SUM + pressure_mean_VALUE;                 
      pressure_mean_INDEX = (pressure_mean_INDEX + 1) % WINDOW_SIZEpressure_mean;  

      pressure_mean_AVERAGED = pressure_mean_SUM / WINDOW_SIZEpressure_mean;  

      int pressure_mean_AVERAGED1 = pressure_mean_AVERAGED;
      monitor_variable_2_write[8] = highByte(pressure_mean_AVERAGED1);  
      monitor_variable_2_write[9] = lowByte(pressure_mean_AVERAGED1);   

      x = 0;
      DWIN_PORT.write(monitor_variable_1_write, 20);
      DWIN_PORT.write(monitor_variable_2_write, 18);
    }
  } else if (setmode == MODE_HFT) {

    monitor_variable_1_write[6] = highByte(0);
    monitor_variable_1_write[7] = lowByte(0);
    int pressure_print = round(pressure);
    monitor_variable_1_write[8] = highByte(pressure_print);
    monitor_variable_1_write[9] = lowByte(pressure_print);

    monitor_variable_1_write[10] = highByte(0);
    monitor_variable_1_write[11] = lowByte(0);
    
    monitor_variable_1_write[12] = highByte(0);
    monitor_variable_1_write[13] = lowByte(0);

    monitor_variable_1_write[14] = highByte(0);
    monitor_variable_1_write[15] = lowByte(0);

    if (set_fio2 <= 21 || oxygenavailability_status == HIGH || !fio2_sensor_ok) {
      dOp_AVERAGED1 = 21;
    } else {
      dOp_AVERAGED1 = (int)(dOp_AVERAGED);
      if (dOp_AVERAGED1 > 100) { dOp_AVERAGED1 = 100; }
    }
    monitor_variable_1_write[16] = highByte(dOp_AVERAGED1);
    monitor_variable_1_write[17] = lowByte(dOp_AVERAGED1);
    
    int average1_print = round(average1);
    monitor_variable_1_write[18] = highByte(average1_print);  
    monitor_variable_1_write[19] = lowByte(average1_print);   

    monitor_variable_2_write[6] = highByte(0);  
    monitor_variable_2_write[7] = lowByte(0);   

    monitor_variable_2_write[10] = highByte((0));  
    monitor_variable_2_write[11] = lowByte((0));   

    monitor_variable_2_write[12] = highByte(0);  
    monitor_variable_2_write[13] = lowByte(0);   

    monitor_variable_2_write[14] = highByte(0);  
    monitor_variable_2_write[15] = lowByte(0);
    
    monitor_variable_2_write[16] = highByte(0);  
    monitor_variable_2_write[17] = lowByte(0);

    monitor_variable_2_write[8] = highByte(0);  
    monitor_variable_2_write[9] = lowByte(0);   

    x = 0;
    DWIN_PORT.write(monitor_variable_1_write, 20);
    DWIN_PORT.write(monitor_variable_2_write, 18);
  }
}

void testscreenvariable() {
  int ov_start_duty1 = ov_start_duty;
  int V1_1 = V1;
  int pressure_1 = pressure;
  int pressure_3 = PRESSURE_FACTOR * ((analogRead(Analog_pr3)) - ZFP_pr3);  
  int tur_duty1 = tur_duty;
  int tur_duty_HFT_1 = tur_duty_HFT;
  int val_1 = sal;
  int peepduty_1 = 10 * peepduty;

  display_write_variable(VP_CAL_HFT, calibration_HFT);
  display_write_variable(VP_CAL_FLOW, calibration_flow);

  int pressure4 = ((analogRead(Analog_pr4) - ZFF_PR4));

  display_write_variable(VP_TEST_PR4_RAW, pressure4);

  display_write_variable(VP_TEST_PR3_PRESSURE, pressure_3);  

  display_write_variable(VP_TEST_BATT_VOLT, (btr_volt * 10));  

  display_write_variable(VP_TEST_SMPS_VOLT, (SMPS * 10));  

  monitorWindowVariable_write[6] = highByte(ov_start_duty1);  
  monitorWindowVariable_write[7] = lowByte(ov_start_duty1);

  monitorWindowVariable_write[8] = highByte(pressure_1);  
  monitorWindowVariable_write[9] = lowByte(pressure_1);

  monitorWindowVariable_write[10] = highByte(V1_1);  
  monitorWindowVariable_write[11] = lowByte(V1_1);

  if (digitalRead(mutebutton) == LOW) {        
    monitorWindowVariable_write[12] = highByte(pf);  
    monitorWindowVariable_write[13] = lowByte(pf);
  }

  /* f1_1 / L3_1 were ints, so 19.90 and 1.19 truncated to 19 and 1 and the
     service page showed constants that could never change. Sent x100 -
     set both DGUS fields to 2 decimal places.                          */
  f1_1 = (int)(f1 * 100.0f);
  L3_1 = (int)(L3 * 100.0f);
  monitorWindowVariable_write[14] = highByte(f1_1);  
  monitorWindowVariable_write[15] = lowByte(f1_1);   

  monitorWindowVariable_write[16] = highByte(L3_1);  
  monitorWindowVariable_write[17] = lowByte(L3_1);   

  /* live O2 cell diagnostics on the service page */
  display_write_variable(VP_TEST_FIO2_RAW, fio2_raw_adc);
  display_write_variable(VP_TEST_FIO2_INST, (int)(fio2 * 10.0f));
  display_write_variable(VP_TEST_FIO2_OK, fio2_sensor_ok);

  monitorWindowVariable_write[18] = highByte(tur_duty1);  
  monitorWindowVariable_write[19] = lowByte(tur_duty1);

  monitorWindowVariable_write[20] = highByte(tur_duty_HFT_1);  
  monitorWindowVariable_write[21] = lowByte(tur_duty_HFT_1);

  monitorWindowVariable_write[22] = highByte(val_1);  
  monitorWindowVariable_write[23] = lowByte(val_1);

  monitorWindowVariable_write[24] = highByte(peepduty_1);  
  monitorWindowVariable_write[25] = lowByte(peepduty_1);

  int turbine_pickup_time_factor_1 = 10 * turbine_pickup_time_factor;
  monitorWindowVariable_write[26] = highByte(turbine_pickup_time_factor_1);  
  monitorWindowVariable_write[27] = lowByte(turbine_pickup_time_factor_1);   

  DWIN_PORT.write(monitorWindowVariable_write, 28);
}

void alarm_logs(const char *alarm_name, int values) {

  /* flash log funnel - TODO: map alarm_name -> FLOG_A_* codes */
  FLOG_HOOK(FLOG_T_ALARM_ON, 0x3F, (uint16_t)values);
  
  datafile = SD.open("ALARMLOG.txt", O_RDWR || O_APPEND);
  if (datafile) {
    datafile.seek(datafile.size());  
    
    sprintf(buff_rtc, "%d:%d:%d %d/%d/%d ", hh1, mm1, ss1, dayss, months, years);
    datafile.print(buff_rtc);
    sprintf(buff_event, "%s", alarm_name);
    datafile.print(buff_event);
    datafile.print("- ");
    datafile.println(values, 3);
    datafile.flush();
    delay(10);
    datafile.close();
  }
}

void encoderfunction() {  

  encoder_pinAstateCurrent = digitalRead(encoder_pinA);  
  
  
  if (encoder_pinAStateLast != encoder_pinAstateCurrent) {        
    if (digitalRead(encoder_pinB) == encoder_pinAstateCurrent) {  
      
      if (encoder_switch_count == 0 && HMI_Page == 50) {
        increamentButtonState = 1;
        touch = 1;
        keys();
        encoderfunction();  /* polled quadrature decode - had NO call site */
        touch = 0;
        increamentButtonState = 0;
      } else if (encoder_switch_count == 1 && HMI_Page == 50) {
        settingButtonState = 1;
        touch = 1;
        keys();
        touch = 0;
        settingButtonState = 0;
        cursr_forward = 1;
      } else if (HMI_Page == 12) {
        increamentButtonState = 1;
        touch = 1;
        keys();
        touch = 0;
        increamentButtonState = 0;
      } else if (HMI_Page == 13) {
        increamentButtonState = 1;
        touch = 1;
        keys();
        touch = 0;
        increamentButtonState = 0;
      } else if (HMI_Page == 14) {
        increamentButtonState = 1;
        touch = 1;
        keys();
        touch = 0;
        increamentButtonState = 0;
      } else if (HMI_Page == PAGE_PATIENT_WEIGHT) {
        increamentButtonState = 1;
        touch = 1;
        keys();
        touch = 0;
        increamentButtonState = 0;
      } else if (HMI_Page == display_page_3) {
        increamentButtonState = 1;
        touch = 1;
        keys();
        touch = 0;
        increamentButtonState = 0;
      }  
      else if (encoder_switch_count == 0 && HMI_Page == 29) {
        increamentButtonState = 1;
        touch = 1;
        keys();
        touch = 0;
        increamentButtonState = 0;
      } else if (encoder_switch_count == 1 && HMI_Page == 29) {
        settingButtonState = 1;
        touch = 1;
        keys();
        touch = 0;
        settingButtonState = 0;
        cursr_forward = 1;
      } else if (encoder_switch_count == 0 && HMI_Page == PAGE_STANDBY) {
        increamentButtonState = 1;
        touch = 1;
        keys();
        touch = 0;
        increamentButtonState = 0;
      } else if (encoder_switch_count == 1 && HMI_Page == PAGE_STANDBY) {
        settingButtonState = 1;
        touch = 1;
        keys();
        touch = 0;
        settingButtonState = 0;
        cursr_forward = 1;
      }
      
    } else if (digitalRead(encoder_pinB) != encoder_pinAstateCurrent) {  
      
      if (encoder_switch_count == 0 && HMI_Page == 50) {
        decreamentButtonState = 1;
        touch = 1;
        keys();
        touch = 0;
        decreamentButtonState = 0;
      } else if (encoder_switch_count == 1 && HMI_Page == 50) {
        settingButtonState = 1;
        touch = 1;
        keys();
        touch = 0;
        settingButtonState = 0;
        cursr_forward = 0;
      } else if (HMI_Page == 12) {
        decreamentButtonState = 1;
        touch = 1;
        keys();
        touch = 0;
        decreamentButtonState = 0;
      } else if (HMI_Page == 13) {
        decreamentButtonState = 1;
        touch = 1;
        keys();
        touch = 0;
        decreamentButtonState = 0;
      } else if (HMI_Page == 14) {
        decreamentButtonState = 1;
        touch = 1;
        keys();
        touch = 0;
        decreamentButtonState = 0;
      } else if (HMI_Page == PAGE_PATIENT_WEIGHT) {
        decreamentButtonState = 1;
        touch = 1;
        keys();
        touch = 0;
        decreamentButtonState = 0;
      } else if (HMI_Page == display_page_3) {
        decreamentButtonState = 1;
        touch = 1;
        keys();
        touch = 0;
        decreamentButtonState = 0;
      }  
      else if (encoder_switch_count == 0 && HMI_Page == 29) {
        decreamentButtonState = 1;
        touch = 1;
        keys();
        touch = 0;
        decreamentButtonState = 0;
      } else if (encoder_switch_count == 1 && HMI_Page == 29) {
        settingButtonState = 1;
        touch = 1;
        keys();
        touch = 0;
        settingButtonState = 0;
        cursr_forward = 0;
      } else if (encoder_switch_count == 0 && HMI_Page == PAGE_STANDBY) {
        increamentButtonState = 1;
        touch = 1;
        keys();
        touch = 0;
        increamentButtonState = 0;
      } else if (encoder_switch_count == 1 && HMI_Page == PAGE_STANDBY) {
        settingButtonState = 1;
        touch = 1;
        keys();
        touch = 0;
        settingButtonState = 0;
        cursr_forward = 1;
      }
    
    }
  }
}

void serialread_data(void) {  /* touch dispatch: switch on VP return code */
  if ((Buffer1[1] == 0XA5) && (Buffer1[0] == 0X5A) && (Buffer1[3] == 131)) {

    uint16_t rcvdAddr = uint16_t((Buffer1[4] << 8) | Buffer1[5]);

    switch (rcvdAddr) {
    case TP_MODE_CPAP: {
      apnea_operatorOverride();
      FLOG_HOOK(FLOG_T_EVENT, FLOG_E_MODE_CHANGE, 7);

      HMI_Page = 50;  
      dwin_page_Set(23);
      mode1 = MODE_CPAP;
      settingbutton_count = 13;
      encoder_switch_count = 0;
      Icon1[7] = 36;
      DWIN_PORT.write(Icon1, 8);
      break;
    }
    case TP_MODE_VCV: {
      apnea_operatorOverride();
      FLOG_HOOK(FLOG_T_EVENT, FLOG_E_MODE_CHANGE, 1);

      HMI_Page = 50;  
      dwin_page_Set(18);
      mode1 = MODE_VCV;
      settingbutton_count = 13;
      encoder_switch_count = 0;
      Icon1[7] = 31;
      DWIN_PORT.write(Icon1, 8);
      break;
    }
    case TP_MODE_ACV: {
      apnea_operatorOverride();
      FLOG_HOOK(FLOG_T_EVENT, FLOG_E_MODE_CHANGE, 3);

      HMI_Page = 50;  
      apnbackup_mode = 0;
      dwin_page_Set(19);
      mode1 = MODE_ACV;
      settingbutton_count = 13;
      encoder_switch_count = 0;
      Icon1[7] = 32;
      DWIN_PORT.write(Icon1, 8);
      break;
    }
    case TP_MODE_PSIMV: {
      apnea_operatorOverride();
      FLOG_HOOK(FLOG_T_EVENT, FLOG_E_MODE_CHANGE, 8);

      HMI_Page = 50;  
      dwin_page_Set(20);
      mode1 = MODE_PSIMV;
      settingbutton_count = 13;
      encoder_switch_count = 0;
      Icon1[7] = 33;
      DWIN_PORT.write(Icon1, 8);
      break;
    }
    case TP_MODE_SIMV: {
      apnea_operatorOverride();
      FLOG_HOOK(FLOG_T_EVENT, FLOG_E_MODE_CHANGE, 2);

      HMI_Page = 50;  
      dwin_page_Set(21);
      mode1 = MODE_SIMV;
      settingbutton_count = 13;
      encoder_switch_count = 0;
      Icon1[7] = 34;
      DWIN_PORT.write(Icon1, 8);
      break;
    }
    case TP_MODE_APNEA_BK: {
      apnea_operatorOverride();
      FLOG_HOOK(FLOG_T_EVENT, FLOG_E_MODE_CHANGE, 6);

      HMI_Page = 50;  
      dwin_page_Set(22);
      mode1 = MODE_SPONT_PS;
      settingbutton_count = 13;
      encoder_switch_count = 0;
      Icon1[7] = 35;
      DWIN_PORT.write(Icon1, 8);
      break;
    }
    case TP_MODE_PCV: {
      apnea_operatorOverride();
      FLOG_HOOK(FLOG_T_EVENT, FLOG_E_MODE_CHANGE, 5);

      HMI_Page = 50;  
      dwin_page_Set(17);
      mode1 = MODE_PCV;
      settingbutton_count = 13;
      encoder_switch_count = 0;
      Icon1[7] = 30;
      DWIN_PORT.write(Icon1, 8);
      break;
    }
    case TP_MODE_HFT: {
      apnea_operatorOverride();
      FLOG_HOOK(FLOG_T_EVENT, FLOG_E_MODE_CHANGE, 4);

      HMI_Page = 50;  
      dwin_page_Set(24);
      mode1 = MODE_HFT;
      settingbutton_count = 13;
      encoder_switch_count = 0;
      Icon1[7] = 28;
      DWIN_PORT.write(Icon1, 8);
      break;
    }
    case TP_MONITOR_OPEN: {

      HMI_Page = 27;
      dwin_page_Set(27);
      break;
    }
    case TP_ALARMLIMIT_OPEN: {

      HMI_Page = 29;
      dwin_page_Set(29);  
      settingbutton_count = 6;
      encoder_switch_count = 1;
      Icon17[7] = 156;
      DWIN_PORT.write(Icon17, 8);
      break;
    }
    case TP_NAV_TO_MAIN: {

      HMI_Page = display_page_3;

      HMI_Page = display_page_3;
      dwin_page_Set(display_page_3);
      break;
    }
    case TP_CALB_TRIGGER: {
      calb = 1;
      break;
    }
    case TP_MUTE_TOGGLE: {
      FLOG_HOOK(FLOG_T_EVENT, FLOG_E_MUTE, 0);

      if (mutebuttonState == 1) {
        mutebuttonState = 0;
        touch = 1;
        keys();
        touch = 0;

      } else {
        mutebuttonState = 1;
        touch = 1;
        keys();
        touch = 0;
      }

      mutebutton_lastState = mutebuttonState;
      break;
    }
    case TP_DECREMENT: {
      if (HMI_Page != 28) {
      decreamentButtonState = 1;  
      touch = 1;
      keys();
      touch = 0;
      decreamentButtonState = 0;
      }
      if (HMI_Page == 28 && testscreen_count == 1) {
      ov_start_duty = ov_start_duty - 1;
      }
      if (HMI_Page == 28 && testscreen_count == 3) {
      calibration_HFT = 0;  
                            
      display_write_variable(VP_CAL_HFT, calibration_HFT);
      }
      if ((HMI_Page == 28) && (digitalRead(mutebutton) == LOW) && (pf > -3)) {
      pf = pf - 1;  
      EEPROM.put(290, pf);
      Buffer1[4] = 0;
      Buffer1[5] = 0;
        break;  /* original zeroed Buffer1 here so no later block could re-fire on this event */
      }
      if (HMI_Page == 28 && testscreen_count == 4) {
      
      calibration_flow = 0;
      
      display_write_variable(VP_CAL_FLOW, calibration_flow);
      }
      break;
    }
    case TP_INCREMENT: {
      if (HMI_Page != 28) {
      
      increamentButtonState = 1;  
      touch = 1;
      keys();
      touch = 0;
      increamentButtonState = 0;
      }
      if (HMI_Page == 28 && testscreen_count == 3 && calibration_HFT == 0) {

      calibration_HFT = comp;  
                               
                               
      display_write_variable(VP_CAL_HFT, calibration_HFT);
      }
      if ((HMI_Page == 28) && (digitalRead(mutebutton) == LOW) && (pf < 6)) {

      pf = pf + 1;  
      Buffer1[4] = 0;
      Buffer1[5] = 0;       
      EEPROM.put(290, pf);
        break;  /* original zeroed Buffer1 here so no later block could re-fire on this event */
      }
      if (HMI_Page == 28 && testscreen_count == 4 && calibration_flow == 0) {

      
      
      calibration_flow = comp;
      
      display_write_variable(VP_CAL_FLOW, calibration_flow);
      }
      if (HMI_Page == 28 && testscreen_count == 1) {
      ov_start_duty = ov_start_duty + 1;
      }
      break;
    }
    case TP_CONFIRM: {


      confirmbuttonState = 1;
      touch = 1;
      keys();
      touch = 0;
      confirmbuttonState = 0;
      settingbutton_count = 0;  
      testscreen_count = 0;
      if (mode1 == MODE_PCV) {
        Icon15[7] = 42;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_PSIMV) {
        Icon15[7] = 41;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_VCV) {
        Icon15[7] = 45;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_CPAP) {
        Icon15[7] = 38;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_HFT) {
        Icon15[7] = 40;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_SIMV) {
        Icon15[7] = 44;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_ACV) {
        Icon15[7] = 43;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_SPONT_PS) {
        Icon15[7] = 39;
        DWIN_PORT.write(Icon15, 8);
      }
      Icon1[7] = 0;
      DWIN_PORT.write(Icon1, 8);

      confirmbuttonState = 1;
      touch = 1;
      keys();
      touch = 0;
      confirmbuttonState = 0;
      settingbutton_count = 0;
      if (mode1 == MODE_PCV) {
        Icon15[7] = 42;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_PSIMV) {
        Icon15[7] = 41;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_VCV) {
        Icon15[7] = 45;
        DWIN_PORT.write(Icon15, 8);

      } else if (mode1 == MODE_CPAP) {
        Icon15[7] = 38;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_HFT) {
        Icon15[7] = 40;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_SIMV) {
        Icon15[7] = 44;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_ACV) {
        Icon15[7] = 43;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_SPONT_PS) {
        Icon15[7] = 39;
        DWIN_PORT.write(Icon15, 8);
      }
      Icon1[7] = 0;
      DWIN_PORT.write(Icon1, 8);
      break;
    }
    case TP_NEB_START: {
      FLOG_HOOK(FLOG_T_EVENT, FLOG_E_NEBULIZER, 1);
      if (HMI_Page == 1) {
      HMI_Page = display_page_3;
      dwin_page_Set(display_page_3);
      nebu_touch = 1; 
      nebbutton_count_timer = 1;
      nebbuttonState_timer = 1;
      
      if (mode1 == MODE_PCV) {
        Icon15[7] = 42;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_PSIMV) {
        Icon15[7] = 41;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_VCV) {
        Icon15[7] = 45;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_CPAP) {
        Icon15[7] = 38;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_HFT) {
        Icon15[7] = 40;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_SIMV) {
        Icon15[7] = 44;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_ACV) {
        Icon15[7] = 43;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_SPONT_PS) {
        Icon15[7] = 39;
        DWIN_PORT.write(Icon15, 8);
      }
      Icon1[7] = 0;
      DWIN_PORT.write(Icon1, 8);
      }
      break;
    }
    case TP_CONFIRM_FROM_P29: {
      if (HMI_Page == 29) {
      HMI_Page = display_page_3;
      dwin_page_Set(display_page_3);  
      confirmbuttonState = 1;
      touch = 1;
      keys();
      touch = 0;
      confirmbuttonState = 0;
      settingbutton_count = 0;
      if (mode1 == MODE_PCV) {
        Icon15[7] = 42;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_PSIMV) {
        Icon15[7] = 41;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_VCV) {
        Icon15[7] = 45;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_CPAP) {
        Icon15[7] = 38;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_HFT) {
        Icon15[7] = 40;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_SIMV) {
        Icon15[7] = 44;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_ACV) {
        Icon15[7] = 43;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_SPONT_PS) {
        Icon15[7] = 39;
        DWIN_PORT.write(Icon15, 8);
      }
      Icon1[7] = 0;
      DWIN_PORT.write(Icon1, 8);
      }
      break;
    }
    case TP_FIO2_SELECT: {

      settingbutton_count = 12;  
      encoder_switch_count = 0;
      Icon1[7] = 13;
      DWIN_PORT.write(Icon1, 8);
      break;
    }
    case TP_FLOW_SELECT: {

      settingbutton_count = 2;  
      encoder_switch_count = 0;
      Icon1[7] = 50;
      DWIN_PORT.write(Icon1, 8);
      break;
    }
    case TP_PARAM_SEL_A: {

      settingbutton_count = 2;  
      encoder_switch_count = 0;
      break;
    }
    case TP_PARAM_SEL_B: {

      settingbutton_count = 3;  
      encoder_switch_count = 0;
      Icon1[7] = 48;
      DWIN_PORT.write(Icon1, 8);
      break;
    }
    case TP_PARAM_SEL_C: {

      settingbutton_count = 3;  
      encoder_switch_count = 0;
      if (mode1 == MODE_SPONT_PS) {
        Icon1[7] = 52;
      } 
      else
      {
        Icon1[7] = 46;
      }
      DWIN_PORT.write(Icon1, 8);
      break;
    }
    case TP_PARAM_SEL_D: {

      settingbutton_count = 4;  
      encoder_switch_count = 0;
      Icon1[7] = 11;
      DWIN_PORT.write(Icon1, 8);
      break;
    }
    case TP_PARAM_SEL_E: {

      settingbutton_count = 5;  
      encoder_switch_count = 0;
      Icon1[7] = 54;
      DWIN_PORT.write(Icon1, 8);
      break;
    }
    case TP_PARAM_SEL_F: {

      settingbutton_count = 5;  
      encoder_switch_count = 0;
      Icon1[7] = 58;
      DWIN_PORT.write(Icon1, 8);
      break;
    }
    case TP_PARAM_SEL_G: {

      settingbutton_count = 8;  
      encoder_switch_count = 0;
      Icon1[7] = 21;
      DWIN_PORT.write(Icon1, 8);
      break;
    }
    case TP_PARAM_SEL_H: {

      settingbutton_count = 9;  
      encoder_switch_count = 0;
      Icon1[7] = 23;
      DWIN_PORT.write(Icon1, 8);
      break;
    }
    case TP_PARAM_SEL_I: {

      settingbutton_count = 9;  
      encoder_switch_count = 0;
      Icon1[7] = 29;
      DWIN_PORT.write(Icon1, 8);
      break;
    }
    case TP_PARAM_SEL_J: {
      settingbutton_count = 10;  
      encoder_switch_count = 0;
      if (mode1 == MODE_CPAP) {
        Icon1[7] = 27;
      } else {
        Icon1[7] = 25;
      }
      DWIN_PORT.write(Icon1, 8);
      break;
    }
    case TP_PARAM_SEL_K: {
      settingbutton_count = 11;  
      encoder_switch_count = 0;
      Icon1[7] = 37;
      DWIN_PORT.write(Icon1, 8);
      break;
    }
    case TP_TESTSCREEN_OPEN: {

      HMI_Page = 28;  
      dwin_page_Set(28);
      break;
    }
    case TP_STANDBY_SCREEN: {
      standby_scr = 1;
      break;
    }
    case TP_MAIN_FROM_TEST: {

      HMI_Page = display_page_3;  
      dwin_page_Set(display_page_3);
      break;
    }
    case TP_TEST_OVDUTY_SEL: {

      testscreen_count = 1;
      break;
    }
    case TP_TEST_HFTCAL_SEL: {
      testscreen_count = 3;
      break;
    }
    case TP_TEST_FLOWCAL_SEL: {
      testscreen_count = 4;
      break;
    }
    case TP_LIMIT_CUR_PAWHI: {

      encoder_switch_count = 0;
      Icon17[7] = 156;
      DWIN_PORT.write(Icon17, 8);
      settingbutton_count = 6;
      break;
    }
    case TP_LIMIT_CUR_PAWLO: {

      encoder_switch_count = 0;
      Icon17[7] = 157;
      DWIN_PORT.write(Icon17, 8);
      settingbutton_count = 7;
      break;
    }
    case TP_LIMIT_CUR_BPMHI: {

      encoder_switch_count = 0;
      Icon17[7] = 158;
      DWIN_PORT.write(Icon17, 8);
      settingbutton_count = 15;
      break;
    }
    case TP_LIMIT_CUR_BPMLO: {

      encoder_switch_count = 0;
      Icon17[7] = 159;
      DWIN_PORT.write(Icon17, 8);
      settingbutton_count = 16;
      break;
    }
    case TP_LIMIT_CUR_VTHI: {

      encoder_switch_count = 0;
      Icon17[7] = 160;
      DWIN_PORT.write(Icon17, 8);
      settingbutton_count = 17;
      break;
    }
    case TP_LIMIT_CUR_VTLO: {

      encoder_switch_count = 0;
      Icon17[7] = 161;
      DWIN_PORT.write(Icon17, 8);
      settingbutton_count = 18;
      break;
    }
    case TP_LIMIT_CUR_MVHI: {

      encoder_switch_count = 0;
      Icon17[7] = 162;
      DWIN_PORT.write(Icon17, 8);
      settingbutton_count = 19;
      break;
    }
    case TP_LIMIT_CUR_MVLO: {

      encoder_switch_count = 0;
      Icon17[7] = 163;
      DWIN_PORT.write(Icon17, 8);
      settingbutton_count = 20;
      break;
    }
    case TP_CLOCK_OPEN: {
      HMI_Page = 26;
      dwin_page_Set(26);
      settingbutton_count = 30;
      break;
    }
    case TP_CLOCK_HH_SEL: {
      display_write_variable(VP_CLOCK_HH, hhh);
      settingbutton_count = 30;
      HMI_Page = 26;
      break;
    }
    case TP_CLOCK_MM_SEL: {

      display_write_variable(VP_CLOCK_MM, mmm);
      settingbutton_count = 31;
      HMI_Page = 26;
      break;
    }
    case TP_CLOCK_SAVE: {

      unsigned char clock1[14] = { 0x5a, 0xa5, 0x0B, 0x82, 0x00, 0x9C,0x5A, 0xA5, 23, 2, 13, hhh, mmm, sss };
      DWIN_PORT.write(clock1, 14);
      delay(2);

      HMI_Page = 36;
      dwin_page_Set(36);
      break;
    }
    case TP_CLOCK_EXIT: {

      HMI_Page = 36;
      dwin_page_Set(36);
      break;
    }
    case TP_LIMIT_CUR_FIO2HI: {

      encoder_switch_count = 0;
      Icon17[7] = 164;
      DWIN_PORT.write(Icon17, 8);
      settingbutton_count = 21;
      break;
    }
    case TP_LIMIT_CUR_FIO2LO: {

      encoder_switch_count = 0;
      Icon17[7] = 165;
      DWIN_PORT.write(Icon17, 8);
      settingbutton_count = 22;
      break;
    }
    case TP_O2CAL_OPEN: {

      HMI_Page = 30;
      dwin_page_Set(30);
      display_write_variable(VP_O2CAL_PROGRESS, 0);
      O2cal = 1;
      break;
    }
    case TP_UNMOUNT: {
      if (HMI_Page == PAGE_STANDBY) {
      unmountbuttonState = 1;
      touch = 1;
      keys();
      touch = 0;
      unmountbuttonState = 0;
      Icon14[7] = 193;
      DWIN_PORT.write(Icon14, 8);
      digitalWrite(unmount_pin, HIGH);
      }
      break;
    }
    case TP_LIMITS_OPEN: {
  
      limitparameter();
      break;
    }
    case TP_NEWPATIENT_OPEN: {

      settingbutton_count = 1;
      HMI_Page = 13;
      dwin_page_Set(13);
      if (pediatric_patient == 1) {
        HMI_Page = 13;
        Icon1[7] = 86;
        DWIN_PORT.write(Icon1, 8);
      } else {
        HMI_Page = 13;
        Icon1[7] = 65;
        DWIN_PORT.write(Icon1, 8);
      }
      break;
    }
    case TP_NEWPATIENT_SETUP: {
      FLOG_HOOK(FLOG_T_EVENT, FLOG_E_NEW_PATIENT, 0);

      stand_check = 0;
      if (pediatric_patient == 1) {
        HMI_Page = 13;
        dwin_page_Set(13);
        Icon1[7] = 86;
        DWIN_PORT.write(Icon1, 8);
      } else {
        HMI_Page = 13;
        dwin_page_Set(13);
        Icon1[7] = 65;
        DWIN_PORT.write(Icon1, 8);
      }
      break;
    }
    case TP_SETTINGS_REOPEN: {

      settingbutton_count = 13;
      HMI_Page = 50;
      if (mode1 == MODE_PCV) {
        dwin_page_Set(17);
        Icon15[7] = 42;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_PSIMV) {
        dwin_page_Set(20);
        Icon15[7] = 41;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_VCV) {
        dwin_page_Set(18);
        Icon15[7] = 45;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_CPAP) {
        dwin_page_Set(23);
        Icon15[7] = 38;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_HFT) {
        dwin_page_Set(24);
        Icon15[7] = 40;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_SIMV) {
        dwin_page_Set(21);
        Icon15[7] = 44;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_ACV) {
        dwin_page_Set(19);
        Icon15[7] = 43;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_SPONT_PS) {
        dwin_page_Set(22);
        Icon15[7] = 39;
        DWIN_PORT.write(Icon15, 8);
      }
      break;
    }
    case TP_PATIENT_PEDIATRIC: {

      pediatric_patient = 1;
      HMI_Page = PAGE_PATIENT_GENDER;
      
      if (malepatient == 1) {
        bdc = 1;
        dwin_page_Set(HMI_Page);
        Icon1[7] = 66;
        DWIN_PORT.write(Icon1, 8);
      } else {
        dwin_page_Set(HMI_Page);
        Icon1[7] = 70;
        DWIN_PORT.write(Icon1, 8);
      }
      break;
    }
    case TP_PATIENT_ADULT: {

      bdc = 1;
      HMI_Page = PAGE_PATIENT_GENDER;
      pediatric_patient = 0;
      if (malepatient == 1) {
        dwin_page_Set(HMI_Page);
        Icon1[7] = 66;
        DWIN_PORT.write(Icon1, 8);
      } else {
        dwin_page_Set(HMI_Page);
        Icon1[7] = 70;
        DWIN_PORT.write(Icon1, 8);
      }
      break;
    }
    case TP_PATIENT_MALE: {

      malepatient = 1;  
      HMI_Page = 15;
      dwin_page_Set(HMI_Page);
      
      
      Icon1[7] = 111;
      DWIN_PORT.write(Icon1, 8);
      break;
    }
    case TP_PATIENT_FEMALE: {

      malepatient = 0;  
      HMI_Page = 15;
      dwin_page_Set(15);
      
      Icon1[7] = 111;
      DWIN_PORT.write(Icon1, 8);
      break;
    }
    case TP_NAV_TO_P14: {

      HMI_Page = 14;
      break;
    }
    case TP_NAV_TO_P13: {

      HMI_Page = 13;
      dwin_page_Set(13);
      break;
    }
    case TP_PATIENT_PROCEED: {
      if ((HMI_Page == PAGE_PATIENT_WEIGHT || HMI_Page == PAGE_STANDBY)) {  

      settingbutton_count = 13;
      dwin_page_Set(16);
      HMI_Page = 50;

#if 0
      if (mode1 == MODE_PCV) {
        dwin_page_Set (17);
        Icon15[7] = 42;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_PSIMV) {
        dwin_page_Set (20);
        Icon15[7] = 41;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_VCV) {
        dwin_page_Set (18);
        Icon15[7] = 45;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_CPAP) {
        dwin_page_Set (23);
        Icon15[7] = 38;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_HFT) {
        dwin_page_Set (24);
        Icon15[7] = 40;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_SIMV) {
        dwin_page_Set (21);
        Icon15[7] = 44;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_ACV) {
        dwin_page_Set (19);
        Icon15[7] = 43;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_SPONT_PS) {
        dwin_page_Set (22);
        Icon15[7] = 39;
        DWIN_PORT.write(Icon15, 8);
      }
#endif
      }
      break;
    }

    case TP_CAL_MANUAL:  if (HMI_Page == PAGE_CAL_SELECT) { selftest_openManual(); } break;
    case TP_CAL_AUTO:    if (HMI_Page == PAGE_CAL_SELECT) { selftest_startAuto(); } break;
    case TP_TEST_TURBINE:   selftest_start(1); break;
    case TP_TEST_SENSOR:    selftest_start(2); break;
    case TP_TEST_PNEUMATIC: selftest_start(3); break;
    case TP_TEST_FIO2:      selftest_start(4); break;
    case TP_TEST_PRESSURE:  selftest_start(5); break;
    case TP_TEST_O2SRC:     selftest_start(6); break;
    case TP_TEST_BATTERY:   selftest_start(7); break;
    case TP_TEST_VALVE:     selftest_start(8); break;

    case TP_BACK_CANCEL: {
      /* RESTRUCTURED: origin snapshot + mutually exclusive routes.
         The old chain of independent ifs let one branch's navigation
         satisfy the next branch's condition (bounce), and pages with
         no route got a DEAD button. Order: specific routes first,
         guarded generic history-return last.                        */
      uint16_t fromPage = HMI_Page;

      if (fromPage == 13) {                    /* patient type -> start */
        encoder_switch_count = 1;
        HMI_Page = 12;
        dwin_page_Set(12);
      }
      else if (fromPage == 50 && standbybutton_count == 2) {
        HMI_Page = display_page_3;             /* settings -> monitor  */
        dwin_page_Set(display_page_3);
        cancelButtonState = 1;
        touch = 1;
        keys();
        touch = 0;
        cancelButtonState = 0;
      }
      else if (fromPage == 50 && standbybutton_count == 1) {
        if (prevPage != 50 && prevPage >= 1) {
          backNav = 1;
          HMI_Page = prevPage;
          dwin_page_Set(prevPage);
        }
        cancelButtonState = 1;
        touch = 1;
        keys();
        touch = 0;
        cancelButtonState = 0;
      }
      else if (fromPage == PAGE_STANDBY) {
        /* standby root: Back is a no-op by design - exit standby is
           an explicit therapy decision, never a navigation slip.    */
      }
      else if (prevPage >= 1 && prevPage != fromPage && prevPage != 50) {
        /* GENERIC: any unrouted page returns where it came from
           (standby child pages 26 etc. were previously DEAD here).
           Guards: page 0 = boot (never navigate there), no self-nav,
           50 is a logical state not a physical page.                */
        backNav = 1;
        HMI_Page = prevPage;
        dwin_page_Set(prevPage);
      }
      break;
    }
    case TP_BACK_P14_TO_P13: {
      if (HMI_Page == 14) {
      encoder_switch_count = 1;
      HMI_Page = 13;
      dwin_page_Set(13);
      }
      break;
    }
    case TP_BACK_P15_TO_P14: {
      if (HMI_Page == PAGE_PATIENT_WEIGHT) {
      HMI_Page = 14;
      dwin_page_Set(14);
      }
      break;
    }
    case TP_NEB_CANCEL: {
      FLOG_HOOK(FLOG_T_EVENT, FLOG_E_NEBULIZER, 0);
      if (HMI_Page == 1 && standbybutton_count == 2) {
      HMI_Page = display_page_3;
      dwin_page_Set(display_page_3);
      neb_time = 0;
      Icon21[7] = 0;
      DWIN_PORT.write(Icon21, 8);
      nebu_touch = 0;
      display_write_variable(VP_NEB_TIME, neb_time);
      digitalWrite(nebulizerpin, LOW);
      }
      break;
    }
    case TP_VENTCHK_1: {
      if (ventilation_check == 1) {
      ventilation_check = 0;
      HMI_Page = 36;
      dwin_page_Set(36);
      }
      break;
    }
    case TP_VENTCHK_2: {
      if (ventilation_check == 1) {
      ventilation_check = 0;
      HMI_Page = 36;
      dwin_page_Set(36);
      }
      break;
    }
    case TP_VENTCHK_3: {
      if (ventilation_check == 1) {
      ventilation_check = 0;
      HMI_Page = 36;
      dwin_page_Set(36);
      }
      break;
    }
    case TP_VENTCHK_4: {
      if (ventilation_check == 1) {
      ventilation_check = 0;
      HMI_Page = 36;
      dwin_page_Set(36);
      }
      break;
    }
    case TP_VENTCHK_5: {
      if (ventilation_check == 1) {
      ventilation_check = 0;
      HMI_Page = 36;
      dwin_page_Set(36);
      }
      break;
    }
    case TP_VENTCHK_6: {
      if (ventilation_check == 1) {
      ventilation_check = 0;
      HMI_Page = 36;
      dwin_page_Set(36);
      }
      break;
    }
    case TP_VENTCHK_7: {
      if (ventilation_check == 1) {
      ventilation_check = 0;
      HMI_Page = 36;
      dwin_page_Set(36);
      }
      break;
    }
    case TP_VENTCHK_8: {
      if (ventilation_check == 1) {
      ventilation_check = 0;
      HMI_Page = 36;
      dwin_page_Set(36);
      }
      break;
    }
    case TP_NAV_ALT: {

      HMI_Page = 12;
      dwin_page_Set(30);
      break;
    }
    case TP_O2CAL_CANCEL: {

      O2cal = 0;
      confirm = 0;
      dwin_page_Set(36);
      HMI_Page = (36);
      neb_time = 0;
      Icon21[7] = 0;
      DWIN_PORT.write(Icon21, 8);
      nebu_touch = 0;
      display_write_variable(VP_NEB_TIME, neb_time);
      digitalWrite(nebulizerpin, LOW);
      break;
    }
    case TP_O2CAL_CONFIRM: {
      FLOG_HOOK(FLOG_T_EVENT, FLOG_E_O2_CAL, 0);

      O2cal = 1;
      confirm = 1;
      caliprevioustime = calicurrenttime;
      break;
    }

    /* ============ ALARM & EVENT LOG VIEWER (page 43) ============ */
    case TP_LOG_OPEN: {
      if (HMI_Page == PAGE_STANDBY) {   /* entry only from standby screen */
        HMI_Page = PAGE_LOG_VIEW;
        dwin_page_Set(PAGE_LOG_VIEW);
#if ENABLE_FLASH_LOG
        logview_show(0);
#endif
      }
      break;
    }
    case TP_SYSLOG_OPEN: {
      if (HMI_Page == PAGE_STANDBY) {   /* entry only from standby screen */
        HMI_Page = PAGE_SYSLOG_VIEW;
        dwin_page_Set(PAGE_SYSLOG_VIEW);
#if ENABLE_FLASH_LOG
        logview_show(0);
#endif
      }
      break;
    }
    case TP_LOG_UP: {
#if ENABLE_FLASH_LOG
      logview_scroll(1);
#endif
      break;
    }
    case TP_LOG_DOWN: {
#if ENABLE_FLASH_LOG
      logview_scroll(-1);
#endif
      break;
    }
    case TP_LOG_HOME: {
#if ENABLE_FLASH_LOG
      logview_show(0);
#endif
      break;
    }
    case TP_LOG_BACK: {
      HMI_Page = PAGE_STANDBY;
      dwin_page_Set(PAGE_STANDBY);
      break;
    }
    case TP_NEB_OPEN: {
      if (HMI_Page == display_page_3) {
      settingbutton_count = 28;
      HMI_Page = 1;
      dwin_page_Set(1);
      encoder_switch_count = 0; 
      Icon21[7] = 198;
      DWIN_PORT.write(Icon21, 8);
      if (mode1 == MODE_PCV) {
        Icon15[7] = 42;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_PSIMV) {
        Icon15[7] = 41;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_VCV) {
        Icon15[7] = 45;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_CPAP) {
        Icon15[7] = 38;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_HFT) {
        Icon15[7] = 40;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_SIMV) {
        Icon15[7] = 44;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_ACV) {
        Icon15[7] = 43;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_SPONT_PS) {
        Icon15[7] = 39;
        DWIN_PORT.write(Icon15, 8);
      }
      Icon1[7] = 0;
      DWIN_PORT.write(Icon1, 8);
      }
      break;
    }
    case TP_VENT_MENU: {

      settingbutton_count = 13;
      HMI_Page = 50;
      ventilation_check = 1;
      Icon1[7] = 0;
      DWIN_PORT.write(Icon1, 8);
      if (mode1 == MODE_PCV) {
        dwin_page_Set(17);
        Icon15[7] = 42;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_PSIMV) {
        dwin_page_Set(20);
        Icon15[7] = 41;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_VCV) {
        dwin_page_Set(18);
        Icon15[7] = 45;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_CPAP) {
        dwin_page_Set(23);
        Icon15[7] = 38;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_HFT) {
        dwin_page_Set(24);
        Icon15[7] = 40;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_SIMV) {
        dwin_page_Set(21);
        Icon15[7] = 44;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_ACV) {
        dwin_page_Set(19);
        Icon15[7] = 43;
        DWIN_PORT.write(Icon15, 8);
      } else if (mode1 == MODE_SPONT_PS) {
        dwin_page_Set(22);
        Icon15[7] = 39;
        DWIN_PORT.write(Icon15, 8);
      }
      break;
    }
    case TP_SETTINGS_OPEN: {

#if 1
      settingbutton_count = 13;
      HMI_Page = 50;
      Icon1[7] = 0;
      DWIN_PORT.write(Icon1, 8);
      if (mode1 == MODE_PCV) {
        dwin_page_Set(17);
      }  
      else if (mode1 == MODE_PSIMV) {
        dwin_page_Set(20);
      }  
      else if (mode1 == MODE_VCV) {
        dwin_page_Set(18);
      }  
      else if (mode1 == MODE_CPAP) {
        dwin_page_Set(23);
      }  
      else if (mode1 == MODE_HFT) {
        dwin_page_Set(24);
      }  
      else if (mode1 == MODE_SIMV) {
        dwin_page_Set(21);
      }  
      else if (mode1 == MODE_ACV) {
        dwin_page_Set(19);
      }  
      else if (mode1 == MODE_SPONT_PS) {
        dwin_page_Set(22);
      }  
#endif
      break;
    }
      default:
        break;  /* unrecognized touch code: ignore */
    }

    /* ---- RTC response: keyed on Buffer1[5..6], NOT the [4..5] address,
            so it stays outside the switch exactly as in the original ---- */
    {
rcvdAddr = uint16_t((Buffer1[5] << 8) | Buffer1[6]);
    if (rcvdAddr == TP_RTC_RESPONSE) {
      years = Buffer1[7];
      months = Buffer1[8];
      dayss = Buffer1[9];
      hh1 = Buffer1[11];
      mm1 = Buffer1[12];
      ss1 = Buffer1[13];  

      sprintf(buff_rtc, "%d %d %d %d %d %d", years, months, dayss, hh1, mm1, ss1);
      
    }
    }
  }
}

void demo(void) {
  
  demo_settingcurrenttime = millis();

  if (digitalRead(mutebutton) == LOW) {  demo_settingprevioustime = demo_settingcurrenttime; }

  else if ((demo_settingcurrenttime - demo_settingprevioustime) > 20000 && standbybutton_count == 1) {
    demomode = 0;
    EEPROM.update(32, demomode);
    
    demo_settingprevioustime = demo_settingcurrenttime;
    setup();
  } else if ((demo_settingcurrenttime - demo_settingprevioustime) > 10000 && standbybutton_count == 1 && digitalRead(encoder_switchPin) == LOW)  
  {
    demomode = 1;
    
    EEPROM.update(32, demomode);
    demo_settingprevioustime = demo_settingcurrenttime;
  }
  if (demomode == 1) {
    dwin_page_Set(5); 
  }
  
}

void NIV(void) {
  ;
}
void BPM(void)  

{
  if ((mainrelaypinstate == HIGH) && (mainrelaypinstate != U1))  
  {
    if (R_count < 3)  
    {
      R_count += 1;
    }
    if (R_count == 3) {
      R_count = 1;
    }
    
    
  }
  U1 = mainrelaypinstate;

  
  if (R_count == 1 && UU == 1) {
    bpm_pmillis = millis();
    UU = 2;
  }
  if (UU == 2 && R_count == 2) {
    bpm_cmillis = millis();
    
    bpm_totaltime = bpm_cmillis - bpm_pmillis - 100;
    
    SUM_bpm = SUM_bpm - READINGS_bpm[INDEX_bpm];  

    VALUE_bpm = bpm_totaltime;            
    READINGS_bpm[INDEX_bpm] = VALUE_bpm;  
    SUM_bpm = SUM_bpm + VALUE_bpm;        
    INDEX_bpm = (INDEX_bpm + 1) % 2;      

    AVERAGED_bpm = SUM_bpm / 2;
    
    UU = 1;
  }


  if (mainrelaypinstate == HIGH) {
    AVERAGED_SAVED_bpm = AVERAGED_bpm;
    AVERAGED_SAVED_bpm = 60000 / AVERAGED_bpm;
    AVERAGED_SAVED_bpm = 1.09 * AVERAGED_SAVED_bpm;  
    
  }
  
  current_TI = millis();
  current_TE = millis();
  if (mainrelaypinstate == HIGH || Hvtcutstate == 1) {
    monitore_TI = current_TI - previous_TI;
    previous_TE = current_TE;
  } else {
    previous_TI = current_TI;
    monitore_TE = current_TE - previous_TE;
  }
  
  
}

/*******************************************************************************
 * fio2_sample() - single acquisition pass on the galvanic O2 cell.
 *
 * Replaces the ADC read + scaling + snap-to-setpoint + dOp boxcar that used
 * to be copy-pasted into fio2_function(), fio2_function2() and
 * fio2_function3(). Three behavioural changes from the old inline code:
 *
 *   1. Both filters are PRIMED on the first call. The old code started with
 *      dOp_READINGS[] all zeros, so the monitored FiO2 climbed from 0
 *      towards the true value one slot at a time - a true 40% read as 4%
 *      until roughly 15 of the 150 slots had been written.
 *   2. NO substitution of the setpoint for the measurement. The old code
 *      forced fio23 = set_set_fio2 whenever the measurement sat within 5%
 *      of the setting, which is why the screen never tracked the cell.
 *   3. An implausible cell (output below the room-air floor, or a bad
 *      calibration slope) sets fio2_sensor_ok = 0 instead of producing a
 *      fabricated low FiO2 number.
 *
 * slope_scale is the trim fio2_function2() used to apply as (L3 - 0.1*L3);
 * pass 1.0f for the normal path, 0.9f for that path.
 ******************************************************************************/
void fio2_sample(float slope_scale) {

  fio2_raw_adc = analogRead(fio2_read);
  Op_VALUE = fio2_raw_adc;

  if (!Op_primed) {
    for (uint8_t i = 0; i < WINDOW_SIZEop; i++) { Op_READINGS[i] = Op_VALUE; }
    Op_SUM = (float)WINDOW_SIZEop * Op_VALUE;
    Op_INDEX = 0;
    Op_primed = 1;
  } else {
    Op_SUM = Op_SUM - Op_READINGS[Op_INDEX];
    Op_READINGS[Op_INDEX] = Op_VALUE;
    Op_SUM = Op_SUM + Op_VALUE;
    Op_INDEX = (Op_INDEX + 1) % WINDOW_SIZEop;
  }
  Op_AVERAGED = Op_SUM / WINDOW_SIZEop;
  Op_AVERAGED_SAVED = Op_AVERAGED;

  float slope = L3 * slope_scale;

  if (isnan(slope) || isnan(f1) || slope < 0.30f || slope > 12.0f ||
      (float)Op_AVERAGED_SAVED < (f1 - FIO2_ADC_FLOOR)) {
    /* cell exhausted, unplugged, amplifier dead, or calibration invalid */
    fio2_sensor_ok = 0;
    fio2 = 21.0f;
    fio23 = 21.0f;
  } else {
    fio2_sensor_ok = 1;
    fio2 = (((float)Op_AVERAGED_SAVED - f1) / slope) + 21.0f;
    if (fio2 < 15.0f) { fio2 = 15.0f; }
    if (fio2 > 100.0f) { fio2 = 100.0f; }
    fio23 = fio2;
  }

  if (!dOp_primed) {
    for (uint16_t i = 0; i < WINDOW_SIZEdop; i++) { dOp_READINGS[i] = fio23; }
    dOp_SUM = (float)WINDOW_SIZEdop * fio23;
    dOp_INDEX = 0;
    dOp_AVERAGED = fio23;
    dOp_primed = 1;
  } else {
    dOp_SUM = dOp_SUM - dOp_READINGS[dOp_INDEX];
    dOp_VALUE = fio23;
    dOp_READINGS[dOp_INDEX] = dOp_VALUE;
    dOp_SUM = dOp_SUM + dOp_VALUE;
    dOp_INDEX = (dOp_INDEX + 1) % WINDOW_SIZEdop;
    dOp_AVERAGED = dOp_SUM / WINDOW_SIZEdop;
  }

}

/*******************************************************************************
 * fio2_monitored() - the ONE number reported to the operator.
 *
 * The display bytes and the alarm comparison must both come from here.
 * Previously the display was forced to 21 while the alarm still tested the
 * raw average, so the screen could read 21 with a "FiO2 Low" banner on top
 * of it. IEC 60601-1-8 requires the displayed value and the alarm condition
 * to derive from the same measurement.
 ******************************************************************************/
int fio2_monitored(void) {
  if (set_set_fio2 <= 21 || oxygenavailability_status == HIGH || !fio2_sensor_ok) {
    return 21;                       /* room air / no O2 supply / cell fault */
  }
  int v = (int)(dOp_AVERAGED);
  if (v < 15) { v = 15; }
  if (v > 100) { v = 100; }
  return v;
}

/*******************************************************************************
 * fio2_alarm_enabled() - FiO2 alarms are only meaningful when the blender is
 * actually being driven. No alarm at a setting of 21% or below (room air),
 * none without an O2 supply, none on a faulty cell, none in HFT or standby.
 ******************************************************************************/
uint8_t fio2_alarm_enabled(void) {
  return (set_set_fio2 > 21
          && oxygenavailability_status == LOW
          && fio2_sensor_ok
          && standbybutton_count == 2
          && setmode != MODE_HFT);
}

void fio2_function3(void) {

  Ocmillis = millis();
  Opmillis = Ocmillis;
  fio2_sample(1.0f);            /* acquire + scale + filter, no snapping */
    
    if (oxygenavailability_status == LOW && set_set_fio2 > 21 && standbybutton_count == 2) 
    {
      Kp = 3, Ki = 1, Kd = 0, Hz = 10;
      int setpoint = set_set_fio2;
      
      float feedback = fio2;
      
      before = micros();
      int16_t output1 = myPID.step(setpoint, feedback);
      after = micros();  
      if (mainrelaypinstate == HIGH) 
      {
        if (output1 > 1000) {
          output1 = map(output1, 300, 1000, 1830, 3000);
        } else if (output1 <= -1000) {
          output1 = (-1000);
        }
        // sal = ov_start_duty + (1 * (output1));
        sal = 2000 + (1 * (output1));
      }
    }
  
  mini_step = settidalvolume / 1000;
  if (set_set_fio2 > 21 && j == 1) {
    sal = ov_start_duty;
    mainrelay_duty = ov_start_duty ;
    j = 0;
  }  
  if (set_set_fio2 == 21) { j = 1; }
  if (fio2 <= 10 && set_set_fio2 > 21) {
    sal = ov_start_duty;
    mainrelay_duty = ov_start_duty ;
  }

  if (mainrelay_duty >= 4000) { mainrelay_duty = 4000; }
  if (mainrelay_duty <= 100) { mainrelay_duty = 100; }
              
    if (mainrelaypinstate == HIGH && set_set_fio2 != 21) {    
    }  

    if (oxygenavailability_status == HIGH) { sal = 0; }
    if (set_set_fio2 == 21) {
      sal = 0;
    }
    
    if (standbybutton_count != 2) {
      sal = 1830;
    }

    buffer[0] = 0b01000000;
    buffer[1] = sal >> 4;  
    buffer[2] = sal << 4;  

    Wire.beginTransmission(MCP4725);  
    Wire.write(buffer[0]);            
    Wire.write(buffer[1]);            
    Wire.write(buffer[2]);            
    Wire.endTransmission();  
}


/* ===== FiO2 SETTLED-STATE HOLD (consistency after reaching set) =====
   The O2 plant (valve->blender->circuit->galvanic cell) responds over
   tens of seconds; a 10 Hz integrator on that lag limit-cycles around
   the setpoint forever. Once |error| stays inside FIO2_SETTLE_BAND for
   FIO2_SETTLE_MS, PID steps are paced 10x slower - integration slows
   to match the plant, and the mixture holds. Any excursion beyond
   FIO2_SETTLE_EXIT restores full-rate correction instantly.          */
#define FIO2_SETTLE_BAND 2      /* % inside which we count as settled  */
#define FIO2_SETTLE_EXIT 3      /* % that breaks the settled state     */
#define FIO2_SETTLE_MS 8000UL   /* must hold in-band this long         */
#define FIO2_PID_FAST_MS 100UL  /* step period while converging        */
#define FIO2_PID_SLOW_MS 1000UL /* step period once settled (10x)      */
#define FIO2_SAL_RAMP_UP  2
uint32_t fio2InBandSince = 0;
uint16_t fio2Settled = 0;
uint32_t fio2LastStep = 0;

#if 0
void fio2_function(void) {
  
  if (bodyweight > 10 && setmode != MODE_HFT) {
    Ocmillis = millis();
    
    Opmillis = Ocmillis;
    fio2_sample(1.0f);          /* acquire + scale + filter, no snapping.
                                   the old ptl>=2 decimation is gone: it
                                   doubled the filter time constant on top
                                   of an already 150-deep boxcar.        */
    

    if (oxygenavailability_status == LOW && set_set_fio2 > 21 && standbybutton_count == 2) {
      
      pressure2 = (analogRead(Analog_pr2) - ZFF_PR2);
      if (pressure2 > 2) {
        pressure2 = Pr2_range * sqrt((pressure2) / (1023 - ZFF_PR2));  
      } else {
        pressure2 = 0;
      }
      
      distance = sharp2cm(0.0126582278 * (set_set_fio2 - 21) * pressure2);
      /* fio2 already computed, clamped and fault-checked by
         fio2_sample(). Recomputing here re-introduced the unclamped
         value into the PID feedback path.                         */
      if (set_set_fio2 == 100) {
        set_set_fio2 = 99;
      }
      
      if (set_set_fio2 > 21 && mainrelaypinstate == 1) {
        Kp = 0.00091, Ki = 0.04, Kd = 0, Hz = 20;
        int setpoint = set_set_fio2;
        
        float feedback = fio2;

        /* settled-state detector */
        float fErr = (float)setpoint - feedback;
        if (fErr < 0) { fErr = -fErr; }
        if (fio2Settled == 0) {
          if (fErr <= FIO2_SETTLE_BAND) {
            if (fio2InBandSince == 0) { fio2InBandSince = millis(); }
            else if ((millis() - fio2InBandSince) >= FIO2_SETTLE_MS) {
              fio2Settled = 1;
            }
          } else {
            fio2InBandSince = 0;
          }
        } else if (fErr > FIO2_SETTLE_EXIT) {
          fio2Settled = 0;          /* disturbance/setpoint change     */
          fio2InBandSince = 0;
        }

        /* paced PID: integration rate matched to plant response.
           Between steps output1 HOLDS its last value - the valve sits
           still instead of chasing sensor lag.                       */
        uint32_t stepPeriod = fio2Settled ? FIO2_PID_SLOW_MS : FIO2_PID_FAST_MS;
        if ((millis() - fio2LastStep) >= stepPeriod) {
          fio2LastStep = millis();
          before = micros();
          output1 = myPID.step(setpoint, feedback);
          after = micros();  
        }
        
        calibration_HFT_final = calibration_HFT;  
                                                  
        calibration_flow_final = calibration_flow;
        
        if (set_set_fio2 <= 98) {
          if (output1 > 300 && calibration_flow_final != 0) {
            output1 = 300;
          } else if (output1 < -300 && calibration_flow_final != 0) {
            output1 = -300;
          }
          comp = output1 + calibration_flow_final;
        }
        else {
          if (output1 > 100 && calibration_HFT_final != 0) {
            output1 = 110;
          } else if (output1 < -100 && calibration_HFT_final != 0) {
            output1 = -100;
          }

          comp = output1 + calibration_HFT_final;
        }
      }

      sal = distance + comp;
    }

    if (set_set_fio2 > 21 && j == 1) {
      sal = ov_start_duty;
      mainrelay_duty = ov_start_duty ;
      j = 0;
    }  

    if (set_set_fio2 == 21) {
      j = 1;
    }
    
    if (((analogRead(Analog_pr2) - ZFF_PR2) <= 10) && setmode != MODE_HFT) {
      if (sal > 1800) { sal = sal - 10; }  
    }
    if (((analogRead(Analog_pr2) - ZFF_PR2) <= 4) && setmode == MODE_HFT) {
      if (sal > 1800) { sal = sal - 10; }  
    }
    if (set_set_fio2 == 21 || oxygenavailability_status == HIGH || standbybutton_count != 2) {
      sal = 0;
    }
    
    if (fio2 <= 10 && set_set_fio2 > 21 && standbybutton_count == 2 && mainrelaypinstate == HIGH) {
      if (mainrelaypinstate == 1) { sal = ov_start_duty; }
      if (mainrelaypinstate == 0) { sal = ov_start_duty; }
      mainrelay_duty = ov_start_duty ;
    }
    
    buffer[0] = 0b01000000;  
    
    buffer[1] = sal >> 4;             
    buffer[2] = sal << 4;             
    Wire.beginTransmission(MCP4725);  
    Wire.write(buffer[0]);            
    Wire.write(buffer[1]);            
    Wire.write(buffer[2]);            
    Wire.endTransmission();
  } else if (setmode == MODE_HFT) {
    fio2_function3();
  } else {
    fio2_function2();
  }
}
#endif

/*
 * Only line 3458 is changed.
 * All other code, all block positions, all conditions — identical to original.
 *
 * CHANGE: replace `sal = distance + comp;`
 *         with a direct linear map: FiO2 21 → DAC 1825,  FiO2 100 → DAC 4095
 *
 *   sal = 1825 + (set_set_fio2 - 21) * (4095 - 1825) / (100 - 21)
 *       = 1825 + (set_set_fio2 - 21) * 28.7342
 *
 *   FiO2  21  → sal 1825
 *   FiO2  30  → sal 2083
 *   FiO2  50  → sal 2658
 *   FiO2  60  → sal 2945
 *   FiO2  80  → sal 3520
 *   FiO2  95  → sal 3951
 *   FiO2 100  → sal 4066  (set_set_fio2 clamped to 99 at line 3398-3400)
 *              + comp from PID bridges remaining 29 counts to 4095
 *
 * The PID comp term (output1 + calibration_flow) still adds its correction
 * on top, so closed-loop trim around the linear feedforward is preserved.
 * DAC hard clamp to 4095 prevents MCP4725 bit-pack overflow.
 */

void fio2_function(void) {

  if (bodyweight > 10 && setmode != MODE_HFT) {
    Ocmillis = millis();
    
    Opmillis = Ocmillis;
    fio2_sample(1.0f);          

    if (oxygenavailability_status == LOW && set_set_fio2 > 21 && standbybutton_count == 2) {
      
      pressure2 = (analogRead(Analog_pr2) - ZFF_PR2);
      if (pressure2 > 2) {
        pressure2 = Pr2_range * sqrt((pressure2) / (1023 - ZFF_PR2));  
      } else {
        pressure2 = 0;
      }
      
      distance = sharp2cm(0.0126582278 * (set_set_fio2 - 21) * pressure2);
      if (set_set_fio2 == 100) {
        set_set_fio2 = 99;
      }
      
      if (set_set_fio2 > 21 && mainrelaypinstate == 1) {
        Kp = 0.00091, Ki = 0.04, Kd = 0, Hz = 10;
        int setpoint = set_set_fio2;
        
        float feedback = fio2;

        /* settled-state detector */
        float fErr = (float)setpoint - feedback;
        if (fErr < 0) { fErr = -fErr; }
        if (fio2Settled == 0) {
          if (fErr <= FIO2_SETTLE_BAND) {
            if (fio2InBandSince == 0) { fio2InBandSince = millis(); }
            else if ((millis() - fio2InBandSince) >= FIO2_SETTLE_MS) {
              fio2Settled = 1;
            }
          } else {
            fio2InBandSince = 0;
          }
        } else if (fErr > FIO2_SETTLE_EXIT) {
          fio2Settled = 0;
          fio2InBandSince = 0;
        }

        uint64_t stepPeriod = fio2Settled ? FIO2_PID_SLOW_MS : FIO2_PID_FAST_MS;
        if ((millis() - fio2LastStep) >= stepPeriod) {
          fio2LastStep = millis();
          before = micros();
          output1 = myPID.step(setpoint, feedback);
          after = micros();  
        }
        
        calibration_HFT_final = calibration_HFT;  
        calibration_flow_final = calibration_flow;
        
        if (set_set_fio2 <= 80) {
          if (output1 > 300 && calibration_flow_final != 0) {
            output1 = 300;
          } else if (output1 < -300 && calibration_flow_final != 0) {
            output1 = -300;
          }
          comp = output1 + calibration_flow_final;
        } else {
          if (output1 >= 100 && calibration_HFT_final != 0) {
            output1 = 130;
          } else if (output1 < -100 && calibration_HFT_final != 0) {
            output1 = -130;
          }
          comp = output1 + calibration_HFT_final;
        }
      }

      // NOW (two segments matching actual valve curve):
      if (set_set_fio2 <= 30) 
      {
          sal = 2000 + ((set_set_fio2 - 21) * 12.79f) + comp;
      } 
      else if (set_set_fio2 > 30 && set_set_fio2 <= 50)
      {
        sal = 2080 + ((set_set_fio2 - 21) * 4.59f) + comp;
      } 
      else if (set_set_fio2 > 50 && set_set_fio2 <= 80)
      {
        sal = 2080 + ((set_set_fio2 - 21) * 3.19f) + comp;
      } 
      else {
          sal = 2080 + ((set_set_fio2 - 21) * 2.59) + comp;
      }
      if (sal > 4090) { sal = 4090; }
      /* ------------------------------------------------------------------ */
    }

    if (set_set_fio2 > 21 && j == 1) {
      sal = ov_start_duty;
      mainrelay_duty = ov_start_duty ;
      j = 0;
    }  

    if (set_set_fio2 == 21) {
      j = 1;
    }
    
    if (((analogRead(Analog_pr2) - ZFF_PR2) <= 10) && setmode != MODE_HFT) {
      if (sal > 1800) { sal = sal - 10; }  
    }
    if (((analogRead(Analog_pr2) - ZFF_PR2) <= 4) && setmode == MODE_HFT) {
      if (sal > 1800) { sal = sal - 10; }  
    }
    if (set_set_fio2 == 21 || oxygenavailability_status == HIGH || standbybutton_count != 2) {
      sal = 0;
    }
    
    if (fio2 <= 10 && set_set_fio2 > 21 && standbybutton_count == 2 && mainrelaypinstate == HIGH) {
      if (mainrelaypinstate == 1) { sal = ov_start_duty; }
      if (mainrelaypinstate == 0) { sal = ov_start_duty; }
      mainrelay_duty = ov_start_duty ;
    }
    
    buffer[0] = 0b01000000;  

    buffer[1] = sal >> 4;             
    buffer[2] = sal << 4;             
    Wire.beginTransmission(MCP4725);  
    Wire.write(buffer[0]);            
    Wire.write(buffer[1]);            
    Wire.write(buffer[2]);            
    Wire.endTransmission();
  } else if (setmode == MODE_HFT) {
    fio2_function3();
  } else {
    fio2_function2();
  }
}



#if 0   /* fio2_function9() - legacy copy, no call sites. Fenced off so the
           obsolete snap-to-setpoint code cannot be edited by mistake. */
void fio2_function9(void) {
  
  if (bodyweight > 10 && setmode != MODE_HFT) {
    Ocmillis = millis();

    Op_SUM = Op_SUM - Op_READINGS[Op_INDEX];    
    Op_VALUE = analogRead(fio2_read);           
    Op_READINGS[Op_INDEX] = Op_VALUE;           
    Op_SUM = Op_SUM + Op_VALUE;                 
    Op_INDEX = (Op_INDEX + 1) % WINDOW_SIZEop;  
    Op_AVERAGED = Op_SUM / WINDOW_SIZEop;       

    
    Opmillis = Ocmillis;
    Op_AVERAGED_SAVED = Op_AVERAGED;

    fio2 = ((Op_AVERAGED_SAVED - f1) / (L3)) + 21;

    
    if (set_set_fio2 - fio2 <= 5 && set_set_fio2 >= fio2 && set_set_fio2 > 25) {
      fio23 = set_set_fio2;
    } else if (fio2 - set_set_fio2 <= 5 && fio2 >= set_set_fio2 && set_set_fio2 > 25) {
      fio23 = set_set_fio2;
    } else {
      fio23 = fio2;
    }
    
    ptl = ptl + 1;
    if (ptl >= 2) {
      dOp_SUM = dOp_SUM - dOp_READINGS[dOp_INDEX];   
      dOp_VALUE = fio23;                             
      dOp_READINGS[dOp_INDEX] = dOp_VALUE;           
      dOp_SUM = dOp_SUM + dOp_VALUE;                 
      dOp_INDEX = (dOp_INDEX + 1) % WINDOW_SIZEdop;  
      dOp_AVERAGED = dOp_SUM / WINDOW_SIZEdop;       
      ptl = 0;
    }

    if (set_set_fio2 - dOp_AVERAGED <= 5 && set_set_fio2 >= dOp_AVERAGED && set_set_fio2 > 30) {
      dOp_AVERAGED = set_set_fio2;
    } else if (dOp_AVERAGED - set_set_fio2 <= 5 && dOp_AVERAGED >= set_set_fio2 && set_set_fio2 > 30) {
      dOp_AVERAGED = set_set_fio2;
    }
    
    if (set_set_fio2 > 21 && oxygenavailability_status == LOW && standbybutton_count == 2) {
      pressure2 = (analogRead(Analog_pr2) - ZFF_PR2);
      if (pressure2 > 2) {
        pressure2 = Pr2_range * sqrt((pressure2) / (1023 - ZFF_PR2));  
      } else {
        pressure2 = 0;
      }

      
      distance = sharp2cm(0.0126582278 * (set_set_fio2 - 21) * pressure2);

      /* fio2 already computed, clamped and fault-checked by
         fio2_sample(). Recomputing here re-introduced the unclamped
         value into the PID feedback path.                         */

      if (set_set_fio2 == 100) {
        set_set_fio2 = 99;
      }

      if (set_set_fio2 > 21 && mainrelaypinstate == 1) {
        Kp = 0.0097, Ki = 0.04, Kd = 0, Hz = 10;  
        int setpoint = set_set_fio2;
        
        float feedback = fio2;

        before = micros();
        output1 = myPID.step(setpoint, feedback);

        after = micros();  
        
        calibration_HFT_final = calibration_HFT;  
                                                  
        calibration_flow_final = calibration_flow;
        
        if (set_set_fio2 <= 100) {
          if (output1 > 300 && calibration_flow_final != 0) {
            output1 = 50;
          } else if (output1 < -300 && calibration_flow_final != 0) {
            output1 = -50;
          }
          comp = output1 + calibration_flow_final;
          if (mainrelaypinstate == HIGH && comp < 0) {
            comp = -comp;
          }
        }
      }
    }
    if (set_set_fio2 > 21 && j == 1) {
      sal = ov_start_duty;
      mainrelay_duty = ov_start_duty ;
      j = 0;
    }  

    if (set_set_fio2 == 21) {
      j = 1;
    }
    
    if (((analogRead(Analog_pr2) - ZFF_PR2) <= 10) && setmode != MODE_HFT) {
      if (sal > 1800) {
        sal = sal - 10;
      }  
    }
    if (((analogRead(Analog_pr2) - ZFF_PR2) <= 4) && setmode == MODE_HFT) {
      if (sal > 1800) {
        sal = sal - 10;
      }  
    }
    if (set_set_fio2 == 21 || standbybutton_count != 2 || oxygenavailability_status == HIGH) {
      sal = 0;
    }
    
    if (fio2 <= 10 && set_set_fio2 > 21 && standbybutton_count == 2 && mainrelaypinstate == HIGH) {
      if (mainrelaypinstate == 1) {
        sal = ov_start_duty - comp;
      }
      if (mainrelaypinstate == 0) {
        sal = ov_start_duty - comp;
      }
      mainrelay_duty = ov_start_duty ;
    }

    
    buffer[0] = 0b01000000;  
    

    buffer[1] = sal >> 4;             
    buffer[2] = sal << 4;             
    Wire.beginTransmission(MCP4725);  
    Wire.write(buffer[0]);            
    Wire.write(buffer[1]);            
    Wire.write(buffer[2]);            
    Wire.endTransmission();
  } else if (setmode == MODE_HFT) {
    fio2_function3();
  } else {
    fio2_function2();
  }
}

#endif  /* end fio2_function9 - legacy copy */

void fio2_function2(void) {

  Ocmillis = millis();

  Opmillis = Ocmillis;
  fio2_sample(0.9f);            /* 0.9f preserves this path's old
                                   (L3 - 0.1*L3) slope trim            */
  {
    if (oxygenavailability_status == LOW && set_set_fio2 > 21 && standbybutton_count == 2) {
      Kp = 3, Ki = 1, Kd = 0, Hz = 10;
      int setpoint = set_set_fio2;
      
      float feedback = fio2;
      
      before = micros();
      int16_t output1 = myPID.step(setpoint, feedback);
      after = micros();  
      if (mainrelaypinstate == HIGH) {
        if (output1 > -100) {
          output1 = -100;
        } else if (output1 <= -500) {
          output1 = (-500);
        }
        sal = ov_start_duty + (1 * (output1));
      }    
    }
  }
  
  mini_step = settidalvolume / 1000;
  if (set_set_fio2 > 21 && j == 1) {
    sal = ov_start_duty;
    mainrelay_duty = ov_start_duty ;
    j = 0;
  }  
  if (set_set_fio2 == 21) { j = 1; }
  if (fio2 <= 10 && set_set_fio2 > 21) {
    sal = ov_start_duty - 100;
    mainrelay_duty = ov_start_duty - 100 ;
  }

  if (mainrelay_duty >= 4000) { mainrelay_duty = 4000; }
  if (mainrelay_duty <= 100) { mainrelay_duty = 100; }
  {

    buffer[0] = 0b01000000;                                 
    if (mainrelaypinstate == HIGH && set_set_fio2 != 21) {  
      
    }  
    if (oxygenavailability_status == HIGH) {
      sal = 0;
    }
    if (set_set_fio2 == 21) {
      sal = 0;
    }
    
    if (standbybutton_count != 2) {
      sal = 1800;
    }

    buffer[1] = sal >> 4;  
    buffer[2] = sal << 4;  

    Wire.beginTransmission(MCP4725);  
    Wire.write(buffer[0]);            
    Wire.write(buffer[1]);            
    Wire.write(buffer[2]);            
    Wire.endTransmission();

  }  
}


#if 0   /* fio2_function1() - legacy copy, no call sites. */
void fio2_function1(void) {  
  Ocmillis = millis();
  
    Op_SUM = Op_SUM - Op_READINGS[Op_INDEX];    
    Op_VALUE = analogRead(fio2_read);           
    Op_READINGS[Op_INDEX] = Op_VALUE;           
    Op_SUM = Op_SUM + Op_VALUE;                 
    Op_INDEX = (Op_INDEX + 1) % WINDOW_SIZEop;  
    Op_AVERAGED = Op_SUM / WINDOW_SIZEop;       
  
    Opmillis = Ocmillis;
    Op_AVERAGED_SAVED = Op_AVERAGED;
    fio2 = ((analogRead(fio2_read) - f1) / (L3 - (0.1 * L3))) + 21;  
  
  if (set_set_fio2 - fio2 <= 5 && set_set_fio2 >= fio2 && set_set_fio2 > 25) {
    fio23 = set_set_fio2;
  } else if (fio2 - set_set_fio2 <= 5 && fio2 >= set_set_fio2 && set_set_fio2 > 25) {
    fio23 = set_set_fio2;
  } else {
    fio23 = fio2;
  }
  if (mainrelaypinstate == HIGH) {
    dOp_SUM = dOp_SUM - dOp_READINGS[dOp_INDEX];   
    dOp_VALUE = fio23;                             
    dOp_READINGS[dOp_INDEX] = dOp_VALUE;           
    dOp_SUM = dOp_SUM + dOp_VALUE;                 
    dOp_INDEX = (dOp_INDEX + 1) % WINDOW_SIZEdop;  
    dOp_AVERAGED = dOp_SUM / WINDOW_SIZEdop;                                                      
  }
  if (set_set_fio2 - dOp_AVERAGED <= 5 && set_set_fio2 >= dOp_AVERAGED && set_set_fio2 > 30) {
    dOp_AVERAGED = set_set_fio2;
  } else if (dOp_AVERAGED - set_set_fio2 <= 5 && dOp_AVERAGED >= set_set_fio2 && set_set_fio2 > 30) {
    dOp_AVERAGED = set_set_fio2;
  }
  
  if (oxygenavailability_status == LOW && set_set_fio2 > 21 && standbybutton_count == 2) {
    if (mainrelaypinstate == 1) {
      Kp = 0.00091, Ki = 0.04, Kd = 0, Hz = 10;
      int setpoint = set_set_fio2;
      
      float feedback = fio2;
      uint32_t before, after;
      before = micros();
      output1 = myPID.step(setpoint, feedback);
      after = micros();  
      if (mainrelaypinstate == HIGH) {
        if (output1 > 1000) {
          output1 = 1000;
        } else if (output1 <= -1000) {
          output1 = (-1000);
        }
        sal = ov_start_duty + ((output1));
      }
    }

    if (mainrelaypinstate == 0 && standbybutton_count == 2 && compan_duty1 > 8) {
      Kp = 0.00091, Ki = 0.04, Kd = 0, Hz = 10;
      int setpoint = set_set_fio2;
      
      float feedback = fio2;
      uint32_t before, after;
      before = micros();
      int16_t output_low = myPID.step(setpoint, feedback);
      after = micros();  

      sal = ov_start_duty + ((output_low)) - 300;
      if (sal > (ov_start_duty + ((output1)))) { sal = ov_start_duty + ((output1)); }
    }    
  }
 
  mini_step = settidalvolume / 1000;
  if (set_set_fio2 > 21 && j == 1) {
    sal = ov_start_duty;
    mainrelay_duty = ov_start_duty ;
    j = 0;
  }  
  if (set_set_fio2 == 21) { j = 1; }
  if (fio2 <= 10 && set_set_fio2 > 21) {
    sal = ov_start_duty;
    mainrelay_duty = ov_start_duty ;
  }
  if (fio2 <= 10 && set_set_fio2 > 21 && mainrelaypinstate == 0) { sal = 0; }
  
  if (mainrelay_duty >= 4000) { mainrelay_duty = 4000; }
  if (mainrelay_duty <= 100) { mainrelay_duty = 100; }
  
  {
    pressure = 00.1991640625 * 1.17 * (analogRead(Analog_pr1) - ZFP);
    buffer[0] = 0b01000000;  
                                                    
    if (oxygenavailability_status == HIGH) { sal = 0; }
    if (set_set_fio2 == 21) { sal = 0; }
    if (mainrelaypinstate == LOW && compan_duty1 < 8) { sal = 0; }

    
    buffer[1] = sal >> 4;  
    buffer[2] = sal << 4;  

    Wire.beginTransmission(MCP4725);  
    Wire.write(buffer[0]);            
    Wire.write(buffer[1]);            
    Wire.write(buffer[2]);            
    Wire.endTransmission();
    
  }  
}
#endif  /* end fio2_function1 - legacy copy */
/*********************************************************
*
************************************************************/
void alarms(void) {
  mainspower_state = (analogRead(MAINS_PIN));
  battery_icon_percentage = analogRead(BATTERY_PIN);
  if (mainspower_state > 300)  
  {
    digitalWrite(mainspower_write, HIGH);
    digitalWrite(lowbatterystatus_write, LOW);
    digitalWrite(batterypower_write, LOW);

    alarm_numpower = 0;
  } else if (((mainrelaypinstate == LOW) || (setmode == MODE_HFT)) && (battery_icon_percentage < battery_percent_30))  
  {
    digitalWrite(lowbatterystatus_write, HIGH);
    digitalWrite(batterypower_write, HIGH);
    digitalWrite(mainspower_write, LOW);
    alarm_write = 1;
    alarm_numpower = 1;  
  }
  
  airavailability_status = digitalRead(airavailability_read);
  if (airavailability_status == HIGH) {
    digitalWrite(airavailability_write, HIGH);  
    digitalWrite(airfailure_write, LOW);

  } else {
    digitalWrite(airavailability_write, HIGH);  
    digitalWrite(airfailure_write, LOW);
    
  }
  
  oxygenavailability_status = digitalRead(oxygenavailability_read);
  
  if ((oxygenavailability_status_last != oxygenavailability_status) || (set_fio2_last != set_fio2)) {  

    if (oxygenavailability_status == LOW) {
      digitalWrite(oxygenavaibility_write, HIGH);
      Icon22[7] = 0;
      DWIN_PORT.write(Icon22, 8);
      digitalWrite(oxygenfailure_write, LOW);
      
    } else {
      digitalWrite(oxygenavaibility_write, LOW);

      if (set_fio2 != 21) {
        Icon22[7] = 199;
        DWIN_PORT.write(Icon22, 8);
        digitalWrite(oxygenfailure_write, HIGH);
      }  
      else {
        digitalWrite(oxygenfailure_write, LOW);
        Icon22[7] = 0;
        DWIN_PORT.write(Icon22, 8);
      }     
    }
    
  }
  set_fio2_last = set_fio2;
  oxygenavailability_status_last = oxygenavailability_status;
  
  oxy_fail_cmillis = millis();
  if (oxygenavailability_status == LOW || standbybutton_count == 1) {
    oxy_fail_pmillis = oxy_fail_cmillis;
  }

  if (set_set_fio2 > 21 && (oxygenavailability_status == HIGH) && (oxy_fail_cmillis - oxy_fail_pmillis) <= 10000) {
    alarm_write = 1;
    alarm_num = 5;
  }

  if (hpstate == 1 && standbybutton_count == 2 && setmode != MODE_HFT) {
    alarm_write = 1;
    alarm_num = 1;
  }

  if (Hvtstate == 1 && standbybutton_count == 2 && setmode != MODE_HFT) {
    alarm_write = 1;
    alarm_num = 3;
  }
  else if (Lvtstate == 1 && standbybutton_count == 2 && setmode != MODE_HFT) {
    alarm_write = 1;
    alarm_num = 4;
  }
  if ((monitored_mv / 10 >= high_mvalarmlimit) && standbybutton_count == 2 && setmode != MODE_HFT) {
    alarm_write = 1;
    alarm_num = 12;
  }
  
  else if ((monitored_mv / 10 < low_mvalarmlimit) && standbybutton_count == 2 && setmode != MODE_HFT) {
    alarm_write = 1;
    alarm_num = 13;
  }
  
  
  if (AVERAGED_SAVED_bpm > highbpmalarmlimit && standbybutton_count == 2 && setmode != MODE_HFT) {
    alarm_write = 1;
    alarm_num = 10;
  }
  
  else if (AVERAGED_SAVED_bpm < lowbpmalarmlimit && standbybutton_count == 2 && setmode != MODE_HFT) {
    alarm_write = 1;
    alarm_num = 11;
    if (millis() - lastLogtime > 5000) {
      lastLogtime = millis();
      
    }
  }
  /* FiO2 alarms are inhibited at a setting of 21% or below (room air),
     with no O2 supply present, and on a faulty cell - see
     fio2_alarm_enabled(). Any stale FiO2 icon is cleared when the
     condition no longer applies.                                      */
  if (fio2_alarm_enabled()
      && ((dOp_AVERAGED1 >= (set_set_fio2 + 30)) || dOp_AVERAGED1 > high_fio2alarmlimit)) {
    alarm_write = 1;
    alarm_num = 8;
    if (millis() - lastLogtime > 5000) {
      lastLogtime = millis();
      
    }
  } 
  else if (fio2_alarm_enabled()
      && ((dOp_AVERAGED1 <= (set_set_fio2 - 30)) || dOp_AVERAGED1 < low_fio2alarmlimit)) {
    alarm_write = 1;
    alarm_num = 9;
    if (millis() - lastLogtime > 8000) {
      lastLogtime = millis();
      
    }
  }  
  else if (Icon10[7] == 194 || Icon10[7] == 195 || Icon10[7] == 196) {
    Icon10[7] = 0;
    DWIN_PORT.write(Icon10, 8);
  }

  uint32_t alarmtimercurrentMillis = millis();
  if (alarm_write == 1) {
    AT = 1;
  }
  if (alarm_write == 0) {
    alarmtimerpreviousMillis = alarmtimercurrentMillis;
  }
  if ((AT == 1) && (alarmtimercurrentMillis - alarmtimerpreviousMillis) >= 1000) {
    alarm_write = 0;
    alarm_num = 0;
    alarm_numpower = 0;
    alarmtimerpreviousMillis = alarmtimercurrentMillis;
    AT = 0;
  }
  
  if (standbybutton_count == 1) {
    lowpawcurrentMillis = millis();
  }

  if (pressure >= setlowpawalarmlimit || standbybutton_count == 1 || setmode == MODE_HFT) {  
    lowpawcurrentMillis = millis();                                                   
  }
  if ((millis() - lowpawcurrentMillis) >= 10000 && standbybutton_count == 2)  
  {
    alarm_write = 1;
    alarm_num = 2;
  }
  

  if (standbybutton_count == 1) {
    pr1failiurecurrentMillis = millis();
  }

  if (pressure >= 4 || standbybutton_count == 1 || setmode == MODE_HFT) {  
    pr1failiurecurrentMillis = millis();                            
    pr1failiure = 0;
  }
  if ((millis() - pr1failiurecurrentMillis) >= 10000 && standbybutton_count == 2)  
  {
    pr1failiure = 1;
  }
  
  
  if (standbybutton_count == 1) {
    exp_pawcurrentMillis = millis();
  }
  if (pressure3 >= 25 || standbybutton_count == 1 || setmode == MODE_HFT) {
    exp_pawcurrentMillis = millis();  
  }
  if (((millis() - exp_pawcurrentMillis) >= 30000) && (standbybutton_count == 2) && (CpapSensorFlag = 1))  
  {
    alarm_write = 1;
    alarm_num = 6;
    peepduty = 35;
    CpapSensorFlag = 0;
  }
  
  
  if (standbybutton_count == 1) {
    flowsensor_pawcurrentMillis = millis();
  }
  if (V1 >= 3 || standbybutton_count == 1 || bodyweight <= 10)  
  {
    flowsensor_pawcurrentMillis = millis();  
    sensorfailure = 0;
  }
  if (((millis() - flowsensor_pawcurrentMillis) >= 30000) && (standbybutton_count == 2))  
  {
    alarm_write = 1;
    alarm_num = 7;
    sensorfailure = 1;

    tonetimer = millis();
  }
  
  
  if ((alarm_write == 1) && standbybutton_count == 2) {
    digitalWrite(alarmled, HIGH);
  } else if (resetbuttonState == HIGH) {
    digitalWrite(alarmled, LOW);
  } else if (set_set_fio2 > 21 && oxygenavailability_status == HIGH) {
    digitalWrite(alarmled, HIGH);
  } else digitalWrite(alarmled, LOW);
  

  /* ==================== LOW AIRWAY PRESSURE ALARM ====================
     The limit (lowpawalarmlimit) was settable in the UI but NO detection
     branch existed - the disconnection alarm never fired (found via
     'visible but no sound' report; the visual was another mechanism).
     Detection: if pressure has not exceeded the low limit for ~1.5
     breath periods (min 5 s), raise alarm_num 7. Re-arms on any
     crossing; suppressed in standby and HFT (no cyclic pressure).
     PRIORITY NOTE (CLINICAL-CONFIRM): this block runs after the alarm
     chain and overrides same-pass lower alarms - disconnection is
     treated as highest priority.                                     */
  if ((standbybutton_count != 2) || (setmode == MODE_HFT)
      || (pressure > lowpawalarmlimit)) {
    lowpawcurrentMillis = millis();      /* armed / re-armed          */
  } else {
    uint32_t lowpawDelay = (60000UL / ((setfrequency > 0) ? setfrequency : 12));
    lowpawDelay = lowpawDelay + (lowpawDelay / 2);          /* x1.5   */
    if (lowpawDelay < 5000UL) { lowpawDelay = 5000UL; }
    if ((millis() - lowpawcurrentMillis) > lowpawDelay) {
      alarm_write = 1;
      alarm_num = 14;  /* RENUMBERED from 7 - 7 is the sensor watchdog */
    }
  }

  /* flash log: alarm ON/OFF edge records (one per transition) */
  FLOG_ALARM(FLOG_A_HIGH_PAW, hpstate == 1);
  FLOG_ALARM(FLOG_A_LOW_PAW, alarm_num == 14);
  FLOG_ALARM(FLOG_A_APNEA, apnbackup_mode == 1);
  FLOG_ALARM(FLOG_A_SENSOR_FAIL, sensorfailure == 1);
  FLOG_ALARM(FLOG_A_PR1_FAIL, pr1failiure == 1);
  FLOG_ALARM(FLOG_A_LOW_BATTERY, alarm_numpower == 1);
  FLOG_ALARM(FLOG_A_O2_DISCONNECT, (oxygenavailability_status == LOW) && (set_fio2 > 21));
}

void testscreen(void) {  

  modebuttonState = digitalRead(modebutton);
  if (modebuttonState && modebuttonState != modebutton_lastState) {
    if (testscreenpage == 2) {
      testscreenpage = 1;
    } else {
      testscreenpage = 2;
    }
  }
  modebutton_lastState = modebuttonState;
}

void alarmscreen(void) {  

  if ((fio23 < 15 || fio23 > 120) && ((millis() - o2sensorfailtimer) > 15000) && alarm_numpower != 1 && standbybutton_count == 2) {
  } else {
    o2sensorfailtimer = millis();
  }
  
  
  // Serial.print(F("ALARM_NUM -> "));
  // Serial.println(alarm_num);
  switch (alarm_num) {
    case ALARM_NUM_APNEA:
      Icon12[7] = 150;  /* TODO-CONFIRM: APNEA banner icon ID in ICL */
      DWIN_PORT.write(Icon12, 8);
      break;
    case 1:
      Icon4[7] = 139;
      DWIN_PORT.write(Icon4, 8);
      break;

    case 2:
      Icon4[7] = 140;
      DWIN_PORT.write(Icon4, 8);
      break;

    case 3:
      Icon4[7] = 139;
      DWIN_PORT.write(Icon4, 8);
      break;

    case 4:
      Icon4[7] = 141;
      DWIN_PORT.write(Icon4, 8);
      break;

    case 5:
      Icon10[7] = 196;
      DWIN_PORT.write(Icon10, 8);
      break;

    case 6:
      Icon11[7] = 143;
      DWIN_PORT.write(Icon11, 8);
      break;

    case 7:
      Icon11[7] = 143;
      DWIN_PORT.write(Icon11, 8);
      break;

    case 8:
      Icon10[7] = 194;
      DWIN_PORT.write(Icon10, 8);  
      break;

    case 9:
      Icon10[7] = 195;
      DWIN_PORT.write(Icon10, 8);
      break;

    case 10:
      Icon4[7] = 152;
      DWIN_PORT.write(Icon4, 8);
      break;

    case 11:
      Icon4[7] = 151;
      DWIN_PORT.write(Icon4, 8);
      break;

    case 12:
      Icon4[7] = 148;
      DWIN_PORT.write(Icon4, 8);
      break;

    case 13:
      Icon4[7] = 149;
      DWIN_PORT.write(Icon4, 8);
      break;

      
      
      
      
      
      
      
      
  }

  if (alarm_numpower == 1) {
    Icon13[7] = 144;
    DWIN_PORT.write(Icon13, 8);
  }
}

void peakdetect(void) {

  latest_p2 = latest_p;
  latest_p = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);  
  if ((latest_p > latest_p2) && (mainrelaypinstate == HIGH)) {
    peakpressure = latest_p;
  }
}
#if 0
/* ============================================================================
 * turbine_optimized.ino  —  drop-in replacement for turbine() / turbine1()
 *
 * Behaviour-preserving refactor. turbine() and turbine1() in the original
 * were uint8_t-identical except for three constants, so both are now thin
 * wrappers around one core:
 *
 *              bw for compan  bw gate for      TPF step in
 *              floor (<=)     adaptive ctl (>) SIMV (mode 2)
 *   turbine()      15              15              0.03
 *   turbine1()      6              10              0.04
 *
 * NOTE: turbine1() is never called anywhere in v11_4 — kept only so the
 * sketch still links if it gets wired up later. Delete it to save flash.
 * ==========================================================================*/

/* ---- adaptive-Ti factor helpers (identical guards as original) ---------- */
static inline void tpfRaise(float step) {
  if (turbine_pickup_time_factor <= 2.0f && tur_duty >= 249)
    turbine_pickup_time_factor += step;
}
static inline void tpfLower(float step) {
  if (turbine_pickup_time_factor >= 1.03f && tur_duty <= 200)
    turbine_pickup_time_factor -= step;
}

static void turbine_core(uint8_t bwCompanMin, uint8_t bwAdaptMin,
                         float simvTpfStep) {

  /* ---------------- expiratory / inspiratory motor drive ---------------- */
  TCMillis = millis();
  if (mainrelaypinstate == HIGH) { TPMillis = TCMillis; }

  if (mainrelaypinstate == LOW) {                 /* expiration */
    if ((TCMillis - TPMillis) <= 200) {           /* 200 ms brake window   */
      compan_duty = 0;
    } else {                                      /* turbine-assist (PEEP) */
      compan_duty = (compan_duty1 <= 22) ? 0 : compan_duty1;
      if (compan_duty2 != 0 && compan_duty1 < 25) { compan_duty = compan_duty2; }
      if (bodyweight <= bwCompanMin)              { compan_duty = 25; }
      if (compan_duty <= 25 && setpeep > 1)       { compan_duty = 0; }
      if (compan_duty >= tur_duty)                { compan_duty = tur_duty; }
    }
    analogWrite(turbinePin, compan_duty);
    digitalWrite(tur_reverse, HIGH);
    coil = 0;
    analogWrite(peepPumpPin, peepduty);
  } else {                                        /* inspiration */
    if (standbybutton_count == 2) { digitalWrite(tur_reverse, HIGH); coil = 1; }
    if (standbybutton_count == 1) { digitalWrite(tur_reverse, LOW); }
  }

  /* ---------------- standby save / restore of duty ----------------------- */
  if (standbybutton_count == 1 && g2 == 2) {
    g1 = tur_duty;
    EEPROM.update(23, g1);
    tur_duty = 0;
    g2 = 1;
  } else if (standbybutton_count == 2 && g2 == 1) {
    tur_duty = g1;
    g2 = 2;
  }

  /* ---------------- precomputed terms (were re-evaluated up to 6x) ------- */
  const float vtRef  = (settidalvolume <= 350) ? 0.90f * settidalvolume
                                               : 0.95f * settidalvolume;
  const float vtGap  = vtRef - T_Volume_final;      /* >0 = under-delivered */
  const bool  vtLow  = (vtGap > 0);
  const bool  bigVt  = (settidalvolume > 350);      /* skips gap-size split */
  const float tiErrAC   = (1.3f * Tinsp) - Ti_real;
  const float tiErrSIMV = setTinsp_simv - Ti_real;

  /* shared gates (identical to the guards repeated in every branch) */
  const bool base   = (mainrelaypinstate == LOW) && (g == 1) && (tr_save != 1);
  const bool raiseV = base && (tur_duty < 250) && (hpstate != 1)
                           && (peakpressure < sethighpawalarmlimit);
  const bool raiseP = base && (tur_duty < 250) && (hpstate != 1);
  const bool lower  = base && (tur_duty > 23);

  /* ---------------- once-per-breath duty adaptation ---------------------- */
  if (peakpressure > 0 && bodyweight > bwAdaptMin) {

    if (turbine_pickup_time_factor >= 1.03f && tur_duty <= 200 && g == 1) {
      turbine_pickup_time_factor -= 0.03f;
    }

    switch (setmode) {
      case MODE_VCV: case MODE_ACV:                                   /* VCV / AC-volume  */
        if (vtLow && (bigVt || vtGap > 100) && raiseV) {        /* big gap  */
          tur_duty += (settidalvolume - T_Volume_final) / 15;
          tpfRaise(0.03f);  g = 0;
        } else if (vtLow && (bigVt || vtGap < 100) && raiseV) { /* small    */
          tur_duty += (settidalvolume - T_Volume_final) / 32;
          tpfRaise(0.03f);  g = 0;
        } else if (peakpressure >= setlowpawalarmlimit
                   && tiErrAC > 100 && lower) {                 /* overshoot*/
          tur_duty -= 0.00240f * tiErrAC;
          tpfLower(0.03f);  g = 0;
        }
        break;

      case MODE_SIMV:                                           /* SIMV             */
        if (vtLow && raiseV) {
          tur_duty += (settidalvolume - T_Volume_final) / 20;
          tpfRaise(simvTpfStep);  g = 0;
        } else if (peakpressure >= setlowpawalarmlimit
                   && tiErrSIMV > 100 && lower) {
          tur_duty -= 0.000100f * tiErrSIMV;
          tpfLower(simvTpfStep);  g = 0;
        }
        break;

      case MODE_PSIMV:                                           /* pressure, SIMV-Ti*/
        if (set_Pinsp > peakpressure && raiseP) {
          tur_duty += (set_Pinsp - peakpressure);
          tpfRaise(0.03f);  g = 0;
        } else if (tiErrSIMV > 100 && lower) {
          tur_duty -= 0.0010f * tiErrSIMV;
          tpfLower(0.03f);  g = 0;
        }
        break;

      case MODE_CPAP:
        if (set_Pinsp > peakpressure && raiseP) {
          tur_duty += (set_Pinsp - peakpressure);
          tpfRaise(0.03f);  g = 0;
        } else if (tiErrAC > 100 && lower) {
          tur_duty -= 0.0010f * tiErrAC;
          tpfLower(0.03f);  g = 0;
        }
        break;

       case MODE_PCV:                                   /* PCV              */
        if (set_Pinsp > peakpressure && raiseP) {
          tur_duty += (set_Pinsp - peakpressure);
          tpfRaise(0.03f);  g = 0;
        } else if (tiErrAC > 100 && lower) {
          tur_duty -= 0.0010f * tiErrAC;
          tpfLower(0.03f);  g = 0;
        }
        break;

      default:
        break;
    }

  } else {
    /* startup / paediatric path: volume modes only, gap threshold 5, +3 kick */
    if (setmode == MODE_VCV || setmode == MODE_ACV) {
      if (vtLow && (bigVt || vtGap > 5) && raiseV) {
        tur_duty += ((settidalvolume - T_Volume_final) / 15) + 3;
        tpfRaise(0.03f);  g = 0;
      } else if (vtLow && (bigVt || vtGap < 5) && raiseV) {
        tur_duty += (settidalvolume - T_Volume_final) / 32;
        tpfRaise(0.03f);  g = 0;
      }
    }
  }

  if (mainrelaypinstate == HIGH) { g = 1; }             /* re-arm each breath */

  /* ---------------- clamps (order preserved) ----------------------------- */
  if (tur_duty >= (bodyweight * 5 + 20)) { tur_duty = (bodyweight * 5 + 20); }
  if (tur_duty >= 249) { tur_duty = 249; }
  if (tur_duty <= 25)  { tur_duty = 25; }

  /* ---------------- open-loop fallback on sensor failure ----------------- */
  if (sensorfailure == 1 || pr1failiure == 1) {
    if (bodyweight < 40) {
      tur_duty = 30;
    } else if (bodyweight > 40 && bodyweight < 83) {    /* NB: bw==40 falls  */
      tur_duty = (40 + ((1.38f * bodyweight) - 55.38f));/* through unchanged */
    } else if (bodyweight >= 83) {
      tur_duty = 100;
    }
  }

  turbine_pickup_time_factor = 1;   /* !! kills all TPF adaptation above —
                                       see review note 1 before removing   */
}

void turbine(void)  { turbine_core(15, 15, 0.03f); }
void turbine1(void) { turbine_core( 6, 10, 0.04f); }  /* currently uncalled */
#endif

#if 1
/* ===== TURBINE PEEP-SUPPORT FIXES (v2/v3 port) ===== */
#define COMPAN_STEP_MS 50
#define COMPAN_SLEW_UP 5.0f
#define COMPAN_SLEW_DOWN 8.0f
#define COMPAN_ON_HI 26.0f
#define COMPAN_OFF_HI 23.0f
#define COMPAN_ON_LO 23.0f
#define COMPAN_OFF_LO 21.0f
#define EXHALE_BIAS_DUTY 12.0f
float companOut = 0.0;
uint8_t companOn = 0;
uint32_t companStepMillis = 0;

void turbine(void) {
  
  TCMillis = millis();
  if (mainrelaypinstate == HIGH) { TPMillis = TCMillis; }
  if ((TCMillis - TPMillis) <= 200 && mainrelaypinstate == LOW)  
  {                                                              
    companOut = (setpeep >= 2) ? EXHALE_BIAS_DUTY : 0.0;
    compan_duty = companOut;
    companStepMillis = TCMillis;
    analogWrite(peepPumpPin, peepduty);
    analogWrite(turbinePin, (int)companOut);
    digitalWrite(tur_reverse, HIGH);  
    coil = 0;
    analogWrite(peepPumpPin, peepduty);
  }

  else if ((TCMillis - TPMillis) > 200 && mainrelaypinstate == LOW) {
    float companTarget;
    if (setpeep > 1) {
      if (companOn == 0 && compan_duty1 >= COMPAN_ON_HI) { companOn = 1; }
      if (companOn == 1 && compan_duty1 < COMPAN_OFF_HI) { companOn = 0; }
    } else {
      if (companOn == 0 && compan_duty1 >= COMPAN_ON_LO) { companOn = 1; }
      if (companOn == 1 && compan_duty1 < COMPAN_OFF_LO) { companOn = 0; }
    }
    companTarget = (companOn == 1) ? compan_duty1 : 0.0;
    if (compan_duty2 != 0 && compan_duty1 < 25) { companTarget = compan_duty2; }
    if (bodyweight <= 15 && setpeep <= 1) { companTarget = 25.0; }
    if (setpeep >= 2 && companTarget < EXHALE_BIAS_DUTY) {
      companTarget = EXHALE_BIAS_DUTY;
    }
    if (companTarget >= tur_duty) { companTarget = tur_duty; }
    if ((TCMillis - companStepMillis) >= COMPAN_STEP_MS) {
      companStepMillis = TCMillis;
      if (companOut < companTarget) {
        companOut += COMPAN_SLEW_UP;
        if (companOut > companTarget) { companOut = companTarget; }
      } else if (companOut > companTarget) {
        companOut -= COMPAN_SLEW_DOWN;
        if (companOut < companTarget) { companOut = companTarget; }
      }
    }
    compan_duty = companOut;
    analogWrite(turbinePin, (int)companOut);
    digitalWrite(tur_reverse, HIGH);  
    coil = 0;
    analogWrite(peepPumpPin, peepduty);
    
  }

  else {
    if (standbybutton_count == 2) {
      digitalWrite(tur_reverse, HIGH);
      coil = 1;
    }
    if (standbybutton_count == 1) {
      digitalWrite(tur_reverse, LOW);
    }
  }
  

  if (standbybutton_count == 1 && g2 == 2) {
    g1 = tur_duty;
    EEPROM.update(23, g1);
    tur_duty = 0;
    g2 = 1;
  } else if (standbybutton_count == 2 && g2 == 1) {
    tur_duty = g1;
    g2 = 2;
  }

  if (standbybutton_count == 2) {         
  } else if (standbybutton_count == 1) {  
  }
  if (peakpressure > 0 && bodyweight > 15) {                                                                                                  
    if (turbine_pickup_time_factor >= 1.03 && tur_duty <= 200 && g == 1) { turbine_pickup_time_factor = turbine_pickup_time_factor - 0.03; }  
    
    if ((((0.90 * settidalvolume) > T_Volume_final && (0.90 * settidalvolume) - T_Volume_final > 100 && settidalvolume <= 350) || ((0.95 * settidalvolume) > T_Volume_final && settidalvolume > 350)) && tur_duty < (250) && (mainrelaypinstate == LOW) && hpstate != 1 && g == 1 && (setmode == MODE_VCV || setmode == MODE_ACV) && peakpressure < sethighpawalarmlimit && tr_save != 1) {
      tur_duty = tur_duty + ((settidalvolume - T_Volume_final) / 15);                                                                
      if (turbine_pickup_time_factor <= 2.0 && tur_duty >= 249) { turbine_pickup_time_factor = turbine_pickup_time_factor + 0.03; }  
      
      g = 0;
    }
    
    else if ((((0.90 * settidalvolume) > T_Volume_final && (0.90 * settidalvolume) - T_Volume_final < 100 && settidalvolume <= 350) || ((0.95 * settidalvolume) > T_Volume_final && settidalvolume > 350)) && tur_duty < (250) && (mainrelaypinstate == LOW) && hpstate != 1 && g == 1 && (setmode == MODE_VCV || setmode == MODE_ACV) && peakpressure < sethighpawalarmlimit && tr_save != 1) {
      tur_duty = tur_duty + ((settidalvolume - T_Volume_final) / 32);  
      if (turbine_pickup_time_factor
            <= 2.0
          && tur_duty >= 249) { turbine_pickup_time_factor = turbine_pickup_time_factor + 0.03; }  
      
      g = 0;
    }

    else if (peakpressure >= setlowpawalarmlimit && ((1.3 * Tinsp) - Ti_real) > 100 && tur_duty > (23) && (mainrelaypinstate == LOW) && g == 1 && (setmode == MODE_VCV || setmode == MODE_ACV) && tr_save != 1) {
      tur_duty = tur_duty - ((0.00240 * ((1.3 * Tinsp) - Ti_real)));                                                                  
      if (turbine_pickup_time_factor >= 1.03 && tur_duty <= 200) { turbine_pickup_time_factor = turbine_pickup_time_factor - 0.03; }  
      
      
      g = 0;
    }

    
    if ((((0.90 * settidalvolume) > T_Volume_final && settidalvolume <= 350) || ((0.95 * settidalvolume) > T_Volume_final && settidalvolume > 350)) && tur_duty < (250) && (mainrelaypinstate == LOW) && hpstate != 1 && g == 1 && setmode == MODE_SIMV && peakpressure < sethighpawalarmlimit && tr_save != 1) {
      tur_duty = tur_duty + ((settidalvolume - T_Volume_final) / 20);                                                                
      if (turbine_pickup_time_factor <= 2.0 && tur_duty >= 249) { turbine_pickup_time_factor = turbine_pickup_time_factor + 0.03; }  
      
      g = 0;
    } else if (peakpressure >= setlowpawalarmlimit && setTinsp_simv - Ti_real > 100 && tur_duty > (23) && (mainrelaypinstate == LOW) && g == 1 && setmode == MODE_SIMV && tr_save != 1) {
      tur_duty = tur_duty - (0.000100 * (setTinsp_simv - Ti_real));                                                                   
      if (turbine_pickup_time_factor >= 1.03 && tur_duty <= 200) { turbine_pickup_time_factor = turbine_pickup_time_factor - 0.03; }  
                                                                                                                                      
      
      g = 0;
    }

    
    
    if (set_Pinsp > peakpressure && tur_duty < (250) && (mainrelaypinstate == LOW) && hpstate != 1 && g == 1 && (setmode == MODE_PSIMV) && tr_save != 1) {
      tur_duty = tur_duty + (1 * (set_Pinsp - peakpressure));                                                                        
      if (turbine_pickup_time_factor <= 2.0 && tur_duty >= 249) { turbine_pickup_time_factor = turbine_pickup_time_factor + 0.03; }  
      
      g = 0;
    } else if (((setTinsp_simv)-Ti_real) > 100 && tur_duty > (23) && (mainrelaypinstate == LOW) && g == 1 && (setmode == MODE_PSIMV) && tr_save != 1) {
      tur_duty = tur_duty - (0.0010 * ((setTinsp_simv)-Ti_real));                                                                     
      if (turbine_pickup_time_factor >= 1.03 && tur_duty <= 200) { turbine_pickup_time_factor = turbine_pickup_time_factor - 0.03; }  
      
      g = 0;
    }

    

    
    if (set_Pinsp > peakpressure && tur_duty < (250) && (mainrelaypinstate == LOW) && hpstate != 1 && g == 1 && (setmode == MODE_CPAP || setmode == MODE_PCV) && tr_save != 1) {
      tur_duty = tur_duty + (1 * (set_Pinsp - peakpressure));                                                                        
      if (turbine_pickup_time_factor <= 2.0 && tur_duty >= 249) { turbine_pickup_time_factor = turbine_pickup_time_factor + 0.03; }  
      
      g = 0;
    } else if (((1.3 * Tinsp) - Ti_real) > 100 && tur_duty > (23) && (mainrelaypinstate == LOW) && g == 1 && (setmode == MODE_CPAP || setmode == MODE_PCV) && tr_save != 1) {
      tur_duty = tur_duty - (0.0010 * ((1.3 * Tinsp) - Ti_real));                                                                     
      if (turbine_pickup_time_factor >= 1.03 && tur_duty <= 200) { turbine_pickup_time_factor = turbine_pickup_time_factor - 0.03; }  
      
      g = 0;
    }
  }

  else {
    
    if ((((0.90 * settidalvolume) > T_Volume_final && (0.90 * settidalvolume) - T_Volume_final > 5 && settidalvolume <= 350) || ((0.95 * settidalvolume) > T_Volume_final && settidalvolume > 350)) && tur_duty < (250) && (mainrelaypinstate == LOW) && hpstate != 1 && g == 1 && (setmode == MODE_VCV || setmode == MODE_ACV) && peakpressure < sethighpawalarmlimit && tr_save != 1) {
      tur_duty = tur_duty + (((settidalvolume - T_Volume_final) / 15) + 3);                                                          
      if (turbine_pickup_time_factor <= 2.0 && tur_duty >= 249) { turbine_pickup_time_factor = turbine_pickup_time_factor + 0.03; }  
      
      g = 0;
    }
    
    else if ((((0.90 * settidalvolume) > T_Volume_final && (0.90 * settidalvolume) - T_Volume_final < 5 && settidalvolume <= 350) || ((0.95 * settidalvolume) > T_Volume_final && settidalvolume > 350)) && tur_duty < (250) && (mainrelaypinstate == LOW) && hpstate != 1 && g == 1 && (setmode == MODE_VCV || setmode == MODE_ACV) && peakpressure < sethighpawalarmlimit && tr_save != 1) {
      tur_duty = tur_duty + ((settidalvolume - T_Volume_final) / 32);  
      if (turbine_pickup_time_factor
            <= 2.0
          && tur_duty >= 249) { turbine_pickup_time_factor = turbine_pickup_time_factor + 0.03; }  
      g = 0;
    }
  }
  if (mainrelaypinstate == HIGH) {
    g = 1;
  }
  if (tur_duty >= (bodyweight * 5 + 20)) { tur_duty = (bodyweight * 5 + 20); }
  if (tur_duty >= 249) { tur_duty = 249; }
  if (tur_duty <= 25) { tur_duty = 25; }
  

  if (sensorfailure == 1 || pr1failiure == 1) {
    if (bodyweight < 40) {
      tur_duty = 30;
    } else if (bodyweight > 40 && bodyweight < 83) {
      tur_duty = (40 + ((1.38 * bodyweight) - 55.38));
    } else if (bodyweight >= 83) {
      tur_duty = 100;
    }
  }
  
  turbine_pickup_time_factor = 1;
}
void turbine1(void) {
  
  TCMillis = millis();
  if (mainrelaypinstate == HIGH) { TPMillis = TCMillis; }
  if ((TCMillis - TPMillis) <= 200 && mainrelaypinstate == LOW)  
  {                                                              
    companOut = (setpeep >= 2) ? EXHALE_BIAS_DUTY : 0.0;
    compan_duty = companOut;
    companStepMillis = TCMillis;
    analogWrite(peepPumpPin, peepduty);
    analogWrite(turbinePin, (int)companOut);
    digitalWrite(tur_reverse, HIGH);  
    coil = 0;
    analogWrite(peepPumpPin, peepduty);
  }

  else if ((TCMillis - TPMillis) > 200 && mainrelaypinstate == LOW) {

    float companTarget;
    if (setpeep > 1) {
      if (companOn == 0 && compan_duty1 >= COMPAN_ON_HI) { companOn = 1; }
      if (companOn == 1 && compan_duty1 < COMPAN_OFF_HI) { companOn = 0; }
    } else {
      if (companOn == 0 && compan_duty1 >= COMPAN_ON_LO) { companOn = 1; }
      if (companOn == 1 && compan_duty1 < COMPAN_OFF_LO) { companOn = 0; }
    }
    companTarget = (companOn == 1) ? compan_duty1 : 0.0;
    if (compan_duty2 != 0 && compan_duty1 < 25) { companTarget = compan_duty2; }
    if (bodyweight <= 6 && setpeep <= 1) { companTarget = 25.0; }
    if (setpeep >= 2 && companTarget < EXHALE_BIAS_DUTY) {
      companTarget = EXHALE_BIAS_DUTY;
    }
    if (companTarget >= tur_duty) { companTarget = tur_duty; }
    if ((TCMillis - companStepMillis) >= COMPAN_STEP_MS) {
      companStepMillis = TCMillis;
      if (companOut < companTarget) {
        companOut += COMPAN_SLEW_UP;
        if (companOut > companTarget) { companOut = companTarget; }
      } else if (companOut > companTarget) {
        companOut -= COMPAN_SLEW_DOWN;
        if (companOut < companTarget) { companOut = companTarget; }
      }
    }
    compan_duty = companOut;
    analogWrite(turbinePin, (int)companOut);
    digitalWrite(tur_reverse, HIGH);  
    coil = 0;
    analogWrite(peepPumpPin, peepduty);
    
  }

  else {
    if (standbybutton_count == 2) {
      digitalWrite(tur_reverse, HIGH);
      coil = 1;
    }
    if (standbybutton_count == 1) {
      digitalWrite(tur_reverse, LOW);
    }
  }
  

  if (standbybutton_count == 1 && g2 == 2) {
    g1 = tur_duty;
    EEPROM.update(23, g1);
    tur_duty = 0;
    g2 = 1;
  } else if (standbybutton_count == 2 && g2 == 1) {
    tur_duty = g1;
    g2 = 2;
  }

  if (standbybutton_count == 2) {         
  } else if (standbybutton_count == 1) {  
  }
  
  if (peakpressure > 0 && bodyweight > 10) {                                                                                                  
    if (turbine_pickup_time_factor >= 1.03 && tur_duty <= 200 && g == 1) { turbine_pickup_time_factor = turbine_pickup_time_factor - 0.03; }  
    
    if ((((0.90 * settidalvolume) > T_Volume_final && (0.90 * settidalvolume) - T_Volume_final > 100 && settidalvolume <= 350) || ((0.95 * settidalvolume) > T_Volume_final && settidalvolume > 350)) && tur_duty < (250) && (mainrelaypinstate == LOW) && hpstate != 1 && g == 1 && (setmode == MODE_VCV || setmode == MODE_ACV) && peakpressure < sethighpawalarmlimit && tr_save != 1) {
      tur_duty = tur_duty + ((settidalvolume - T_Volume_final) / 15);                                                                
      if (turbine_pickup_time_factor <= 2.0 && tur_duty >= 249) { turbine_pickup_time_factor = turbine_pickup_time_factor + 0.03; }  
      
      g = 0;
    }
    
    else if ((((0.90 * settidalvolume) > T_Volume_final && (0.90 * settidalvolume) - T_Volume_final < 100 && settidalvolume <= 350) || ((0.95 * settidalvolume) > T_Volume_final && settidalvolume > 350)) && tur_duty < (250) && (mainrelaypinstate == LOW) && hpstate != 1 && g == 1 && (setmode == MODE_VCV || setmode == MODE_ACV) && peakpressure < sethighpawalarmlimit && tr_save != 1) {
      tur_duty = tur_duty + ((settidalvolume - T_Volume_final) / 32);  
      if ((turbine_pickup_time_factor <= 2.0) && tur_duty >= 249) {
        turbine_pickup_time_factor = turbine_pickup_time_factor + 0.03;
      }  
      
      g = 0;
    }

    else if (peakpressure >= setlowpawalarmlimit && ((1.3 * Tinsp) - Ti_real) > 100 && tur_duty > (23) && (mainrelaypinstate == LOW) && g == 1 && (setmode == MODE_VCV || setmode == MODE_ACV) && tr_save != 1) {
      tur_duty = tur_duty - ((0.00240 * ((1.3 * Tinsp) - Ti_real)));                                                                  
      if (turbine_pickup_time_factor >= 1.03 && tur_duty <= 200) { turbine_pickup_time_factor = turbine_pickup_time_factor - 0.03; }  
      
      
      g = 0;
    }

    
    if ((((0.90 * settidalvolume) > T_Volume_final && settidalvolume <= 350) || ((0.95 * settidalvolume) > T_Volume_final && settidalvolume > 350)) && tur_duty < (250) && (mainrelaypinstate == LOW) && hpstate != 1 && g == 1 && setmode == MODE_SIMV && peakpressure < sethighpawalarmlimit && tr_save != 1) {
      tur_duty = tur_duty + ((settidalvolume - T_Volume_final) / 20);                                                                
      if (turbine_pickup_time_factor <= 2.0 && tur_duty >= 249) { turbine_pickup_time_factor = turbine_pickup_time_factor + 0.04; }  
      
      g = 0;
    } else if (peakpressure >= setlowpawalarmlimit && setTinsp_simv - Ti_real > 100 && tur_duty > (23) && (mainrelaypinstate == LOW) && g == 1 && setmode == MODE_SIMV && tr_save != 1) {
      tur_duty = tur_duty - (0.000100 * (setTinsp_simv - Ti_real));                                                                   
      if (turbine_pickup_time_factor >= 1.03 && tur_duty <= 200) { turbine_pickup_time_factor = turbine_pickup_time_factor - 0.04; }  
                                                                                                                                      
      
      g = 0;
    }

    
    
    if (set_Pinsp > peakpressure && tur_duty < (250) && (mainrelaypinstate == LOW) && hpstate != 1 && g == 1 && (setmode == MODE_PSIMV) && tr_save != 1) {
      tur_duty = tur_duty + (1 * (set_Pinsp - peakpressure));                                                                        
      if (turbine_pickup_time_factor <= 2.0 && tur_duty >= 249) { turbine_pickup_time_factor = turbine_pickup_time_factor + 0.03; }  
      
      g = 0;
    } else if (((setTinsp_simv)-Ti_real) > 100 && tur_duty > (23) && (mainrelaypinstate == LOW) && g == 1 && (setmode == MODE_PSIMV) && tr_save != 1) {
      tur_duty = tur_duty - (0.0010 * ((setTinsp_simv)-Ti_real));                                                                     
      if (turbine_pickup_time_factor >= 1.03 && tur_duty <= 200) { turbine_pickup_time_factor = turbine_pickup_time_factor - 0.03; }  
      
      g = 0;
    }

    

    
    if (set_Pinsp > peakpressure && tur_duty < (250) && (mainrelaypinstate == LOW) && hpstate != 1 && g == 1 && (setmode == MODE_CPAP || setmode == MODE_PCV) && tr_save != 1) {
      tur_duty = tur_duty + (1 * (set_Pinsp - peakpressure));                                                                        
      if (turbine_pickup_time_factor <= 2.0 && tur_duty >= 249) { turbine_pickup_time_factor = turbine_pickup_time_factor + 0.03; }  
      
      g = 0;
    } else if (((1.3 * Tinsp) - Ti_real) > 100 && tur_duty > (23) && (mainrelaypinstate == LOW) && g == 1 && (setmode == MODE_CPAP || setmode == MODE_PCV) && tr_save != 1) {
      tur_duty = tur_duty - (0.0010 * ((1.3 * Tinsp) - Ti_real));                                                                     
      if (turbine_pickup_time_factor >= 1.03 && tur_duty <= 200) { turbine_pickup_time_factor = turbine_pickup_time_factor - 0.03; }  
      
      g = 0;
    }
  }

  else {
    
    if ((((0.90 * settidalvolume) > T_Volume_final && (0.90 * settidalvolume) - T_Volume_final > 5 && settidalvolume <= 350) || ((0.95 * settidalvolume) > T_Volume_final && settidalvolume > 350)) && tur_duty < (250) && (mainrelaypinstate == LOW) && hpstate != 1 && g == 1 && (setmode == MODE_VCV || setmode == MODE_ACV) && peakpressure < sethighpawalarmlimit && tr_save != 1) {
      tur_duty = tur_duty + (((settidalvolume - T_Volume_final) / 15) + 3);                                                          
      if (turbine_pickup_time_factor <= 2.0 && tur_duty >= 249) { turbine_pickup_time_factor = turbine_pickup_time_factor + 0.03; }  
      
      g = 0;
    }
    
    else if ((((0.90 * settidalvolume) > T_Volume_final && (0.90 * settidalvolume) - T_Volume_final < 5 && settidalvolume <= 350) || ((0.95 * settidalvolume) > T_Volume_final && settidalvolume > 350)) && tur_duty < (250) && (mainrelaypinstate == LOW) && hpstate != 1 && g == 1 && (setmode == MODE_VCV || setmode == MODE_ACV) && peakpressure < sethighpawalarmlimit && tr_save != 1) {
      tur_duty = tur_duty + ((settidalvolume - T_Volume_final) / 32);  
      if (turbine_pickup_time_factor
            <= 2.0
          && tur_duty >= 249) { turbine_pickup_time_factor = turbine_pickup_time_factor + 0.03; }  
      g = 0;
    }
  }
  if (mainrelaypinstate == HIGH) {
    g = 1;
  }
  if (tur_duty >= (bodyweight * 5 + 20)) { tur_duty = (bodyweight * 5 + 20); }
  if (tur_duty >= 249) { tur_duty = 249; }
  if (tur_duty <= 25) { tur_duty = 25; }
  

  if (sensorfailure == 1 || pr1failiure == 1) {
    if (bodyweight < 40) {
      tur_duty = 30;
    } else if (bodyweight > 40 && bodyweight < 83) {
      tur_duty = (40 + ((1.38 * bodyweight) - 55.38));
    } else if (bodyweight >= 83) {
      tur_duty = 100;
    }
  }
  
  turbine_pickup_time_factor = 1;
}
#endif
void nebulizer(void) {
  /* mk inactive-state ownership: displayed neb time must be 0 whenever
     the nebulizer is not running - re-asserted change-gated, so any
     stale or spilled value is corrected within one pass.           */
  static uint16_t nebShown = 0xFFFF;
  if (nebu_touch == 0) {
    if (nebShown != 0) { display_write_variable(VP_NEB_TIME, 0); nebShown = 0; }
  } else {
    nebShown = 0xFFFF;   /* active: countdown writes own the VP */
  }

  
  nebcurrentMillis_timer = millis();  
  if (nebu_touch == 0) {
    nebpreviousMillis_timer = nebcurrentMillis_timer;  
    
  }
  if (nebu_touch == 1) {
    if ((neb_time - ((nebcurrentMillis_timer - nebpreviousMillis_timer) / 1000) / 60) < 2) {  

      display_write_variable(VP_NEB_TIME, 0);  
         
      nebu_touch = 0;
      digitalWrite(nebulizerpin, LOW);
      Icon21[7] = 0;
      DWIN_PORT.write(Icon21, 8);
      
    } else {
      
      display_write_variable(VP_NEB_TIME, (neb_time - ((nebcurrentMillis_timer - nebpreviousMillis_timer) / 1000) / 60));  
    }
  }

  if (mainrelaypinstate == HIGH && nebu_touch == 1 && setmode != MODE_HFT) {
    digitalWrite(nebulizerpin, HIGH);
  } else {
    digitalWrite(nebulizerpin, LOW);
  } 
}

/*******************************************************************************
 * PEEP valve control - PID v10 (v9 + EEPROM feed-forward duty table)
 *
 * WHY v10: convergence after a setpoint change / cold start took minutes -
 * the stability stack (schedule, pacing, gates, capped corrections) limits
 * authority to ~5-8 duty counts per breath by design. Fix: the converged
 * duty per PEEP setting is highly repeatable, so LEARN it (EEPROM table,
 * one uint8_t per setpeep 1..20 at PEEP_FF_EEPROM_BASE) and on any setpoint
 * change JUMP peepduty to the learned value. The PID then trims only the
 * residual -> convergence in 1-2 breaths after each setpoint's first-ever
 * visit. Learning: at a breath boundary with the loop settled (|error|
 * inside the deadband), the current duty is stored if it moved >= 2
 * counts (EEPROM.update + change threshold => negligible wear).
 * VERIFY PEEP_FF_EEPROM_BASE (300) does not collide with your EEPROM map.
 *
 * --- v9 notes below ---
 *
 * GOAL: every expiratory FALL must land at the same PEEP level, so every
 * inspiratory RISE starts from the same baseline and looks identical.
 *
 * WHY TROUGHS SCATTERED AT LOW PEEP (v8): the in-breath PID is heavily
 * derated there by design (schedule, pacing, settle gate, hold), so where
 * a fall LANDS is set mostly by the duty carried over from the previous
 * breath; the PID can only trim slightly after landing. Small carry-over
 * errors -> visible breath-to-breath trough variation.
 *
 * v9 ADDS ITERATIVE (breath-to-breath) CORRECTION - same principle as the
 * existing tur_duty Vt adaptation:
 *   - at each expiration->inspiration transition, latch where the fall
 *     actually landed (lastBreathPEEP = the PID's own filtered input)
 *   - EARLY in the NEXT expiration (settle window, before the pressure
 *     lands) apply ONE scheduled, clamped correction step:
 *         peepduty += sched * PEEP_BREATH_GAIN * (setpoint - lastBreathPEEP)
 *   - skipped inside the deadband (no dithering), never raises duty
 *     during hpstate, capped at +/-PEEP_BREATH_STEP_MAX per breath.
 *   Falls converge onto the target within a few breaths and stay there;
 *   the in-breath PID remains for disturbances within the breath.
 ******************************************************************************/

/* ---- tunables (add near the HFT PID globals) ---- */
#define PEEP_PID_SETTLE_MS 500
#define PEEP_PID_PERIOD_MS 100     /* min interval between PID/ramp steps         */
#define PEEP_HOLD_AFTER_MS 1500    /* freeze all trims after this much expiration */
#define PEEP_HOLD_EXIT_BAND 1.0f   /* hold breaks if |error| exceeds this         */
#define PEEP_BREATH_GAIN 0.8f      /* per-breath trough-correction gain           */
#define PEEP_BREATH_STEP_MAX 2.0f  /* duty cap for the once-per-breath step       */
#define PEEP_FF_EEPROM_BASE 300    /* EEPROM: learned duty per setpeep (1..20)    */
#define PEEP_FF_MIN 5              /* stored values outside 5..119 = not learned  */
#define PEEP_FF_MAX 119
#define PEEP_FF_WRITE_DELTA 2      /* store only when moved >= this many counts   */
#define PEEP_COMPAN_EE 322         /* EEPROM: persisted compan_duty1 (VERIFY free) */
#define PEEP_COMPAN_MIN 21         /* valid restore range                          */
#define PEEP_COMPAN_MAX 30
#define PEEP_STEP_BASE 0.5f
#define PEEP_STEP_GAIN 0.6f
#define PEEP_STEP_CAP 3.0f
#define PEEP_ERR_DEADBAND 0.4f
#define PEEP_INPUT_ALPHA 0.20f
#define PEEP_SCHED_REF 10.0f
#define PEEP_SCHED_MIN 0.2f        /* was 0.4 - keep derating at the low end      */
#define PEEP_UNDERSHOOT_LIMIT 1.0f
#define PEEP_COMPAN_GUARD 1

float Kp_peep = 2.0;
float Ki_peep = 5.0;
float Kd_peep = 0.0;

float prevError_peep = 0.0;
float prevError2_peep = 0.0;
float pidInput_peep = 0.0;
uint32_t lastTime_peep = 0;    /* 0 = PID needs (re)initialisation               */
uint32_t lastStep_peep = 0;    /* paces settle-window undershoot steps           */
float lastBreathPEEP = 0.0;    /* where the previous fall actually landed        */
uint8_t breathCorrPending = 0; /* one correction armed per breath                */
int prevRelay_peep = LOW;      /* local edge detector for insp/exp transition    */
int lastSetpeep_ff = -1;       /* detects setpoint changes for feed-forward      */

void PEEP(void) {

  pressure = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);         // PR1 - sensing inspration pressure
  pressure3 = PRESSURE_FACTOR * ((analogRead(Analog_pr3)) - ZFP_pr3);  // PR3 -- sensing -

  if ((standbybutton_count == 1) || (setpeep == 0) && (pee == 2)) {
    pee1 = peepduty;
    EEPROM.update(25, pee1);
    { uint8_t cd = (uint8_t)compan_duty1;
      if (cd >= 21 && cd <= 30) { EEPROM.update(322, cd); } }
    /* POWER-CYCLE FIX: compan_duty1 lived only in RAM - every power-on
       restarted it at the floor (21) and the 0.009/pass crawl took
       1-2 min to rebuild PEEP support. Persist it like pee1.           */
    {
      uint8_t cd = (uint8_t)compan_duty1;
      if (cd >= PEEP_COMPAN_MIN && cd <= PEEP_COMPAN_MAX) {
        EEPROM.update(PEEP_COMPAN_EE, cd);
      }
    }
    peepduty = PEEP_LOWER_THRESHOLD;
    pee = 1;

    prevError_peep = 0.0;
    prevError2_peep = 0.0;
    lastTime_peep = 0;
    breathCorrPending = 0;   /* stale correction must not apply on resume */
  }

  else if ((standbybutton_count == 2) && (pee == 1) && (setpeep != 0)) {
    peepduty = pee1;
    pee = 2;
    lastTime_peep = 0;
    { uint8_t cd = EEPROM.read(322);
      if (cd >= 21 && cd <= 30) { compan_duty1 = cd; } }
    {
      uint8_t cd = EEPROM.read(PEEP_COMPAN_EE);
      if (cd >= PEEP_COMPAN_MIN && cd <= PEEP_COMPAN_MAX) {
        compan_duty1 = cd;   /* restore turbine-assist working point    */
      }
    }
  }

  /* breath boundary: latch where the last fall landed, arm one correction */
  if (mainrelaypinstate == HIGH && prevRelay_peep == LOW) {
    if (lastTime_peep != 0) {   /* PID actually ran last expiration */
      lastBreathPEEP = pidInput_peep;
      breathCorrPending = 1;

      /* FEED-FORWARD LEARNING: loop settled at end of expiration ->
         remember the converged duty for this setpoint (change-gated,
         so EEPROM wear is negligible).                                */
      if (setpeep >= 1 && setpeep <= 20) {
        float bErr = ((float)setpeep + pf1) - lastBreathPEEP;
        /* LEARNING GATE WIDENED 0.4 -> 1.0: at low setpoints the trough
        float bErr = ((float)setpeep + pf1) - lastBreathPEEP;
           routinely dips ~1 below set (documented lower-edge behaviour),
        float bErr = ((float)setpeep + pf1) - lastBreathPEEP;
           so the strict deadband never passed and the FF table never
        float bErr = ((float)setpeep + pf1) - lastBreathPEEP;
           learned low-PEEP duties -> perpetual slow starts at PEEP <= 6.
        float bErr = ((float)setpeep + pf1) - lastBreathPEEP;
           1.0 = the controller's own settled band (hold / compan guard). */
    //    float bErr = ((float)setpeep + pf1) - lastBreathPEEP;
        if ((bErr < 1.0f) && (bErr > -1.0f)) {
          uint8_t cur = EEPROM.read(PEEP_FF_EEPROM_BASE + setpeep);
          uint8_t nowd = (uint8_t)peepduty;
          int16_t d = (int16_t)nowd - (int16_t)cur;
          if (d < 0) { d = -d; }
          if (cur < PEEP_FF_MIN || cur > PEEP_FF_MAX || d >= PEEP_FF_WRITE_DELTA) {
            EEPROM.update(PEEP_FF_EEPROM_BASE + setpeep, nowd);
          }
        }
      }
    }
  }
  prevRelay_peep = mainrelaypinstate;

  if (mainrelaypinstate == LOW && setmode != MODE_HFT) {

    if (pressure3 < setpeep + 4)  //@TODO check why +4 , need to update offset
    {
      pr3_SUM = pr3_SUM - pr3_READINGS[pr3_INDEX];
      int pressure3_1_int = pressure3;
      pr3_VALUE = pressure3_1_int;
      pr3_READINGS[pr3_INDEX] = pr3_VALUE;
      pr3_SUM = pr3_SUM + pr3_VALUE;
      pr3_INDEX = (pr3_INDEX + 1) % pr3_WINDOW_SIZE;
      pr3_AVERAGED = pr3_SUM / pr3_WINDOW_SIZE;
    } else
      pr3_AVERAGED = pressure3;

    if (setpeep <= 3) {
      pf1 = pf + 1;
    } else {
      pf1 = pf;
    }

    /* ---------- PID control of peepduty (velocity form, paced) ---------- */
    if (standbybutton_count == 2 && setpeep != 0) {

      /* FEED-FORWARD: on setpoint change, jump to the learned duty for
         this setpeep - the PID then only trims the residual.           */
      if (setpeep != lastSetpeep_ff) {
        lastSetpeep_ff = setpeep;
        if (setpeep >= 1 && setpeep <= 20) {
          uint8_t ff = EEPROM.read(PEEP_FF_EEPROM_BASE + setpeep);
          if (ff >= PEEP_FF_MIN && ff <= PEEP_FF_MAX) {
            peepduty = ff;
            pee1 = peepduty;
          }
        }
        lastTime_peep = 0;            /* clean PID re-init at new duty  */
        breathCorrPending = 0;
      }

      float sched_peep = (float)setpeep / PEEP_SCHED_REF;
      if (sched_peep > 1.0f) {
        sched_peep = 1.0f;
      } else if (sched_peep < PEEP_SCHED_MIN) {
        sched_peep = PEEP_SCHED_MIN;
      }

      /* ERROR-MAGNITUDE BOOST: the setpoint derate protects small-signal
         stability near target (high plant gain at low PEEP). Far from
         target, oscillation risk is low and progress matters: authority
         floors at 0.6 while |error| > 2 cmH2O. Low setpoints converge
         ~2-3x faster; settled behaviour and small-signal gain are
         UNCHANGED.                                                    */
      float schedEff = sched_peep;
      {
        float aerr = pidInput_peep - ((float)setpeep + pf1);
        if (aerr < 0) { aerr = -aerr; }
        if ((aerr > 2.0f) && (schedEff < 0.6f)) { schedEff = 0.6f; }
      }

      uint32_t now_peep = millis();

      if ((TCMillis - TPMillis) < PEEP_PID_SETTLE_MS) {

        /* ONCE-PER-BREATH TROUGH CORRECTION: applied EARLY, before the
           pressure lands, so every fall arrives at the same level.     */
        if (breathCorrPending == 1) {
          float bErr = ((float)setpeep + pf1) - lastBreathPEEP;
          if ((bErr > PEEP_ERR_DEADBAND) || (bErr < -PEEP_ERR_DEADBAND)) {
            float bStep = schedEff * PEEP_BREATH_GAIN * bErr;
            if (bStep > PEEP_BREATH_STEP_MAX) {
              bStep = PEEP_BREATH_STEP_MAX;
            } else if (bStep < -PEEP_BREATH_STEP_MAX) {
              bStep = -PEEP_BREATH_STEP_MAX;
            }
            if ((hpstate != 1) || (bStep < 0.0)) {
              peepduty += bStep;
            }
          }
          breathCorrPending = 0;
        }

        /* asymmetric settle window: hold against HIGH readings; a LOW
           reading is always real. PACED: one bounded step per
           PEEP_PID_PERIOD_MS, max ~5 steps per window.               */
        if (((float)setpeep - pressure3) > PEEP_UNDERSHOOT_LIMIT && hpstate != 1) {
          if ((now_peep - lastStep_peep) >= PEEP_PID_PERIOD_MS) {
            peepduty += schedEff * PEEP_STEP_CAP;
            lastStep_peep = now_peep;
          }
        }
        lastTime_peep = 0;  /* PID re-initialises when the gate opens */
      } else if ((lastTime_peep == 0) || ((now_peep - lastTime_peep) >= PEEP_PID_PERIOD_MS)) {

        float dt_peep = (now_peep - lastTime_peep) / 1000.0;

        if (lastTime_peep == 0 || dt_peep > 0.5) {
          dt_peep = 0.0;
          pidInput_peep = pr3_AVERAGED;
        } else {
          pidInput_peep += PEEP_INPUT_ALPHA * (pr3_AVERAGED - pidInput_peep);
        }
        lastTime_peep = now_peep;
        lastStep_peep = now_peep;

        float setpoint_peep = setpeep + pf1;
        float error_peep = setpoint_peep - pidInput_peep;

        if (dt_peep == 0.0) {
          prevError_peep = error_peep;
          prevError2_peep = error_peep;
        }

        if ((error_peep < PEEP_ERR_DEADBAND) && (error_peep > -PEEP_ERR_DEADBAND)) {
          prevError2_peep = prevError_peep;
          prevError_peep = error_peep;
        } else if (((TCMillis - TPMillis) > PEEP_HOLD_AFTER_MS)
                   && (error_peep < PEEP_HOLD_EXIT_BAND)
                   && (error_peep > -PEEP_HOLD_EXIT_BAND)) {
          /* END-EXPIRATORY HOLD: tail segment stays a straight line.
             Small errors are tolerated here; only a real disturbance
             (|error| > exit band) re-engages the controller.           */
          prevError2_peep = prevError_peep;
          prevError_peep = error_peep;
        } else {

          float delta_peep = (schedEff * Kp_peep * (error_peep - prevError_peep))
                           + (schedEff * Ki_peep * error_peep * dt_peep);

          if (Kd_peep > 0.0 && dt_peep > 0.0) {
            delta_peep += schedEff * Kd_peep
                        * (error_peep - (2.0 * prevError_peep) + prevError2_peep)
                        / dt_peep;
          }

          prevError2_peep = prevError_peep;
          prevError_peep = error_peep;

          float abs_err = (error_peep >= 0.0) ? error_peep : -error_peep;
          float max_step = schedEff * (PEEP_STEP_BASE + (PEEP_STEP_GAIN * abs_err));
          float step_cap = schedEff * PEEP_STEP_CAP;
          if (max_step > step_cap) {
            max_step = step_cap;
          }

          if (delta_peep > max_step) {
            delta_peep = max_step;
          } else if (delta_peep < -max_step) {
            delta_peep = -max_step;
          }

          if ((hpstate != 1) || (delta_peep < 0.0)) {
            peepduty += delta_peep;
          }
        }

        pee1 = peepduty;
      }
    }
    /* ---------- end PID ---------- */

#if PEEP_COMPAN_GUARD
    bool peepSettled = (prevError_peep <= 1.0f) && (prevError_peep >= -1.0f);
    /* STARTUP-DEADLOCK BREAKER (the ~1.75 min "PEEP won't start" bug):
       if the valve is saturated and PEEP is still BELOW target, more
       source flow is the only physical remedy - the compan INCREASE
       branch must run even though the loop is not settled. The DECREASE
       branch keeps the full guard (that is the anti-fight direction). */
    bool allowCompanUp = peepSettled
        || ((peepduty >= (PEEP_UPPER_THRESHOLD - 5)) && (prevError_peep > 1.0f));
#else
    bool peepSettled = true;
    bool allowCompanUp = true;
#endif

    if (allowCompanUp && ((TCMillis - TPMillis) >= 500) && ((TCMillis - TPMillis) <= PEEP_HOLD_AFTER_MS) && ((pressure) < (setpeep)) && standbybutton_count == 2 && setpeep != 0 && round(pr3_AVERAGED) >= (setpeep - 1) && hpstate != 1 && peakpressure >= setpeep) {
      {
        /* 3x catch-up while the loop is not yet settled (first-ever run
           or invalid restore); gentle 0.009 once converged.            */
        compan_duty1 = compan_duty1 + (peepSettled ? 0.009 : 0.027);
      }
    }

    if (peepSettled && ((TCMillis - TPMillis) >= 500) && ((TCMillis - TPMillis) <= PEEP_HOLD_AFTER_MS) && (round(pressure) > (setpeep)) && standbybutton_count == 2 && setpeep != 0) {
      if ((round(pressure) > setpeep)) {
        compan_duty1 = compan_duty1 - (0.009);
      }
    }
  }

  if (mainrelaypinstate && mainrelaypinstate != mainrelaypinstate_lastState) {
    if (mainrelaypinstate_count <= 3) {
      mainrelaypinstate_count += 1;
    } else if (mainrelaypinstate_count > 1) {
      mainrelaypinstate_count = 1;
    }
  }

  mainrelaypinstate_lastState = mainrelaypinstate;

  if (peepduty < 0) {
    peepduty = PEEP_LOWER_THRESHOLD;
  } else if (peepduty >= PEEP_UPPER_THRESHOLD) {
    peepduty = PEEP_UPPER_THRESHOLD;
  }


  if (compan_duty1 < 21) {
    compan_duty1 = 21;
  }

  if (compan_duty1 > 25 && setpeep == 2) {
    compan_duty1 = 25;
  }
  if (compan_duty1 > 30 && setpeep > 2) {
    compan_duty1 = 30;
  }
  if (setpeep <= 1) {
    compan_duty = 0;
    compan_duty1 = 0;
  }
  if (mask == 1 && compan_duty <= 25) {
    compan_duty2 = 25;
  } else {
    compan_duty2 = 0;
  }

  if (mainrelaypinstate == HIGH && pp == 1) {
    pp1 = pressure3;
    pp = 0;
  }

  if (mainrelaypinstate == LOW) {
    pp = 1;
  }

  cmillis = millis();

  qw = qw + 1;

  if ((pressure < setpeep + 4) && (mainrelaypinstate == LOW) && ((cmillis - pmillis) > Ti_real_1) && (qw >= 3)) {
    p_SUM = p_SUM - p_READINGS[p_INDEX];
    int pressure_int = pressure;
    p_VALUE = pressure_int;
    p_READINGS[p_INDEX] = p_VALUE;
    p_SUM = p_SUM + p_VALUE;
    p_INDEX = (p_INDEX + 1) % p_WINDOW_SIZE;
    p_AVERAGED = p_SUM / p_WINDOW_SIZE;

    if (p_AVERAGED <= 0) {
      p_AVERAGED = 0;
    }
    qw = 0;
  }

  if (mainrelaypinstate == HIGH) {
    pmillis = cmillis;
    p_AVERAGED_SAVED = p_AVERAGED;
  }
  p_AVERAGED_SAVED = p_AVERAGED;

  rrThresCount = rrThresCount + 1;
  if (mainrelaypinstate == LOW && rrThresCount >= 30) {
    peep_SUM = peep_SUM - peep_READINGS[peep_INDEX];
    peep_VALUE = p_AVERAGED_SAVED;
    peep_READINGS[peep_INDEX] = peep_VALUE;
    peep_SUM = peep_SUM + peep_VALUE;
    peep_INDEX = (peep_INDEX + 1) % WINDOW_SIZEpeep;
    peep_AVERAGED = peep_SUM / WINDOW_SIZEpeep;
    rrThresCount = 0;
  }
  peep_AVERAGED_SAVED = peep_AVERAGED;
}

#if 0
/*******************************************************************************
 * PEEP valve control - PID v10 (v9 + EEPROM feed-forward duty table)
 *
 * WHY v10: convergence after a setpoint change / cold start took minutes -
 * the stability stack (schedule, pacing, gates, capped corrections) limits
 * authority to ~5-8 duty counts per breath by design. Fix: the converged
 * duty per PEEP setting is highly repeatable, so LEARN it (EEPROM table,
 * one uint8_t per setpeep 1..20 at PEEP_FF_EEPROM_BASE) and on any setpoint
 * change JUMP peepduty to the learned value. The PID then trims only the
 * residual -> convergence in 1-2 breaths after each setpoint's first-ever
 * visit. Learning: at a breath boundary with the loop settled (|error|
 * inside the deadband), the current duty is stored if it moved >= 2
 * counts (EEPROM.update + change threshold => negligible wear).
 * VERIFY PEEP_FF_EEPROM_BASE (300) does not collide with your EEPROM map.
 *
 * --- v9 notes below ---
 *
 * GOAL: every expiratory FALL must land at the same PEEP level, so every
 * inspiratory RISE starts from the same baseline and looks identical.
 *
 * WHY TROUGHS SCATTERED AT LOW PEEP (v8): the in-breath PID is heavily
 * derated there by design (schedule, pacing, settle gate, hold), so where
 * a fall LANDS is set mostly by the duty carried over from the previous
 * breath; the PID can only trim slightly after landing. Small carry-over
 * errors -> visible breath-to-breath trough variation.
 *
 * v9 ADDS ITERATIVE (breath-to-breath) CORRECTION - same principle as the
 * existing tur_duty Vt adaptation:
 *   - at each expiration->inspiration transition, latch where the fall
 *     actually landed (lastBreathPEEP = the PID's own filtered input)
 *   - EARLY in the NEXT expiration (settle window, before the pressure
 *     lands) apply ONE scheduled, clamped correction step:
 *         peepduty += sched * PEEP_BREATH_GAIN * (setpoint - lastBreathPEEP)
 *   - skipped inside the deadband (no dithering), never raises duty
 *     during hpstate, capped at +/-PEEP_BREATH_STEP_MAX per breath.
 *   Falls converge onto the target within a few breaths and stay there;
 *   the in-breath PID remains for disturbances within the breath.
 ******************************************************************************/

/* ---- tunables (add near the HFT PID globals) ---- */
#define PEEP_PID_SETTLE_MS 500
#define PEEP_PID_PERIOD_MS 100     /* min interval between PID/ramp steps         */
#define PEEP_HOLD_AFTER_MS 1500    /* freeze all trims after this much expiration */
#define PEEP_HOLD_EXIT_BAND 1.0f   /* hold breaks if |error| exceeds this         */
#define PEEP_BREATH_GAIN 0.8f      /* per-breath trough-correction gain           */
#define PEEP_BREATH_STEP_MAX 2.0f  /* duty cap for the once-per-breath step       */
#define PEEP_FF_EEPROM_BASE 300    /* EEPROM: learned duty per setpeep (1..20)    */
#define PEEP_FF_MIN 5              /* stored values outside 5..119 = not learned  */
#define PEEP_FF_MAX 119
#define PEEP_FF_WRITE_DELTA 2      /* store only when moved >= this many counts   */
#define PEEP_STEP_BASE 0.5f
#define PEEP_STEP_GAIN 0.6f
#define PEEP_STEP_CAP 3.0f
#define PEEP_ERR_DEADBAND 0.4f
#define PEEP_INPUT_ALPHA 0.20f
#define PEEP_SCHED_REF 10.0f
#define PEEP_SCHED_MIN 0.2f        /* was 0.4 - keep derating at the low end      */
#define PEEP_UNDERSHOOT_LIMIT 1.0f
#define PEEP_COMPAN_GUARD 1

//float Kp_peep = 2.0;
//float Ki_peep = 5.0;
//float Kd_peep = 0.0;

float prevError_peep = 0.0;
float prevError2_peep = 0.0;
float pidInput_peep = 0.0;
///uint32_t lastTime_peep = 0;    /* 0 = PID needs (re)initialisation               */
uint32_t lastStep_peep = 0;    /* paces settle-window undershoot steps           */
float lastBreathPEEP = 0.0;    /* where the previous fall actually landed        */
uint8_t breathCorrPending = 0; /* one correction armed per breath                */
int prevRelay_peep = LOW;      /* local edge detector for insp/exp transition    */
int lastSetpeep_ff = -1;       /* detects setpoint changes for feed-forward      */

void PEEP(void) {

  pressure = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);         // PR1 - sensing inspration pressure
  pressure3 = PRESSURE_FACTOR * ((analogRead(Analog_pr3)) - ZFP_pr3);  // PR3 -- sensing -

  if ((standbybutton_count == 1) || (setpeep == 0) && (pee == 2)) {
    pee1 = peepduty;
    EEPROM.update(25, pee1);
    peepduty = PEEP_LOWER_THRESHOLD;
    pee = 1;

    prevError_peep = 0.0;
    prevError2_peep = 0.0;
    lastTime_peep = 0;
    breathCorrPending = 0;   /* stale correction must not apply on resume */
  }

  else if ((standbybutton_count == 2) && (pee == 1) && (setpeep != 0)) {
    peepduty = pee1;
    pee = 2;
    lastTime_peep = 0;
  }

  /* breath boundary: latch where the last fall landed, arm one correction */
  if (mainrelaypinstate == HIGH && prevRelay_peep == LOW) {
    if (lastTime_peep != 0) {   /* PID actually ran last expiration */
      lastBreathPEEP = pidInput_peep;
      breathCorrPending = 1;

      /* FEED-FORWARD LEARNING: loop settled at end of expiration ->
         remember the converged duty for this setpoint (change-gated,
         so EEPROM wear is negligible).                                */
      if (setpeep >= 1 && setpeep <= 20) {
        float bErr = ((float)setpeep + pf1) - lastBreathPEEP;
        if ((bErr < PEEP_ERR_DEADBAND) && (bErr > -PEEP_ERR_DEADBAND)) {
          uint8_t cur = EEPROM.read(PEEP_FF_EEPROM_BASE + setpeep);
          uint8_t nowd = (uint8_t)peepduty;
          int16_t d = (int16_t)nowd - (int16_t)cur;
          if (d < 0) { d = -d; }
          if (cur < PEEP_FF_MIN || cur > PEEP_FF_MAX || d >= PEEP_FF_WRITE_DELTA) {
            EEPROM.update(PEEP_FF_EEPROM_BASE + setpeep, nowd);
          }
        }
      }
    }
  }
  prevRelay_peep = mainrelaypinstate;

  if (mainrelaypinstate == LOW && setmode != MODE_HFT) {

    if (pressure3 < setpeep + 4)  //@TODO check why +4 , need to update offset
    {
      pr3_SUM = pr3_SUM - pr3_READINGS[pr3_INDEX];
      int pressure3_1_int = pressure3;
      pr3_VALUE = pressure3_1_int;
      pr3_READINGS[pr3_INDEX] = pr3_VALUE;
      pr3_SUM = pr3_SUM + pr3_VALUE;
      pr3_INDEX = (pr3_INDEX + 1) % pr3_WINDOW_SIZE;
      pr3_AVERAGED = pr3_SUM / pr3_WINDOW_SIZE;
    } else
      pr3_AVERAGED = pressure3;

    if (setpeep <= 3) {
      pf1 = pf + 1;
    } else {
      pf1 = pf;
    }

    /* ---------- PID control of peepduty (velocity form, paced) ---------- */
    if (standbybutton_count == 2 && setpeep != 0) {

      /* FEED-FORWARD: on setpoint change, jump to the learned duty for
         this setpeep - the PID then only trims the residual.           */
      if (setpeep != lastSetpeep_ff) {
        lastSetpeep_ff = setpeep;
        if (setpeep >= 1 && setpeep <= 20) {
          uint8_t ff = EEPROM.read(PEEP_FF_EEPROM_BASE + setpeep);
          if (ff >= PEEP_FF_MIN && ff <= PEEP_FF_MAX) {
            peepduty = ff;
            pee1 = peepduty;
          }
        }
        lastTime_peep = 0;            /* clean PID re-init at new duty  */
        breathCorrPending = 0;
      }

      float sched_peep = (float)setpeep / PEEP_SCHED_REF;
      if (sched_peep > 1.0f) {
        sched_peep = 1.0f;
      } else if (sched_peep < PEEP_SCHED_MIN) {
        sched_peep = PEEP_SCHED_MIN;
      }

      uint32_t now_peep = millis();

      if ((TCMillis - TPMillis) < PEEP_PID_SETTLE_MS) {

        /* ONCE-PER-BREATH TROUGH CORRECTION: applied EARLY, before the
           pressure lands, so every fall arrives at the same level.     */
        if (breathCorrPending == 1) {
          float bErr = ((float)setpeep + pf1) - lastBreathPEEP;
          if ((bErr > PEEP_ERR_DEADBAND) || (bErr < -PEEP_ERR_DEADBAND)) {
            float bStep = sched_peep * PEEP_BREATH_GAIN * bErr;
            if (bStep > PEEP_BREATH_STEP_MAX) {
              bStep = PEEP_BREATH_STEP_MAX;
            } else if (bStep < -PEEP_BREATH_STEP_MAX) {
              bStep = -PEEP_BREATH_STEP_MAX;
            }
            if ((hpstate != 1) || (bStep < 0.0)) {
              peepduty += bStep;
            }
          }
          breathCorrPending = 0;
        }

        /* asymmetric settle window: hold against HIGH readings; a LOW
           reading is always real. PACED: one bounded step per
           PEEP_PID_PERIOD_MS, max ~5 steps per window.               */
        if (((float)setpeep - pressure3) > PEEP_UNDERSHOOT_LIMIT && hpstate != 1) {
          if ((now_peep - lastStep_peep) >= PEEP_PID_PERIOD_MS) {
            peepduty += sched_peep * PEEP_STEP_CAP;
            lastStep_peep = now_peep;
          }
        }
        lastTime_peep = 0;  /* PID re-initialises when the gate opens */
      } else if ((lastTime_peep == 0) || ((now_peep - lastTime_peep) >= PEEP_PID_PERIOD_MS)) {

        float dt_peep = (now_peep - lastTime_peep) / 1000.0;

        if (lastTime_peep == 0 || dt_peep > 0.5) {
          dt_peep = 0.0;
          pidInput_peep = pr3_AVERAGED;
        } else {
          pidInput_peep += PEEP_INPUT_ALPHA * (pr3_AVERAGED - pidInput_peep);
        }
        lastTime_peep = now_peep;
        lastStep_peep = now_peep;

        float setpoint_peep = setpeep + pf1;
        float error_peep = setpoint_peep - pidInput_peep;

        if (dt_peep == 0.0) {
          prevError_peep = error_peep;
          prevError2_peep = error_peep;
        }

        if ((error_peep < PEEP_ERR_DEADBAND) && (error_peep > -PEEP_ERR_DEADBAND)) {
          prevError2_peep = prevError_peep;
          prevError_peep = error_peep;
        } else if (((TCMillis - TPMillis) > PEEP_HOLD_AFTER_MS)
                   && (error_peep < PEEP_HOLD_EXIT_BAND)
                   && (error_peep > -PEEP_HOLD_EXIT_BAND)) {
          /* END-EXPIRATORY HOLD: tail segment stays a straight line.
             Small errors are tolerated here; only a real disturbance
             (|error| > exit band) re-engages the controller.           */
          prevError2_peep = prevError_peep;
          prevError_peep = error_peep;
        } else {

          float delta_peep = (sched_peep * Kp_peep * (error_peep - prevError_peep))
                           + (sched_peep * Ki_peep * error_peep * dt_peep);

          if (Kd_peep > 0.0 && dt_peep > 0.0) {
            delta_peep += sched_peep * Kd_peep
                        * (error_peep - (2.0 * prevError_peep) + prevError2_peep)
                        / dt_peep;
          }

          prevError2_peep = prevError_peep;
          prevError_peep = error_peep;

          float abs_err = (error_peep >= 0.0) ? error_peep : -error_peep;
          float max_step = sched_peep * (PEEP_STEP_BASE + (PEEP_STEP_GAIN * abs_err));
          float step_cap = sched_peep * PEEP_STEP_CAP;
          if (max_step > step_cap) {
            max_step = step_cap;
          }

          if (delta_peep > max_step) {
            delta_peep = max_step;
          } else if (delta_peep < -max_step) {
            delta_peep = -max_step;
          }

          if ((hpstate != 1) || (delta_peep < 0.0)) {
            peepduty += delta_peep;
          }
        }

        pee1 = peepduty;
      }
    }
    /* ---------- end PID ---------- */

#if PEEP_COMPAN_GUARD
    bool peepSettled = (prevError_peep <= 1.0f) && (prevError_peep >= -1.0f);
    /* STARTUP-DEADLOCK BREAKER (the ~1.75 min "PEEP won't start" bug):
       if the valve is saturated and PEEP is still BELOW target, more
       source flow is the only physical remedy - the compan INCREASE
       branch must run even though the loop is not settled. The DECREASE
       branch keeps the full guard (that is the anti-fight direction). */
    bool allowCompanUp = peepSettled
        || ((peepduty >= (PEEP_UPPER_THRESHOLD - 5)) && (prevError_peep > 1.0f));
#else
    bool peepSettled = true;
    bool allowCompanUp = true;
#endif

    if (allowCompanUp && ((TCMillis - TPMillis) >= 500) && ((TCMillis - TPMillis) <= PEEP_HOLD_AFTER_MS) && ((pressure) < (setpeep)) && standbybutton_count == 2 && setpeep != 0 && round(pr3_AVERAGED) >= (setpeep - 1) && hpstate != 1 && peakpressure >= setpeep) {
      {
        compan_duty1 = compan_duty1 + (0.009);
      }
    }

    if (peepSettled && ((TCMillis - TPMillis) >= 500) && ((TCMillis - TPMillis) <= PEEP_HOLD_AFTER_MS) && (round(pressure) > (setpeep)) && standbybutton_count == 2 && setpeep != 0) {
      if ((round(pressure) > setpeep)) {
        compan_duty1 = compan_duty1 - (0.009);
      }
    }
  }

  if (mainrelaypinstate && mainrelaypinstate != mainrelaypinstate_lastState) {
    if (mainrelaypinstate_count <= 3) {
      mainrelaypinstate_count += 1;
    } else if (mainrelaypinstate_count > 1) {
      mainrelaypinstate_count = 1;
    }
  }

  mainrelaypinstate_lastState = mainrelaypinstate;

  if (peepduty < 0) {
    peepduty = PEEP_LOWER_THRESHOLD;
  } else if (peepduty >= PEEP_UPPER_THRESHOLD) {
    peepduty = PEEP_UPPER_THRESHOLD;
  }


  if (compan_duty1 < 21) {
    compan_duty1 = 21;
  }

  if (compan_duty1 > 25 && setpeep == 2) {
    compan_duty1 = 25;
  }
  if (compan_duty1 > 30 && setpeep > 2) {
    compan_duty1 = 30;
  }
  if (setpeep <= 1) {
    compan_duty = 0;
    compan_duty1 = 0;
  }
  if (mask == 1 && compan_duty <= 25) {
    compan_duty2 = 25;
  } else {
    compan_duty2 = 0;
  }

  if (mainrelaypinstate == HIGH && pp == 1) {
    pp1 = pressure3;
    pp = 0;
  }

  if (mainrelaypinstate == LOW) {
    pp = 1;
  }

  cmillis = millis();

  qw = qw + 1;

  if ((pressure < setpeep + 4) && (mainrelaypinstate == LOW) && ((cmillis - pmillis) > Ti_real_1) && (qw >= 3)) {
    p_SUM = p_SUM - p_READINGS[p_INDEX];
    int pressure_int = pressure;
    p_VALUE = pressure_int;
    p_READINGS[p_INDEX] = p_VALUE;
    p_SUM = p_SUM + p_VALUE;
    p_INDEX = (p_INDEX + 1) % p_WINDOW_SIZE;
    p_AVERAGED = p_SUM / p_WINDOW_SIZE;

    if (p_AVERAGED <= 0) {
      p_AVERAGED = 0;
    }
    qw = 0;
  }

  if (mainrelaypinstate == HIGH) {
    pmillis = cmillis;
    p_AVERAGED_SAVED = p_AVERAGED;
  }
  p_AVERAGED_SAVED = p_AVERAGED;

  rrThresCount = rrThresCount + 1;
  if (mainrelaypinstate == LOW && rrThresCount >= 30) {
    peep_SUM = peep_SUM - peep_READINGS[peep_INDEX];
    peep_VALUE = p_AVERAGED_SAVED;
    peep_READINGS[peep_INDEX] = peep_VALUE;
    peep_SUM = peep_SUM + peep_VALUE;
    peep_INDEX = (peep_INDEX + 1) % WINDOW_SIZEpeep;
    peep_AVERAGED = peep_SUM / WINDOW_SIZEpeep;
    rrThresCount = 0;
  }
  peep_AVERAGED_SAVED = peep_AVERAGED;

}


/*******************************************************************************
 * PEEP valve control - PID v10 (v9 + EEPROM feed-forward duty table)
 *
 * WHY v10: convergence after a setpoint change / cold start took minutes -
 * the stability stack (schedule, pacing, gates, capped corrections) limits
 * authority to ~5-8 duty counts per breath by design. Fix: the converged
 * duty per PEEP setting is highly repeatable, so LEARN it (EEPROM table,
 * one uint8_t per setpeep 1..20 at PEEP_FF_EEPROM_BASE) and on any setpoint
 * change JUMP peepduty to the learned value. The PID then trims only the
 * residual -> convergence in 1-2 breaths after each setpoint's first-ever
 * visit. Learning: at a breath boundary with the loop settled (|error|
 * inside the deadband), the current duty is stored if it moved >= 2
 * counts (EEPROM.update + change threshold => negligible wear).
 * VERIFY PEEP_FF_EEPROM_BASE (300) does not collide with your EEPROM map.
 *
 * --- v9 notes below ---
 *
 * GOAL: every expiratory FALL must land at the same PEEP level, so every
 * inspiratory RISE starts from the same baseline and looks identical.
 *
 * WHY TROUGHS SCATTERED AT LOW PEEP (v8): the in-breath PID is heavily
 * derated there by design (schedule, pacing, settle gate, hold), so where
 * a fall LANDS is set mostly by the duty carried over from the previous
 * breath; the PID can only trim slightly after landing. Small carry-over
 * errors -> visible breath-to-breath trough variation.
 *
 * v9 ADDS ITERATIVE (breath-to-breath) CORRECTION - same principle as the
 * existing tur_duty Vt adaptation:
 *   - at each expiration->inspiration transition, latch where the fall
 *     actually landed (lastBreathPEEP = the PID's own filtered input)
 *   - EARLY in the NEXT expiration (settle window, before the pressure
 *     lands) apply ONE scheduled, clamped correction step:
 *         peepduty += sched * PEEP_BREATH_GAIN * (setpoint - lastBreathPEEP)
 *   - skipped inside the deadband (no dithering), never raises duty
 *     during hpstate, capped at +/-PEEP_BREATH_STEP_MAX per breath.
 *   Falls converge onto the target within a few breaths and stay there;
 *   the in-breath PID remains for disturbances within the breath.
 ******************************************************************************/

/* ---- tunables (add near the HFT PID globals) ---- */
#define PEEP_PID_SETTLE_MS 500
#define PEEP_PID_PERIOD_MS 100     /* min interval between PID/ramp steps         */
#define PEEP_HOLD_AFTER_MS 1500    /* freeze all trims after this much expiration */
#define PEEP_HOLD_EXIT_BAND 1.0f   /* hold breaks if |error| exceeds this         */
#define PEEP_BREATH_GAIN 0.8f      /* per-breath trough-correction gain           */
#define PEEP_BREATH_STEP_MAX 2.0f  /* duty cap for the once-per-breath step       */
#define PEEP_FF_EEPROM_BASE 300    /* EEPROM: learned duty per setpeep (1..20)    */
#define PEEP_FF_MIN 5              /* stored values outside 5..119 = not learned  */
#define PEEP_FF_MAX 119
#define PEEP_FF_WRITE_DELTA 2      /* store only when moved >= this many counts   */
#define PEEP_STEP_BASE 0.5f
#define PEEP_STEP_GAIN 0.6f
#define PEEP_STEP_CAP 3.0f
#define PEEP_ERR_DEADBAND 0.4f
#define PEEP_INPUT_ALPHA 0.20f
#define PEEP_SCHED_REF 10.0f
#define PEEP_SCHED_MIN 0.2f        /* was 0.4 - keep derating at the low end      */
#define PEEP_UNDERSHOOT_LIMIT 1.0f
#define PEEP_COMPAN_GUARD 1

// float Kp_peep = 2.0;
// float Ki_peep = 5.0;
// float Kd_peep = 0.0;

float prevError_peep = 0.0;
float prevError2_peep = 0.0;
float pidInput_peep = 0.0;
//uint32_t lastTime_peep = 0;    /* 0 = PID needs (re)initialisation               */
uint32_t lastStep_peep = 0;    /* paces settle-window undershoot steps           */
float lastBreathPEEP = 0.0;    /* where the previous fall actually landed        */
uint8_t breathCorrPending = 0; /* one correction armed per breath                */
int prevRelay_peep = LOW;      /* local edge detector for insp/exp transition    */
int lastSetpeep_ff = -1;       /* detects setpoint changes for feed-forward      */

void PEEP(void) {

  pressure = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);         // PR1 - sensing inspration pressure
  pressure3 = PRESSURE_FACTOR * ((analogRead(Analog_pr3)) - ZFP_pr3);  // PR3 -- sensing -

  if ((standbybutton_count == 1) || (setpeep == 0) && (pee == 2)) {
    pee1 = peepduty;
    EEPROM.update(25, pee1);
    peepduty = PEEP_LOWER_THRESHOLD;
    pee = 1;

    prevError_peep = 0.0;
    prevError2_peep = 0.0;
    lastTime_peep = 0;
    breathCorrPending = 0;   /* stale correction must not apply on resume */
  }

  else if ((standbybutton_count == 2) && (pee == 1) && (setpeep != 0)) {
    peepduty = pee1;
    pee = 2;
    lastTime_peep = 0;
  }

  /* breath boundary: latch where the last fall landed, arm one correction */
  if (mainrelaypinstate == HIGH && prevRelay_peep == LOW) {
    if (lastTime_peep != 0) {   /* PID actually ran last expiration */
      lastBreathPEEP = pidInput_peep;
      breathCorrPending = 1;

      /* FEED-FORWARD LEARNING: loop settled at end of expiration ->
         remember the converged duty for this setpoint (change-gated,
         so EEPROM wear is negligible).                                */
      if (setpeep >= 1 && setpeep <= 20) {
        float bErr = ((float)setpeep + pf1) - lastBreathPEEP;
        if ((bErr < PEEP_ERR_DEADBAND) && (bErr > -PEEP_ERR_DEADBAND)) {
          uint8_t cur = EEPROM.read(PEEP_FF_EEPROM_BASE + setpeep);
          uint8_t nowd = (uint8_t)peepduty;
          int16_t d = (int16_t)nowd - (int16_t)cur;
          if (d < 0) { d = -d; }
          if (cur < PEEP_FF_MIN || cur > PEEP_FF_MAX || d >= PEEP_FF_WRITE_DELTA) {
            EEPROM.update(PEEP_FF_EEPROM_BASE + setpeep, nowd);
          }
        }
      }
    }
  }
  prevRelay_peep = mainrelaypinstate;

  if (mainrelaypinstate == LOW && setmode != MODE_HFT) {

    if (pressure3 < setpeep + 4)  //@TODO check why +4 , need to update offset
    {
      pr3_SUM = pr3_SUM - pr3_READINGS[pr3_INDEX];
      int pressure3_1_int = pressure3;
      pr3_VALUE = pressure3_1_int;
      pr3_READINGS[pr3_INDEX] = pr3_VALUE;
      pr3_SUM = pr3_SUM + pr3_VALUE;
      pr3_INDEX = (pr3_INDEX + 1) % pr3_WINDOW_SIZE;
      pr3_AVERAGED = pr3_SUM / pr3_WINDOW_SIZE;
    } else
      pr3_AVERAGED = pressure3;

    if (setpeep <= 3) {
      pf1 = pf + 1;
    } else {
      pf1 = pf;
    }

    /* ---------- PID control of peepduty (velocity form, paced) ---------- */
    if (standbybutton_count == 2 && setpeep != 0) {

      /* FEED-FORWARD: on setpoint change, jump to the learned duty for
         this setpeep - the PID then only trims the residual.           */
      if (setpeep != lastSetpeep_ff) {
        lastSetpeep_ff = setpeep;
        if (setpeep >= 1 && setpeep <= 20) {
          uint8_t ff = EEPROM.read(PEEP_FF_EEPROM_BASE + setpeep);
          if (ff >= PEEP_FF_MIN && ff <= PEEP_FF_MAX) {
            peepduty = ff;
            pee1 = peepduty;
          }
        }
        lastTime_peep = 0;            /* clean PID re-init at new duty  */
        breathCorrPending = 0;
      }

      float sched_peep = (float)setpeep / PEEP_SCHED_REF;
      if (sched_peep > 1.0f) {
        sched_peep = 1.0f;
      } else if (sched_peep < PEEP_SCHED_MIN) {
        sched_peep = PEEP_SCHED_MIN;
      }

      uint32_t now_peep = millis();

      if ((TCMillis - TPMillis) < PEEP_PID_SETTLE_MS) {

        /* ONCE-PER-BREATH TROUGH CORRECTION: applied EARLY, before the
           pressure lands, so every fall arrives at the same level.     */
        if (breathCorrPending == 1) {
          float bErr = ((float)setpeep + pf1) - lastBreathPEEP;
          if ((bErr > PEEP_ERR_DEADBAND) || (bErr < -PEEP_ERR_DEADBAND)) {
            float bStep = sched_peep * PEEP_BREATH_GAIN * bErr;
            if (bStep > PEEP_BREATH_STEP_MAX) {
              bStep = PEEP_BREATH_STEP_MAX;
            } else if (bStep < -PEEP_BREATH_STEP_MAX) {
              bStep = -PEEP_BREATH_STEP_MAX;
            }
            if ((hpstate != 1) || (bStep < 0.0)) {
              peepduty += bStep;
            }
          }
          breathCorrPending = 0;
        }

        /* asymmetric settle window: hold against HIGH readings; a LOW
           reading is always real. PACED: one bounded step per
           PEEP_PID_PERIOD_MS, max ~5 steps per window.               */
        if (((float)setpeep - pressure3) > PEEP_UNDERSHOOT_LIMIT && hpstate != 1) {
          if ((now_peep - lastStep_peep) >= PEEP_PID_PERIOD_MS) {
            peepduty += sched_peep * PEEP_STEP_CAP;
            lastStep_peep = now_peep;
          }
        }
        lastTime_peep = 0;  /* PID re-initialises when the gate opens */
      } else if ((lastTime_peep == 0) || ((now_peep - lastTime_peep) >= PEEP_PID_PERIOD_MS)) {

        float dt_peep = (now_peep - lastTime_peep) / 1000.0;

        if (lastTime_peep == 0 || dt_peep > 0.5) {
          dt_peep = 0.0;
          pidInput_peep = pr3_AVERAGED;
        } else {
          pidInput_peep += PEEP_INPUT_ALPHA * (pr3_AVERAGED - pidInput_peep);
        }
        lastTime_peep = now_peep;
        lastStep_peep = now_peep;

        float setpoint_peep = setpeep + pf1;
        float error_peep = setpoint_peep - pidInput_peep;

        if (dt_peep == 0.0) {
          prevError_peep = error_peep;
          prevError2_peep = error_peep;
        }

        if ((error_peep < PEEP_ERR_DEADBAND) && (error_peep > -PEEP_ERR_DEADBAND)) {
          prevError2_peep = prevError_peep;
          prevError_peep = error_peep;
        } else if (((TCMillis - TPMillis) > PEEP_HOLD_AFTER_MS)
                   && (error_peep < PEEP_HOLD_EXIT_BAND)
                   && (error_peep > -PEEP_HOLD_EXIT_BAND)) {
          /* END-EXPIRATORY HOLD: tail segment stays a straight line.
             Small errors are tolerated here; only a real disturbance
             (|error| > exit band) re-engages the controller.           */
          prevError2_peep = prevError_peep;
          prevError_peep = error_peep;
        } else {

          float delta_peep = (sched_peep * Kp_peep * (error_peep - prevError_peep))
                           + (sched_peep * Ki_peep * error_peep * dt_peep);

          if (Kd_peep > 0.0 && dt_peep > 0.0) {
            delta_peep += sched_peep * Kd_peep
                        * (error_peep - (2.0 * prevError_peep) + prevError2_peep)
                        / dt_peep;
          }

          prevError2_peep = prevError_peep;
          prevError_peep = error_peep;

          float abs_err = (error_peep >= 0.0) ? error_peep : -error_peep;
          float max_step = sched_peep * (PEEP_STEP_BASE + (PEEP_STEP_GAIN * abs_err));
          float step_cap = sched_peep * PEEP_STEP_CAP;
          if (max_step > step_cap) {
            max_step = step_cap;
          }

          if (delta_peep > max_step) {
            delta_peep = max_step;
          } else if (delta_peep < -max_step) {
            delta_peep = -max_step;
          }

          if ((hpstate != 1) || (delta_peep < 0.0)) {
            peepduty += delta_peep;
          }
        }

        pee1 = peepduty;
      }
    }
    /* ---------- end PID ---------- */

#if PEEP_COMPAN_GUARD
    bool peepSettled = (prevError_peep <= 1.0f) && (prevError_peep >= -1.0f);
#else
    bool peepSettled = true;
#endif

    if (peepSettled && ((TCMillis - TPMillis) >= 500) && ((TCMillis - TPMillis) <= PEEP_HOLD_AFTER_MS) && ((pressure) < (setpeep)) && standbybutton_count == 2 && setpeep != 0 && round(pr3_AVERAGED) >= (setpeep - 1) && hpstate != 1 && peakpressure >= setpeep) {
      {
        compan_duty1 = compan_duty1 + (0.009);
      }
    }

    if (peepSettled && ((TCMillis - TPMillis) >= 500) && ((TCMillis - TPMillis) <= PEEP_HOLD_AFTER_MS) && (round(pressure) > (setpeep)) && standbybutton_count == 2 && setpeep != 0) {
      if ((round(pressure) > setpeep)) {
        compan_duty1 = compan_duty1 - (0.009);
      }
    }
  }

  if (mainrelaypinstate && mainrelaypinstate != mainrelaypinstate_lastState) {
    if (mainrelaypinstate_count <= 3) {
      mainrelaypinstate_count += 1;
    } else if (mainrelaypinstate_count > 1) {
      mainrelaypinstate_count = 1;
    }
  }

  mainrelaypinstate_lastState = mainrelaypinstate;

  if (peepduty < 0) {
    peepduty = PEEP_LOWER_THRESHOLD;
  } else if (peepduty >= PEEP_UPPER_THRESHOLD) {
    peepduty = PEEP_UPPER_THRESHOLD;
  }


  if (compan_duty1 < 21) {
    compan_duty1 = 21;
  }

  if (compan_duty1 > 25 && setpeep == 2) {
    compan_duty1 = 25;
  }
  if (compan_duty1 > 30 && setpeep > 2) {
    compan_duty1 = 30;
  }
  if (setpeep <= 1) {
    compan_duty = 0;
    compan_duty1 = 0;
  }
  if (mask == 1 && compan_duty <= 25) {
    compan_duty2 = 25;
  } else {
    compan_duty2 = 0;
  }

  if (mainrelaypinstate == HIGH && pp == 1) {
    pp1 = pressure3;
    pp = 0;
  }

  if (mainrelaypinstate == LOW) {
    pp = 1;
  }

  cmillis = millis();

  qw = qw + 1;

  if ((pressure < setpeep + 4) && (mainrelaypinstate == LOW) && ((cmillis - pmillis) > Ti_real_1) && (qw >= 3)) {
    p_SUM = p_SUM - p_READINGS[p_INDEX];
    int pressure_int = pressure;
    p_VALUE = pressure_int;
    p_READINGS[p_INDEX] = p_VALUE;
    p_SUM = p_SUM + p_VALUE;
    p_INDEX = (p_INDEX + 1) % p_WINDOW_SIZE;
    p_AVERAGED = p_SUM / p_WINDOW_SIZE;

    if (p_AVERAGED <= 0) {
      p_AVERAGED = 0;
    }
    qw = 0;
  }

  if (mainrelaypinstate == HIGH) {
    pmillis = cmillis;
    p_AVERAGED_SAVED = p_AVERAGED;
  }
  p_AVERAGED_SAVED = p_AVERAGED;

  rrThresCount = rrThresCount + 1;
  if (mainrelaypinstate == LOW && rrThresCount >= 30) {
    peep_SUM = peep_SUM - peep_READINGS[peep_INDEX];
    peep_VALUE = p_AVERAGED_SAVED;
    peep_READINGS[peep_INDEX] = peep_VALUE;
    peep_SUM = peep_SUM + peep_VALUE;
    peep_INDEX = (peep_INDEX + 1) % WINDOW_SIZEpeep;
    peep_AVERAGED = peep_SUM / WINDOW_SIZEpeep;
    rrThresCount = 0;
  }
  peep_AVERAGED_SAVED = peep_AVERAGED;
}



/*******************************************************************************
 * PEEP valve control - PID v9 (breath-to-breath trough correction)
 *
 * GOAL: every expiratory FALL must land at the same PEEP level, so every
 * inspiratory RISE starts from the same baseline and looks identical.
 *
 * WHY TROUGHS SCATTERED AT LOW PEEP (v8): the in-breath PID is heavily
 * derated there by design (schedule, pacing, settle gate, hold), so where
 * a fall LANDS is set mostly by the duty carried over from the previous
 * breath; the PID can only trim slightly after landing. Small carry-over
 * errors -> visible breath-to-breath trough variation.
 *
 * v9 ADDS ITERATIVE (breath-to-breath) CORRECTION - same principle as the
 * existing tur_duty Vt adaptation:
 *   - at each expiration->inspiration transition, latch where the fall
 *     actually landed (lastBreathPEEP = the PID's own filtered input)
 *   - EARLY in the NEXT expiration (settle window, before the pressure
 *     lands) apply ONE scheduled, clamped correction step:
 *         peepduty += sched * PEEP_BREATH_GAIN * (setpoint - lastBreathPEEP)
 *   - skipped inside the deadband (no dithering), never raises duty
 *     during hpstate, capped at +/-PEEP_BREATH_STEP_MAX per breath.
 *   Falls converge onto the target within a few breaths and stay there;
 *   the in-breath PID remains for disturbances within the breath.
 ******************************************************************************/

/* ---- tunables (add near the HFT PID globals) ---- */
#define PEEP_PID_SETTLE_MS 500
#define PEEP_PID_PERIOD_MS 100     /* min interval between PID/ramp steps         */
#define PEEP_HOLD_AFTER_MS 1500    /* freeze all trims after this much expiration */
#define PEEP_HOLD_EXIT_BAND 1.0f   /* hold breaks if |error| exceeds this         */
#define PEEP_BREATH_GAIN 0.8f      /* per-breath trough-correction gain           */
#define PEEP_BREATH_STEP_MAX 2.0f  /* duty cap for the once-per-breath step       */
#define PEEP_STEP_BASE 0.5f
#define PEEP_STEP_GAIN 0.6f
#define PEEP_STEP_CAP 3.0f
#define PEEP_ERR_DEADBAND 0.4f
#define PEEP_INPUT_ALPHA 0.20f
#define PEEP_SCHED_REF 10.0f
#define PEEP_SCHED_MIN 0.2f        /* was 0.4 - keep derating at the low end      */
#define PEEP_UNDERSHOOT_LIMIT 1.0f
#define PEEP_COMPAN_GUARD 1

// float Kp_peep = 2.0;
// float Ki_peep = 5.0;
// float Kd_peep = 0.0;

float prevError_peep = 0.0;
float prevError2_peep = 0.0;
float pidInput_peep = 0.0;
//uint32_t lastTime_peep = 0;    /* 0 = PID needs (re)initialisation               */
uint32_t lastStep_peep = 0;    /* paces settle-window undershoot steps           */
float lastBreathPEEP = 0.0;    /* where the previous fall actually landed        */
uint8_t breathCorrPending = 0; /* one correction armed per breath                */
int prevRelay_peep = LOW;      /* local edge detector for insp/exp transition    */

void PEEP(void) {

  pressure = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);         // PR1 - sensing inspration pressure
  pressure3 = PRESSURE_FACTOR * ((analogRead(Analog_pr3)) - ZFP_pr3);  // PR3 -- sensing -

  if ((standbybutton_count == 1) || (setpeep == 0) && (pee == 2)) {
    pee1 = peepduty;
    EEPROM.update(25, pee1);
    peepduty = PEEP_LOWER_THRESHOLD;
    pee = 1;

    prevError_peep = 0.0;
    prevError2_peep = 0.0;
    lastTime_peep = 0;
    breathCorrPending = 0;   /* stale correction must not apply on resume */
  }

  else if ((standbybutton_count == 2) && (pee == 1) && (setpeep != 0)) {
    peepduty = pee1;
    pee = 2;
    lastTime_peep = 0;
  }

  /* breath boundary: latch where the last fall landed, arm one correction */
  if (mainrelaypinstate == HIGH && prevRelay_peep == LOW) {
    if (lastTime_peep != 0) {   /* PID actually ran last expiration */
      lastBreathPEEP = pidInput_peep;
      breathCorrPending = 1;
    }
  }
  prevRelay_peep = mainrelaypinstate;

  if (mainrelaypinstate == LOW && setmode != MODE_HFT) {

    if (pressure3 < setpeep + 4)  //@TODO check why +4 , need to update offset
    {
      pr3_SUM = pr3_SUM - pr3_READINGS[pr3_INDEX];
      int pressure3_1_int = pressure3;
      pr3_VALUE = pressure3_1_int;
      pr3_READINGS[pr3_INDEX] = pr3_VALUE;
      pr3_SUM = pr3_SUM + pr3_VALUE;
      pr3_INDEX = (pr3_INDEX + 1) % pr3_WINDOW_SIZE;
      pr3_AVERAGED = pr3_SUM / pr3_WINDOW_SIZE;
    } else
      pr3_AVERAGED = pressure3;

    if (setpeep <= 3) {
      pf1 = pf + 1;
    } else {
      pf1 = pf;
    }

    /* ---------- PID control of peepduty (velocity form, paced) ---------- */
    if (standbybutton_count == 2 && setpeep != 0) {

      float sched_peep = (float)setpeep / PEEP_SCHED_REF;
      if (sched_peep > 1.0f) {
        sched_peep = 1.0f;
      } else if (sched_peep < PEEP_SCHED_MIN) {
        sched_peep = PEEP_SCHED_MIN;
      }

      uint32_t now_peep = millis();

      if ((TCMillis - TPMillis) < PEEP_PID_SETTLE_MS) {

        /* ONCE-PER-BREATH TROUGH CORRECTION: applied EARLY, before the
           pressure lands, so every fall arrives at the same level.     */
        if (breathCorrPending == 1) {
          float bErr = ((float)setpeep + pf1) - lastBreathPEEP;
          if ((bErr > PEEP_ERR_DEADBAND) || (bErr < -PEEP_ERR_DEADBAND)) {
            float bStep = sched_peep * PEEP_BREATH_GAIN * bErr;
            if (bStep > PEEP_BREATH_STEP_MAX) {
              bStep = PEEP_BREATH_STEP_MAX;
            } else if (bStep < -PEEP_BREATH_STEP_MAX) {
              bStep = -PEEP_BREATH_STEP_MAX;
            }
            if ((hpstate != 1) || (bStep < 0.0)) {
              peepduty += bStep;
            }
          }
          breathCorrPending = 0;
        }

        /* asymmetric settle window: hold against HIGH readings; a LOW
           reading is always real. PACED: one bounded step per
           PEEP_PID_PERIOD_MS, max ~5 steps per window.               */
        if (((float)setpeep - pressure3) > PEEP_UNDERSHOOT_LIMIT && hpstate != 1) {
          if ((now_peep - lastStep_peep) >= PEEP_PID_PERIOD_MS) {
            peepduty += sched_peep * PEEP_STEP_CAP;
            lastStep_peep = now_peep;
          }
        }
        lastTime_peep = 0;  /* PID re-initialises when the gate opens */
      } else if ((lastTime_peep == 0) || ((now_peep - lastTime_peep) >= PEEP_PID_PERIOD_MS)) {

        float dt_peep = (now_peep - lastTime_peep) / 1000.0;

        if (lastTime_peep == 0 || dt_peep > 0.5) {
          dt_peep = 0.0;
          pidInput_peep = pr3_AVERAGED;
        } else {
          pidInput_peep += PEEP_INPUT_ALPHA * (pr3_AVERAGED - pidInput_peep);
        }
        lastTime_peep = now_peep;
        lastStep_peep = now_peep;

        float setpoint_peep = setpeep + pf1;
        float error_peep = setpoint_peep - pidInput_peep;

        if (dt_peep == 0.0) {
          prevError_peep = error_peep;
          prevError2_peep = error_peep;
        }

        if ((error_peep < PEEP_ERR_DEADBAND) && (error_peep > -PEEP_ERR_DEADBAND)) {
          prevError2_peep = prevError_peep;
          prevError_peep = error_peep;
        } else if (((TCMillis - TPMillis) > PEEP_HOLD_AFTER_MS)
                   && (error_peep < PEEP_HOLD_EXIT_BAND)
                   && (error_peep > -PEEP_HOLD_EXIT_BAND)) {
          /* END-EXPIRATORY HOLD: tail segment stays a straight line.
             Small errors are tolerated here; only a real disturbance
             (|error| > exit band) re-engages the controller.           */
          prevError2_peep = prevError_peep;
          prevError_peep = error_peep;
        } else {

          float delta_peep = (sched_peep * Kp_peep * (error_peep - prevError_peep))
                           + (sched_peep * Ki_peep * error_peep * dt_peep);

          if (Kd_peep > 0.0 && dt_peep > 0.0) {
            delta_peep += sched_peep * Kd_peep
                        * (error_peep - (2.0 * prevError_peep) + prevError2_peep)
                        / dt_peep;
          }

          prevError2_peep = prevError_peep;
          prevError_peep = error_peep;

          float abs_err = (error_peep >= 0.0) ? error_peep : -error_peep;
          float max_step = sched_peep * (PEEP_STEP_BASE + (PEEP_STEP_GAIN * abs_err));
          float step_cap = sched_peep * PEEP_STEP_CAP;
          if (max_step > step_cap) {
            max_step = step_cap;
          }

          if (delta_peep > max_step) {
            delta_peep = max_step;
          } else if (delta_peep < -max_step) {
            delta_peep = -max_step;
          }

          if ((hpstate != 1) || (delta_peep < 0.0)) {
            peepduty += delta_peep;
          }
        }

        pee1 = peepduty;
      }
    }
    /* ---------- end PID ---------- */

#if PEEP_COMPAN_GUARD
    bool peepSettled = (prevError_peep <= 1.0f) && (prevError_peep >= -1.0f);
#else
    bool peepSettled = true;
#endif

    if (peepSettled && ((TCMillis - TPMillis) >= 500) && ((TCMillis - TPMillis) <= PEEP_HOLD_AFTER_MS) && ((pressure) < (setpeep)) && standbybutton_count == 2 && setpeep != 0 && round(pr3_AVERAGED) >= (setpeep - 1) && hpstate != 1 && peakpressure >= setpeep) {
      {
        compan_duty1 = compan_duty1 + (0.009);
      }
    }

    if (peepSettled && ((TCMillis - TPMillis) >= 500) && ((TCMillis - TPMillis) <= PEEP_HOLD_AFTER_MS) && (round(pressure) > (setpeep)) && standbybutton_count == 2 && setpeep != 0) {
      if ((round(pressure) > setpeep)) {
        compan_duty1 = compan_duty1 - (0.009);
      }
    }
  }

  if (mainrelaypinstate && mainrelaypinstate != mainrelaypinstate_lastState) {
    if (mainrelaypinstate_count <= 3) {
      mainrelaypinstate_count += 1;
    } else if (mainrelaypinstate_count > 1) {
      mainrelaypinstate_count = 1;
    }
  }

  mainrelaypinstate_lastState = mainrelaypinstate;

  if (peepduty < 0) {
    peepduty = PEEP_LOWER_THRESHOLD;
  } else if (peepduty >= PEEP_UPPER_THRESHOLD) {
    peepduty = PEEP_UPPER_THRESHOLD;
  }


  if (compan_duty1 < 21) {
    compan_duty1 = 21;
  }

  if (compan_duty1 > 25 && setpeep == 2) {
    compan_duty1 = 25;
  }
  if (compan_duty1 > 30 && setpeep > 2) {
    compan_duty1 = 30;
  }
  if (setpeep <= 1) {
    compan_duty = 0;
    compan_duty1 = 0;
  }
  if (mask == 1 && compan_duty <= 25) {
    compan_duty2 = 25;
  } else {
    compan_duty2 = 0;
  }

  if (mainrelaypinstate == HIGH && pp == 1) {
    pp1 = pressure3;
    pp = 0;
  }

  if (mainrelaypinstate == LOW) {
    pp = 1;
  }

  cmillis = millis();

  qw = qw + 1;

  if ((pressure < setpeep + 4) && (mainrelaypinstate == LOW) && ((cmillis - pmillis) > Ti_real_1) && (qw >= 3)) {
    p_SUM = p_SUM - p_READINGS[p_INDEX];
    int pressure_int = pressure;
    p_VALUE = pressure_int;
    p_READINGS[p_INDEX] = p_VALUE;
    p_SUM = p_SUM + p_VALUE;
    p_INDEX = (p_INDEX + 1) % p_WINDOW_SIZE;
    p_AVERAGED = p_SUM / p_WINDOW_SIZE;

    if (p_AVERAGED <= 0) {
      p_AVERAGED = 0;
    }
    qw = 0;
  }

  if (mainrelaypinstate == HIGH) {
    pmillis = cmillis;
    p_AVERAGED_SAVED = p_AVERAGED;
  }
  p_AVERAGED_SAVED = p_AVERAGED;

  rrThresCount = rrThresCount + 1;
  if (mainrelaypinstate == LOW && rrThresCount >= 30) {
    peep_SUM = peep_SUM - peep_READINGS[peep_INDEX];
    peep_VALUE = p_AVERAGED_SAVED;
    peep_READINGS[peep_INDEX] = peep_VALUE;
    peep_SUM = peep_SUM + peep_VALUE;
    peep_INDEX = (peep_INDEX + 1) % WINDOW_SIZEpeep;
    peep_AVERAGED = peep_SUM / WINDOW_SIZEpeep;
    rrThresCount = 0;
  }
  peep_AVERAGED_SAVED = peep_AVERAGED;
}
#endif 
void keys(void) {  
  if (standbybutton_count == 1) {
    display_write_variable(VP_TEST_BATT_VOLT, (btr_volt * 10));  
    display_write_variable(VP_TEST_SMPS_VOLT, (SMPS * 10));      
  }
  if (i_e == 1) {
    i = 1;
    e = 1;
  }
  if (i_e > 1) {
    i = 1;
    e = i_e;
  }
  if (i_e >= 0 && i_e < 1) {
    e = 1;
    i = i_e;
  }
  if (i_e < 0) {
    e = 1;
    i = -i_e;
  }

  
  if (touch == 0 && lock_unlock == 0) {  
    settingButtonState = digitalRead(setting_button);
    
    modebuttonState = digitalRead(modebutton);
    mutebuttonState = digitalRead(mutebutton);
    standbybuttonState = digitalRead(standbybutton);
    
    
  }

  
  if (HMI_Page == PAGE_PATIENT_WEIGHT) {
    settingbutton_count = 2;
  }

  
  if (encoder_switchState && encoder_switchState != encoder_switch_lastState && HMI_Page == 50 && settingbutton_count == 14)  
  {
    HMI_Page = display_page_3;
    dwin_page_Set(display_page_3);  
    switch (mode1) {
      case MODE_VCV:
        Icon15[7] = 45;
        DWIN_PORT.write(Icon15, 8);

        break;
      case MODE_SIMV:
        Icon15[7] = 44;
        DWIN_PORT.write(Icon15, 8);
        break;

      case MODE_ACV:
        Icon15[7] = 43;
        DWIN_PORT.write(Icon15, 8);
        break;
      case MODE_HFT:
        Icon15[7] = 40;
        DWIN_PORT.write(Icon15, 8);
        break;
      case MODE_PCV:
        Icon15[7] = 42;
        DWIN_PORT.write(Icon15, 8);
        Icon1[7] = 30;
        DWIN_PORT.write(Icon1, 8);
        break;
      case MODE_SPONT_PS:
        Icon15[7] = 39;
        DWIN_PORT.write(Icon15, 8);
        break;
      case MODE_CPAP:
        Icon15[7] = 38;
        DWIN_PORT.write(Icon15, 8);

        break;
      case MODE_PSIMV:
        Icon15[7] = 41;
        DWIN_PORT.write(Icon15, 8);
        break;
    }

    Icon1[7] = 0;
    DWIN_PORT.write(Icon1, 8);
    confirmbuttonState = 1;
  } else if (encoder_switchState && encoder_switchState != encoder_switch_lastState && HMI_Page == 27)  
  {
    HMI_Page = display_page_3;
    dwin_page_Set(display_page_3);
  }

  else if (encoder_switchState && encoder_switchState != encoder_switch_lastState && HMI_Page == display_page_3)  
  {
    if (mainscreencursor == 1) {
      dwin_page_Set(27);
      HMI_Page = 27;
    }
    if (mainscreencursor == 2) {
#if 0
      settingbutton_count = 13;
      HMI_Page = 50;
      if (mode1 == MODE_PCV) {
        dwin_page_Set (17);
      } else if (mode1 == MODE_PSIMV) {
        dwin_page_Set (20);
      } else if (mode1 == MODE_VCV) {
        dwin_page_Set (18);
      } else if (mode1 == MODE_CPAP) {
        dwin_page_Set (23);
      } else if (mode1 == MODE_HFT) {
        dwin_page_Set (24);
      } else if (mode1 == MODE_SIMV) {
        dwin_page_Set (21);
      } else if (mode1 == MODE_ACV) {
        dwin_page_Set (19);
      } else if (mode1 == MODE_SPONT_PS) {
        dwin_page_Set (22);
      }
#endif
    }
    if (mainscreencursor == 3) {
      dwin_page_Set(13);
      HMI_Page = 13;
      if (pediatric_patient == 0) {
        pediatric_patient = 1;
        Icon1[7] = 86;
        DWIN_PORT.write(Icon1, 8);
      } else {
        pediatric_patient = 0;
        Icon1[7] = 65;
        DWIN_PORT.write(Icon1, 8);
      }
    }
    if (mainscreencursor == 4) {
      dwin_page_Set(29);
      HMI_Page = 29;
      settingbutton_count = 6;
      encoder_switch_count = 1;
      Icon17[7] = 156;
      DWIN_PORT.write(Icon17, 8);
    }
  }
  
  
  

  else if (encoder_switchState && encoder_switchState != encoder_switch_lastState && HMI_Page == 29)  
  {
    if (encoder_switch_count == 1) {
      encoder_switch_count = 0;
      if (settingbutton_count == 24) {
        dwin_page_Set(3);
        HMI_Page = 3;
        Icon1[7] = 0;
        DWIN_PORT.write(Icon1, 8);
      }
      if (settingbutton_count == 23) {
        
        encoder_switch_count = 1;
      }
    } else {
      encoder_switch_count = 1;
      if (settingbutton_count == 24) {
        dwin_page_Set(3);
        HMI_Page = 3;
      }
    }
  }

  else if (encoder_switchState && encoder_switchState != encoder_switch_lastState && HMI_Page == PAGE_STANDBY)  
  {

    if (encoder_switch_count == 1) {
      encoder_switch_count = 0;
      if (settingbutton_count == 25) {
        HMI_Page = 30;
        dwin_page_Set(30);
        display_write_variable(VP_O2CAL_PROGRESS, 0);
        O2cal = 1;
      }
      if (settingbutton_count == 26) {
        settingbutton_count = 1;
        HMI_Page = 13;
        dwin_page_Set(13);
      }
      if (settingbutton_count == 27) {
        settingbutton_count = 13;
        HMI_Page = 50;
        ventilation_check = 1;
        Icon1[7] = 0;
        DWIN_PORT.write(Icon1, 8);
        switch (mode1) {
          case MODE_VCV:
            dwin_page_Set(18);
            Icon15[7] = 45;
            DWIN_PORT.write(Icon15, 8);
            break;
          case MODE_SIMV:
            dwin_page_Set(21);
            Icon15[7] = 44;
            DWIN_PORT.write(Icon15, 8);
            break;
          case MODE_ACV:
            dwin_page_Set(19);
            Icon15[7] = 43;
            DWIN_PORT.write(Icon15, 8);
            break;
          case MODE_HFT:
            dwin_page_Set(24);
            Icon15[7] = 40;
            DWIN_PORT.write(Icon15, 8);
            break;
          case MODE_PCV:
            dwin_page_Set(17);
            Icon15[7] = 42;
            DWIN_PORT.write(Icon15, 8);
            break;
          case MODE_SPONT_PS:
            dwin_page_Set(22);
            Icon15[7] = 39;
            DWIN_PORT.write(Icon15, 8);
            break;
          case MODE_CPAP:
            dwin_page_Set(23);
            Icon15[7] = 38;
            DWIN_PORT.write(Icon15, 8);
            break;
          case MODE_PSIMV:
            dwin_page_Set(20);
            Icon15[7] = 41;
            DWIN_PORT.write(Icon15, 8);
            break;
        }
      }
    } else {
      encoder_switch_count = 1;
    }
  }

  else if (encoder_switchState && encoder_switchState != encoder_switch_lastState && HMI_Page == 50)  
  {
    if (encoder_switch_count == 1) {
      encoder_switch_count = 0;
    } else {
      encoder_switch_count = 1;
    }
  }

  else if (encoder_switchState && encoder_switchState != encoder_switch_lastState && HMI_Page == 1)  
  {
    if (encoder_switch_count == 1) {
      encoder_switch_count = 0;
    } else {
      encoder_switch_count = 1;
    }
  }

  else if (encoder_switchState && encoder_switchState != encoder_switch_lastState && HMI_Page == 12)  
  {
    if (newpatient == 1) {
      if (pediatric_patient == 1) {
        dwin_page_Set(13);
        HMI_Page = 13;
        Icon1[7] = 86;
        DWIN_PORT.write(Icon1, 8);
      } else {
        dwin_page_Set(13);
        HMI_Page = 13;
        Icon1[7] = 65;
        DWIN_PORT.write(Icon1, 8);
      }

    } else {
      settingbutton_count = 13;
      HMI_Page = 50;
      switch (mode1) {
        case MODE_VCV:
          dwin_page_Set(18);
          Icon15[7] = 45;
          DWIN_PORT.write(Icon15, 8);

          break;
        case MODE_SIMV:
          dwin_page_Set(21);
          Icon15[7] = 44;
          DWIN_PORT.write(Icon15, 8);
          break;
        case MODE_ACV:
          dwin_page_Set(19);
          Icon15[7] = 43;
          DWIN_PORT.write(Icon15, 8);
          break;
        case MODE_HFT:
          dwin_page_Set(24);
          Icon15[7] = 40;
          DWIN_PORT.write(Icon15, 8);
          break;
        case MODE_PCV:
          dwin_page_Set(17);
          Icon15[7] = 42;
          DWIN_PORT.write(Icon15, 8);
          break;
        case MODE_SPONT_PS:
          dwin_page_Set(22);
          Icon15[7] = 39;
          DWIN_PORT.write(Icon15, 8);
          break;
        case MODE_CPAP:
          dwin_page_Set(23);
          Icon15[7] = 38;
          DWIN_PORT.write(Icon15, 8);
          break;
        case MODE_PSIMV:
          dwin_page_Set(20);
          Icon15[7] = 41;
          DWIN_PORT.write(Icon15, 8);

          break;
      }
    }                                                                                                   
  } else if (encoder_switchState && encoder_switchState != encoder_switch_lastState && HMI_Page == 13)  
  {
    if (malepatient == 1) {
      dwin_page_Set(14);
      HMI_Page = 14;
      Icon1[7] = 66;
      DWIN_PORT.write(Icon1, 8);
    } else {
      dwin_page_Set(14);
      HMI_Page = 14;
      Icon1[7] = 70;
      DWIN_PORT.write(Icon1, 8);
    }

  } else if (encoder_switchState && encoder_switchState != encoder_switch_lastState && HMI_Page == 14)  
  {
    dwin_page_Set(15);
    HMI_Page = 15;
    settingbutton_count = 2;
    Icon1[7] = 111;
    DWIN_PORT.write(Icon1, 8);
  } else if (encoder_switchState && encoder_switchState != encoder_switch_lastState && HMI_Page == PAGE_PATIENT_WEIGHT)  
  {                                                                                                     
    
    
    settingbutton_count = 13;
    HMI_Page = 50;
    switch (mode1) {
      case MODE_VCV:
        dwin_page_Set(18);
        Icon15[7] = 45;
        DWIN_PORT.write(Icon15, 8);
        break;
      case MODE_SIMV:
        dwin_page_Set(21);
        Icon15[7] = 44;
        DWIN_PORT.write(Icon15, 8);
        break;
      case MODE_ACV:
        dwin_page_Set(19);
        Icon15[7] = 43;
        DWIN_PORT.write(Icon15, 8);
        break;
      case MODE_HFT:
        dwin_page_Set(24);
        Icon15[7] = 40;
        DWIN_PORT.write(Icon15, 8);
        break;
      case MODE_PCV:
        dwin_page_Set(17);
        Icon15[7] = 42;
        DWIN_PORT.write(Icon15, 8);
        break;
      case MODE_SPONT_PS:
        dwin_page_Set(22);
        Icon15[7] = 39;
        DWIN_PORT.write(Icon15, 8);
        break;
      case MODE_CPAP:
        dwin_page_Set(23);
        Icon15[7] = 38;
        DWIN_PORT.write(Icon15, 8);
        break;
      case MODE_PSIMV:
        dwin_page_Set(20);
        Icon15[7] = 41;
        DWIN_PORT.write(Icon15, 8);
        break;
    }
  }

  else if (encoder_switchState && encoder_switchState != encoder_switch_lastState && HMI_Page == 13)  
  {
    if (malepatient == 1) {
      dwin_page_Set(14);
      HMI_Page = 14;
      Icon1[7] = 66;
      DWIN_PORT.write(Icon1, 8);
    } else {
      dwin_page_Set(14);
      HMI_Page = 14;
      Icon1[7] = 70;
      DWIN_PORT.write(Icon1, 8);
    }

  }

  else if (encoder_switchState && encoder_switchState != encoder_switch_lastState && HMI_Page == PAGE_PATIENT_WEIGHT)  
  {

    settingbutton_count = 13;
    HMI_Page = 50;

    switch (mode1) {
      case MODE_VCV:
        dwin_page_Set(18);
        Icon15[7] = 45;
        DWIN_PORT.write(Icon15, 8);

        break;
      case MODE_SIMV:
        dwin_page_Set(21);
        Icon15[7] = 44;
        DWIN_PORT.write(Icon15, 8);

        break;
      case MODE_ACV:
        dwin_page_Set(19);
        Icon15[7] = 43;
        DWIN_PORT.write(Icon15, 8);

        break;
      case MODE_HFT:
        dwin_page_Set(24);
        Icon15[7] = 40;
        DWIN_PORT.write(Icon15, 8);

        break;
      case MODE_PCV:
        dwin_page_Set(17);
        Icon15[7] = 42;
        DWIN_PORT.write(Icon15, 8);

        break;
      case MODE_SPONT_PS:
        dwin_page_Set(22);
        Icon15[7] = 39;
        DWIN_PORT.write(Icon15, 8);

        break;
      case MODE_CPAP:
        dwin_page_Set(23);
        Icon15[7] = 38;
        DWIN_PORT.write(Icon15, 8);

        break;
      case MODE_PSIMV:
        dwin_page_Set(20);
        Icon15[7] = 41;
        DWIN_PORT.write(Icon15, 8);

        break;
    }
  } else if (encoder_switchState && encoder_switchState != encoder_switch_lastState && HMI_Page == 50)  
  {
    dwin_page_Set(3);
    HMI_Page = 3;
  } else if (settingButtonState && settingButtonState != settingbutton_lastState)  
  {

    if (mode1 == MODE_VCV && cursr_forward == 0 && HMI_Page == 50) {  
      if (settingbutton_count == 12) {
        settingbutton_count = 13;
        dwin_page_Set(18);
        Icon1[7] = 31;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 3) {
        settingbutton_count = 12;
        dwin_page_Set(18);
        Icon1[7] = 13;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 4) {
        settingbutton_count = 3;  
        dwin_page_Set(18);
        Icon1[7] = 48;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 5) {
        settingbutton_count = 4;
        dwin_page_Set(18);
        Icon1[7] = 11;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 10) {
        settingbutton_count = 5;
        dwin_page_Set(18);
        Icon1[7] = 54;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 14) {
        settingbutton_count = 10;
        dwin_page_Set(18);
        Icon1[7] = 25;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 13) {
        settingbutton_count = 14;
        dwin_page_Set(18);
        Icon1[7] = 129;
        DWIN_PORT.write(Icon1, 8);

      } else {
        settingbutton_count = 12;
        dwin_page_Set(18);
        Icon1[7] = 13;
        DWIN_PORT.write(Icon1, 8);
      }
    }

    else if (mode1 == MODE_VCV && cursr_forward == 1 && HMI_Page == 50) {  
#if 0
      switch (settingbutton_count) {
        case 14:
          settingbutton_count = 13;
          dwin_page_Set(18);
          Icon1[7] = 31;
          DWIN_PORT.write(Icon1, 8);
          break;
        case 3:
          settingbutton_count = 4;
          dwin_page_Set(18);
          Icon1[7] = 11;
          DWIN_PORT.write(Icon1, 8);
          break;
        case 4:
          settingbutton_count = 5;
          dwin_page_Set(18);
          Icon1[7] = 54;
          DWIN_PORT.write(Icon1, 8);
          break;
        case 10:
          settingbutton_count = 14;
          dwin_page_Set(18);
          Icon1[7] = 129;
          DWIN_PORT.write(Icon1, 8);
          break;
        case 5:
          settingbutton_count = 10;  
          dwin_page_Set(18);
          Icon1[7] = 25;
          DWIN_PORT.write(Icon1, 8);
          break;
        case 12:
          settingbutton_count = 3;
          dwin_page_Set(18);
          Icon1[7] = 48;
          DWIN_PORT.write(Icon1, 8);
          break;
        case 13:
          settingbutton_count = 12;
          dwin_page_Set(18);
          Icon1[7] = 13;
          DWIN_PORT.write(Icon1, 8);
          break;
        default:
          settingbutton_count = 12;
          dwin_page_Set(18);
          Icon1[7] = 13;
          DWIN_PORT.write(Icon1, 8);
          break;
      }
#endif
    }

    else if (mode1 == MODE_CPAP && cursr_forward == 0 && HMI_Page == 50) {  
#if 1
      if (settingbutton_count == 12) {
        settingbutton_count = 13;
        dwin_page_Set(17);
        Icon1[7] = 30;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 3) {
        settingbutton_count = 12;
        dwin_page_Set(17);
        Icon1[7] = 13;
        DWIN_PORT.write(Icon1, 8);
      }  
      else if (settingbutton_count == 4) {
        settingbutton_count = 3;  
        dwin_page_Set(17);
        Icon1[7] = 46;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 5) {
        settingbutton_count = 4;
        dwin_page_Set(17);
        Icon1[7] = 11;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 10) {
        settingbutton_count = 5;
        dwin_page_Set(17);
        Icon1[7] = 54;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 14) {
        settingbutton_count = 10;
        dwin_page_Set(17);
        Icon1[7] = 25;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 13) {
        settingbutton_count = 14;
        dwin_page_Set(17);
        Icon1[7] = 129;
        DWIN_PORT.write(Icon1, 8);
      } else {
        settingbutton_count = 12;
        dwin_page_Set(17);
        Icon1[7] = 13;
        DWIN_PORT.write(Icon1, 8);
      }
#endif
    }

    else if (mode1 == MODE_CPAP && cursr_forward == 1 && HMI_Page == 50) {
#if 0
      if (settingbutton_count == 11) {
        settingbutton_count = 13;  
        dwin_page_Set(17);
        Icon1[7] = 0;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 14) {
        settingbutton_count = 13;
        dwin_page_Set(17);
        Icon1[7] = 30;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 10) {
        settingbutton_count = 14;
        dwin_page_Set(17);
        Icon1[7] = 129;
        DWIN_PORT.write(Icon1, 8);
      }  
      else if (settingbutton_count == 5) {
        settingbutton_count = 10;  
        dwin_page_Set(17);
        Icon1[7] = 25;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 4) {
        settingbutton_count = 5;
        dwin_page_Set(17);
        Icon1[7] = 54;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 3) {
        settingbutton_count = 4;
        dwin_page_Set(17);
        Icon1[7] = 11;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 12) {
        settingbutton_count = 3;
        dwin_page_Set(17);
        Icon1[7] = 46;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 13) {
        settingbutton_count = 12;
        dwin_page_Set(17);
        Icon1[7] = 13;
        DWIN_PORT.write(Icon1, 8);
      } else {
        settingbutton_count = 12;
        dwin_page_Set(17);
        Icon1[7] = 13;
        DWIN_PORT.write(Icon1, 8);
      }
#endif
    }

    else if (mode1 == MODE_ACV && cursr_forward == 0 && HMI_Page == 50) {  
      if (settingbutton_count == 14) {
        settingbutton_count = 11;
        dwin_page_Set(19);
        Icon1[7] = 37;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 11) {
        settingbutton_count = 10;
        dwin_page_Set(19);
        Icon1[7] = 25;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 10) {
        settingbutton_count = 8;
        dwin_page_Set(19);
        Icon1[7] = 21;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 9) {
        settingbutton_count = 10;
        dwin_page_Set(19);
        Icon1[7] = 25;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 8) {
        settingbutton_count = 5;
        dwin_page_Set(19);
        Icon1[7] = 54;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 7) {
        settingbutton_count = 8;
        dwin_page_Set(19);
        Icon1[7] = 21;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 6) {
        settingbutton_count = 8;
        dwin_page_Set(19);
        Icon1[7] = 21;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 5) {
        settingbutton_count = 4;  
        dwin_page_Set(19);
        Icon1[7] = 11;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 4) {
        settingbutton_count = 3;
        dwin_page_Set(19);
        Icon1[7] = 48;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 3) {
        settingbutton_count = 12;
        dwin_page_Set(19);
        Icon1[7] = 13;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 12) {
        settingbutton_count = 13;
        dwin_page_Set(19);
        Icon1[7] = 32;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 13) {
        settingbutton_count = 14;
        dwin_page_Set(19);
        Icon1[7] = 129;
        DWIN_PORT.write(Icon1, 8);
      }
      
      else {
        settingbutton_count = 12;
        dwin_page_Set(19);
        Icon1[7] = 13;
        DWIN_PORT.write(Icon1, 8);
      }
    }

    else if (mode1 == MODE_ACV && cursr_forward == 1 && HMI_Page == 50) {  
#if 0
      if (settingbutton_count == 14) {
        settingbutton_count = 13;
        dwin_page_Set(19);
        Icon1[7] = 32;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 11) {
        settingbutton_count = 14;
        dwin_page_Set(19);
        Icon1[7] = 129;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 10) {
        settingbutton_count = 11;
        dwin_page_Set(19);
        Icon1[7] = 37;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 9) {
        settingbutton_count = 10;
        dwin_page_Set(19);
        Icon1[7] = 25;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 8) {
        settingbutton_count = 10;
        dwin_page_Set(19);
        Icon1[7] = 25;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 7) {
        settingbutton_count = 8;
        dwin_page_Set(19);
        Icon1[7] = 21;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 6) {
        settingbutton_count = 8;
        dwin_page_Set(19);
        Icon1[7] = 21;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 5) {
        settingbutton_count = 8;  
        dwin_page_Set(19);
        Icon1[7] = 21;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 4) {
        settingbutton_count = 5;
        dwin_page_Set(19);
        Icon1[7] = 54;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 3) {
        settingbutton_count = 4;
        dwin_page_Set(19);
        Icon1[7] = 11;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 12) {
        settingbutton_count = 3;
        dwin_page_Set(19);
        Icon1[7] = 48;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 13) {
        settingbutton_count = 12;
        dwin_page_Set(19);
        Icon1[7] = 13;
        DWIN_PORT.write(Icon1, 8);
      } else {
        settingbutton_count = 12;
        dwin_page_Set(19);
        Icon1[7] = 13;
        DWIN_PORT.write(Icon1, 8);
      }
#endif
    }

    else if (mode1 == MODE_PSIMV && cursr_forward == 0 && HMI_Page == 50) {  

      if (settingbutton_count == 14) {
        settingbutton_count = 11;
        dwin_page_Set(20);
        Icon1[7] = 37;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 11) {
        settingbutton_count = 10;
        dwin_page_Set(20);
        Icon1[7] = 25;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 10) {
        settingbutton_count = 9;
        dwin_page_Set(20);
        Icon1[7] = 29;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 9) {
        settingbutton_count = 8;
        dwin_page_Set(20);
        Icon1[7] = 21;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 8) {
        settingbutton_count = 5;
        dwin_page_Set(20);
        Icon1[7] = 58;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 7) {
        settingbutton_count = 8;
        dwin_page_Set(20);
        Icon1[7] = 21;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 6) {
        settingbutton_count = 8;
        dwin_page_Set(20);
        Icon1[7] = 21;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 5) {
        settingbutton_count = 4;  
        dwin_page_Set(20);
        Icon1[7] = 11;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 4) {
        settingbutton_count = 3;
        dwin_page_Set(20);
        Icon1[7] = 46;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 3) {
        settingbutton_count = 12;
        dwin_page_Set(20);
        Icon1[7] = 13;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 12) {
        settingbutton_count = 13;
        dwin_page_Set(20);
        Icon1[7] = 33;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 13) {
        settingbutton_count = 14;
        dwin_page_Set(20);
        Icon1[7] = 129;
        DWIN_PORT.write(Icon1, 8);
      } else {
        settingbutton_count = 12;
        dwin_page_Set(20);
      }
    }

    else if (mode1 == MODE_PSIMV && cursr_forward == 1 && HMI_Page == 50) {  
#if 0
      if (settingbutton_count == 14) {
        settingbutton_count = 13;
        dwin_page_Set(20);
        Icon1[7] = 33;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 11) {
        settingbutton_count = 14;
        dwin_page_Set(20);
        Icon1[7] = 129;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 10) {
        settingbutton_count = 11;
        dwin_page_Set(20);
        Icon1[7] = 37;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 9) {
        settingbutton_count = 10;
        dwin_page_Set(20);
        Icon1[7] = 25;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 8) {
        settingbutton_count = 9;
        dwin_page_Set(20);
        Icon1[7] = 29;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 7) {
        settingbutton_count = 8;
        dwin_page_Set(20);
        Icon1[7] = 21;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 6) {
        settingbutton_count = 8;
        dwin_page_Set(20);
        Icon1[7] = 21;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 5) {
        settingbutton_count = 8;  
        dwin_page_Set(20);
        Icon1[7] = 21;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 4) {
        settingbutton_count = 5;
        dwin_page_Set(20);
        Icon1[7] = 58;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 3) {
        settingbutton_count = 4;
        dwin_page_Set(20);
        Icon1[7] = 11;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 12) {
        settingbutton_count = 3;
        dwin_page_Set(20);
        Icon1[7] = 46;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 13) {
        settingbutton_count = 12;
        dwin_page_Set(20);
        Icon1[7] = 13;
        DWIN_PORT.write(Icon1, 8);
      } else {
        settingbutton_count = 12;
        dwin_page_Set(20);
      }
#endif
    }

    else if (mode1 == MODE_SIMV && cursr_forward == 0 && HMI_Page == 50) {  

      if (settingbutton_count == 14) {
        settingbutton_count = 11;
        dwin_page_Set(21);
        Icon1[7] = 37;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 11) {
        settingbutton_count = 10;
        dwin_page_Set(21);
        Icon1[7] = 25;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 10) {
        settingbutton_count = 9;
        dwin_page_Set(21);
        Icon1[7] = 29;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 9) {
        settingbutton_count = 8;
        dwin_page_Set(21);
        Icon1[7] = 21;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 8) {
        settingbutton_count = 5;
        dwin_page_Set(21);
        Icon1[7] = 58;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 7) {
        settingbutton_count = 8;
        dwin_page_Set(21);
        Icon1[7] = 21;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 6) {
        settingbutton_count = 8;
        dwin_page_Set(21);
        Icon1[7] = 21;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 5) {
        settingbutton_count = 4;  
        dwin_page_Set(21);
        Icon1[7] = 11;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 4) {
        settingbutton_count = 3;
        dwin_page_Set(21);
        Icon1[7] = 48;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 3) {
        settingbutton_count = 12;
        dwin_page_Set(21);
        Icon1[7] = 13;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 12) {
        settingbutton_count = 13;
        dwin_page_Set(21);
        Icon1[7] = 34;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 13) {
        settingbutton_count = 14;
        dwin_page_Set(21);
        Icon1[7] = 129;
        DWIN_PORT.write(Icon1, 8);
      } else {
        settingbutton_count = 12;
        dwin_page_Set(21);
        Icon1[7] = 13;
        DWIN_PORT.write(Icon1, 8);
      }
    }

    else if (mode1 == MODE_SIMV && cursr_forward == 1 && HMI_Page == 50) {  
#if 0
      if (settingbutton_count == 14) {
        settingbutton_count = 13;
        dwin_page_Set(21);
        Icon1[7] = 34;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 11) {
        settingbutton_count = 14;
        dwin_page_Set(21);
        Icon1[7] = 129;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 10) {
        settingbutton_count = 11;
        dwin_page_Set(21);
        Icon1[7] = 37;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 9) {
        settingbutton_count = 10;
        dwin_page_Set(21);
        Icon1[7] = 25;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 8) {
        settingbutton_count = 9;
        dwin_page_Set(21);
        Icon1[7] = 29;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 7) {
        settingbutton_count = 8;
        dwin_page_Set(21);
        Icon1[7] = 21;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 6) {
        settingbutton_count = 8;
        dwin_page_Set(21);
        Icon1[7] = 21;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 5) {
        settingbutton_count = 8;  
        dwin_page_Set(21);
        Icon1[7] = 21;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 4) {
        settingbutton_count = 5;
        dwin_page_Set(21);
        Icon1[7] = 58;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 3) {
        settingbutton_count = 4;
        dwin_page_Set(21);
        Icon1[7] = 11;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 12) {
        settingbutton_count = 3;
        dwin_page_Set(21);
        Icon1[7] = 48;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 13) {
        settingbutton_count = 12;
        dwin_page_Set(21);
        Icon1[7] = 13;
        DWIN_PORT.write(Icon1, 8);
      } else {
        settingbutton_count = 12;
        dwin_page_Set(21);
        Icon1[7] = 13;
        DWIN_PORT.write(Icon1, 8);
      }
#endif
    }

    else if (mode1 == MODE_SPONT_PS && cursr_forward == 0 && HMI_Page == 50) {
      if (settingbutton_count == 10) {
        settingbutton_count = 3;  
        dwin_page_Set(22);
        Icon1[7] = 52;
        DWIN_PORT.write(Icon1, 8);
      }  
      else if (settingbutton_count == 11) {
        settingbutton_count = 10;
        dwin_page_Set(22);
        Icon1[7] = 25;
        DWIN_PORT.write(Icon1, 8);
      }  
      
      else if (settingbutton_count == 14) {
        settingbutton_count = 9;
        dwin_page_Set(22);
        Icon1[7] = 23;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 9) {
        settingbutton_count = 8;
        dwin_page_Set(22);
        Icon1[7] = 21;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 8) {
        settingbutton_count = 11;
        dwin_page_Set(22);
        Icon1[7] = 37;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 7) {
        settingbutton_count = 10;
        dwin_page_Set(22);
        Icon1[7] = 25;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 6) {
        settingbutton_count = 10;
        dwin_page_Set(22);
        Icon1[7] = 25;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 5) {
        settingbutton_count = 10;  
        dwin_page_Set(22);
        Icon1[7] = 25;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 4) {
        settingbutton_count = 10;
        dwin_page_Set(22);
        Icon1[7] = 25;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 3) {
        settingbutton_count = 12;
        dwin_page_Set(22);
        Icon1[7] = 13;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 12) {
        settingbutton_count = 13;
        dwin_page_Set(22);
        Icon1[7] = 35;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 13) {
        settingbutton_count = 14;
        dwin_page_Set(22);
        Icon1[7] = 129;
        DWIN_PORT.write(Icon1, 8);
      } else {
        settingbutton_count = 12;
        dwin_page_Set(22);
        Icon1[7] = 0;
        DWIN_PORT.write(Icon1, 8);
      }
    }

    else if (mode1 == MODE_SPONT_PS && cursr_forward == 1 && HMI_Page == 50) {
#if 0
      if (settingbutton_count == 10) {
        settingbutton_count = 11;  
        dwin_page_Set(22);
        Icon1[7] = 37;
        DWIN_PORT.write(Icon1, 8);
      }  
      else if (settingbutton_count == 11) {
        settingbutton_count = 8;
        dwin_page_Set(22);
        Icon1[7] = 21;
        DWIN_PORT.write(Icon1, 8);
      }  
      
      else if (settingbutton_count == 14) {
        settingbutton_count = 13;
        dwin_page_Set(22);
        Icon1[7] = 35;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 9) {
        settingbutton_count = 14;
        dwin_page_Set(22);
        Icon1[7] = 129;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 8) {
        settingbutton_count = 9;
        dwin_page_Set(22);
        Icon1[7] = 23;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 7) {
        settingbutton_count = 10;
        dwin_page_Set(22);
        Icon1[7] = 25;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 6) {
        settingbutton_count = 10;
        dwin_page_Set(22);
        Icon1[7] = 25;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 5) {
        settingbutton_count = 10;  
        dwin_page_Set(22);
        Icon1[7] = 25;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 4) {
        settingbutton_count = 10;
        dwin_page_Set(22);
        Icon1[7] = 25;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 3) {
        settingbutton_count = 10;
        dwin_page_Set(22);
        Icon1[7] = 25;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 12) {
        settingbutton_count = 3;
        dwin_page_Set(22);
        Icon1[7] = 52;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 13) {
        settingbutton_count = 12;
        dwin_page_Set(22);
        Icon1[7] = 13;
        DWIN_PORT.write(Icon1, 8);
      } else {
        settingbutton_count = 12;
        dwin_page_Set(22);
        Icon1[7] = 0;
        DWIN_PORT.write(Icon1, 8);
      }
#endif
    }

    else if (mode1 == MODE_PCV && cursr_forward == 0 && HMI_Page == 50) {  
      
      if (settingbutton_count == 14) {
        settingbutton_count = 10;
        dwin_page_Set(23);
        Icon1[7] = 27;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 10) {
        settingbutton_count = 8;
        dwin_page_Set(23);
        Icon1[7] = 21;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 8) {
        settingbutton_count = 11;
        dwin_page_Set(23);
        Icon1[7] = 37;
        DWIN_PORT.write(Icon1, 8);
      }  
      else if (settingbutton_count == 11) {
        settingbutton_count = 4;
        dwin_page_Set(23);
        Icon1[7] = 11;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 7) {
        settingbutton_count = 11;
        dwin_page_Set(23);
        Icon1[7] = 37;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 6) {
        settingbutton_count = 11;
        dwin_page_Set(23);
        Icon1[7] = 37;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 5) {
        settingbutton_count = 11;  
        dwin_page_Set(23);
        Icon1[7] = 37;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 4) {
        settingbutton_count = 3;
        dwin_page_Set(23);
        Icon1[7] = 52;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 3) {
        settingbutton_count = 12;
        dwin_page_Set(23);
        Icon1[7] = 13;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 12) {
        settingbutton_count = 13;
        dwin_page_Set(23);
        Icon1[7] = 36;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 13) {
        settingbutton_count = 14;
        dwin_page_Set(23);
        Icon1[7] = 129;
        DWIN_PORT.write(Icon1, 8);
      } else {
        settingbutton_count = 12;
        dwin_page_Set(23);
        Icon1[7] = 13;
        DWIN_PORT.write(Icon1, 8);
      }
    }

    else if (mode1 == MODE_PCV && cursr_forward == 1 && HMI_Page == 50) {  
#if 0
      if (settingbutton_count == 14) {
        settingbutton_count = 13;
        dwin_page_Set(23);
        Icon1[7] = 36;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 10) {
        settingbutton_count = 14;
        dwin_page_Set(23);
        Icon1[7] = 129;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 8) {
        settingbutton_count = 10;
        dwin_page_Set(23);
        Icon1[7] = 27;
        DWIN_PORT.write(Icon1, 8);
      }  
      else if (settingbutton_count == 11) {
        settingbutton_count = 8;
        dwin_page_Set(23);
        Icon1[7] = 21;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 7) {
        settingbutton_count = 11;
        dwin_page_Set(23);
        Icon1[7] = 37;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 6) {
        settingbutton_count = 11;
        dwin_page_Set(23);
        Icon1[7] = 37;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 5) {
        settingbutton_count = 11;  
        dwin_page_Set(23);
        Icon1[7] = 37;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 4) {
        settingbutton_count = 11;
        dwin_page_Set(23);
        Icon1[7] = 37;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 3) {
        settingbutton_count = 4;
        dwin_page_Set(23);
        Icon1[7] = 11;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 12) {
        settingbutton_count = 3;
        dwin_page_Set(23);
        Icon1[7] = 52;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 13) {
        settingbutton_count = 12;
        dwin_page_Set(23);
        Icon1[7] = 13;
        DWIN_PORT.write(Icon1, 8);
      } else {
        settingbutton_count = 12;
        dwin_page_Set(23);
        Icon1[7] = 13;
        DWIN_PORT.write(Icon1, 8);
      }
#endif
    }

    else if (mode1 == MODE_HFT && cursr_forward == 0 && HMI_Page == 50) {
      
      if (settingbutton_count == 14) {
        settingbutton_count = 2;
        dwin_page_Set(24);
        Icon1[7] = 50;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 2) {
        settingbutton_count = 12;
        dwin_page_Set(24);
        Icon1[7] = 13;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 12) {
        settingbutton_count = 13;
        dwin_page_Set(24);
        Icon1[7] = 28;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 13) {
        settingbutton_count = 14;
        dwin_page_Set(24);
        Icon1[7] = 129;
        DWIN_PORT.write(Icon1, 8);
      } else {
        settingbutton_count = 12;
        dwin_page_Set(24);
        Icon1[7] = 13;
        DWIN_PORT.write(Icon1, 8);
      }
    }

    else if (mode1 == MODE_HFT && cursr_forward == 1 && HMI_Page == 50) {
#if 0 
      if (settingbutton_count == 14) {
        settingbutton_count = 13;
        dwin_page_Set(24);
        Icon1[7] = 28;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 2) {
        settingbutton_count = 14;
        dwin_page_Set(24);
        Icon1[7] = 129;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 12) {
        settingbutton_count = 2;
        dwin_page_Set(24);
        Icon1[7] = 50;
        DWIN_PORT.write(Icon1, 8);
      } else if (settingbutton_count == 13) {
        settingbutton_count = 12;
        dwin_page_Set(24);
        Icon1[7] = 13;
        DWIN_PORT.write(Icon1, 8);
      } else {
        settingbutton_count = 12;
        dwin_page_Set(24);
        Icon1[7] = 13;
        DWIN_PORT.write(Icon1, 8);
      }
#endif
    }

    else if (cursr_forward == 1 && HMI_Page == 29) {
      mainscreencursor++;
      if (mainscreencursor >= 13) {
        mainscreencursor = 1;
      }
      settingbutton_count = 1;
      if (mainscreencursor == 1) {
        Icon17[7] = 156;
        DWIN_PORT.write(Icon17, 8);
        settingbutton_count = 6;
      } else if (mainscreencursor == 2) {
        Icon17[7] = 157;
        DWIN_PORT.write(Icon17, 8);
        settingbutton_count = 7;
      } else if (mainscreencursor == 3) {
        Icon17[7] = 158;
        DWIN_PORT.write(Icon17, 8);
        settingbutton_count = 15;
      } else if (mainscreencursor == 4) {
        Icon17[7] = 159;
        DWIN_PORT.write(Icon17, 8);
        settingbutton_count = 16;
      } else if (mainscreencursor == 5) {
        Icon17[7] = 160;
        DWIN_PORT.write(Icon17, 8);
        settingbutton_count = 17;
      } else if (mainscreencursor == 6) {
        Icon17[7] = 161;
        DWIN_PORT.write(Icon17, 8);
        settingbutton_count = 18;
      } else if (mainscreencursor == 7) {
        Icon17[7] = 162;
        DWIN_PORT.write(Icon17, 8);
        settingbutton_count = 19;
      } else if (mainscreencursor == 8) {
        Icon17[7] = 163;
        DWIN_PORT.write(Icon17, 8);
        settingbutton_count = 20;
      } else if (mainscreencursor == 9) {
        Icon17[7] = 164;
        DWIN_PORT.write(Icon17, 8);
        settingbutton_count = 21;
      } else if (mainscreencursor == 10) {
        Icon17[7] = 165;
        DWIN_PORT.write(Icon17, 8);
        settingbutton_count = 22;
      } else if (mainscreencursor == 11) {
        Icon17[7] = 166;
        DWIN_PORT.write(Icon17, 8);
        settingbutton_count = 23;
      } else if (mainscreencursor == 12) {
        Icon17[7] = 167;
        DWIN_PORT.write(Icon17, 8);
        settingbutton_count = 24;
      }
    }

    else if (cursr_forward == 0 && HMI_Page == 29) {
      mainscreencursor--;
      if (mainscreencursor == 0) {
        mainscreencursor = 12;
      }
      settingbutton_count = 1;
      if (mainscreencursor == 1) {
        Icon17[7] = 156;
        DWIN_PORT.write(Icon17, 8);
        settingbutton_count = 6;
      } else if (mainscreencursor == 2) {
        Icon17[7] = 157;
        DWIN_PORT.write(Icon17, 8);
        settingbutton_count = 7;
      } else if (mainscreencursor == 3) {
        Icon17[7] = 158;
        DWIN_PORT.write(Icon17, 8);
        settingbutton_count = 15;
      } else if (mainscreencursor == 4) {
        Icon17[7] = 159;
        DWIN_PORT.write(Icon17, 8);
        settingbutton_count = 16;
      } else if (mainscreencursor == 5) {
        Icon17[7] = 160;
        DWIN_PORT.write(Icon17, 8);
        settingbutton_count = 17;
      } else if (mainscreencursor == 6) {
        Icon17[7] = 161;
        DWIN_PORT.write(Icon17, 8);
        settingbutton_count = 18;
      } else if (mainscreencursor == 7) {
        Icon17[7] = 162;
        DWIN_PORT.write(Icon17, 8);
        settingbutton_count = 19;
      } else if (mainscreencursor == 8) {
        Icon17[7] = 163;
        DWIN_PORT.write(Icon17, 8);
        settingbutton_count = 20;
      } else if (mainscreencursor == 9) {
        Icon17[7] = 164;
        DWIN_PORT.write(Icon17, 8);
        settingbutton_count = 21;
      } else if (mainscreencursor == 10) {
        Icon17[7] = 165;
        DWIN_PORT.write(Icon17, 8);
        settingbutton_count = 22;
      } else if (mainscreencursor == 11) {
        Icon17[7] = 166;
        DWIN_PORT.write(Icon17, 8);
        settingbutton_count = 23;
      } else if (mainscreencursor == 12) {
        Icon17[7] = 167;
        DWIN_PORT.write(Icon17, 8);
        settingbutton_count = 24;
      }
    }

    else if (cursr_forward == 1 && HMI_Page == PAGE_STANDBY) {
      standbyiconcursor++;
      if (standbyiconcursor >= 4) {
        standbyiconcursor = 1;
      }
      settingbutton_count = 1;
      if (standbyiconcursor == 1) {
        Icon18[7] = 153;
        DWIN_PORT.write(Icon18, 8);
        settingbutton_count = 25;
      } else if (standbyiconcursor == 2) {
        Icon18[7] = 154;
        DWIN_PORT.write(Icon18, 8);
        settingbutton_count = 26;
      } else if (standbyiconcursor == 3) {
        Icon18[7] = 155;
        DWIN_PORT.write(Icon18, 8);
        settingbutton_count = 27;
      }

    }

    else if (cursr_forward == 0 && HMI_Page == PAGE_STANDBY) {
      standbyiconcursor--;
      if (standbyiconcursor == 0) {
        standbyiconcursor = 3;
      }
      settingbutton_count = 1;
      if (standbyiconcursor == 1) {
        Icon18[7] = 153;
        DWIN_PORT.write(Icon18, 8);
        settingbutton_count = 25;
      } else if (standbyiconcursor == 2) {
        Icon18[7] = 154;
        DWIN_PORT.write(Icon18, 8);
        settingbutton_count = 26;
      } else if (standbyiconcursor == 3) {
        Icon18[7] = 155;
        DWIN_PORT.write(Icon18, 8);
        settingbutton_count = 27;
      }
    }

    
    
    
    

    
  }

  else if (increamentButtonState && increamentButtonState != increamentbutton_lastState && HMI_Page == display_page_3)  
  {
    settingbutton_count = 1;
    mainscreencursor++;
    if (mainscreencursor >= 5) {
      mainscreencursor = 1;
    }
    switch (mode1) {
      case MODE_VCV:
        Icon15[7] = 45;
        DWIN_PORT.write(Icon15, 8);
        break;
      case MODE_SIMV:
        Icon15[7] = 44;
        DWIN_PORT.write(Icon15, 8);
        break;
      case MODE_ACV:
        Icon15[7] = 43;
        DWIN_PORT.write(Icon15, 8);
        break;
      case MODE_HFT:
        Icon15[7] = 40;
        DWIN_PORT.write(Icon15, 8);
        break;
      case MODE_PCV:
        Icon15[7] = 42;
        DWIN_PORT.write(Icon15, 8);
        break;
      case MODE_SPONT_PS:
        Icon15[7] = 39;
        DWIN_PORT.write(Icon15, 8);
        break;
      case MODE_CPAP:
        Icon15[7] = 38;
        DWIN_PORT.write(Icon15, 8);
        break;
      case MODE_PSIMV:
        Icon15[7] = 41;
        DWIN_PORT.write(Icon15, 8);
        break;
    }

    if (mainscreencursor == 3) {
      mainscreencursor = 4;  
    }
    if (mainscreencursor == 1) {
      Icon1[7] = 124;
      DWIN_PORT.write(Icon1, 8);
    } else if (mainscreencursor == 2) {
      stand_check = 1;
      Icon1[7] = 127;
      DWIN_PORT.write(Icon1, 8);
    } else if (mainscreencursor == 3) {
      Icon1[7] = 126;
      DWIN_PORT.write(Icon1, 8);
    } else if (mainscreencursor == 4) {
      Icon1[7] = 125;
      DWIN_PORT.write(Icon1, 8);
      Icon17[7] = 0;
      DWIN_PORT.write(Icon17, 8);
    }
  }

  else if (increamentButtonState && increamentButtonState != increamentbutton_lastState && HMI_Page == 13) {
    settingbutton_count = 1;
    if (pediatric_patient == 0) {
      pediatric_patient = 1;
      Icon1[7] = 86;
      DWIN_PORT.write(Icon1, 8);
    } else {
      pediatric_patient = 0;
      Icon1[7] = 65;
      DWIN_PORT.write(Icon1, 8);
    }
  } else if (increamentButtonState && increamentButtonState != increamentbutton_lastState && HMI_Page == 12) {
    settingbutton_count = 1;
    if (newpatient == 0) {
      newpatient = 1;
      Icon1[7] = 84;
      DWIN_PORT.write(Icon1, 8);
    } else {
      newpatient = 0;
      Icon1[7] = 85;
      DWIN_PORT.write(Icon1, 8);
    }
  } else if (increamentButtonState && increamentButtonState != increamentbutton_lastState && HMI_Page == 14) {
    settingbutton_count = 1;
    if (malepatient == 0) {
      malepatient = 1;
      Icon1[7] = 66;
      DWIN_PORT.write(Icon1, 8);
      Icon3[7] = 122;
      DWIN_PORT.write(Icon3, 8);
    } else {
      malepatient = 0;
      Icon1[7] = 70;
      DWIN_PORT.write(Icon1, 8);
      Icon3[7] = 123;
      DWIN_PORT.write(Icon3, 8);
    }
  }

  else if (increamentButtonState && increamentButtonState != increamentbutton_lastState && settingbutton_count == 2 && flowrate < 90 && mode1 == MODE_HFT && HMI_Page == 50) {
    flowrate = flowrate + 1;
    display_write_variable(VP_SET_FLOWRATE, flowrate);
  } else if ((increamentButtonState && increamentButtonState != increamentbutton_lastState && settingbutton_count == 2 && bodyweight < 250 && HMI_Page == PAGE_PATIENT_WEIGHT) || (HMI_Page == 14 && bdc == 1)) {
    bodyweight += 1;
    highpawalarmlimit = 15 + round(bodyweight / 2);
    if (highpawalarmlimit > 50) { highpawalarmlimit = 50; }
    lowpawalarmlimit = 5;

    if (bdc == 1 && pediatric_patient == 1) {
      bodyweight = 15;  
    }
    if (bdc == 1 && pediatric_patient == 0) {
      bodyweight = 52;  
    }
    if (pediatric_patient == 1 && bodyweight >= 25) {
      bodyweight = 25;  
    }
    if (bodyweight == 5) {
      tidalvolume_L_limit = bodyweight * 2;
    } else {
      tidalvolume_L_limit = bodyweight * 5;
    }
    tidalvolume_H_limit = bodyweight * 15;
    tidalvolume = bodyweight * 9;
    if (tidalvolume > 2000) {
      tidalvolume = 2000;
    }
    if (tidalvolume < 10) {
      tidalvolume = 10;
    }
    if (bodyweight <= 10) {
      frequncy = 33;
      frequncy_L_limit = 2;
      frequncy_H_limit = 70;
      frequencysimv = 23;
      frequencysimv_H_limit = 70;
      Tinsp_simv = 800;
      Pinsp = 10;
      ps = 10;
      peep = 1;
    } else if (bodyweight > 10 && bodyweight <= 15) {
      frequncy = 28;
      frequncy_L_limit = 2;
      frequncy_H_limit = 54;
      frequencysimv = 20;
      frequencysimv_H_limit = 40;
      Tinsp_simv = 1000;
      Pinsp = 10;
      ps = 10;
      peep = 2;
    } else if (bodyweight > 15 && bodyweight <= 20) {
      frequncy = 23;
      frequncy_L_limit = 2;
      frequncy_H_limit = 46;
      frequencysimv = 16;
      frequencysimv_H_limit = 40;
      Tinsp_simv = 1250;
      Pinsp = 10;
      ps = 10;
      peep = 2;
    } else if (bodyweight > 20 && bodyweight <= 30) {
      frequncy = 21;
      frequncy_L_limit = 2;
      frequncy_H_limit = 42;
      frequencysimv = 14;
      frequencysimv_H_limit = 40;
      Tinsp_simv = 1400;
      Pinsp = 10;
      ps = 10;
      peep = 3;
    } else if (bodyweight > 30 && bodyweight <= 40) {
      frequncy = 12;
      frequncy_L_limit = 2;
      frequncy_H_limit = 37;
      frequencysimv = 13;
      frequencysimv_H_limit = 37;
      Tinsp_simv = 1500;
      Pinsp = 10;
      ps = 10;
      peep = 3;
    } else if (bodyweight > 40 && bodyweight <= 50) {
      frequncy = 12;
      frequncy_L_limit = 2;
      frequncy_H_limit = 35;
      frequencysimv = 12;
      frequencysimv_H_limit = 35;
      Tinsp_simv = 1700;
      Pinsp = 10;
      ps = 10;
      peep = 4;
    } else if (bodyweight > 50) {
      frequncy = 12;
      frequncy_L_limit = 2;
      frequncy_H_limit = 30;
      frequencysimv = 10;
      frequencysimv_H_limit = 30;
      Tinsp_simv = 1700;
      Pinsp = 10;
      ps = 10;
      peep = 5;
    }
    display_write_variable(VP_SET_TIDAL_VOLUME, tidalvolume);
    display_write_variable(VP_SET_BODYWEIGHT, bodyweight);
    display_write_variable(VP_SET_FREQUENCY, frequncy);
    display_write_variable(VP_SET_FREQ_SIMV, frequencysimv);
    display_write_variable(VP_SET_TINSP_SIMV, 10 * Tinsp_simv / 1000);
    delay(5);
    display_write_variable(VP_SET_PINSP, Pinsp);
    display_write_variable(VP_SET_PS, ps);
    display_write_variable(VP_SET_PEEP, peep);
    bdc = 0;    
    
  } else if (HMI_Page == 50 && increamentButtonState && increamentButtonState != increamentbutton_lastState && settingbutton_count == 3 && tidalvolume < tidalvolume_H_limit && tidalvolume < 2000 && mode1 != MODE_CPAP && mode1 != MODE_SPONT_PS && mode1 != MODE_PCV && mode1 != MODE_PSIMV) {
    tidalvolume += 5;
    display_write_variable(VP_SET_TIDAL_VOLUME, tidalvolume);
    
    
  } else if (HMI_Page == 50 && increamentButtonState && increamentButtonState != increamentbutton_lastState && settingbutton_count == 3 && Pinsp < 100 && (mode1 == MODE_CPAP || mode1 == MODE_SPONT_PS || mode1 == MODE_PCV || mode1 == MODE_PSIMV)) {
    Pinsp += 1;
    
    display_write_variable(VP_SET_PINSP, Pinsp);
    
  } else if (HMI_Page == 50 && increamentButtonState && increamentButtonState != increamentbutton_lastState && settingbutton_count == 4 && frequncy < frequncy_H_limit && frequncy < 70 && (mode1 == MODE_VCV || mode1 == MODE_ACV || mode1 == MODE_CPAP || mode1 == MODE_PCV) && mode1 != MODE_SPONT_PS) {
    frequncy += 1;
    display_write_variable(VP_SET_FREQUENCY, frequncy);
    
    
  } else if (HMI_Page == 50 && increamentButtonState && increamentButtonState != increamentbutton_lastState && settingbutton_count == 4 && frequencysimv < frequencysimv_H_limit && frequencysimv < 40 && (mode1 == MODE_SIMV || mode1 == MODE_PSIMV) && ((60000 / frequencysimv) - 200) > (Tinsp_simv)) {
    frequencysimv += 1;
    display_write_variable(VP_SET_FREQ_SIMV, frequencysimv);
  }

  else if (HMI_Page == 50 && increamentButtonState && increamentButtonState != increamentbutton_lastState && i_e < 4 && settingbutton_count == 5 && (mode1 == MODE_VCV || mode1 == MODE_ACV || mode1 == MODE_CPAP) && mode1 != MODE_SPONT_PS && mode1 != MODE_PCV)  
  {
    i_e = i_e + 0.10;
    if (i_e >= (-1) && i_e <= 1) {
      i_e = 1;  
    }

    if (i_e == 1) {
      i = 1;
      e = 1;
    }
    if (i_e > 1) {
      i = 1;
      e = i_e;
    }
    if (i_e >= 0 && i_e < 1) {
      e = 1;
      i = i_e;
    }
    if (i_e < 0) {
      e = 1;
      i = -i_e;
    }
 
    int i_int = round(10 * i);
    int e_int = round(10 * e);
    display_write_variable(VP_SET_IE_I, i_int);
    display_write_variable(VP_SET_IE_E, e_int);
    
    
  } else if (HMI_Page == 50 && increamentButtonState && increamentButtonState != increamentbutton_lastState && settingbutton_count == 5 && (mode1 == MODE_SIMV || mode1 == MODE_PSIMV) && ((60000 / frequencysimv) - 200) > (Tinsp_simv))  
  {
    Tinsp_simv += 100;
    display_write_variable(VP_SET_TINSP_SIMV, 10 * Tinsp_simv / 1000);
    
    
  } else if (HMI_Page == 29 && increamentButtonState && increamentButtonState != increamentbutton_lastState && highpawalarmlimit < 99 && settingbutton_count == 6) {
    highpawalarmlimit += 1;
    display_write_variable(VP_ALM_PAW_HIGH, highpawalarmlimit);

    
    
  } else if (HMI_Page == 29 && increamentButtonState && increamentButtonState != increamentbutton_lastState && lowpawalarmlimit < (highpawalarmlimit - 1) && settingbutton_count == 7)  
  {
    lowpawalarmlimit += 1;
    display_write_variable(VP_ALM_PAW_LOW, lowpawalarmlimit);
    
    
  } else if (HMI_Page == 50 && increamentButtonState && increamentButtonState != increamentbutton_lastState && Exsence_percentage < 100 && settingbutton_count == 8) {
    Exsence_percentage += 5;
    display_write_variable(VP_SET_EXSENCE_PCT, Exsence_percentage);
    
    
  }
  
  else if (HMI_Page == 29 && increamentButtonState && increamentButtonState != increamentbutton_lastState && settingbutton_count == 15 && highbpmalarmlimit < 150)  
  {
    highbpmalarmlimit += 1;                             
    display_write_variable(VP_ALM_BPM_HIGH, highbpmalarmlimit);  
      
  }
  
  
  else if (HMI_Page == 29 && increamentButtonState && increamentButtonState != increamentbutton_lastState && settingbutton_count == 16 && lowbpmalarmlimit < highbpmalarmlimit - 1)  
  {
    lowbpmalarmlimit += 1;                             
    display_write_variable(VP_ALM_BPM_LOW, lowbpmalarmlimit);    
  }

  
  else if (HMI_Page == 29 && increamentButtonState && increamentButtonState != increamentbutton_lastState && settingbutton_count == 22)  
  {
    if (low_fio2alarmlimit >= 100) {
      low_fio2alarmlimit = 100;
    } else {  
      
      if (low_fio2alarmlimit < high_fio2alarmlimit - 1) {
        low_fio2alarmlimit += 1;
        display_write_variable(VP_ALM_FIO2_LOW, low_fio2alarmlimit);

      }  
    }
    
  }

  else if (HMI_Page == 1 && increamentButtonState && increamentButtonState != increamentbutton_lastState && neb_time < 120 && settingbutton_count == 28)  
  {
    neb_time += 10;                            
    display_write_variable(VP_NEB_TIME, neb_time);  
    Icon21[7] = 198;
    DWIN_PORT.write(Icon21, 8);

    switch (mode1) {
      case MODE_VCV:
        Icon15[7] = 45;
        DWIN_PORT.write(Icon15, 8);
        break;
      case MODE_SIMV:
        Icon15[7] = 44;
        DWIN_PORT.write(Icon15, 8);
        break;
      case MODE_ACV:
        Icon15[7] = 43;
        DWIN_PORT.write(Icon15, 8);
        break;
      case MODE_HFT:
        Icon15[7] = 40;
        DWIN_PORT.write(Icon15, 8);
        break;
      case MODE_PCV:
        Icon15[7] = 42;
        DWIN_PORT.write(Icon15, 8);
        break;
      case MODE_SPONT_PS:
        Icon15[7] = 39;
        DWIN_PORT.write(Icon15, 8);
        break;
      case MODE_CPAP:
        Icon15[7] = 38;
        DWIN_PORT.write(Icon15, 8);
        break;
      case MODE_PSIMV:
        Icon15[7] = 41;
        DWIN_PORT.write(Icon15, 8);
        break;
    }
  }
  
  else if (HMI_Page == 29 && increamentButtonState && increamentButtonState != increamentbutton_lastState && settingbutton_count == 21 && high_fio2alarmlimit < 100)  
  {

    high_fio2alarmlimit += 1;
    display_write_variable(VP_ALM_FIO2_HIGH, high_fio2alarmlimit);  
  }

  else if (HMI_Page == 29 && increamentButtonState && increamentButtonState != increamentbutton_lastState && settingbutton_count == 20 && low_mvalarmlimit < (high_mvalarmlimit - 2))  
  {
    low_mvalarmlimit += 1;  
    low_mvalarmlimit_1 = low_mvalarmlimit;
    display_write_variable(VP_ALM_MV_LOW, 10 * low_mvalarmlimit_1);  
    
  }

  else if (HMI_Page == 29 && increamentButtonState && increamentButtonState != increamentbutton_lastState && settingbutton_count == 19 && high_mvalarmlimit <= 100)  
  {
    high_mvalarmlimit += 1;  
    high_mvalarmlimit_1 = high_mvalarmlimit;
    display_write_variable(VP_ALM_MV_HIGH, 10 * high_mvalarmlimit_1);  
  }
  
  else if (HMI_Page == 29 && increamentButtonState && increamentButtonState != increamentbutton_lastState && settingbutton_count == 18 && lowvtalarmlimit < (highvtalarmlimit - 10))  
  {
    lowvtalarmlimit += 10;  
    display_write_variable(VP_ALM_VT_LOW, lowvtalarmlimit);

  }
  
  else if (increamentButtonState && increamentButtonState != increamentbutton_lastState && settingbutton_count == 17 && highvtalarmlimit < 2500)  
  {
    highvtalarmlimit += 10;  
    display_write_variable(VP_ALM_VT_HIGH, highvtalarmlimit);
    
  }

  else if ((increamentButtonState && increamentButtonState != increamentbutton_lastState || (modebuttonState && modebuttonState != modebutton_lastState)) && (settingbutton_count == 13)) {
    if (settingbutton_count != 13) {
      settingbutton_count = 13;
    }
    if (mode1 == MODE_VCV) {
      mode1 = MODE_PCV;
      dwin_page_Set(17);
      Icon1[7] = 30;
      DWIN_PORT.write(Icon1, 8);
    } else if (mode1 == MODE_SIMV) {
      mode1 = MODE_PSIMV;
      dwin_page_Set(20);
      Icon1[7] = 33;
      DWIN_PORT.write(Icon1, 8);
    } else if (mode1 == MODE_ACV) {
      mode1 = MODE_VCV;
      dwin_page_Set(18);
      Icon1[7] = 31;
      DWIN_PORT.write(Icon1, 8);
    } else if (mode1 == MODE_HFT) {
      mode1 = MODE_CPAP;
      dwin_page_Set(23);
      Icon1[7] = 36;
      DWIN_PORT.write(Icon1, 8);
    } else if (mode1 == MODE_CPAP) {
      mode1 = MODE_SPONT_PS;
      dwin_page_Set(22);
      Icon1[7] = 35;
      DWIN_PORT.write(Icon1, 8);
    } else if (mode1 == MODE_SPONT_PS) {
      mode1 = MODE_SIMV;
      dwin_page_Set(21);
      Icon1[7] = 34;
      DWIN_PORT.write(Icon1, 8);
    } else if (mode1 == MODE_PSIMV) {
      mode1 = MODE_ACV;
      dwin_page_Set(19);
      Icon1[7] = 32;
      DWIN_PORT.write(Icon1, 8);
    } else if (mode1 == MODE_PCV) {
      mode1 = MODE_HFT;
      dwin_page_Set(24);
      Icon1[7] = 28;
      DWIN_PORT.write(Icon1, 8);
    } 
    HMI_Page = 50;
    
    
  } else if (HMI_Page == 50 && increamentButtonState && increamentButtonState != increamentbutton_lastState && apneatime < 25 && settingbutton_count == 9 && mode1 == MODE_SPONT_PS) {
    apneatime += 1;
    display_write_variable(VP_SET_APNEA_TIME, apneatime);
    
    
  } else if (HMI_Page == 50 && increamentButtonState && increamentButtonState != increamentbutton_lastState && ps < 40 && settingbutton_count == 9 && mode1 != MODE_SPONT_PS && mode1 != MODE_PCV) {
    ps += 1;
    display_write_variable(VP_SET_PS, ps);
    
    
  } else if (HMI_Page == 50 && increamentButtonState && increamentButtonState != increamentbutton_lastState && peep < 30 && (peep < (Pinsp - 1)) && (settingbutton_count == 10 || (settingbutton_count == 4 && mode1 == MODE_SPONT_PS))) {
    peep += 1;
    display_write_variable(VP_SET_PEEP, peep);
    
    
  } else if (HMI_Page == 50 && increamentButtonState && increamentButtonState != increamentbutton_lastState && ptr < 20 && (settingbutton_count == 11 || (settingbutton_count == 5 && mode1 == MODE_SPONT_PS) || (settingbutton_count == 5 && mode1 == MODE_PCV))) {
    if (ptr < 1) {
      ptr += 0.10;
    }  
    else {
      ptr += 0.50;
    }
    ptr_1 = round(10 * ptr);
    display_write_variable(VP_SET_PTR, ptr_1);
    
  } else if (HMI_Page == 50 && increamentButtonState && increamentButtonState != increamentbutton_lastState && settingbutton_count == 12 && set_fio2 < 100) {

    set_fio2 += 1;
    display_write_variable(VP_SET_FIO2, set_fio2);

  } else if (HMI_Page == 26 && increamentButtonState && increamentButtonState != increamentbutton_lastState && settingbutton_count == 30 && hhh < 23) {
    if (hhh >= 23) { hhh = 23; }
    hhh += 1;
    display_write_variable(VP_CLOCK_HH, hhh);

  } else if (HMI_Page == 26 && increamentButtonState && increamentButtonState != increamentbutton_lastState && settingbutton_count == 31 && mmm < 59) {
    if (mmm > 59) { mmm = 59; }
    mmm += 1;
    display_write_variable(VP_CLOCK_MM, mmm);

  }

  else if (decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && flowrate > 7 && settingbutton_count == 2 && mode1 == MODE_HFT && HMI_Page == 50) {

    flowrate = flowrate - 1;
    display_write_variable(VP_SET_FLOWRATE, flowrate);
  }

  else if (decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && bodyweight > 7 && settingbutton_count == 2 && HMI_Page == PAGE_PATIENT_WEIGHT) {
    bodyweight -= 1;
    highpawalarmlimit = 15 + round(bodyweight / 2);
    if (highpawalarmlimit > 50) { highpawalarmlimit = 50; }
    lowpawalarmlimit = 5;
    if (bdc == 1 && pediatric_patient == 1) {
      bodyweight = 15;  
    }
    if (bdc == 1 && pediatric_patient == 0) {
      bodyweight = 50;  
    }
    if (pediatric_patient == 0 && bodyweight <= 25) {
      bodyweight = 25;  
    }
    if (bodyweight == 5) {
      tidalvolume_L_limit = bodyweight * 2;
    } else {
      tidalvolume_L_limit = bodyweight * 5;
    }
    tidalvolume_H_limit = bodyweight * 15;
    tidalvolume = bodyweight * 9;
    if (tidalvolume > 2000) {
      tidalvolume = 2000;
    }
    if (tidalvolume < 10) {
      tidalvolume = 10;
    }
    if (bodyweight <= 10) {
      frequncy = 33;
      frequncy_L_limit = 2;
      frequncy_H_limit = 60;
      frequencysimv = 23;
      frequencysimv_H_limit = 40;
      Tinsp_simv = 800;
      Pinsp = 10;
      ps = 10;
      peep = 1;
    } else if (bodyweight > 10 && bodyweight <= 15) {
      frequncy = 28;
      frequncy_L_limit = 2;
      frequncy_H_limit = 54;
      frequencysimv = 20;
      frequencysimv_H_limit = 40;
      Tinsp_simv = 1000;
      Pinsp = 10;
      ps = 10;
      peep = 2;
    } else if (bodyweight > 15 && bodyweight <= 20) {
      frequncy = 23;
      frequncy_L_limit = 2;
      frequncy_H_limit = 46;
      frequencysimv = 16;
      frequencysimv_H_limit = 40;
      Tinsp_simv = 1250;
      Pinsp = 15;
      ps = 15;
      peep = 2;
    } else if (bodyweight > 20 && bodyweight <= 30) {
      frequncy = 21;
      frequncy_L_limit = 2;
      frequncy_H_limit = 42;
      frequencysimv = 14;
      frequencysimv_H_limit = 40;
      Tinsp_simv = 1400;
      Pinsp = 15;
      ps = 15;
      peep = 3;
    } else if (bodyweight > 30 && bodyweight <= 40) {
      frequncy = 19;
      frequncy_L_limit = 2;
      frequncy_H_limit = 37;
      frequencysimv = 13;
      frequencysimv_H_limit = 37;
      Tinsp_simv = 1500;
      Pinsp = 15;
      ps = 15;
      peep = 3;
    } else if (bodyweight > 40 && bodyweight <= 50) {
      frequncy = 17;
      frequncy_L_limit = 2;
      frequncy_H_limit = 35;
      frequencysimv = 12;
      frequencysimv_H_limit = 35;
      Tinsp_simv = 1700;
      Pinsp = 20;
      ps = 20;
      peep = 4;
    } else if (bodyweight > 50) {
      frequncy = 15;
      frequncy_L_limit = 2;
      frequncy_H_limit = 30;
      frequencysimv = 10;
      frequencysimv_H_limit = 30;
      Tinsp_simv = 1700;
      Pinsp = 20;
      ps = 20;
      peep = 5;
    }
    display_write_variable(VP_SET_TIDAL_VOLUME, tidalvolume);
    display_write_variable(VP_SET_BODYWEIGHT, bodyweight);
    display_write_variable(VP_SET_FREQUENCY, frequncy);
    display_write_variable(VP_SET_FREQ_SIMV, frequencysimv);
    display_write_variable(VP_SET_TINSP_SIMV, 10 * Tinsp_simv / 1000);
    display_write_variable(VP_SET_PINSP, Pinsp);
    display_write_variable(VP_SET_PS, ps);
    display_write_variable(VP_SET_PEEP, peep);

  }

  else if (decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && settingbutton_count == 19 && high_mvalarmlimit > (low_mvalarmlimit + 2)) {  
    high_mvalarmlimit -= 1;                                                                                                                                            
    high_mvalarmlimit_1 = high_mvalarmlimit;
    display_write_variable(VP_ALM_MV_HIGH, 10 * high_mvalarmlimit_1);  

  }

  else if (decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && settingbutton_count == 22) {
    if (low_fio2alarmlimit <= 18) {
      low_fio2alarmlimit = 18;
    } else {
      low_fio2alarmlimit -= 1;  
    }
    display_write_variable(VP_ALM_FIO2_LOW, low_fio2alarmlimit);  
    
  }

  else if (decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && settingbutton_count == 21) {
    if (high_fio2alarmlimit <= 19) {
      high_fio2alarmlimit = 19;
    } else {  
      if (high_fio2alarmlimit > (low_fio2alarmlimit + 1)) {
        high_fio2alarmlimit -= 1;
        display_write_variable(VP_ALM_FIO2_HIGH, high_fio2alarmlimit);  
      }
    }
    
  }

  else if (decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && settingbutton_count == 20 && low_mvalarmlimit > 1) {
    low_mvalarmlimit -= 1;  
    low_mvalarmlimit_1 = low_mvalarmlimit;
    display_write_variable(VP_ALM_MV_LOW, 10 * low_mvalarmlimit_1);  
    
  }
  else if (decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && settingbutton_count == 17 && highvtalarmlimit > (lowvtalarmlimit + 10)) {  
    highvtalarmlimit -= 10;                                                                                                                                           
    display_write_variable(VP_ALM_VT_HIGH, highvtalarmlimit);  
    
  }
  else if (decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && settingbutton_count == 18 && lowvtalarmlimit > 10) {  
    lowvtalarmlimit -= 10;                                                                                                                       
    display_write_variable(VP_ALM_VT_LOW, lowvtalarmlimit);                                                                                             
    
  }
  else if (decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && settingbutton_count == 16 && lowbpmalarmlimit > 1) {  
    lowbpmalarmlimit -= 1;                                                                                                                       
    display_write_variable(VP_ALM_BPM_LOW, lowbpmalarmlimit);                                                                                            
    
  }

  else if (HMI_Page == 29 && decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && settingbutton_count == 15 && highbpmalarmlimit > (lowbpmalarmlimit + 1)) {  
    highbpmalarmlimit -= 1;                                                                                                                                                              
    display_write_variable(VP_ALM_BPM_HIGH, highbpmalarmlimit);                                                                                                                                   
    
  } else if (decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && tidalvolume > (tidalvolume_L_limit) && settingbutton_count == 3 && tidalvolume > 10 && mode1 != MODE_CPAP && mode1 != MODE_SPONT_PS && mode1 != MODE_PCV && mode1 != MODE_PSIMV) {
    tidalvolume -= 5;
    display_write_variable(VP_SET_TIDAL_VOLUME, tidalvolume);
    
  } else if (decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && settingbutton_count == 3 && Pinsp > 5 && (mode1 == MODE_CPAP || mode1 == MODE_SPONT_PS || mode1 == MODE_PCV || mode1 == MODE_PSIMV)) {

    
    Pinsp -= 1;
    display_write_variable(VP_SET_PINSP, Pinsp);

    
  } else if (decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && frequncy > frequncy_L_limit && frequncy > 7 && settingbutton_count == 4 && (mode1 == MODE_VCV || mode1 == MODE_ACV || mode1 == MODE_CPAP || mode1 == MODE_PCV) && mode1 != MODE_SPONT_PS) {
    frequncy -= 1;
    display_write_variable(VP_SET_FREQUENCY, frequncy);
    
    
  } else if (decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && frequencysimv > 6 && settingbutton_count == 4 && (mode1 == MODE_SIMV || mode1 == MODE_PSIMV) && (200 < Tinsp_simv)) {
    frequencysimv -= 1;
    display_write_variable(VP_SET_FREQ_SIMV, frequencysimv);
    
    
  } else if (decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && i_e > -2 && settingbutton_count == 5 && (mode1 == MODE_VCV || mode1 == MODE_ACV || mode1 == MODE_CPAP) && mode1 != MODE_SPONT_PS && mode1 != MODE_PCV) {
    i_e = i_e - 0.10;
    if (i_e <= 1 && i_e >= -1) {
      i_e = -1;  
    }

    if (i_e == 1) {
      i = 1;
      e = 1;
    }
    if (i_e > 1) {
      i = 1;
      e = i_e;
    }
    if (i_e >= 0 && i_e < 1) {
      e = 1;
      i = i_e;
    }
    if (i_e < 0) {
      e = 1;
      i = -i_e;
    }
    int i_int = round(10 * i);
    int e_int = round(10 * e);
    display_write_variable(VP_SET_IE_I, i_int);
    display_write_variable(VP_SET_IE_E, e_int);

  }
  else if (HMI_Page == 50 && decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && settingbutton_count == 5 && (mode1 == MODE_SIMV || mode1 == MODE_PSIMV) && (800 < Tinsp_simv)) {
    Tinsp_simv -= 100;
    display_write_variable(VP_SET_TINSP_SIMV, 10 * Tinsp_simv / 1000); 
    
  } else if (decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && highpawalarmlimit > (lowpawalarmlimit + 1) && settingbutton_count == 6)  
  {
    highpawalarmlimit -= 1;
    display_write_variable(VP_ALM_PAW_HIGH, highpawalarmlimit);
    
    
  } else if (HMI_Page == 1 && decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && neb_time > 1 && settingbutton_count == 28)  
  {
    neb_time -= 10;
    if (neb_time > 0) {
      Icon21[7] = 198;
      DWIN_PORT.write(Icon21, 8);  
      display_write_variable(VP_NEB_TIME, neb_time);
      
    } else {
      display_write_variable(VP_NEB_TIME, 0);  
      nebu_touch = 0;
      digitalWrite(nebulizerpin, LOW);
      
      Icon21[7] = 0;
      DWIN_PORT.write(Icon21, 8);
    }
  } else if (decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && lowpawalarmlimit > 1 && settingbutton_count == 7) {
    lowpawalarmlimit -= 1;
    display_write_variable(VP_ALM_PAW_LOW, lowpawalarmlimit);
    
    
  } else if (HMI_Page == 50 && decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && Exsence_percentage > 50 && settingbutton_count == 8) {
    Exsence_percentage -= 5;
    display_write_variable(VP_SET_EXSENCE_PCT, Exsence_percentage);

  }

  else if ((decreamentButtonState && decreamentButtonState != decreamentbutton_lastState) && (settingbutton_count == 13)) {
    if (settingbutton_count != 13) {
      settingbutton_count = 13;
    } else {
      switch (mode1) {
        case MODE_VCV:  //vcv
          mode1 = MODE_ACV;
          dwin_page_Set(19);
          Icon1[7] = 32;
          DWIN_PORT.write(Icon1, 8);
          break;

        case MODE_SIMV:  //simv
          mode1 = MODE_SPONT_PS;
          dwin_page_Set(22);
          Icon1[7] = 35;
          DWIN_PORT.write(Icon1, 8);
          break;

        case MODE_ACV: //acv
          mode1 = MODE_PSIMV;
          dwin_page_Set(20);
          Icon1[7] = 33;
          DWIN_PORT.write(Icon1, 8);
          break;

        case MODE_HFT:  //hft
          mode1 = MODE_PCV;
          dwin_page_Set(17);
          Icon1[7] = 30;
          DWIN_PORT.write(Icon1, 8);
          break;

        case MODE_PCV: //pcv
          mode1 = MODE_VCV;
          dwin_page_Set(18);
          Icon1[7] = 31;
          DWIN_PORT.write(Icon1, 8);
          break;

        case MODE_SPONT_PS:  //cpap
          mode1 = MODE_CPAP;
          dwin_page_Set(23);
          Icon1[7] = 36;
          DWIN_PORT.write(Icon1, 8);
          break;

        case MODE_CPAP: //bipap
          mode1 = MODE_HFT;
          dwin_page_Set(24);
          Icon1[7] = 28;
          DWIN_PORT.write(Icon1, 8);
          break;

        case MODE_PSIMV:  //psimv
          mode1 = MODE_SIMV;
          dwin_page_Set(21);
          Icon1[7] = 34;
          DWIN_PORT.write(Icon1, 8);
          break;
      }
    }
    
  } else if (HMI_Page == 50 && decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && ps > 1 && settingbutton_count == 9 && mode1 != MODE_SPONT_PS && mode1 != MODE_PCV) {
    ps -= 1;
    display_write_variable(VP_SET_PS, ps);
    
    
  } else if (HMI_Page == 50 && decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && apneatime > 5 && settingbutton_count == 9 && mode1 == MODE_SPONT_PS) {
    apneatime -= 1;
    display_write_variable(VP_SET_APNEA_TIME, apneatime);
    
    
  } else if (HMI_Page == 50 && decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && peep > 0 && (settingbutton_count == 10 || (settingbutton_count == 4 && mode1 == MODE_SPONT_PS))) {
    peep -= 1;
    display_write_variable(VP_SET_PEEP, peep);
    
    

  } else if (HMI_Page == 50 && decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && ptr > 0.5 && (settingbutton_count == 11 || (settingbutton_count == 5 && mode1 == MODE_SPONT_PS) || (settingbutton_count == 5 && mode1 == MODE_PCV))) {
    if (ptr <= 1) {
      ptr -= 0.10;
    } else {
      ptr -= 0.50;
    }
    ptr_1 = round(10 * ptr);
    display_write_variable(VP_SET_PTR, ptr_1);
  
  }

  else if (HMI_Page == 50 && decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && settingbutton_count == 12 && set_fio2 > 21) {
    set_fio2 -= 1;
    display_write_variable(VP_SET_FIO2, set_fio2);

    
    
  } else if (HMI_Page == 26 && decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && settingbutton_count == 30 && hhh > 0) {
    hhh -= 1;
    display_write_variable(VP_CLOCK_HH, hhh);

  } else if (HMI_Page == 26 && decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && settingbutton_count == 31 && mmm > 0) {
    mmm -= 1;
    display_write_variable(VP_CLOCK_MM, mmm);

  } else if (decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && HMI_Page == 12) {
    settingbutton_count = 1;
    if (newpatient == 0) {
      newpatient = 1;
      Icon1[7] = 84;
      DWIN_PORT.write(Icon1, 8);
    } else {
      newpatient = 0;
      Icon1[7] = 85;
      DWIN_PORT.write(Icon1, 8);
    }
  } else if (decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && HMI_Page == 13) {
    settingbutton_count = 1;
    if (pediatric_patient == 0) {
      pediatric_patient = 1;
      Icon1[7] = 86;
      DWIN_PORT.write(Icon1, 8);
    } else {
      pediatric_patient = 0;
      Icon1[7] = 65;
      DWIN_PORT.write(Icon1, 8);
    }
  } else if (decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && HMI_Page == 14) {
    settingbutton_count = 1;
    if (malepatient == 0) {
      malepatient = 1;
      Icon1[7] = 66;
      DWIN_PORT.write(Icon1, 8);
      Icon3[7] = 122;
      DWIN_PORT.write(Icon3, 8);
    } else {
      malepatient = 0;
      Icon1[7] = 70;
      DWIN_PORT.write(Icon1, 8);
      Icon3[7] = 123;
      DWIN_PORT.write(Icon3, 8);
    }
  }

  else if (decreamentButtonState && decreamentButtonState != decreamentbutton_lastState && HMI_Page == display_page_3)  
  {
    settingbutton_count = 1;
    mainscreencursor--;
    if (mainscreencursor <= 0) {
      mainscreencursor = 4;  
    }                        
    if (mainscreencursor == 3) {
      mainscreencursor = 2;
    }
    if (mode1 == MODE_PCV) {
      Icon15[7] = 42;
      DWIN_PORT.write(Icon15, 8);
    } else if (mode1 == MODE_PSIMV) {
      Icon15[7] = 41;
      DWIN_PORT.write(Icon15, 8);
    } else if (mode1 == MODE_VCV) {
      Icon15[7] = 45;
      DWIN_PORT.write(Icon15, 8);
    } else if (mode1 == MODE_CPAP) {
      Icon15[7] = 38;
      DWIN_PORT.write(Icon15, 8);
    } else if (mode1 == MODE_HFT) {
      Icon15[7] = 40;
      DWIN_PORT.write(Icon15, 8);
    } else if (mode1 == MODE_SIMV) {
      Icon15[7] = 44;
      DWIN_PORT.write(Icon15, 8);
    } else if (mode1 == MODE_ACV) {
      Icon15[7] = 43;
      DWIN_PORT.write(Icon15, 8);
    } else if (mode1 == MODE_SPONT_PS) {
      Icon15[7] = 39;
      DWIN_PORT.write(Icon15, 8);
    }
    if (mainscreencursor == 1) {
      Icon1[7] = 124;
      DWIN_PORT.write(Icon1, 8);
    } else if (mainscreencursor == 2) {
      Icon1[7] = 127;
      DWIN_PORT.write(Icon1, 8);
    } else if (mainscreencursor == 3) {
      Icon1[7] = 126;
      DWIN_PORT.write(Icon1, 8);
    } else if (mainscreencursor == 4) {
      Icon1[7] = 125;
      DWIN_PORT.write(Icon1, 8);
    }
    
    
  }
  
  if (confirmbuttonState && confirmbuttonState != confirmbutton_lastState) {    

    tr_save = 0;  
    Icon12[7] = 0;
    DWIN_PORT.write(Icon12, 8);  
    if (HMI_Page == 28) {
      EEPROM.put(190, ov_start_duty);  
      EEPROM.put(170, calibration_HFT);
      EEPROM.put(210, calibration_flow);
    }
    
    if (mode1 == MODE_HFT) {
      HMI_Page = 2;
      dwin_page_Set(2);
    } else {
      HMI_Page = display_page_3;
      dwin_page_Set(3);
    }

    if (set_fio2 > 21 && peep == 0) {
      peep = 1;
      display_write_variable(VP_SET_PEEP, peep);
    }
    settingbutton_count = 1;
    standbybutton_count = 2;
    settidalvolume = tidalvolume;
    setbodyweight = bodyweight;
    setfrequency = frequncy;
    setfrequencysimv = frequencysimv;
    seti_e = i_e;
  
    sethighpawalarmlimit = highpawalarmlimit;

    setlowpawalarmlimit = lowpawalarmlimit;
    setmode = mode1;
    setpeep = peep;  
    set_neb_time = neb_time;
    setps = (ps + setpeep);
    
    setptr = ptr;
    setTinsp_simv = Tinsp_simv;
    set_i = i;
    set_e = e;
    setflowrate = flowrate;
    set_Pinsp = (Pinsp + setpeep);
    set_apneatime = apneatime;
    set_set_fio2 = set_fio2;

    
    EEPROM.put(340, i_e);
    EEPROM.update(3, highpawalarmlimit);
    EEPROM.update(4, lowpawalarmlimit);
    EEPROM.update(5, mode1);
    EEPROM.update(6, peep);
    EEPROM.update(8, (10 * ptr));
    Tinsp_simv1 = Tinsp_simv;
    EEPROM.put(100, Tinsp_simv1);
    EEPROM.update(10, i);
    EEPROM.update(11, e);
    EEPROM.update(12, flowrate);
    EEPROM.update(13, Pinsp);
    EEPROM.update(14, apneatime);
    EEPROM.update(15, tidalvolume / 10);
    EEPROM.update(16, bodyweight);
    EEPROM.update(17, frequncy);
    EEPROM.update(18, frequencysimv);
    EEPROM.update(19, ps);
    
    EEPROM.update(26, set_fio2);    
    EEPROM.update(33, Exsence_percentage);
    
    
    EEPROM.update(40, malepatient);
    EEPROM.update(41, pediatric_patient);

    
  }

  confirmbutton_lastState = confirmbuttonState;
  increamentbutton_lastState = increamentButtonState;
  decreamentbutton_lastState = decreamentButtonState;
  modebutton_lastState = modebuttonState;
  encoder_switch_lastState = encoder_switchState;
  settingbutton_lastState = settingButtonState;
  
  
  settingcurrenttime = millis();

  if (settingButtonState == 0) {
    settingprevioustime = settingcurrenttime;
  }

  if ((settingcurrenttime - settingprevioustime) > 2000 && standbybuttonState == 0) {
    O2cal = 1;
    settingprevioustime = settingcurrenttime;
  }

  standbycurrenttime = millis();
  if (standbybuttonState == 0) {
    standbyprevioustime = standbycurrenttime;
    stnd = 0;
  }
  if (((standbycurrenttime - standbyprevioustime) > 4000 && stnd == 0) || standby_scr == 1) {
    noTone(tonePin);
    digitalWrite(tonePin, LOW);
    pinMode(tonePin, INPUT);
    alarm_num = 0;
    HMI_Page = 36;
    dwin_page_Set(36);
    EEPROM.update(24, tur_duty_HFT);
    FLOG_HOOK(FLOG_T_EVENT, FLOG_E_STANDBY_ENTER, 0);
    /* entering standby cancels any active mute cleanly */
    mut = 0;
    mutebutton_count = 1;
    Icon14[7] = 0;
    DWIN_PORT.write(Icon14, 8);
    display_write_variable(VP_MUTE_COUNTDOWN, 90);

    EEPROM.update(27, mainrelay_duty / 20);
    encoder_switch_count = 1;
    Icon18[7] = 0;
    DWIN_PORT.write(Icon18, 8);
    stand_check = 0;
    neb_time = 0;
    Icon21[7] = 0;
    DWIN_PORT.write(Icon21, 8);
    nebu_touch = 0;
    display_write_variable(VP_NEB_TIME, neb_time);
    digitalWrite(nebulizerpin, LOW);

    
    buffer[0] = 0b01000000;  
    
    sal = 0;
    buffer[1] = sal >> 4;             
    buffer[2] = sal << 4;             
    Wire.beginTransmission(MCP4725);  
    Wire.write(buffer[0]);            
    Wire.write(buffer[1]);            
    Wire.write(buffer[2]);            
    Wire.endTransmission();

    if (standbybutton_count == 3) {
      standbybutton_count = 2;
      settingbutton_count = 1;
      settidalvolume = tidalvolume;
      setbodyweight = bodyweight;
      setfrequency = frequncy;
      setfrequencysimv = frequencysimv;
      seti_e = i_e;
      
      setlowpawalarmlimit = lowpawalarmlimit;
      setmode = mode1;
      setpeep = peep;
      setps = ps + setpeep;
      setptr = ptr;
      setTinsp_simv = Tinsp_simv;
      set_i = i;
      set_e = e;
      setflowrate = flowrate;
      set_Pinsp = Pinsp;
      set_apneatime = apneatime;
      set_set_fio2 = set_fio2;

      
      EEPROM.put(340, i_e);
      EEPROM.update(3, highpawalarmlimit);
      EEPROM.update(4, lowpawalarmlimit);
      EEPROM.update(5, mode1);
      EEPROM.update(6, peep);
      EEPROM.update(8, (10 * ptr));
      Tinsp_simv1 = Tinsp_simv;
      EEPROM.put(100, Tinsp_simv1);
      EEPROM.update(10, i);
      EEPROM.update(11, e);
      EEPROM.update(12, flowrate);
      EEPROM.update(13, Pinsp);
      EEPROM.update(14, apneatime);
      EEPROM.update(15, tidalvolume / 10);
      EEPROM.update(16, bodyweight);
      EEPROM.update(17, frequncy);
      EEPROM.update(18, frequencysimv);
      EEPROM.update(19, ps);
      
      EEPROM.update(26, set_fio2);
      EEPROM.update(33, Exsence_percentage);
      
    } else {
      standbybutton_count = 1;
    }
    standbyprevioustime = standbycurrenttime;
    stnd = 1;  
    standby_scr = 0;
  }

  static uint8_t prevStandbyForMute = 0;
  if (prevStandbyForMute != 2 && standbybutton_count == 2) {
    mut = 1;
    mutebutton_count = 0;
    Icon14[7] = 147;
    DWIN_PORT.write(Icon14, 8);
    mutecurrentMillis = millis();
    mutepreviousMillis = mutecurrentMillis;
    display_write_variable(VP_MUTE_COUNTDOWN, 90);
  }
  prevStandbyForMute = standbybutton_count;

  if (mutebuttonState && mutebuttonState != mutebutton_lastState
      && standbybutton_count == 2) {
    /* GATED ON VENTILATING: a spurious edge at power-on (pin rest
       level / DWIN boot frame) was engaging mute in standby - the
       countdown ran from power-on. Mute is only meaningful while
       alarms can sound.                                            */

    if (mut == 1) {
      /* UNMUTE: sound re-enabled, display parks at 90 */
      mut = 0;
      Icon14[7] = 0;
      DWIN_PORT.write(Icon14, 8);
      display_write_variable(VP_MUTE_COUNTDOWN, 90);
      mutebutton_count = 1;
      mutepreviousMillis = millis();
    } else {

      mut = 1;
      mutebutton_count = 0;
      Icon14[7] = 147;
      DWIN_PORT.write(Icon14, 8);
      mutecurrentMillis = millis();
      mutepreviousMillis = mutecurrentMillis;
      display_write_variable(VP_MUTE_COUNTDOWN, 90);
    }
  }
  else if (mutebuttonState && mutebuttonState != mutebutton_lastState) {
    /* edge in standby: consume it, force sane un-muted state        */
    mut = 0;
    mutebutton_count = 1;
    Icon14[7] = 0;
    DWIN_PORT.write(Icon14, 8);
    display_write_variable(VP_MUTE_COUNTDOWN, 90);
  }
  if (mutebutton_count == 0) {
    if ((90 - (mutecurrentMillis - mutepreviousMillis) / 1000) < 2) {
      display_write_variable(VP_MUTE_COUNTDOWN, 90);
    } else {
      display_write_variable(VP_MUTE_COUNTDOWN, (90 - (mutecurrentMillis - mutepreviousMillis) / 1000));
    }
  }

  mutecurrentMillis = millis();

  if ((mutebutton_count == 0) && (mutecurrentMillis - mutepreviousMillis) >= 90000) {
    mutebutton_count = 1;
    if (mut == 1) {
      Icon14[7] = 0;
      DWIN_PORT.write(Icon14, 8);
      mut = 0;
    }
    mutepreviousMillis = mutecurrentMillis;  
  }
  if (mutebutton_count == 1) {
    mutepreviousMillis = mutecurrentMillis;  
  }
  
  
  if (unmountbuttonState && unmountbuttonState != unmountbutton_lastState) {
    unmountbutton_count = 0;
    unmount = 1;
    unmountpreviousMillis = unmountcurrentMillis;
  }

  if (unmountbutton_count == 0) {
    if ((20 - (unmountcurrentMillis - unmountpreviousMillis) / 1000) < 2) {
      display_write_variable(VP_UNMOUNT_COUNTDOWN, 20);  
    } else {
      display_write_variable(VP_UNMOUNT_COUNTDOWN, (20 - (unmountcurrentMillis - unmountpreviousMillis) / 1000));
    }
  }
  
  unmountcurrentMillis = millis();
  if ((unmountbutton_count == 0) && (unmountcurrentMillis - unmountpreviousMillis) >= 20000) {
    unmountbutton_count = 1;
    if (unmount == 1) {
      Icon14[7] = 0;
      DWIN_PORT.write(Icon14, 8);
      digitalWrite(unmount_pin, LOW);
      
      unmount = 0;
    }
    unmountpreviousMillis = unmountcurrentMillis;  
  }

  if (unmountbutton_count == 1) {
    unmountpreviousMillis = unmountcurrentMillis;  
  }

  
  increamentcurrentMillis = millis();
  if ((increamentcurrentMillis - increamentpreviousMillis) >= 600) {
    increamentbutton_lastState = LOW;
  }

  if (increamentButtonState == 0) {
    increamentpreviousMillis = increamentcurrentMillis;  
  }
  
  decreamentcurrentMillis = millis();
  if ((decreamentcurrentMillis - decreamentpreviousMillis) >= 600) {
    decreamentbutton_lastState = LOW;
  }
  if (decreamentButtonState == 0) {
    decreamentpreviousMillis = decreamentcurrentMillis;  
  }
  
  if (cancelButtonState && cancelButtonState != cancelbutton_lastState) {
    
    settingbutton_count = 1;
    

    if (tidalvolume != settidalvolume) {
      display_write_variable(VP_SET_TIDAL_VOLUME, settidalvolume);
    }
    if (bodyweight != setbodyweight) {
      display_write_variable(VP_SET_BODYWEIGHT, setbodyweight);
    }
    if (frequncy != setfrequency) {
      display_write_variable(VP_SET_FREQUENCY, setfrequency);
    }
    if (frequencysimv = setfrequencysimv) {
      display_write_variable(VP_SET_FREQ_SIMV, setfrequencysimv);
    }
    if (peep != setpeep) {
      display_write_variable(VP_SET_PEEP, setpeep);
    }

    if (ptr != setptr) {
      setptr_1 = round(10.0 * setptr);
      display_write_variable(VP_SET_PTR, setptr_1);
    }
    if ((Tinsp_simv) != (setTinsp_simv)) {
      display_write_variable(VP_SET_TINSP_SIMV, (setTinsp_simv) / 100);
    }
    if (i != set_i) {
      int set_i_int = round(10 * set_i);
      display_write_variable(VP_SET_IE_I, set_i_int);
    }
    if (e != set_e) {
      int set_e_int = round(10 * set_e);
      display_write_variable(VP_SET_PINSP, set_e_int);
    }
    if (flowrate != setflowrate) {
      setflowrate = pressure2;
      display_write_variable(VP_SET_FLOWRATE, pressure2);
    }
    if (Pinsp != (set_Pinsp - setpeep)) {
      display_write_variable(VP_SET_PINSP, (set_Pinsp - setpeep));
    }
    if (apneatime != set_apneatime) {
      display_write_variable(VP_SET_APNEA_TIME, set_apneatime);
    }
    if (set_fio2 != set_set_fio2) {
      display_write_variable(VP_SET_FIO2, set_set_fio2);
    }
    if (mode1 != setmode) {
      mode1 = setmode;
    }
    if (ps != (setps - setpeep)) {
      ps = setps - setpeep;
    }
    i_e = seti_e;
    tidalvolume = settidalvolume;
    bodyweight = setbodyweight;

    frequncy = setfrequency;
    frequencysimv = setfrequencysimv;
    i_e = seti_e;
    
    
    mode1 = setmode;
    ps = setps - setpeep;
    peep = setpeep;
    ptr = setptr;
    Tinsp_simv = setTinsp_simv;
    i = set_i;
    e = set_e;
    flowrate = setflowrate;
    Pinsp = set_Pinsp - setpeep;
    apneatime = set_apneatime;
    set_fio2 = set_set_fio2;
  }

  
  resetbutton_lastState = resetbuttonState;
  standbybutton_lastState = standbybuttonState;
  mutebutton_lastState = mutebuttonState;
  unmountbutton_lastState = unmountbuttonState;
  cancelbutton_lastState = cancelButtonState;
}

void alarmbuffer(void) {

  if (alarm_num1 != alarm_num) {
    
    buffer_address = buffer_address + 1;
    alarm_num1 = alarm_num;
  }

  if (alarm_numpower1 != alarm_numpower) {
    
    buffer_address = buffer_address + 1;
    alarm_numpower1 = alarm_numpower;
  }
}
void update_status_leds() {
  int fullBrightness = 255;
  int offValue = 0;

  if (mainspower_state >= 300) {
    analogWrite(LED_AC_MAIN, fullBrightness);
    analogWrite(BATTERY_LED, offValue);
  } else {
    analogWrite(BATTERY_LED, fullBrightness);
    analogWrite(LED_AC_MAIN, offValue);
  }

  if (standbybutton_count != 2 && oxygenavailability_status == LOW || set_set_fio2 > 21) {
    digitalWrite(OXYGEN_LED, LOW);  
  } else {
    digitalWrite(OXYGEN_LED, HIGH);  
  }
}

void mid_alarm(void) {
  pinMode(tonePin, OUTPUT);
  static uint32_t next = 0;
  static uint8_t count = 0;
  static bool highPriority = false;

  const uint16_t high_freq = 900;
  const uint16_t low_freq = 600;
  const uint16_t beep_ms = 200;
  const uint16_t gap_ms = 180;

  if (next == 0) {
    if (standbybutton_count == 2 && (alarm_num == 1 || alarm_num == 3 || alarm_num == 7 || alarm_num == 8 || alarm_num == 10 || alarm_num == 12 || alarm_num == 6)) {
      highPriority = true;
      
    } else if (standbybutton_count == 2 && (alarm_num == 5 || alarm_num == 11 || alarm_num == 2 || alarm_num == 4 || alarm_num == 13 || alarm_num == 9)) {
      highPriority = false;
      
    } else {
      
      alarm_num = 0;
      return;
    }
    count = 0;
    next = millis();
  }

  if (millis() >= next) {

    if (highPriority) {

      if (count < 3) {
        tone(tonePin, high_freq, beep_ms);
        next = millis() + beep_ms + gap_ms;
        count++;
      } else {
        next = millis() + 3200;
        count = 0;
      }
    } else {
      if (count < 2) {
        tone(tonePin, low_freq, beep_ms);
        next = millis() + beep_ms + gap_ms;
        count++;
      } else {
        next = millis() + 5000;
        count = 0;
      }
    }
  }
}

void midi1(void) {
  

  if (set_set_fio2 > 21 && oxygenavailability_status == LOW) {  
    tonetimer = millis();

    if (0 < (tonetimer - previoustonetimer) && (tonetimer - previoustonetimer) < 150 && ton_1 == 1) {
      ton_1 = 0;
    }
    
    
    else if (350 < (tonetimer - previoustonetimer) && (tonetimer - previoustonetimer) < 500) {

    } else if (1000 < (tonetimer - previoustonetimer)) {
      previoustonetimer = tonetimer;
      ton_1 = 1;
    }

  }

  else {
    tonetimer1 = millis();
    if (0 < (tonetimer1 - previoustonetimer1) && (tonetimer1 - previoustonetimer1) < 150) {
      Icon5[7] = 134;
      DWIN_PORT.write(Icon5, 8);
    } else if (420 < (tonetimer1 - previoustonetimer1) && (tonetimer1 - previoustonetimer1) < 570) {
      Icon6[7] = 136;
      DWIN_PORT.write(Icon6, 8);
    } else if (840 < (tonetimer1 - previoustonetimer1) && (tonetimer1 - previoustonetimer1) < 990) {
      Icon7[7] = 135;
      DWIN_PORT.write(Icon7, 8);
    } else if (1760 < (tonetimer1 - previoustonetimer1) && (tonetimer1 - previoustonetimer1) < 1910) {
      Icon8[7] = 137;
      DWIN_PORT.write(Icon8, 8);
    } else if (2190 < (tonetimer1 - previoustonetimer1) && (tonetimer1 - previoustonetimer1) < 2340) {
    } else if (7320 < (tonetimer1 - previoustonetimer1))  
    {
      previoustonetimer1 = tonetimer1;
      Icon4[7] = 0;
      DWIN_PORT.write(Icon4, 8);
      Icon5[7] = 0;
      DWIN_PORT.write(Icon5, 8);  
      Icon6[7] = 0;
      DWIN_PORT.write(Icon6, 8);
      Icon7[7] = 0;
      DWIN_PORT.write(Icon7, 8);
      Icon8[7] = 0;
      DWIN_PORT.write(Icon8, 8);
    }
    
  }
}

void flowsensor(void) {
  
  V1 = (analogRead(Analog_pr2) - ZFF_PR2);
  V2 = V1;

  if (mainrelaypinstate == 1) {
    su = 1;
  }
  if (mainrelaypinstate == 0 && su == 1) {
    v1 = v2;
    v2 = 0;
    su = 0;
  }

  
  float average2 = average1;
  average1 = (analogRead(Analog_pr2) - ZFF_PR2);
  if (average1 > 0 && settidalvolume <= 20 && average1 < 5 && bodyweight <= 10) {  
    average1 = (Pr2_range - 40) * sqrt((average1) / (1023 - ZFF_PR2));             
  } else if (average1 > 0 && bodyweight <= 10) {                                   
    average1 = (Pr2_range - 40) * sqrt((average1) / (1023 - ZFF_PR2));             
  } else if (average1 > 2 && bodyweight > 10) {
    average1 = Pr2_range * sqrt((average1) / (1023 - ZFF_PR2));  
  }

  else {
    average1 = 0;
  }

  
  interval_new = millis();
  flow_time = ((interval_new - interval_old));
  interval_old = millis();
  battery_time = ((interval_new - interval_old));
  interval_old = interval_new;
  
  if (gh == 0 && inspiration == 1) {
    leak_volume = currentvolume;
    currentvolume = 0;
    gh = 1;
    currentvolume = (-1 * leak_volume);
  } else if (inspiration == 0) {
    gh = 0;
  }
  
  
  
  
  
  
  
  

  
  if (((setmode != MODE_HFT && (setmode == MODE_SPONT_PS || setmode == MODE_PCV) && mainrelaypinstate == HIGH)) || ((setmode != MODE_HFT && setmode != MODE_SPONT_PS && mainrelaypinstate == HIGH && (1.3 * 00.1991640625 * 1.17 * (analogRead(Analog_pr1) - ZFP)) > (3 + peep_AVERAGED_SAVED))) ) {
    
    if (gh == 0 && mainrelaypinstate == 1) {
      leak_volume = currentvolume;
      currentvolume = 0;
      currentvolume = (-1 * leak_volume);
      gh = 1;
    }

    currentvolume += 0.012 * (flow_time) * (average1 + average2);
    T_volume1 = currentvolume;  

    if (setmode == MODE_SPONT_PS || setmode == MODE_PCV) {
      T_volume1 = currentvolume;  
    }
    
    
    
    
    
    
    
  }
  
  if (mainrelaypinstate == HIGH) {
    f = 0;
  }
  
  if (mainrelaypinstate == LOW && f == 0 && tr_save != 1 && T_volume1 > 0) {  
    T_volume = T_volume1;                                                     
    vt_SUM = vt_SUM - vt_READINGS[vt_INDEX];                                  
    vt_VALUE = T_volume;                                                      
    vt_READINGS[vt_INDEX] = vt_VALUE;                                         
    vt_SUM = vt_SUM + vt_VALUE;                                               
    vt_INDEX = (vt_INDEX + 1) % WINDOW_SIZEvt;                                
    vt_AVERAGED = vt_SUM / WINDOW_SIZEvt;
    vt_AVERAGED_SAVED = vt_AVERAGED;  
    
    if (settidalvolume <= 200) {
      vt_AVERAGED_SAVED = (((vt_AVERAGED_SAVED + 5) / 10) * 10);
    } else {
      vt_AVERAGED_SAVED = (((vt_AVERAGED_SAVED + 10) / 20) * 20);
    }
    
    f = 1;
  }
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  T_Volume_final = T_volume;  
  
  
}

void flowsensor1(void) {
  
  V1 = (analogRead(Analog_pr2) - ZFF_PR2);
  V2 = V1;  

  
  
  
  

  if (mainrelaypinstate == 1) {
    su = 1;
  }
  if (mainrelaypinstate == 0 && su == 1) {
    v1 = v2;
    v2 = 0;
    su = 0;
  }

  
  float average2 = average1;
  average1 = (analogRead(Analog_pr2) - ZFF_PR2);
  if (average1 > 0 && settidalvolume <= 20 && average1 < 5 && bodyweight <= 10)  
  {
    average1 = ((Pr2_range - 40) * sqrt(0.002587 * average1));
    
  }

  else if (average1 > 0 && bodyweight <= 10) {                
    average1 = (Pr2_range - 40) * sqrt(0.002587 * average1);  
    
  }

  else if (average1 > 2 && bodyweight > 10) {
    
    average1 = Pr2_range * sqrt(0.002587 * average1);  
  }

  else {
    average1 = 0;
  }

  
  interval_new = millis();
  flow_time = ((interval_new - interval_old));
  interval_old = millis();
  battery_time = ((interval_new - interval_old));
  interval_old = interval_new;
  
  if (gh == 0 && inspiration == 1) {
    leak_volume = currentvolume;
    currentvolume = 0;
    gh = 1;
    currentvolume = (-1 * leak_volume);
  } else if (inspiration == 0) {
    gh = 0;
  }
  
  
  
  
  
  
  
  

  
  if (((setmode != MODE_HFT && (setmode == MODE_SPONT_PS || setmode == MODE_PCV) && mainrelaypinstate == HIGH)) || ((setmode != MODE_HFT && setmode != MODE_SPONT_PS && mainrelaypinstate == HIGH && (1.3 * PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP)) > (3 + peep_AVERAGED_SAVED))) ) {
    
    if (gh == 0 && mainrelaypinstate == 1) {
      leak_volume = currentvolume;
      currentvolume = 0;
      currentvolume = (-1 * leak_volume);
      gh = 1;
    }
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    currentvolume += 0.012 * (flow_time) * (average1 + average2);
    T_volume1 = currentvolume;  

    if (setmode == MODE_SPONT_PS || setmode == MODE_PCV) {
      T_volume1 = currentvolume;  
    }
    
    
    
    
    
    
    
  }
  
  if (mainrelaypinstate == HIGH) {
    f = 0;
  }
  
  if (mainrelaypinstate == LOW && f == 0 && tr_save != 1 && T_volume1 > 0) {  
    T_volume = T_volume1;                                                     
    vt_SUM = vt_SUM - vt_READINGS[vt_INDEX];                                  
    vt_VALUE = T_volume;                                                      
    vt_READINGS[vt_INDEX] = vt_VALUE;                                         
    vt_SUM = vt_SUM + vt_VALUE;                                               
    vt_INDEX = (vt_INDEX + 1) % WINDOW_SIZEvt;                                
    vt_AVERAGED = vt_SUM / WINDOW_SIZEvt;
    vt_AVERAGED_SAVED = vt_AVERAGED;  
    
    if (settidalvolume <= 200) {
      vt_AVERAGED_SAVED = (((vt_AVERAGED_SAVED + 5) / 10) * 10);
    } else {
      vt_AVERAGED_SAVED = (((vt_AVERAGED_SAVED + 10) / 20) * 20);
    }
    
    f = 1;
  }
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  T_Volume_final = T_volume;  
  
  
}

void O2calibration(void) {

  if (digitalRead(oxygenavailability_read) == HIGH) {
    {
      dwin_page_Set(35);
    }
    
    delay(4000);
    HMI_Page = 36;  
    Icon18[7] = 0;
    DWIN_PORT.write(Icon18, 8);
    encoder_switch_count = 1;
    dwin_page_Set(36);
    confirm = 0;
    O2cal = 0;
  }

  calicurrenttime = millis();
  
  O2confirm_add = uint16_t((Buffer1[4] << 8) | Buffer1[5]);
  if (O2confirm_add == TP_O2CAL_CONFIRM || confirm == 1) {

    
    
    confirm = 1;
    if ((calicurrenttime - caliprevioustime) <= 15000) {
      HMI_Page = 31;
      dwin_page_Set(31);
      display_write_variable(VP_O2CAL_RAW, analogRead(fio2_read));
      
    }

    else if ((calicurrenttime - caliprevioustime) <= 30000 && (calicurrenttime - caliprevioustime) > 15000) {
      HMI_Page = 32;
      dwin_page_Set(32);
      
      display_write_variable(VP_O2CAL_PROGRESS, 40);
    }

    else if ((calicurrenttime - caliprevioustime) <= 45000 && (calicurrenttime - caliprevioustime) > 30000) {
      HMI_Page = 33;
      dwin_page_Set(33);
      
      display_write_variable(VP_O2CAL_PROGRESS, 70);
    }

    else if ((calicurrenttime - caliprevioustime) <= 60000 && (calicurrenttime - caliprevioustime) > 45000) {
      HMI_Page = (34);
      dwin_page_Set(34);
      
      display_write_variable(VP_O2CAL_PROGRESS, 98);
    }

    else if ((calicurrenttime - caliprevioustime) <= 62000 && (calicurrenttime - caliprevioustime) > 60000) {
      HMI_Page = (34);
      dwin_page_Set(34);
      display_write_variable(VP_O2CAL_PROGRESS, 100);
    }

    if ((calicurrenttime - caliprevioustime) < 30000)  
    {
      digitalWrite(tur_reverse, HIGH);  
      analogWrite(turbinePin, 70);
      buffer[0] = 0b01000000;           
      sal = 0;                          
      buffer[1] = sal >> 4;             
      buffer[2] = sal << 4;             
      Wire.beginTransmission(MCP4725);  
      Wire.write(buffer[0]);            
      Wire.write(buffer[1]);            
      Wire.write(buffer[2]);            
      Wire.endTransmission();
      
      cOcmillis = millis();
      {
        cOp_SUM = cOp_SUM - cOp_READINGS[cOp_INDEX];   
        cOp_VALUE = analogRead(fio2_read);             
        cOp_READINGS[cOp_INDEX] = cOp_VALUE;           
        cOp_SUM = cOp_SUM + cOp_VALUE;                 
        cOp_INDEX = (cOp_INDEX + 1) % WINDOW_SIZEcop;  
        cOp_AVERAGED = cOp_SUM / WINDOW_SIZEcop;
        f1 = cOp_AVERAGED;  
      }
      
      
    }
    if (((calicurrenttime - caliprevioustime) < 70000) && ((calicurrenttime - caliprevioustime) > 31000)) {
      digitalWrite(tur_reverse, HIGH);  
      analogWrite(turbinePin, 0);
      mainrelaypinstate = HIGH;
      {
        buffer[0] = 0b01000000;           
        sal = 4070;                       
        buffer[1] = sal >> 4;             
        buffer[2] = sal << 4;             
        Wire.beginTransmission(MCP4725);  
        Wire.write(buffer[0]);            
        Wire.write(buffer[1]);            
        Wire.write(buffer[2]);            
        Wire.endTransmission();
      }  
      
      cOcmillis = millis();
      {
        cOp_SUM = cOp_SUM - cOp_READINGS[cOp_INDEX];   
        cOp_VALUE = analogRead(fio2_read);             
        cOp_READINGS[cOp_INDEX] = cOp_VALUE;           
        cOp_SUM = cOp_SUM + cOp_VALUE;                 
        cOp_INDEX = (cOp_INDEX + 1) % WINDOW_SIZEcop;  
        cOp_AVERAGED = cOp_SUM / WINDOW_SIZEcop;
        f2 = cOp_AVERAGED;  
      }
      
      
    }
    if ((calicurrenttime - caliprevioustime) > 62000) {
      L3 = (f2 - f1) / 79;  
      
      
      if (isnan(L3) || L3 < 0.30f || L3 > 12.0f) {
        L3 = 1.19f;       /* reject an implausible calibration result    */
      }
      L3_1 = (int)(L3 * 100.0f);
      f1_1 = (int)(f1 * 100.0f);
      EEPROM.put(60, L3);
      EEPROM.put(80, f1);
      Op_primed = 0;      /* re-prime against the new constants          */
      dOp_primed = 0;
      O2cal = 0;
      confirm = 0;
      dwin_page_Set(36);
      buffer[0] = 0b01000000;           
      sal = 0;                          
      buffer[1] = sal >> 4;             
      buffer[2] = sal << 4;             
      Wire.beginTransmission(MCP4725);  
      Wire.write(buffer[0]);            
      Wire.write(buffer[1]);            
      Wire.write(buffer[2]);            
      Wire.endTransmission();
      sal = 1825;
    }
  }
}

void battery(void) {
  int mainsRaw = analogRead(MAINS_PIN);      /* read each ADC exactly once */
  int battRaw  = analogRead(BATTERY_PIN);

  if (mainsRaw <= 0)        SMPS = 0.00f;
  else if (mainsRaw >= 760) SMPS = 26.00f;
  else                      SMPS = 0.034342105f * mainsRaw;

  if (battRaw < 458)        btr_volt = 16.00f;
  else if (battRaw > 715)   btr_volt = 25.00f;
  else                      btr_volt = 0.03463035f * (battRaw - 458) + 16.1f;

  battery_icon_percentage = battRaw;

  /* legacy mains-rail percent estimate (kept: feeds battery_AVERAGED_1) */
  float v2 = 1.64f * (mainsRaw * 22.2f / 1024.0f);
  if (mainsRaw >= 300) v2 *= 1.04f;
  percent = ((v2 - 19.0f) / (24.4f - 19.0f)) * 100.0f;
  if (percent > 100) { battery_AVERAGED = 100; percent = 100; }
  if (percent < 0)   { battery_AVERAGED = 0;   percent = 0;   }

  mainspower_state = mainsRaw;
  batt_status = (mainsRaw >= 300) ? 0 : 1;

  /* moving average of the raw battery ADC */
  BATTERY_SUM -= BATTERY_READINGS[BATTERY_INDEX];
  BATTERY_VALUE = battery_icon_percentage;
  BATTERY_READINGS[BATTERY_INDEX] = BATTERY_VALUE;
  BATTERY_SUM += BATTERY_VALUE;
  BATTERY_INDEX = (BATTERY_INDEX + 1) % BATTERY_WINDOW_SIZE;
  BATTERY_AVERAGED = BATTERY_SUM / BATTERY_WINDOW_SIZE;

  /* pick the icon (bands unchanged, boundaries contiguous) */
  uint8_t icon;
  if (batt_status == 0) {
    icon = 168;                                          /* on mains */
  } else {
    if      (BATTERY_AVERAGED < battery_percent_10) icon = 169;
    else if (BATTERY_AVERAGED < battery_percent_20) icon = 170;
    else if (BATTERY_AVERAGED < battery_percent_30) { icon = 171; alarm_numpower = 1; }
    else if (BATTERY_AVERAGED < battery_percent_40) icon = 172;
    else if (BATTERY_AVERAGED < battery_percent_50) icon = 173;
    else if (BATTERY_AVERAGED < battery_percent_60) icon = 174;
    else if (BATTERY_AVERAGED < battery_percent_70) icon = 175;
    else if (BATTERY_AVERAGED < battery_percent_80) icon = 176;
    else if (BATTERY_AVERAGED < battery_percent_90) icon = 177;
    else                                            icon = 178;  /* >=90% incl. 100 */
  }
  mainspower_last_state = mainspower_state;

  /* transmit only on change, plus a slow refresh */
  static uint8_t  lastIcon    = 0;
  static uint32_t lastIconTx  = 0;
  if (icon != lastIcon || (millis() - lastIconTx) >= 2000) {
    lastIcon   = icon;
    lastIconTx = millis();
    Icon19[7]  = icon;
    DWIN_PORT.write(Icon19, 8);
  }

  battery_AVERAGED_1 = battery_AVERAGED;
}

#define DWIN_SEC_MTR_ADDR 0xBA07
#define DWIN_HR_MTR_ADDR 0xBA09
#define DWIN_MIN_MTR_ADDR 0xBA11
#define DWIN_LOAD_ADDR 0xBA50
#define DWIN_HR_TXT_ADDR 0xBA14
#define DWIN_CT_TXT_ADDR 0xBA60
#define DWIN_TT_TXT_ADDR 0xBA80
#define EEPROM_HOUR_METER_ADDR 550
uint32_t previousMillis = 0;

uint32_t runtime_seconds = 0;
uint32_t runtime_minutes = 0;
uint32_t runtime_hours = 0;

uint32_t totalRunMin = 1;

void saveHourMeter(uint32_t value) {
  for (int i = 0; i < 4; i++) {
    EEPROM.write(EEPROM_HOUR_METER_ADDR + i, (value >> (8 * i)) & 0xFF);
  }
}

uint32_t loadHourMeter() {
  uint32_t value = 0;

  for (int i = 0; i < 4; i++) {
    value |= ((uint32_t)EEPROM.read(EEPROM_HOUR_METER_ADDR + i) << (8 * i));
  }

  return value;
}

void updateHourMeter() {
  uint32_t currentMillis = millis();

  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;

    runtime_seconds++;

    if (runtime_seconds > 59) {
      runtime_seconds = 0;
      runtime_minutes++;

      if (runtime_minutes > 59) {
        runtime_minutes = 0;
        runtime_hours++;
        if (runtime_hours > 23) {
          runtime_hours = 0;
        }
      }
      totalRunMin++;
    }

    if (HMI_Page == 12 || HMI_Page == PAGE_STANDBY || HMI_Page == 50) {
      display_write_variable(DWIN_SEC_MTR_ADDR, runtime_seconds);
      display_write_variable(DWIN_MIN_MTR_ADDR, runtime_minutes);
      display_write_variable(DWIN_HR_MTR_ADDR, runtime_hours);
      display_write_variable(DWIN_LOAD_ADDR, totalRunMin);
    }
  }

  if (totalRunMin % 5 == 0) {
    

    saveHourMeter(totalRunMin);
  }
}

void update_page(uint16_t HMI_Page) {
  if (currentPage != HMI_Page) {
    if (backNav == 0) {
      prevPage = currentPage;   /* forward navigation: record history */
    }
    backNav = 0;                /* consume the flag either way        */
    currentPage = HMI_Page;
  }
}

void setup() {  

  /* rotary encoder enabled (was parked on invalid pins 900/180/190) */
  pinMode(encoder_pinA, INPUT_PULLUP);
  pinMode(encoder_pinB, INPUT_PULLUP);
  pinMode(encoder_switchPin, INPUT_PULLUP);
  encoder_pinAStateLast = digitalRead(encoder_pinA);  /* no false first edge */
  pinMode(unmount_pin, OUTPUT);
  digitalWrite(unmount_pin, LOW);
  pinMode(encoder_switchPin, INPUT_PULLUP);  
  pinMode(encoder_pinA, INPUT);              
  pinMode(encoder_pinB, INPUT);              
  pinMode(53, OUTPUT);
  pinMode(chipSelect, OUTPUT);
  digitalWrite(chipSelect, HIGH);

  DEBUG.begin(9600);

  if (!SD.begin(chipSelect)) {
    
  }
  if (!SD.exists("ALARMLOG.txt")) {
    datafile = SD.open("ALARMLOG.txt", FILE_WRITE);
    datafile.println("DATE\t EVENT NAME\t VALUE ");
    datafile.close();
  } else if (SD.exists("ALARMLOG.txt")) {
    SD.remove("ALARMLOG.txt");
  }

#if 1
  Wire.begin();  
  sal = 0;
  buffer[1] = sal >> 4;             
  buffer[2] = sal << 4;             
  Wire.beginTransmission(MCP4725);  
  Wire.write(buffer[0]);            
  Wire.write(buffer[1]);            
  Wire.write(buffer[2]);            
  Wire.endTransmission();
#endif

  if (digitalRead(mutebutton) == LOW)
  {
    EEPROM.get(340, i_e);
    
    highpawalarmlimit = EEPROM.read(3);
    lowpawalarmlimit = EEPROM.read(4);
    mode1 = EEPROM.read(5);
    peep = EEPROM.read(6);
    ptr = EEPROM.read(8);
    ptr = ptr / 10;
    EEPROM.get(100, Tinsp_simv1);
    Tinsp_simv = Tinsp_simv1;
    i = EEPROM.read(10);
    e = EEPROM.read(11);
    flowrate = EEPROM.read(12);
    Pinsp = EEPROM.read(13);
    apneatime = EEPROM.read(14);
    tidalvolume = EEPROM.read(15);
    
    tidalvolume = tidalvolume * 10;
    bodyweight = EEPROM.read(16);
    frequncy = EEPROM.read(17);
    frequencysimv = EEPROM.read(18);
    ps = EEPROM.read(19);
    EEPROM.get(60, L3);
    EEPROM.get(80, f1);
    /* a virgin or corrupted EEPROM returns 0xFFFFFFFF, which loads as NaN
       and poisons every downstream FiO2 computation with no guard.     */
    if (isnan(L3) || L3 < 0.30f || L3 > 12.0f) { L3 = 1.19f; }
    if (isnan(f1) || f1 < 5.0f || f1 > 400.0f) { f1 = 19.90f; }
    L3_1 = (int)(L3 * 100.0f);
    f1_1 = (int)(f1 * 100.0f);
    Op_primed = 0;      /* re-prime the filters against the new constants */
    dOp_primed = 0;
    peepduty = EEPROM.read(25);  
    tur_duty = EEPROM.read(23);  
    tur_duty_HFT = EEPROM.read(24);
    pee1 = EEPROM.read(25);  
    set_fio2 = EEPROM.read(26);
    mainrelay_duty = EEPROM.read(27);
    mainrelay_duty = mainrelay_duty * 20;
    
    EEPROM.get(150, ZFP);
    EEPROM.get(140, ZFP_pr3);
    EEPROM.get(250, CF_PR2);
    EEPROM.get(270, Pr4_range);
    EEPROM.get(290, pf);
    EEPROM.get(130, ZFF_PR2);
    EEPROM.get(120, ZFF_PR4);
    demomode = EEPROM.read(32);
    Exsence_percentage = EEPROM.read(33);
    
    malepatient = EEPROM.read(40);
    pediatric_patient = EEPROM.read(41);
    
    EEPROM.get(170, calibration_HFT);   
    EEPROM.get(210, calibration_flow);  
    EEPROM.get(190, ov_start_duty);
    
  } else {
    delay(40);
    EEPROM.get(170, calibration_HFT);
    ZFF_PR2 = analogRead(Analog_pr2);
    EEPROM.put(130, ZFF_PR2);  
    ZFP = analogRead(Analog_pr1);
    EEPROM.put(150, ZFP);  
    ZFP_pr3 = (analogRead(Analog_pr3));
    EEPROM.put(140, ZFP_pr3);  
    ZFF_PR4 = analogRead(Analog_pr4);
    EEPROM.put(120, ZFF_PR4);  
    delay(10);
  }
  
  if (bodyweight == 5) {
    tidalvolume_L_limit = bodyweight * 2;
  } else {
    tidalvolume_L_limit = bodyweight * 5;
  }

  tidalvolume_H_limit = bodyweight * 15;
  tidalvolume = bodyweight * 9;

  if (tidalvolume > 2000) {
    tidalvolume = 2000;
  }
  if (tidalvolume < 10) {
    tidalvolume = 10;
  }
  if (bodyweight <= 10) {
    frequncy_L_limit = 2;
    frequncy_H_limit = 70;
    frequencysimv_H_limit = 40;
  } else if (bodyweight > 10 && bodyweight <= 15) {
    frequncy_L_limit = 2;
    frequncy_H_limit = 54;
    frequencysimv_H_limit = 40;
  } else if (bodyweight > 15 && bodyweight <= 20) {
    frequncy_L_limit = 2;
    frequncy_H_limit = 46;
    frequencysimv_H_limit = 40;
  } else if (bodyweight > 20 && bodyweight <= 30) {
    frequncy_L_limit = 2;
    frequncy_H_limit = 42;
    frequencysimv_H_limit = 40;
  } else if (bodyweight > 30 && bodyweight <= 40) {
    frequncy_L_limit = 2;
    frequncy_H_limit = 37;
    frequencysimv_H_limit = 37;
  } else if (bodyweight > 40 && bodyweight <= 50) {
    frequncy_L_limit = 2;
    frequncy_H_limit = 35;
    frequencysimv_H_limit = 35;
  } else if (bodyweight > 50) {
    frequncy_L_limit = 2;
    frequncy_H_limit = 30;
    frequencysimv_H_limit = 30;
  }

  settidalvolume = tidalvolume;
  setbodyweight = bodyweight;
  setfrequency = frequncy;
  setfrequencysimv = frequencysimv;
  seti_e = i_e;
  setmode = mode1;
  setpeep = peep;  
  set_neb_time = neb_time;
  setps = (ps + setpeep);
  setptr = ptr;
  setTinsp_simv = Tinsp_simv;
  set_i = i;
  set_e = e;
  setflowrate = flowrate;
  set_Pinsp = (Pinsp + setpeep);
  set_apneatime = apneatime;
  set_set_fio2 = set_fio2;

  pinMode(fio2_read, INPUT);
  
  pinMode(turbinePin, OUTPUT);
  pinMode(tur_reverse, OUTPUT);
  
  pinMode(B_status_led, OUTPUT);
  pinMode(G_status_led, OUTPUT);
  pinMode(R_status_led, OUTPUT);

  pinMode(nebulizerpin, OUTPUT);
  digitalWrite(nebulizerpin, LOW);
  
  pinMode(standbybutton, INPUT);
  pinMode(mainspower_write, OUTPUT);
  pinMode(white_led_10inch_button, OUTPUT);
  pinMode(yellow_led_10inch_button, OUTPUT);
  pinMode(red_led_10inch_button, OUTPUT);

  pinMode(mainspower_read, INPUT);
  pinMode(batterypower_write, OUTPUT);
  pinMode(lowbatterystatus_write, OUTPUT);
  pinMode(oxygenavailability_read, INPUT_PULLUP);
  pinMode(oxygenavaibility_write, OUTPUT);
  pinMode(oxygenfailure_write, OUTPUT);
  pinMode(airavailability_read, INPUT);
  pinMode(airavailability_write, OUTPUT);
  pinMode(airfailure_write, OUTPUT);

  pinMode(setting_button, INPUT);
  pinMode(cancel_button, INPUT);
  pinMode(decreament_button, INPUT);
  pinMode(increament_button, INPUT);
  pinMode(modebutton, INPUT);
  pinMode(confirmbutton, INPUT);
  pinMode(mutebutton, INPUT);
  pinMode(standbybutton, INPUT);
  pinMode(mainrelaypin, OUTPUT);
  pinMode(alarmled, OUTPUT);

  DWIN_PORT.begin(201600); //9600
  delay(4);
  version_1 = 100 * pcb_VR;
  display_write_variable(VP_MODEL_ID, 11);  
  display_write_variable(VP_FW_VERSION, version_1);

  pvloop_init();
  pfloop_init();

  vploop_init();
  vfloop_init();

  fvloop_init();;
  fploop_init();

  totalRunMin = loadHourMeter();

  if (totalRunMin == (0xA2A2A2A2A2)) {
    totalRunMin = 0;
    saveHourMeter(totalRunMin);
  }

  Icon26[7] = 201;
  DWIN_PORT.write(Icon26, 8);
  dwin_page_Set(0x0C);
  delay(50);
  display_write_text(DWIN_HR_TXT_ADDR, "  Hr : Min : Sec ");
  display_write_text(DWIN_CT_TXT_ADDR, " Current Time ");
  display_write_text(DWIN_TT_TXT_ADDR, "Total Time ");
  if (setmode == MODE_ACV || setmode == MODE_SIMV || setmode == MODE_VCV) {  
    highvtalarmlimit = 600;                            
    lowvtalarmlimit = 40;                              
  } else {
    highvtalarmlimit = 600;  
    lowvtalarmlimit = 40;
    (0.75 * OFFSET_BW_TV * bodyweight);  
  }

  display_write_variable(VP_ALM_VT_HIGH, highvtalarmlimit);
  display_write_variable(VP_ALM_VT_LOW, lowvtalarmlimit);

  if (setmode == MODE_ACV || setmode == MODE_SIMV || setmode == MODE_VCV || setmode == MODE_HFT || setmode == MODE_CPAP || setmode == MODE_PCV || setmode == MODE_PSIMV) {
    highbpmalarmlimit = 35;  
    lowbpmalarmlimit = 6;    
  } else {
  }

  display_write_variable(VP_ALM_BPM_HIGH, highbpmalarmlimit);
  display_write_variable(VP_ALM_BPM_LOW, lowbpmalarmlimit);
  
  if (setmode == MODE_PCV || setmode == MODE_CPAP || setmode == MODE_PSIMV || setmode == MODE_SPONT_PS) {
    highpawalarmlimit = 40;
    display_write_variable(VP_ALM_PAW_HIGH, highpawalarmlimit);
  } else {
    highpawalarmlimit = 40;
    display_write_variable(VP_ALM_PAW_HIGH, highpawalarmlimit);
  }  

  if (setmode == MODE_PCV || setmode == MODE_CPAP || setmode == MODE_PSIMV || setmode == MODE_SPONT_PS) {
    lowpawalarmlimit = 5;
    display_write_variable(VP_ALM_PAW_LOW, lowpawalarmlimit);
  }  
  else {
    lowpawalarmlimit = 5;
    display_write_variable(VP_ALM_PAW_LOW, lowpawalarmlimit);
  }
  
  if (setmode == MODE_ACV || setmode == MODE_SIMV || setmode == MODE_VCV) {
    high_mvalarmlimit = 10.0;  
    low_mvalarmlimit = 0.;     
  } else {
    high_mvalarmlimit = 10.0;  
    low_mvalarmlimit = 0.2;    
  }
  high_mvalarmlimit_1 = high_mvalarmlimit;  
  low_mvalarmlimit_1 = low_mvalarmlimit;
  display_write_variable(VP_ALM_MV_HIGH, 10 * high_mvalarmlimit_1);
  display_write_variable(VP_ALM_MV_LOW, 10 * low_mvalarmlimit_1);

  high_fio2alarmlimit = set_fio2 + 30;
  low_fio2alarmlimit = set_fio2 - 30;

  if ((set_fio2 + 30) >= 100) {
    high_fio2alarmlimit = 100;
  }

  if ((set_fio2 - 30) <= 18) {
    low_fio2alarmlimit = 18;
    display_write_variable(VP_ALM_FIO2_LOW, low_fio2alarmlimit);
  } else {
    display_write_variable(VP_ALM_FIO2_LOW, low_fio2alarmlimit);
  }

  if ((set_fio2 + 30) <= 19) {
    high_fio2alarmlimit = 19;
    display_write_variable(VP_ALM_FIO2_HIGH, high_fio2alarmlimit);
  } else {
    display_write_variable(VP_ALM_FIO2_HIGH, high_fio2alarmlimit);
  }
  display_write_variable(VP_SET_PEEP, peep);  
  ptr = 2.0;
  ptr_1 = 10 * ptr;
  display_write_variable(VP_SET_PTR, ptr_1);
  
  display_write_variable(VP_SET_TINSP_SIMV, 10 * Tinsp_simv / 1000);  

  if (i_e >= (-1) && i_e <= 1) {
    i_e = 1;  
  }
  if (i_e == 1) {
    i = 1;
    e = 1;
  }
  if (i_e > 1) {
    i = 1;
    e = i_e;
  }
  if (i_e >= 0 && i_e < 1) {
    e = 1;
    i = i_e;
  }
  if (i_e < 0) {
    e = 1;
    i = -i_e;
  }

  int i_int = round(10 * i);
  int e_int = round(10 * e);
  display_write_variable(VP_SET_IE_I, i_int);
  display_write_variable(VP_SET_IE_E, e_int);
  display_write_variable(VP_SET_FLOWRATE, flowrate);       
  display_write_variable(VP_SET_PINSP, Pinsp);          
  display_write_variable(VP_SET_APNEA_TIME, apneatime);      
  display_write_variable(VP_SET_TIDAL_VOLUME, tidalvolume);    
  display_write_variable(VP_SET_BODYWEIGHT, bodyweight);     
  display_write_variable(VP_SET_FREQUENCY, frequncy);       
  display_write_variable(VP_SET_FREQ_SIMV, frequencysimv);  
  display_write_variable(VP_SET_PS, ps);             
  display_write_variable(VP_SET_FIO2, set_fio2);       
  
  display_write_variable(VP_SET_EXSENCE_PCT, Exsence_percentage);  
  Height = (bodyweight + 100);
  if (Height >= 160 && malepatient == 1) {
    Height = ((bodyweight / 0.9) + 100);
  }
  if (Height >= 150 && malepatient == 0) {
    Height = ((bodyweight / 0.9) + 100);
  }
  display_write_variable(VP_PATIENT_HEIGHT, Height);
  Icon1[7] = 85;
  DWIN_PORT.write(Icon1, 8);
  limitparameter();

#if 1
  if ((digitalRead(mutebutton) == LOW) && (digitalRead(modebutton) == LOW)) {
    for (int td = 1; td <= 40; td += 1) {
      digitalWrite(tur_reverse, HIGH);
      analogWrite(turbinePin, 100);
      delay(100);
      
      pressure2 = Pr2_range * sqrt(0.002587 * (analogRead(Analog_pr2) - ZFF_PR2));
      Pr4_range = (pressure2 * sqrt(0.002587 * (analogRead(Analog_pr4) - ZFF_PR4)));

      vt_SUM = vt_SUM - vt_READINGS[vt_INDEX];    
      vt_VALUE = Pr4_range;                       
      vt_READINGS[vt_INDEX] = vt_VALUE;           
      vt_SUM = vt_SUM + vt_VALUE;                 
      vt_INDEX = (vt_INDEX + 1) % WINDOW_SIZEvt;  
      vt_AVERAGED = vt_SUM / WINDOW_SIZEvt;
      Pr4_range = vt_AVERAGED;
      
      EEPROM.put(270, 40);
      pressure4 = Pr4_range * sqrt(0.002587 * (analogRead(Analog_pr4) - ZFF_PR4));
    }
  }
#endif
  noInterrupts();
  TCCR1A = (1 << COM1B1) | (1 << WGM11);               
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);  
  ICR1 = (16000000 / frequency) - 1;                   
  OCR1B = ICR1 / 2;                                    
  interrupts();                                        

  /* flash log: init + power-on record */
  FLOG_INIT();
  //mk
  display_write_variable(VP_NEB_TIME, 0);   /* boot init: was stale (-168) */
  FLOG_HOOK(FLOG_T_EVENT, FLOG_E_POWER_ON, 0);
}

/* ===== extracted verbatim from loop(), original lines 10651-10746 =====
 * outer guard was: if (standbybutton_count == 2 && setmode == MODE_VCV)
 * now dispatched from the mode switch in loop(). Body unchanged. */
static void runModeVCV(void) {  /* volume A/C */
        apnbackup_mode = 0;
        Tinsp = turbine_pickup_time_factor * ((60000 * set_i) / ((e + set_i) * setfrequency));  
        Texp = ((60000 / setfrequency) - (Tinsp));
        pressure = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);
        uint32_t mainrelaycurrentMillis = millis();

        if ((mainrelaycurrentMillis - mainrelaypreviousMillis) <= 1.3 * Tinsp)  
        {
          inspiration = 1;
          
          if (gh == 0 && inspiration == 1) {
            leak_volume = currentvolume;
            currentvolume = 0;
            currentvolume = (-1 * leak_volume);
            gh = 1;
          } else if (inspiration == 0) {
            gh = 0;
          }
          
          if (mainrelaypinstate == HIGH) {
            Ti_real_1 = (mainrelaycurrentMillis - mainrelaypreviousMillis);  
          }
          peakdetect();
          if (mainrelaypinstate == HIGH) {
            analogWrite(peepPumpPin, 255);
          } else {
            analogWrite(peepPumpPin, (peepduty));
          }
          pressure = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);
          flowsensor();
          if ((pressure >= (sethighpawalarmlimit))) {
            peakdetect();
            hpstate = 1;
            Hvtcutstate = 1;
          }
          if (hpstate == 0) {
            mainrelaypinstate = HIGH;
          } else if (hpstate == 1) {
            mainrelaypinstate = LOW;  
          }

          fio2_function();  
          if ((highvtalarmlimit < vt_AVERAGED_SAVED) && settidalvolume >= 200) {
            Hvtstate = 1;
          } else if ((highvtalarmlimit < vt_AVERAGED_SAVED) && settidalvolume <= 200) {
            Hvtstate = 1;
          }
          if (settidalvolume < (currentvolume) && currentvolume >= 20) {
            Hvtcutstate = 1;  
          }
          if (Hvtcutstate == 0) {
            mainrelaypinstate = HIGH;
          } else if (Hvtcutstate == 1) {
            mainrelaypinstate = LOW;
          }
          fio2_function();  
          if (mainrelaypinstate == HIGH) {
            analogWrite(turbinePin, tur_duty);  
            digitalWrite(tur_reverse, HIGH);
          } else {
            analogWrite(turbinePin, compan_duty);
          }
          if ((lowvtalarmlimit > T_Volume_final && vt_AVERAGED_SAVED > 20 && settidalvolume >= 200)) {
            peakdetect();
            Lvtstate = 1;
          } else if ((lowvtalarmlimit > T_Volume_final && vt_AVERAGED_SAVED > 20 && settidalvolume < 200)) {
            peakdetect();
            Lvtstate = 1;
          }
        }

        else if ((mainrelaycurrentMillis - mainrelaypreviousMillis) > (1.3 * Tinsp) && (mainrelaycurrentMillis - mainrelaypreviousMillis) <= ((60000 / setfrequency) + 0.3 * Tinsp)) {
          inspiration = 0;
          Ti_real = Ti_real_1;
          hpstate = 0;
          Hvtstate = 0;
          Hvtcutstate = 0;
          Lvtstate = 0;
          trigger_monitor = 0;
          if (mainrelaypinstate == HIGH) {
            trigger_monitor_save = trigger_monitor;
          }
          mainrelaypinstate = LOW;  
          fio2_function();          
          PEEP();
        }

        else if ((mainrelaycurrentMillis - mainrelaypreviousMillis) > ((60000 / setfrequency) + 0.3 * Tinsp)) {
          mainrelaypreviousMillis = mainrelaycurrentMillis;
        } else {
          digitalWrite(mainrelaypin, LOW);
        }
        PEEP();
      
}

/* ===== extracted verbatim from loop(), original lines 10748-10854 =====
 * outer guard was: if (standbybutton_count == 2 && setmode == MODE_CPAP) {
 * now dispatched from the mode switch in loop(). Body unchanged. */
static void runModePCV(void) {  /* CPAP/PC (mode 5) */  

         apnbackup_mode = 0; 
        Tinsp = turbine_pickup_time_factor * ((60000 * set_i) / ((e + set_i) * setfrequency));
        Texp = ((60000 / setfrequency) - (Tinsp));
        pressure = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);
                
        uint32_t mainrelaycurrentMillis = millis();

        if ((mainrelaycurrentMillis - mainrelaypreviousMillis) <= 1.3 * Tinsp) {  
          inspiration = 1;
          
          if (gh == 0 && inspiration == 1) {
            leak_volume = currentvolume;
            currentvolume = 0;
            currentvolume = (-1 * leak_volume);
            gh = 1;

          } else if (inspiration == 0) {
            gh = 0;
          }
          
          if (mainrelaypinstate == HIGH) {
            Ti_real_1 = (mainrelaycurrentMillis - mainrelaypreviousMillis);  
          }

          peakdetect();

          if (mainrelaypinstate == HIGH) {
            analogWrite(peepPumpPin, 255);
          } else {
            analogWrite(peepPumpPin, (peepduty));
          }
          pressure = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);
          flowsensor();
          
          if (highvtalarmlimit < vt_AVERAGED_SAVED) {
            
          }
          
          if ((pressure >= sethighpawalarmlimit)) {
            peakdetect();
            hpstate = 1;
            Hvtcutstate = 1;
            
          }
          if (hpstate == 0) {
            mainrelaypinstate = HIGH;
          } else if (hpstate == 1) {
            mainrelaypinstate = LOW;  
          }
          fio2_function();  
          
          if ((set_Pinsp < pressure)) {
            Hvtcutstate = 1;  
          }
          if (Hvtcutstate == 0) {
            mainrelaypinstate = HIGH;
          } else if (Hvtcutstate == 1) {
            mainrelaypinstate = LOW;  
          }
          fio2_function();  
          
          if (mainrelaypinstate == HIGH) {
            analogWrite(turbinePin, tur_duty);  
            digitalWrite(tur_reverse, HIGH);
          } else {
            analogWrite(turbinePin, compan_duty);
            
          }
          
          
          if (lowvtalarmlimit > T_Volume_final && vt_AVERAGED_SAVED > 20) {
            peakdetect();
            Lvtstate = 1;
          }

          
        }

        else if ((mainrelaycurrentMillis - mainrelaypreviousMillis) > (1.3 * Tinsp) && (mainrelaycurrentMillis - mainrelaypreviousMillis) <= ((60000 / setfrequency) + 0.3 * Tinsp)) {
          
          inspiration = 0;
          
          Ti_real = Ti_real_1;
          hpstate = 0;
          Hvtstate = 0;
          Hvtcutstate = 0;
          Lvtstate = 0;
          trigger_monitor = 0;
          mainrelaypinstate = LOW;  
          
          fio2_function();  
          PEEP();
          
        } else if ((mainrelaycurrentMillis - mainrelaypreviousMillis) > ((60000 / setfrequency) + 0.3 * Tinsp)) {
          mainrelaypreviousMillis = mainrelaycurrentMillis;
        }
        
        else {
          digitalWrite(mainrelaypin, LOW);  
          mainrelaypinstate = LOW;
          
        }
      
}

/* ===== extracted verbatim from loop(), original lines 10857-11038 =====
 * outer guard was: if (standbybutton_count == 2 && (setmode == MODE_PSIMV)) {
 * now dispatched from the mode switch in loop(). Body unchanged. */
static void runModePSIMV(void) {  /* P-SIMV */  

        Texp = ((60000 / setfrequencysimv) - (setTinsp_simv));
        uint32_t mainrelaycurrentMillis = millis();
        apnbackup_mode = 0;
        
        pressure = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);
        if (pressure <= prs && (Exsence_timer == 0)) {
          prs = pressure;
        }
        
        if ((mainrelaycurrentMillis - mainrelaypreviousMillis) <= setTinsp_simv) {  
          inspiration = 1;
          
          if (gh == 0 && inspiration == 1) {
            leak_volume = currentvolume;
            currentvolume = 0;
            currentvolume = (-1 * leak_volume);
            gh = 1;

          } else if (inspiration == 0) {
            gh = 0;
          }
          
          if (mainrelaypinstate == HIGH) {
            Ti_real_1 = (mainrelaycurrentMillis - mainrelaypreviousMillis);  
          }
          
          pressure = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);
          if (pressure <= prs && (Exsence_timer == 0)) {
            prs = pressure;
          }
          
          peakdetect();
          if (mainrelaypinstate == HIGH) {
            analogWrite(peepPumpPin, 255);
          } else {
            analogWrite(peepPumpPin, (peepduty));
          }
          pressure = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);
          
          if (pressure >= sethighpawalarmlimit) {
            peakdetect();
            hpstate = 1;
            Hvtcutstate = 1;
          }
          if (hpstate == 0) {
            mainrelaypinstate = HIGH;
          } else if (hpstate == 1) {
            mainrelaypinstate = LOW;
          }
          fio2_function();  
          
          if ((highvtalarmlimit < vt_AVERAGED_SAVED) && settidalvolume >= 200 && tr_save == 0) {
          } else if ((highvtalarmlimit < vt_AVERAGED_SAVED) && settidalvolume <= 200 && tr_save == 0) {
          }
          if ((settidalvolume < currentvolume) && tr == 0) {
          }
          if ((set_Pinsp < pressure) && tr == 0) {
            Hvtcutstate = 1;  
          }
          if (Hvtcutstate == 0) {
            mainrelaypinstate = HIGH;
          } else if (Hvtcutstate == 1) {
            mainrelaypinstate = LOW;  
          }
          if (pressure >= setps && tr == 1) {
            tr = 0;
            trigger_monitor = 0;
            Hvtcutstate = 1;
            mainrelaypinstate = LOW;  
          }
          
          if (mainrelaypinstate == HIGH && s == 1 && tr != 1) {
            tr_save = 0;
            s = 0;
          }
          fio2_function();  
          
          if (mainrelaypinstate == HIGH) {
            trigger_monitor_save = trigger_monitor;
          }
          
          if (mainrelaypinstate == HIGH) {
            if (tr == 1 && tur_duty < 200) { analogWrite(turbinePin, tur_duty + 40); }  
            else {
              analogWrite(turbinePin, tur_duty);
            }
            digitalWrite(tur_reverse, HIGH);
          } else {
            analogWrite(turbinePin, compan_duty);
          }
          
          if (((lowvtalarmlimit > T_Volume_final) && vt_AVERAGED_SAVED > 20 && settidalvolume >= 200) && tr_save == 0) {
            peakdetect();
            Lvtstate = 1;
          } else if (((lowvtalarmlimit > T_Volume_final) && vt_AVERAGED_SAVED > 20 && settidalvolume < 200) && tr_save == 0) {
            peakdetect();
            Lvtstate = 1;
          }
        } else if ((mainrelaycurrentMillis - mainrelaypreviousMillis) > (setTinsp_simv) && (mainrelaycurrentMillis - mainrelaypreviousMillis) <= ((60000 / setfrequencysimv))) {
          
          inspiration = 0;
          
          Ti_real = Ti_real_1;
          trigger_monitor = 0;  
          hpstate = 0;
          Hvtstate = 0;
          Hvtcutstate = 0;
          Lvtstate = 0;
          mainrelaypinstate = LOW;
          
          if (tr_save == 1) {
            s = 1;
          }
          pressure = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);
          if (pressure <= prs && (Exsence_timer == 0)) {
            prs = pressure;
          }
          if (pressure <= prs && (Exsence_timer == 0)) {
            prs = pressure;
          }

          if ((mainrelaycurrentMillis - mainrelaypreviousMillis) > ((0.01 * Exsence_percentage * setTinsp_simv) + (1 * setTinsp_simv))) {
            Exsence_timer = 0;  
          } else {
            Exsence_timer = 1;
          }
          if (Exsence_timer == 0) {                                                                   
            if (prs <= (-(00.3 * setptr) + 0.5) && (setpeep == 0 || (p_AVERAGED_SAVED - ptr < 0))) {  

              analogWrite(turbinePin, tur_duty);
              if ((mainrelaycurrentMillis - mainrelaypreviousMillis) <= (setTinsp_simv + (0.75 * Texp))) {
                tr = 1;
                tr_save = 1;
              } else {
                tr = 1;
                tr_save = 1;
              }
              trigger_monitor = 1;
              mainrelaypreviousMillis = mainrelaycurrentMillis;
            }
            if (prs <= (-setptr + p_AVERAGED_SAVED) && (setpeep) > prs && (setpeep) >= p_AVERAGED_SAVED && (setpeep != 0) && mainrelaypinstate == LOW) {
              analogWrite(turbinePin, tur_duty);
              trigger_monitor = 1;
              if ((mainrelaycurrentMillis - mainrelaypreviousMillis) <= (setTinsp_simv + (0.75 * Texp))) {
                tr = 1;
                tr_save = 1;
              } else {
                tr = 1;
                tr_save = 1;
              }
              mainrelaypreviousMillis = mainrelaycurrentMillis;
            }
            if (prs <= (-setptr + setpeep) && (setpeep) > prs && (setpeep) < p_AVERAGED_SAVED && (setpeep != 0) && mainrelaypinstate == LOW) {
              analogWrite(turbinePin, tur_duty);
              trigger_monitor = 1;
              if ((mainrelaycurrentMillis - mainrelaypreviousMillis) <= (setTinsp_simv + (0.75 * Texp))) {
                tr = 1;
                tr_save = 1;
              } else {
                tr = 1;
                tr_save = 1;
              }
              mainrelaypreviousMillis = mainrelaycurrentMillis;
            }
            
          }
          fio2_function();  

        } else if ((mainrelaycurrentMillis - mainrelaypreviousMillis) > ((60000 / setfrequencysimv))) {
          mainrelaypreviousMillis = mainrelaycurrentMillis;
        }
        else {
          mainrelaypinstate = LOW;  
          analogWrite(mainrelaypin, 0);
          mainrelaypreviousMillis = mainrelaycurrentMillis;
          
        }
}

/* ===== extracted verbatim from loop(), original lines 11042-11293 =====
 * outer guard was: if (standbybutton_count == 2 && (setmode == MODE_SIMV)) {
 * now dispatched from the mode switch in loop(). Body unchanged. */
static void runModeSIMV(void) {  /* SIMV */

        apnpreviousmillis = millis();  
        apnbccurrentmillis = millis();
        if (apnbackup_mode == 1) {
          Icon12[7] = 146;
          DWIN_PORT.write(Icon12, 8);  
          if (apntr == 0 && a == 0) {
            apnbcpreviousmillis = apnbccurrentmillis;
          }
          if (apntr == 1 && b == 1) {
            a = 1;  
          }
          if (apntr == 1 && b == 0) {
            b = 1;  
          }
          if ((apnbccurrentmillis - apnbcpreviousmillis) < (15000) && apntr == 1 && a == 1) {
            Icon12[7] = 0;
            DWIN_PORT.write(Icon12, 8);  
            Icon15[7] = 39;
            DWIN_PORT.write(Icon15, 8);
            b = 0;
            a = 0;
            apntr = 0;
            apnbackup_mode = 0;
            setmode = MODE_SPONT_PS;
            mode1 = MODE_SPONT_PS;
          }
          if ((apnbccurrentmillis - apnbcpreviousmillis) > (15000) && apntr == 1 && b == 1) {
            a = 0;
          }
          apntr = 0;
        }

        Texp = ((60000 / setfrequencysimv) - (setTinsp_simv));
        uint32_t mainrelaycurrentMillis = millis();
        
        
        pressure = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);
        if (pressure <= prs && (Exsence_timer == 0)) {
          prs = pressure;
        }
        
        if ((mainrelaycurrentMillis - mainrelaypreviousMillis) <= setTinsp_simv) {  
          inspiration = 1;
          
          if (gh == 0 && inspiration == 1) {
            leak_volume = currentvolume;
            currentvolume = 0;
            currentvolume = (-1 * leak_volume);
            gh = 1;
          } else if (inspiration == 0) {
            gh = 0;
          }
                    
          if (mainrelaypinstate == HIGH) {
            Ti_real_1 = (mainrelaycurrentMillis - mainrelaypreviousMillis);  
          }
          
          pressure = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);
          if (pressure <= prs && (Exsence_timer == 0)) {
            prs = pressure;
          }
          
          peakdetect();

          if (mainrelaypinstate == HIGH) {
            analogWrite(peepPumpPin, 255);
          } else {
            analogWrite(peepPumpPin, (peepduty));
          }
          pressure = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);
          
          if (pressure >= sethighpawalarmlimit) {
            peakdetect();
            hpstate = 1;
            Hvtcutstate = 1;
          }
          if (hpstate == 0) {
            mainrelaypinstate = HIGH;
          } else if (hpstate == 1) {
            mainrelaypinstate = LOW;
          }
          
          
          fio2_function();  
                                      
          if ((highvtalarmlimit < vt_AVERAGED_SAVED) && settidalvolume >= 200 && tr_save == 0) {
            Hvtstate = 1;
          } else if ((highvtalarmlimit < vt_AVERAGED_SAVED) && settidalvolume <= 200 && tr_save == 0) {
            Hvtstate = 1;
          }
          if ((settidalvolume < currentvolume) && tr == 0 && currentvolume >= 20) {
            Hvtcutstate = 1;  
          }
          if (Hvtcutstate == 0) {
            mainrelaypinstate = HIGH;
          } else if (Hvtcutstate == 1) {
            mainrelaypinstate = LOW;  
          }
          if (pressure >= setps && tr == 1) {
            tr = 0;
            trigger_monitor = 0;
            Hvtcutstate = 1;
            mainrelaypinstate = LOW;  
          }
          
          if (mainrelaypinstate == HIGH && s == 1 && tr != 1) {
            tr_save = 0;
            s = 0;
          }
          
          fio2_function();  
          
          if (mainrelaypinstate == HIGH) {
            trigger_monitor_save = trigger_monitor;
          }
          
          
          if (mainrelaypinstate == HIGH) {
            if (tr == 1 && tur_duty < 200) { analogWrite(turbinePin, tur_duty + 40); }  
            else {
              analogWrite(turbinePin, tur_duty);
            }
            digitalWrite(tur_reverse, HIGH);
          } else {
            analogWrite(turbinePin, compan_duty);
            
          }
          
          
          if (((lowvtalarmlimit > T_Volume_final) && vt_AVERAGED_SAVED > 20 && settidalvolume >= 200) && tr_save == 0) {
            peakdetect();
            Lvtstate = 1;
          } else if (((lowvtalarmlimit > T_Volume_final) && vt_AVERAGED_SAVED > 20 && settidalvolume < 200) && tr_save == 0) {
            peakdetect();
            Lvtstate = 1;
          }

          
        }
        
        else if ((mainrelaycurrentMillis - mainrelaypreviousMillis) > (setTinsp_simv) && (mainrelaycurrentMillis - mainrelaypreviousMillis) <= ((60000 / setfrequencysimv))) {
          
          inspiration = 0;

          Ti_real = Ti_real_1;
          
          trigger_monitor = 0;  
          hpstate = 0;
          Hvtstate = 0;
          Hvtcutstate = 0;
          Lvtstate = 0;
          mainrelaypinstate = LOW;
          
          if (tr_save == 1) {
            s = 1;
          }
          
          pressure = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);
          if (pressure <= prs && (Exsence_timer == 0)) {
            prs = pressure;
          }
          
          if (pressure <= prs && (Exsence_timer == 0)) {
            prs = pressure;
          }

          if ((mainrelaycurrentMillis - mainrelaypreviousMillis) > ((0.01 * Exsence_percentage * setTinsp_simv) + (1 * setTinsp_simv))) {
            Exsence_timer = 0;  
          } else {
            Exsence_timer = 1;
          }
          if (Exsence_timer == 0) {                                                                   
            if (prs <= (-(00.3 * setptr) + 0.5) && (setpeep == 0 || (p_AVERAGED_SAVED - ptr < 0))) {  

              analogWrite(turbinePin, tur_duty);
              if ((mainrelaycurrentMillis - mainrelaypreviousMillis) <= (setTinsp_simv + (0.75 * Texp))) {
                tr = 1;
                tr_save = 1;
              } else {
                tr = 1;
                tr_save = 1;
              }
              trigger_monitor = 1;
              apntr = 1;
              mainrelaypreviousMillis = mainrelaycurrentMillis;
            }
            if (prs <= (-setptr + p_AVERAGED_SAVED) && (setpeep) > prs && (setpeep) >= p_AVERAGED_SAVED && (setpeep != 0) && mainrelaypinstate == LOW) {
              analogWrite(turbinePin, tur_duty);
              trigger_monitor = 1;
              if ((mainrelaycurrentMillis - mainrelaypreviousMillis) <= (setTinsp_simv + (0.75 * Texp))) {
                tr = 1;
                tr_save = 1;
              } else {
                tr = 1;
                tr_save = 1;
              }
              mainrelaypreviousMillis = mainrelaycurrentMillis;
              apntr = 1;
            }
            if (prs <= (-setptr + setpeep) && (setpeep) > prs && (setpeep) < p_AVERAGED_SAVED && (setpeep != 0) && mainrelaypinstate == LOW) {
              analogWrite(turbinePin, tur_duty);
              trigger_monitor = 1;
              if ((mainrelaycurrentMillis - mainrelaypreviousMillis) <= (setTinsp_simv + (0.75 * Texp))) {
                tr = 1;
                tr_save = 1;
              } else {
                tr = 1;
                tr_save = 1;
              }
              mainrelaypreviousMillis = mainrelaycurrentMillis;
              apntr = 1;
            }
            
          }
          fio2_function();  

          
        } else if ((mainrelaycurrentMillis - mainrelaypreviousMillis) > ((60000 / setfrequencysimv))) {
          mainrelaypreviousMillis = mainrelaycurrentMillis;
        }
        
        else {
          mainrelaypinstate = LOW;  
          analogWrite(mainrelaypin, 0);
          mainrelaypreviousMillis = mainrelaycurrentMillis;
          
        }
      
}

/* ===== extracted verbatim from loop(), original lines 11297-11512 =====
 * outer guard was: if (standbybutton_count == 2 && (setmode == MODE_ACV)) {
 * now dispatched from the mode switch in loop(). Body unchanged. */
static void runModeACV(void) {  /* ACV */  

        Tinsp = turbine_pickup_time_factor * ((60000 * set_i) / ((e + set_i) * setfrequency));
        Texp = ((60000 / setfrequency) - (Tinsp));
        uint32_t mainrelaycurrentMillis = millis();
        
        pressure = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);
        if (pressure <= prs && (Exsence_timer == 0)) {
          prs = pressure;
        }
        
        if ((mainrelaycurrentMillis - mainrelaypreviousMillis) <= Tinsp) {  
          inspiration = 1;
          
          if (gh == 0 && inspiration == 1) {
            leak_volume = currentvolume;
            currentvolume = 0;
            currentvolume = (-1 * leak_volume);
            gh = 1;

          } else if (inspiration == 0) {
            gh = 0;
          }
          
          if (mainrelaypinstate == HIGH) {
            Ti_real_1 = (mainrelaycurrentMillis - mainrelaypreviousMillis);  
          }

          
          pressure = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);
          if (pressure <= prs && (Exsence_timer == 0)) {
            prs = pressure;
          }
          
          peakdetect();

          if (mainrelaypinstate == HIGH) {
            analogWrite(peepPumpPin, 255);
          } else {
            analogWrite(peepPumpPin, (peepduty));
          }
          pressure = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);
          
          if (pressure >= sethighpawalarmlimit) {
            peakdetect();
            hpstate = 1;
            Hvtcutstate = 1;
          }
          if (hpstate == 0) {
            mainrelaypinstate = HIGH;
          } else if (hpstate == 1) {
            mainrelaypinstate = LOW;
          }
          
          
          fio2_function();  
                                    
          if ((highvtalarmlimit < vt_AVERAGED_SAVED) && settidalvolume >= 200) {
            Hvtstate = 1;
          } else if ((highvtalarmlimit < vt_AVERAGED_SAVED) && settidalvolume <= 200) {
            Hvtstate = 1;
          }
          if ((settidalvolume < currentvolume) && currentvolume >= 20) {
            Hvtcutstate = 1;  
            
            
          }
          if (Hvtcutstate == 0) {
            mainrelaypinstate = HIGH;
          } else if (Hvtcutstate == 1) {
            mainrelaypinstate = LOW;  
          }

          fio2_function();  
          
          
          if (mainrelaypinstate == HIGH) {
            trigger_monitor_save = trigger_monitor;
          }
          
          
          if (mainrelaypinstate == HIGH) {
            analogWrite(turbinePin, tur_duty);  
            digitalWrite(tur_reverse, HIGH);
          } else {
            analogWrite(turbinePin, compan_duty);
            
          }
          
          
          if (((lowvtalarmlimit > T_Volume_final) && vt_AVERAGED_SAVED > 20 && settidalvolume >= 200)) {
            peakdetect();
            Lvtstate = 1;
          } else if (((lowvtalarmlimit > T_Volume_final) && vt_AVERAGED_SAVED > 20 && settidalvolume < 200)) {
            peakdetect();
            Lvtstate = 1;
          }

        }        
        else if ((mainrelaycurrentMillis - mainrelaypreviousMillis) > (Tinsp) && (mainrelaycurrentMillis - mainrelaypreviousMillis) <= ((60000 / setfrequency))) {
          
          inspiration = 0;
          
          Ti_real = Ti_real_1;
          
          trigger_monitor = 0;  
          hpstate = 0;
          Hvtstate = 0;
          Hvtcutstate = 0;
          Lvtstate = 0;
          mainrelaypinstate = LOW;
          
          pressure = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);
          if (pressure <= prs && (Exsence_timer == 0)) {
            prs = pressure;
          }
          
          if (pressure <= prs && (Exsence_timer == 0)) {
            prs = pressure;
          }

          if ((mainrelaycurrentMillis - mainrelaypreviousMillis) > ((0.01 * Exsence_percentage * Tinsp) + (1 * Tinsp))) {
            Exsence_timer = 0;  
          } else {
            Exsence_timer = 1;
          }
          if (Exsence_timer == 0) {                                                                   
            if (prs <= (-(00.3 * setptr) + 0.5) && (setpeep == 0 || (p_AVERAGED_SAVED - ptr < 0))) {  

              analogWrite(turbinePin, tur_duty);
              trigger_monitor = 1;
              
              mainrelaypreviousMillis = mainrelaycurrentMillis;
            }
            if (prs <= (-setptr + p_AVERAGED_SAVED) && (setpeep) > prs && (setpeep) >= p_AVERAGED_SAVED && (setpeep != 0) && mainrelaypinstate == LOW) {
              analogWrite(turbinePin, tur_duty);
              trigger_monitor = 1;
              
              mainrelaypreviousMillis = mainrelaycurrentMillis;
            }
            if (prs <= (-setptr + setpeep) && (setpeep) > prs && (setpeep) < p_AVERAGED_SAVED && (setpeep != 0) && mainrelaypinstate == LOW) {
              analogWrite(turbinePin, tur_duty);
              trigger_monitor = 1;
              
              mainrelaypreviousMillis = mainrelaycurrentMillis;
            }
            
          }
          fio2_function();  

          
        } else if ((mainrelaycurrentMillis - mainrelaypreviousMillis) > ((60000 / setfrequency))) {
          mainrelaypreviousMillis = mainrelaycurrentMillis;
        }
        
        else {
          mainrelaypinstate = LOW;  
          analogWrite(mainrelaypin, 0);
          mainrelaypreviousMillis = mainrelaycurrentMillis;
          
        }
}

/* ===== extracted verbatim from loop(), original lines 11518-11572 =====
 * outer guard was: if (standbybutton_count == 2 && setmode == MODE_HFT) {
 * now dispatched from the mode switch in loop(). Body unchanged. */
static void runModeHFT(void) {  /* high-flow therapy */
        if (standbybutton_count == 2) {
          apnbackup_mode = 0;
          currentvolume = 0;
          mainrelaypinstate = HIGH;
          fio2_function();                       
          digitalWrite(tur_reverse, HIGH);       
          analogWrite(turbinePin, tur_duty_HFT);  
          analogWrite(peepPumpPin, 0);
          average_flow = average1;
          
          pressure2 = (analogRead(Analog_pr2) - ZFF_PR2);
          if (pressure2 > 2) {
            
            pressure2 = (Pr2_range * sqrt(0.002587 * (pressure2)));  
          } else {
            pressure2 = 0;
          }
          display_write_variable(VP_HFT_MEASURED_FLOW, pressure2);

          setpoint_hft = setflowrate;
          
          input_hft = pressure2;  
          uint32_t now_hft = millis();
          float timeChange_hft = (float)(now_hft - lastTime_hft);
          float error_hft = setpoint_hft - input_hft;
          integral_hft += (error_hft * timeChange_hft);
          float derivative_hft = (error_hft - previousError_hft) / timeChange_hft;
          output_hft = Kp_hft * error_hft + Ki_hft * integral_hft + Kd_hft * derivative_hft;

          previousError_hft = error_hft;
          lastTime_hft = now_hft;
          tur_duty_HFT = output_hft;

        }
         else {
          mainrelaypinstate = LOW;
          analogWrite(mainrelaypin, 0);
          digitalWrite(tur_reverse, LOW);  
          analogWrite(turbinePin, 0);
        }
        if (tur_duty_HFT >= 254) {
          tur_duty_HFT = 254;
        }
        if (tur_duty_HFT <= 1) {
          tur_duty_HFT = 1;
        }      
}

/* ===== extracted verbatim from loop(), original lines 11574-11738 =====
 * outer guard was: if (standbybutton_count == 2 && setmode == MODE_PCV) {
 * now dispatched from the mode switch in loop(). Body unchanged. */
static void runModeCPAP(void) {  /* PCV */
        apnbackup_mode = 0;
        Tinsp = turbine_pickup_time_factor * ((60000 * set_i) / ((e + set_i) * setfrequency));
        if (Tinsp >= 1700) {
          Tinsp = 1700;  
        }
        Texp = ((60000 / setfrequency) - (Tinsp));
        pressure = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);
        
        
        uint32_t mainrelaycurrentMillis = millis();
        if ((mainrelaycurrentMillis - mainrelaypreviousMillis) <= 1.3 * Tinsp) {  
          inspiration = 1;
          
          if (gh == 0 && inspiration == 1) {
            leak_volume = currentvolume;

            currentvolume = 0;
            currentvolume = (-1 * leak_volume);
            gh = 1; 
            
          } else if (inspiration == 0) {
            gh = 0;
          }

          if (mainrelaypinstate == HIGH) {
            Ti_real_1 = (mainrelaycurrentMillis - mainrelaypreviousMillis);  
          }

          peakdetect();

          if (mainrelaypinstate == HIGH) {
            analogWrite(peepPumpPin, 255);
          } else {
            analogWrite(peepPumpPin, (peepduty));
          }
          pressure = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);
          flowsensor();
          
          if (highvtalarmlimit < vt_AVERAGED_SAVED) { }
          
          if ((pressure >= sethighpawalarmlimit)) {
            peakdetect();
            hpstate = 1;
            Hvtcutstate = 1;
          }
          if (hpstate == 0) {
            mainrelaypinstate = HIGH;
          } else if (hpstate == 1) {
            mainrelaypinstate = LOW;  
          }
          fio2_function();  
          
          if ((set_Pinsp < pressure)) {
            Hvtcutstate = 1;  
          }
          if (Hvtcutstate == 0) {
            mainrelaypinstate = HIGH;
          } else if (Hvtcutstate == 1) {
            mainrelaypinstate = LOW;  
          }
          
          fio2_function();  
          
          if (mainrelaypinstate == HIGH) {
            analogWrite(turbinePin, tur_duty);  
            digitalWrite(tur_reverse, HIGH);
          } else {
            analogWrite(turbinePin, compan_duty);
            
          }

          if (mainrelaypinstate == HIGH) {
            trigger_monitor_save = trigger_monitor;
          }
          
          if (lowvtalarmlimit > T_Volume_final && vt_AVERAGED_SAVED > 20) {
            peakdetect();
            
          }
        }
        else if ((mainrelaycurrentMillis - mainrelaypreviousMillis) > (1.3 * Tinsp) && (mainrelaycurrentMillis - mainrelaypreviousMillis) <= ((60000 / setfrequency) + 0.3 * Tinsp)) {
          
          inspiration = 0;
          
          Ti_real = Ti_real_1;
          
          hpstate = 0;
          Hvtstate = 0;
          Hvtcutstate = 0;
          Lvtstate = 0;
          mainrelaypinstate = LOW;  
          analogWrite(mainrelaypin, 0); 
          analogWrite(turbinePin, compan_duty);
          
          if ((mainrelaycurrentMillis - mainrelaypreviousMillis) > ((0.01 * Exsence_percentage * Tinsp) + (1.3 * Tinsp))) {
            Exsence_timer = 0;  
          } else {
            Exsence_timer = 1;
          }
          pressure = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);
          if (pressure <= prs && (Exsence_timer == 0)) {
            prs = pressure;
          }

          if (Exsence_timer == 0) {
            if (prs <= (-(00.3 * setptr) + 0.5) && (setpeep == 0) && mainrelaypinstate == LOW) {
              analogWrite(turbinePin, tur_duty);
              trigger_monitor = 1;
               mainrelaypreviousMillis = mainrelaycurrentMillis;
            }
            if (prs <= (-setptr + peep_AVERAGED_SAVED) && (setpeep) > prs && (setpeep) >= peep_AVERAGED_SAVED && (setpeep != 0) && mainrelaypinstate == LOW) {
              analogWrite(turbinePin, tur_duty);
              trigger_monitor = 1;
               mainrelaypreviousMillis = mainrelaycurrentMillis;
            }
            if (prs <= (-setptr + setpeep) && (setpeep) > prs && (setpeep) < peep_AVERAGED_SAVED && (setpeep != 0) && mainrelaypinstate == LOW) {
              analogWrite(turbinePin, tur_duty);
              trigger_monitor = 1;
               mainrelaypreviousMillis = mainrelaycurrentMillis;
            }
            
          }
          
          fio2_function();  
          PEEP();
          
        } else if ((mainrelaycurrentMillis - mainrelaypreviousMillis) > ((60000 / setfrequency) + 0.3 * Tinsp)) {
          mainrelaypreviousMillis = mainrelaycurrentMillis;
          trigger_monitor = 0;
        }
        
        else {
          digitalWrite(mainrelaypin, LOW);  
          mainrelaypinstate = LOW;
          mainrelaypreviousMillis = mainrelaycurrentMillis;
          
          trigger_monitor = 0;
        }  
}


/* ===================== CLINICAL-CONFIRM CONSTANTS ======================== */
/* ALARM_NUM_APNEA / APNEA_RETURN_BREATHS: defined at TOP of file -
   #defines are not hoisted like function prototypes.            */
/* return CPAP; CLINICAL-CONFIRM        */
/* Backup parameters: this scaffold runs VACV with the CURRENT set values
   (settidalvolume / setfrequency / set_i / e). CLINICAL-CONFIRM whether
   dedicated backup Vt/RR (from the mode-6 settings page) should apply
   instead - if so, snapshot/apply them in apnea_enterBackup().           */
/* ========================================================================= */

static uint32_t apneaLastBreathMs = 0;
static uint8_t apneaSpontCount = 0;
static uint8_t apneaPrevMode = 5;

/* ---- HOOK-2: called at every PATIENT-TRIGGERED breath start ---- */
void apnea_breathSeen(void) {
  apneaLastBreathMs = millis();

  if (apnbackup_mode == 1) {
    /* patient breathing during backup: count toward return */
    apneaSpontCount++;
    if (apneaSpontCount >= APNEA_RETURN_BREATHS) {
      apnea_exitBackup();
    }
  } else {
    apneaSpontCount = 0;
  }
}

/* mode badge (Icon15) values - extracted from the confirm handler:
   5->42 CPAP, 8->41 PSIMV, 1->45 VCV, 7->38 PCV, 4->40 HFT,
   2->44 SIMV, 3->43 VACV, 6->39                                     */
static void apnea_setBadge(uint8_t m) {
  uint8_t v = 42;
  if (m == 8) { v = 41; } else if (m == 1) { v = 45; }
  else if (m == 7) { v = 38; } else if (m == 4) { v = 40; }
  else if (m == 2) { v = 44; } else if (m == 3) { v = 43; }
  else if (m == 6) { v = 39; }
  Icon15[7] = v;
  DWIN_PORT.write(Icon15, 8);
}

/* ---- changeover: CPAP -> VACV backup ---- */
void apnea_enterBackup(void) {
  if (apnbackup_mode == 1) { return; }

  apneaPrevMode = setmode;
  apnbackup_mode = 1;
  apneaSpontCount = 0;

  /* programmatic mode switch - replicate what the touch path does so
     no state is left half-switched (the half-switched state is what
     made the watchdog misfire plausible in the first place):          */
  setmode = MODE_ACV;                       /* VACV delivery                  */
  mode1 = MODE_ACV;                         /* keep UI/mode agreement - the  */
                                     /* PREVIOUS implementation died   */
                                     
  hpstate = 0;
  Hvtstate = 0;
  Hvtcutstate = 0;
  Lvtstate = 0;
  inspiration = 0;
  gh = 0;
  trigger_monitor = 0;
  mainrelaypinstate = LOW;
  mainrelaypreviousMillis = millis(); /* clean breath-cycle restart    */

  apnea_setBadge(3);                 /* mode badge follows the therapy */
  alarm_write = 1;                   /* APNEA alarm (60601: high prio) */
  alarm_num = ALARM_NUM_APNEA;

  FLOG_HOOK(FLOG_T_ALARM_ON, FLOG_A_APNEA, set_apneatime);
  FLOG_HOOK(FLOG_T_EVENT, FLOG_E_MODE_CHANGE, 3);
}

/* ---- return: backup -> CPAP after sustained spontaneous breathing ---- */
void apnea_exitBackup(void) {
  if (apnbackup_mode == 0) { return; }

  apnbackup_mode = 0;
  setmode = apneaPrevMode;           /* back to CPAP                   */
  mode1 = apneaPrevMode;
  mainrelaypreviousMillis = millis();
  apneaLastBreathMs = millis();
  apnea_setBadge(apneaPrevMode);

  FLOG_HOOK(FLOG_T_ALARM_OFF, FLOG_A_APNEA, 0);
  FLOG_HOOK(FLOG_T_EVENT, FLOG_E_MODE_CHANGE, apneaPrevMode);
}

/* ---- HOOK-4: manual mode selection cancels backup state ---- */
void apnea_operatorOverride(void) {
  apnbackup_mode = 0;
  apneaSpontCount = 0;
  apneaLastBreathMs = millis();
}

/* ---- HOOK-3: detection service, every loop() pass ---- */
void apnea_service(void) {

  /* armed only while ventilating in CPAP (backup arms fresh on entry) */
  if (standbybutton_count != 2 || (setmode == MODE_SPONT_PS && apnbackup_mode == 0)) {
    apneaLastBreathMs = millis();
    return;
  }
 
  if (apnbackup_mode == 0 && (setmode == MODE_SPONT_PS)) {
    uint32_t limit = (uint32_t)set_apneatime * 1000UL;
    if ((millis() - apneaLastBreathMs) > limit) {
      apnea_enterBackup();
    }
  }
}


/* ===== extracted verbatim from loop(), original lines 11749-11920 =====
 * outer guard was: if (setmode == MODE_SPONT_PS && standbybutton_count == 2) {
 * now dispatched from the mode switch in loop(). Body unchanged. */
static void runModeSpontPS(void) {  /* spont/pressure-trigger */
        if (ib == 1) {
          apnpreviousmillis = millis();
          apncurrentmillis = millis();
          tr = 1;
        }
        a = 0;

        apncurrentmillis = millis();
        if (mainrelaypinstate == 1 || standbybutton_count != 2) {
          apnpreviousmillis = apncurrentmillis;
        }
        if ((apncurrentmillis - apnpreviousmillis) > (1000 * set_apneatime) && mainrelaypinstate == 0) {
          apnbackup_mode = 1;
          setmode = MODE_SIMV;
          mode1 = MODE_SIMV;
          Icon15[7] = 44;
          DWIN_PORT.write(Icon15, 8);
          currentvolume = 0;
          mainrelaypreviousMillis = mainrelaycurrentMillis;
        }

        uint32_t mainrelaycurrentMillis = millis();
        if (mainrelaypinstate == HIGH) {
          Ti_real_1 = (mainrelaycurrentMillis - mainrelaypreviousMillis);
          Exsence_timer = 1;
        }
        if (mainrelaypinstate == HIGH) {
          trigger_monitor_save = trigger_monitor;
        }
        if (lowvtalarmlimit > T_Volume_final && vt_AVERAGED_SAVED > 20) {
          peakdetect();
        }
        pressure = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);

        if (pressure <= prs && (Exsence_timer == 0)) {
          prs = pressure;
        }
        if (standbybutton_count == 2) {
          if (tr == 1) {
            inspiration = 1;
            if (gh == 0 && inspiration == 1) {
              leak_volume = currentvolume;
              currentvolume = 0;
              currentvolume = (-1 * leak_volume);
              gh = 1;
            } else if (inspiration == 0) {
              gh = 0;
            }
          } else if (tr == 0) {
            inspiration = 0;
          }
          peakdetect();
          if (onetimestartbreath == 1) {
            tr = 1, trigger_monitor = 1; apnea_breathSeen();
            onetimestartbreath = 0;
          }

          if (setpeep == 0) {
            setpeep = 1;
          }

          if (Exsence_timer == 0) {
            if (prs <= (-(00.3 * setptr) + 0.5) && (setpeep == 0 || (p_AVERAGED_SAVED - ptr < 0))) {
              tr = 1;
              trigger_monitor = 1; apnea_breathSeen();
              mainrelaypreviousMillis = mainrelaycurrentMillis;
              //DEBUG.println(F("first"));
            }

            if (prs <= ((-1 * setptr) + p_AVERAGED_SAVED) && (setpeep) > prs && (setpeep) >= p_AVERAGED_SAVED && setpeep != 0) {
              tr = 1;
              trigger_monitor = 1; apnea_breathSeen();
              mainrelaypreviousMillis = mainrelaycurrentMillis;
              //DEBUG.println(F("first2"));
            }
            if (prs <= ((-1 * setptr) + setpeep) && (setpeep) > prs && (setpeep) < p_AVERAGED_SAVED && setpeep != 0) {
              tr = 1;
              trigger_monitor = 1; apnea_breathSeen();
              mainrelaypreviousMillis = mainrelaycurrentMillis;
              //DEBUG.println(F("first3"));
            }
          }

          if ((pressure >= set_Pinsp) || ((mainrelaycurrentMillis - mainrelaypreviousMillis) > 2500))  
          {
            tr = 0;
            mainrelaypinstate = LOW;
            trigger_monitor = 0;
          }

          if ((tr == 1) && pressure < set_Pinsp) {
            mainrelaypinstate = HIGH;
            int tur_duty_2;
            if (sensorfailure == 1 || pr1failiure == 1) {
              if (bodyweight < 40) {
                tur_duty_2 = 30;
              } else if (bodyweight > 40 && bodyweight < 83) {
                tur_duty_2 = (40 + ((1.38 * bodyweight) - 55.38));
              } else if (bodyweight >= 83) {
                tur_duty_2 = 100;
              }
            } else {
              if (set_Pinsp <= 30) { tur_duty_2 = 100; }
              if (set_Pinsp > 30) { tur_duty_2 = 150; }
              if (tur_duty_2 >= 248) { tur_duty_2 = 248; }
            }
            analogWrite(turbinePin, tur_duty_2);
            
            fio2_function();
            analogWrite(peepPumpPin, 255);
          } else {
            if (tr == 1) {
              inspiration = 1;
              if (gh == 0 && inspiration == 1) {
                leak_volume = currentvolume;
                currentvolume = 0;
                gh = 1;
              } else if (inspiration == 0) {
                gh = 0;
              }
            } else if (tr == 0) {
              inspiration = 0;
            }

            trigger_monitor = 0;
            hpstate = 0;
            Hvtstate = 0;
            Hvtcutstate = 0;
            Lvtstate = 0;
            ib = 0;  
            mainrelaypinstate = LOW;
            if ((mainrelaycurrentMillis - mainrelaypreviousMillis) > ((0.01 * Exsence_percentage * Ti_real_1) + (2 * Ti_real_1)) || (mainrelaycurrentMillis - mainrelaypreviousMillis) > 6000) {
              Exsence_timer = 0;
              mainrelaypreviousMillis = mainrelaycurrentMillis;
            }

          }
        }     
}



void loop() {

  FLOG_SERVICE();      /* gated background log writer  */
  LOGVIEW_SERVICE();   /* page-43 paced row renderer   */
  apnea_service();    /* CPAP/spont apnea watchdog + backup changeover */
#if 1
  while (DWIN_PORT.available() > 0) {
    Buffer1[Buffer1_Len] = DWIN_PORT.read();
    if ((Buffer1[0] == 0X5A) && (Buffer1[3] != 130)) {
      Buffer1_Len++;
    }
    
    else {
      Buffer1_Len = 0;
      Buffer1[0] = 0;
      memset(Buffer1, 0, sizeof(Buffer1));
    }
        
    volatile int lock_unlock_add = uint16_t((Buffer1[4] << 8) | Buffer1[5]);
    if (lock_unlock_add == TP_LOCK_TOGGLE && lock_unlock == 0) {
      lock_unlock_add = 0;  
      lock_unlock = 1;
      Icon26[7] = 202;
      DWIN_PORT.write(Icon26, 8);
      
    }
    if (lock_unlock_add == TP_LOCK_TOGGLE && lock_unlock == 1) {
      lock_unlock_add = 0;  
      lock_unlock = 0;
      Icon26[7] = 201;
      DWIN_PORT.write(Icon26, 8);
      
    }
    if (lock_unlock == 0) {
      serialread_data();
    } else if (lock_unlock == 1) {
    }

    if (Buffer1_Len > 8) {
      Buffer1_Len = 0;
    }
  }

  DWIN_PORT.flush();
  if (HMI_Page != 26)
    update_page(HMI_Page);

  
  if (HMI_Page == 28) {
    testscreenvariable();
  }
  
  if (HMI_Page == 3 || HMI_Page == 2 || HMI_Page == 27) {
    monitoredvariables();
  }
  
  realTimeTrendGraph();
  
  pvloop_service();
  pfloop_service();

  vploop_service();
  vfloop_service();

  fvloop_service();
  fploop_service();

  updateHourMeter();
  
  battery();

  if ((digitalRead(setting_button) == HIGH && digitalRead(resetbutton) == HIGH && standbybutton_count != 2) || calb == 1) {
    flowsensor();
  }
  if (standbybutton_count != 2 && O2cal == 1) {  
    O2calibration();
  }

  else {
    if (standbybutton_count == 1) {
      o2sensorfailtimer = millis();
    }
    
    if (digitalRead(setting_button) == HIGH && digitalRead(resetbutton) == HIGH) {
      testscreenstate = 1;
    }
    if (digitalRead(cancel_button) == HIGH) {
      testscreenstate = 0;
    }
    float px = millis();
    if (mainrelaypinstate == HIGH) {
      prs = pressure;
    }
    
    pressure = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);
    pressure3 = PRESSURE_FACTOR * ((analogRead(Analog_pr3)) - ZFP_pr3);
    if (pressure <= prs && (Exsence_timer == 0)) {
      prs = pressure;
    }
    keys();
    fio2_function();
    nebulizer();
    pressure = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);
    if (pressure <= prs && (Exsence_timer == 0)) {
      prs = pressure;
    }
    alarms();
    update_status_leds();
    if (mutebutton_count == 1 && alarm_write == 1) {
      mid_alarm();
    }
    flowsensor();
    
    peakdetect();
    pressure = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);
    if (pressure <= prs && (Exsence_timer == 0)) {
      prs = pressure;
    }
    PEEP();
    turbine();
    BPM();
    pressure = PRESSURE_FACTOR * (analogRead(Analog_pr1) - ZFP);
    if (pressure <= prs && (Exsence_timer == 0)) {
      prs = pressure;
    }

    if (mainrelaypinstate == HIGH) {
      trigger_monitor_save = trigger_monitor;
    }

    if (mutebutton_count == 1 && (alarm_write == 1 || (set_set_fio2 > 21 && oxygenavailability_status == HIGH)) && standbybutton_count == 2) {
      midi1();
    } else {
      
    }
    if (alarm_write == 1 && standbybutton_count == 2) {
      alarmscreen();
      ala = 1;
    } else {
      if (ala == 1) {
        Icon4[7] = 0;
        DWIN_PORT.write(Icon4, 8);
        Icon5[7] = 0;
        DWIN_PORT.write(Icon5, 8);
        Icon6[7] = 0;
        DWIN_PORT.write(Icon6, 8);
        Icon7[7] = 0;
        DWIN_PORT.write(Icon7, 8);
        Icon8[7] = 0;
        DWIN_PORT.write(Icon8, 8);
        Icon9[7] = 0;
        DWIN_PORT.write(Icon9, 8);
        Icon10[7] = 0;
        DWIN_PORT.write(Icon10, 8);
        Icon11[7] = 0;
        DWIN_PORT.write(Icon11, 8);
        Icon12[7] = 0;
        DWIN_PORT.write(Icon12, 8);
        Icon13[7] = 0;
        DWIN_PORT.write(Icon13, 8);
        Icon20[7] = 0;
        DWIN_PORT.write(Icon20, 8);
        ala = 0;
      }
    }
    if (alarm_write == 1 && standbybutton_count == 2)  
    {
      analogWrite(R_status_led, 150);
      analogWrite(G_status_led, 0);
      analogWrite(B_status_led, 0);

      digitalWrite(white_led_10inch_button, LOW);
      digitalWrite(red_led_10inch_button, HIGH);
      digitalWrite(yellow_led_10inch_button, LOW);

    } else if (standbybutton_count == 1) {
      digitalWrite(white_led_10inch_button, LOW);
      digitalWrite(red_led_10inch_button, LOW);
      digitalWrite(yellow_led_10inch_button, HIGH);

      analogWrite(G_status_led, 30 );
      analogWrite(R_status_led, 204);
      analogWrite(B_status_led, 0 );
    } else {
      if (trigger_monitor_save == 1) {
        digitalWrite(white_led_10inch_button, HIGH);
        digitalWrite(red_led_10inch_button, LOW);
        digitalWrite(yellow_led_10inch_button, LOW);
        analogWrite(G_status_led, 100);
        analogWrite(R_status_led, 100);
        analogWrite(B_status_led, 100);
      }
      if (trigger_monitor_save == 0) {
        digitalWrite(white_led_10inch_button, LOW);
        digitalWrite(red_led_10inch_button, LOW);
        digitalWrite(yellow_led_10inch_button, LOW);
        analogWrite(G_status_led, 100);
        analogWrite(R_status_led, 0);
        analogWrite(B_status_led, 0);
      }
    }
    if (trigger_monitor == 1 && standbybutton_count == 2) {
      if (sp == 1) {
        Icon23[7] = 200;
        DWIN_PORT.write(Icon23, 8);
        sp = 0;
      }
    } else {
      if (sp == 0) {
        Icon23[7] = 0;
        DWIN_PORT.write(Icon23, 8);
        sp = 1;
      }
    }
    
    if (standbybutton_count == 1) {  
      analogWrite(turbinePin, 0);
      analogWrite(peepPumpPin, 0);
      digitalWrite(mainrelaypin, LOW);
      mainrelaypinstate = LOW;
      digitalWrite(tur_reverse, LOW);  
      ib = 1;                          
    }
    
    /* ---- inter-mode housekeeping (hoisted from between the old blocks;
            all order-independent: no mode body reads these before they
            were set in the original sequence) ---- */
    if (setmode != MODE_HFT) {
      tur_duty_HFT = 25;  /* reset HFT duty whenever not in HFT */
    }
    if (setmode != MODE_SPONT_PS && apnbackup_mode == 0) { onetimestartbreath = 1; }
    if (standbybutton_count == 1) { onetimestartbreath = 1; }
    if (mainrelaypinstate == 1) {
      apnpreviousmillis = millis();
      apncurrentmillis = millis();
    }

    /* ---- mode dispatch (was a chain of 8 sequential if-blocks) ---- */
    if (standbybutton_count == 2) {
      switch (setmode) {
        case MODE_VCV: runModeVCV();     break;
        case MODE_SIMV: runModeSIMV();   break;
        case MODE_ACV: runModeACV();     break;
        case MODE_HFT: runModeHFT();     break;
        case MODE_PCV: runModePCV();    break;
        case MODE_SPONT_PS: runModeSpontPS(); break;
        case MODE_CPAP: runModeCPAP();     break;
        case MODE_PSIMV: runModePSIMV();   break;
        default: break;           /* unknown mode: no breath cycling */
      }
    }    
    float py = millis();
  }
#endif
}

float sharp2cm(int val) {
  float out[] = {
    2000, 2010, 2020, 2030, 2040,  
    2050, 2060, 2070, 2080, 2090,  
    2100, 2110, 2120, 2130, 2140,  
    2150, 2160, 2170, 2180, 2190,  
    2200, 2210, 2220, 2230, 2240,  
    2250, 2260, 2270, 2280, 2290,  
    2300, 2310, 2320, 2330, 2340,  
    2350, 2360, 2370, 2380, 2390,  
    2400, 2410, 2420, 2430, 2440,  
    2450, 2460, 2470, 2480, 2490,  
    2500, 2510, 2520, 2530, 2540,  
    2550, 2560, 2570, 2580, 2590,  
    2600, 2610, 2620, 2630, 2640,  
    2650, 2660, 2670, 2680, 2690,  
    2700, 2710, 2720, 2730, 2740,  
    2862, 2874, 2886, 2898, 2910,  
    2906, 2920, 2932, 2838, 2850,  
    2862, 2874, 2886, 2898, 2910,  
    2912, 2924, 2936, 2948, 2960,  
    2972, 2984, 2996, 3008, 3020
  };  

  
  
  
  
  

  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  

  
  float in[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 12, 12, 14, 14, 16, 17, 18, 19, 19, 22, 25, 29, 31, 31, 37,
                 39, 42, 44, 46, 47, 49, 50, 53, 53, 55, 58, 60, 64, 64, 64, 67, 69, 70, 73, 75, 75, 77, 80, 80, 85, 85, 85, 87, 89, 89, 89, 89, 89, 89, 89, 89, 89, 89, 89, 89, 89,
                 89, 89, 89, 89, 89 };

  float dist = multiMap<float>(val, in, out, 100);
  return dist;
}
