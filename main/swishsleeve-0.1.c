#include <plib.h>
#include <p32xxxx.h>
#include <string.h>

#include <stdio.h>


#include "imu.h"
//#include "battery_monitor.h"  DOES NOT WORK CURRENTLY
#include "delay.h"
#include "i2c_shared.h"
#include "kalman.h"
#include "lcd_16x2.h"
#include "math_helpers.h"


// Configuration Bit settings
//
// SYSCLK = 80 MHz (8MHz Crystal/ FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 40 MHz
// WDT OFF
#pragma config FPLLMUL = MUL_20
#pragma config FPLLIDIV = DIV_2
#pragma config FPLLODIV = DIV_1
#pragma config POSCMOD = HS
#pragma config FNOSC = PRIPLL
#pragma config FWDTEN = OFF // watchdog off
#pragma config FPBDIV = DIV_1

// Clock Constants
#define SYS_CLOCK (80000000L)
#define GetSystemClock()            (SYS_CLOCK)
#define GetInstructionClock()       (SYS_CLOCK)


/*****Constants that are used in initializing and execution of the main program*****/
/* IMU DEFINES AND CONSTANTS */
// IDs and corresponding I2C modules to be used with the IMUs
// These same IDs should be used on the base station
#ifdef I2C3
#define N_IMUS 3
#else
#define N_IMUS 2
#endif

/* KALMAN DEFINES AND CONSTANTS */
// TODO need update rate
#define SAMPLE_FREQ 100.0f  // 1 / 10 ms

#define ID_UPPER_ARM 0
#define I2C_UPPER_ARM I2C1
#define ID_FORE_ARM 1
#define I2C_FORE_ARM I2C2   // FIXME these i2c are probably wrong from diagram
#define ID_HAND 2
// For testing, if we have I2C3 we will use it
#ifdef I2C3
#define I2C_HAND I2C3
#endif
// I2C bus frequency to communicate with IMUs
#define I2C_FREQ 400000
// IMU acceleration and gyroscope settings being used
#define ACC_RANG ACCEL_RANGE_4G         // accelerometer range
#define ACC_BW ACCEL_BW_100             // accelerometer bandwidth
#define GYR_DLPF GYRO_DLPF_LPF_42HZ     // gyro digital low pass filter setting
#define GYR_SAMP_DIV 9                  // sample divider being used
#define GYR_POW_SEL GYRO_PWR_MGM_CLK_SEL_X  // gyro clock selector
// IMU calibration settings
#define CALIBRATE_SAMPLES 128
#define CALIBRATE_DELAY (1 / SAMPLE_FREQ) *

/* BATTERY MONITOR DEFINES AND CONSTANTS */
// *not incorporated

/* XBEE DEFINES AND CONSTANTS */
// TODO all xbee settings here
#define XBEE_UART_PORT UART3
#define XBEE_SETTINGS UART_ENABLE_PINS_TX_RX_ONLY

/* LCD DEFINES AND CONSTANTS */
#define LCD_INPUTS 11
// Settings that are being used
#define CURSOR_VAL LCD_CURSOR_OFF
#define CURSOR_BLINK LCD_CURSOR_BLINK_OFF

#define LCD_DISP_THRESH 1000

typedef struct LCD_PAIR {
    unsigned int bitnum;
    IoPortId port_id;
} LCD_PAIR;

// pins and ports to be used for testing
#define INIT_MESSAGE "Initializing"
#define START_MESSAGE "Swish Sleeve"
#define IMU_MESSAGE "IMUs"
#define FILTER_INIT_MESSAGE "Kalman Filters"
#define CALIB_MESSAGE_A "Hold arms still"
#define CALIB_MESSAGE_B "Calibrating"
#define READY_MESSAGE "System Ready\nAcquiring data..."
#define BATT_PRE "Battery: "
#define RSSI_PRE "RSSI: "
// Success messages
#define SUCC_IMU_MESSAGE " IMUs were\nsuccessful"
// Error display messages
#define ERROR_IMU_MESSAGE "ERR: IMU Failure"
#define BATT_INVAL "NO BATT. SIGNAL"
#define RSS_INVAL "NO RSSI. SIGNAL"
LCD_PAIR lcd_pairs[LCD_INPUTS] = {  // TODO fix these for the actual project
    {BIT_7, IOPORT_F}, \
    {BIT_6, IOPORT_F}, \
    {BIT_9, IOPORT_E}, \
    {BIT_14, IOPORT_D}, \
    {BIT_8, IOPORT_E}, \
    {BIT_15, IOPORT_D}, \
    {BIT_2, IOPORT_A}, \
    {BIT_3, IOPORT_A}, \
    {BIT_13, IOPORT_G}, \
    {BIT_14, IOPORT_G}, \
    {BIT_4, IOPORT_C}};


// Pins and ports used for button reset FIXME: might need to be changed for diagram/working
#define BUTTON1_A_PORT IOPORT_G
#define BUTTON1_A_PIN BIT_2
#define BUTTON1_B_PORT IOPORT_G
#define BUTTON1_B_PIN BIT_3

// Settings for timer interrupt, 10 ms interrupt
#define TIM1_SETTINGS (T1_ON | T1_SOURCE_INT | T1_PS_1_256)
#define TIM1_PERIOD 3124  // Done by hand and tested for 10 ms, SYS_CLOCK / PBFREQ / TIM1_PS / 1000 / (1 / 10 ms)
volatile BOOL UPDATE = FALSE;     // flag for update that timer will change


// Utility functions used in here
void ButtonConfig();
BOOL ButtonCheck();
// Timer interrupt functions
void ConfigTimer1Intrs();
#define SetTimer1Intrs(on) mT1IntEnable(on) // simple macro expansion to the mT1IntEnable
// Interrupt handler for Timer1 is called 'Timer1IntrHandler'
void DetectIMUErrorTrap(IMU_RESULT *imu_results);
void Send2LineDisplay(char *line_1, char *line_2, unsigned char bottomLineStartOffset);
void UpdateLCDStatus(int signalPercent, int batteryPercent);

/*** MAIN SWISH SLEEVE PROGRAM ***/
int main() {
  // Loop variables
  int i;

  // IMUs
  imu_t imu_upper_arm;
  imu_t imu_fore_arm;
  imu_t imu_hand;
  // Determine which IMU to update first, start with IMU 0
  int lastUpdateImu = 0;

  // Kalman filter states
  KALMAN_STATE_MADGWICK k_upper_arm;
  KALMAN_STATE_MADGWICK k_fore_arm;
  KALMAN_STATE_MADGWICK k_hand;

  // Percentage readings from RSSI of Xbee and battery monitor *NOT CONFIGED*
  int rssiPercentage = -1;
  int batteryPercentage = -1;
  
  // Result variables to keep track of statuses from libraries
  IMU_RESULT imu_res[N_IMUS];

  // Actual peripheral bus frequency
  unsigned int pbFreq;

  // Counters for updating IMUs in cyclic order, LCD, battery monitor, and possibly(?) FIXME the RSSI
  unsigned int lcd_update_counter = 0;
  //  unsigned int bmon_update_counter = 0; *UNUSED*
  
  // Const pointers assigned to each IMU for readability
#ifdef I2C3
  imu_t *const p_imus[N_IMUS] = {&imu_upper_arm, &imu_fore_arm, &imu_hand};
#else
  imu_t *const p_imus[N_IMUS] = {&imu_upper_arm, &imu_fore_arm};
#endif

  // Const pointers assigned to each Kalman filter state
#ifdef I2C3
  KALMAN_STATE_MADGWICK *const pKs[N_IMUS] = {&k_upper_arm, &k_fore_arm, &k_hand};
#else
  KALMAN_STATE_MADGWICK *const pKs[N_IMUS] = {&k_upper_arm, &k_fore_arm};
#endif
  
  // Initialize components needed
  pbFreq = SYSTEMConfigPerformance(GetSystemClock());

  // Initialize Delay module first
  DelayInit(pbFreq);
  // Delay for components to boot up
  DelayS(1);

  // Initialize LCD module
  LcdInit(lcd_pairs[0].bitnum, lcd_pairs[0].port_id,
    lcd_pairs[1].bitnum, lcd_pairs[1].port_id,
    lcd_pairs[2].bitnum, lcd_pairs[2].port_id,
    lcd_pairs[3].bitnum, lcd_pairs[3].port_id,
    lcd_pairs[4].bitnum, lcd_pairs[4].port_id,
    lcd_pairs[5].bitnum, lcd_pairs[5].port_id,
    lcd_pairs[6].bitnum, lcd_pairs[6].port_id,
    lcd_pairs[7].bitnum, lcd_pairs[7].port_id,
    lcd_pairs[8].bitnum, lcd_pairs[8].port_id,
    lcd_pairs[9].bitnum, lcd_pairs[9].port_id,
    lcd_pairs[10].bitnum, lcd_pairs[10].port_id,
    LCD_DOTS_5x8);
  LcdInstrSetDisplayMode(LCD_DISPLAY_ON, CURSOR_VAL, CURSOR_BLINK);

  // Display initializing message
  Send2LineDisplay(INIT_MESSAGE, START_MESSAGE, 0);
  DelayS(2);

  // Initialize IMUs
  Send2LineDisplay(INIT_MESSAGE, IMU_MESSAGE, 0);
  // Set IMU IDs
  ImuSetID(p_imus[ID_UPPER_ARM], ID_UPPER_ARM);
  ImuSetID(p_imus[ID_FORE_ARM], ID_FORE_ARM);
  ImuSetID(p_imus[ID_HAND], ID_HAND);
  // Initialize IMUs
  imu_res[ID_UPPER_ARM] = ImuInit(p_imus[ID_UPPER_ARM], I2C_UPPER_ARM, pbFreq, I2C_FREQ, ACC_RANG, ACC_BW, GYR_DLPF, GYR_SAMP_DIV, GYR_POW_SEL);
  imu_res[ID_FORE_ARM] = ImuInit(p_imus[ID_FORE_ARM], I2C_FORE_ARM, pbFreq, I2C_FREQ, ACC_RANG, ACC_BW, GYR_DLPF, GYR_SAMP_DIV, GYR_POW_SEL);
#ifdef I2C3
  imu_res[ID_HAND] = ImuInit(p_imus[ID_HAND], I2C_HAND, pbFreq, I2C_FREQ, ACC_RANG, ACC_BW, GYR_DLPF, GYR_SAMP_DIV, GYR_POW_SEL); // TODO pic32mx360 only has 2 i2c ports
#endif

  /** IF ERROR IN AN IMU, TRAP HERE*/
  DetectIMUErrorTrap(imu_res);

  // Initialize Kalman state - USING MADGWICK FOR NOW
  Send2LineDisplay(INIT_MESSAGE, FILTER_INIT_MESSAGE, 0);
  for (i = 0; i < N_IMUS; i++) {
    Kalman_MadgwickInit(pKs[i]);
  }
  
  // Configure settings for XBee TODO
  //UARTConfigure(XBEE_UART_PORT, XBEE_SETTINGS);

  // Configure reset button
  ButtonConfig();

  // Configure interrupts for the system as multi vector
  INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
  
  // Set timer for interrupts based on how often IMUUpdate and KalmanUpdate occur
  ConfigTimer1Intrs();

  // TODO other imu results/calibrations and also a printout for IMUs that fail and/or successful
  Send2LineDisplay(CALIB_MESSAGE_A, CALIB_MESSAGE_B, 0);
  DelayS(1);
  for (i = 0; i < N_IMUS; i++) {
    imu_res[i] = ImuCalibrate(p_imus[i], TRUE, TRUE, CALIBRATE_SAMPLES, CALIBRATE_DELAY);
  }
  LcdClearAndDisplayData(READY_MESSAGE);
  DelayS(3);
  // Enable int
  errupts
  INTEnableInterrupts();
  
  // Enable Timer1 interrupts
  SetTimer1Intrs(TRUE);

  // Main loop,
  while(1) {
    // Wait until timer interrupt trigures an update
    while(!UPDATE);

    if (ButtonCheck()) {
      // If user has pressed reset button, recalibrate system
      RecalibrateIMUs()
    }

    // Update IMUs
    for (i = 0; i < N_IMUS; i++) {
      // Update in a different order each time for fairness
      ImuUpdate(p_imus[(i + lastUpdateImu) % N_IMUS]);
    }
    // Change which IMU gets updated first
    lastUpdateImu = (lastUpdateImu + 1) % N_IMUS;

    // Kalman filter the data
    for (i = 0; i < N_IMUS; i++) {
      // Order does not matter here
      Kalman_MadgwickUpdate(p_imus[i], pKs[i], SAMPLE_FREQ);
    }

    // Update RSSI if at threshhold
    // TODO

    // Update battery monitor at threshhold
    // *NOT BEING USED RIGHT NOW*

    // Output data via XBee
    
    // Display data to LCD, if at threshhold
    if (lcd_update_counter++ == LCD_DISP_THRESH) {
      UpdateLCDStatus(rssiPercentage, batteryPercentage);
      // Reset counter
      lcd_update_counter = 0;
    }
  }

  return 0;
}

// DONE doc this
void ButtonConfig() {
  PORTSetPinsDigitalIn(BUTTON1_A_PORT, BUTTON1_A_PIN);
  PORTSetPinsDigitalIn(BUTTON1_B_PORT, BUTTON1_B_PIN);
}

// DONE doc this
BOOL ButtonCheck() {
  static BOOL down = FALSE;
  BOOL b1, b2;
  BOOL res = FALSE;;

  b1 = PORTReadBits(BUTTON1_A_PORT, BUTTON1_A_PIN) ? TRUE : FALSE;
  b2 = PORTReadBits(BUTTON1_B_PORT, BUTTON1_B_PIN) ? TRUE : FALSE;

  if (down && (b1 && !b2)) {
    // button relased
    res = TRUE;
  }
  down = !b1 && b2;

  return (res);

  return FALSE;
}

// TODO
void ConfigTimer1Intrs() {
  // Configure timers for our target update rate
  ConfigIntTimer1(T1_INT_OFF | T1_INT_PRIOR_1);
  OpenTimer1(TIM1_SETTINGS, TIM1_PERIOD);
}

// DONE doc this
void __ISR(_TIMER_1_VECTOR, ipl1) Timer1IntrHandler() {
  // Clear interrupt flag
  mT1ClearIntFlag();
  // Handle timer 1 interrupt
  if (UPDATE == TRUE) {
    printf("\nERROR: TIMER1 Interrupt happened before last calculations were completed\n\n");
  }
  UPDATE = TRUE;
}

// DONE doc this
void UpdateLCDStatus(int signalPercent, int batteryPercent) {
  char line1[40];
  char line2[40];
  char percentageval[5];

  // Display signal message
  if (signalPercent <= 0) {
    // invalid signal strength, copy to buffer
    strcpy(line1, RSS_INVAL);
    // Append newline to data for battery message
  } else {
    strcpy(line1, BATT_PRE);
    // Format battery string
    itoa(percentageval, signalPercent, 10);
    strcat(line1, percentageval);
    strcat(line1, percentageval);
    strcat(line1, "%");
  }

  // Display battery message
  if (batteryPercent <= 0) {
    // Concat RSSI error message
    strcat(line2, RSS_INVAL);
  } else {
    strcat(line2, RSSI_PRE);
    // Format battery string
    itoa(percentageval, batteryPercent, 10);
    strcat(line2, percentageval);
    strcat(line2, percentageval);
    strcat(line2, "%");
  }

  // Clear and display this 2 line message
  Send2LineDisplay(line1, line2, 0);
}

void Send2LineDisplay(char *line_1, char *line_2, unsigned char bottomLineStartOffset) {
  LcdInstrClearDisplay();
  LcdDisplayData(line_1);
  LcdInstrSetDDRAMAddress(LCD_LINES_2 + bottomLineStartOffset);
  LcdDisplayData(line_2);
}

void DetectIMUErrorTrap(IMU_RESULT *imu_results) {
  int i;

  for (i = 0; i < N_IMUS; i++) {
    if (imu_results[i] == IMU_FAIL) {
      Send2LineDisplay(ERROR_IMU_MESSAGE, "Device: ", 0);
      LcdDisplayChar(i + 60); // This assumes an IMU ID less that 10
      while (1);  // LOCKS HERE FOR ERRORS
    }
  }
  LcdInstrClearDisplay();
  LcdDisplayChar(N_IMUS + 60);
  LcdDisplayData(SUCC_IMU_MESSAGE);
  DelayS(1);  
}