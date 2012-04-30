#include <plib.h>
#include <p32xxxx.h>
#include <string.h>
// Configuration Bit settings
#include "configs.h"

#include "delay.h"
#include "imu.h"
//#include "battery_monitor.h"  DOES NOT WORK CURRENTLY
#include "kalman.h"
#include "lcd_16x2.h"
#include "math_helpers.h"
#include "xbee.h"



// Clock Constants
#define SYS_CLOCK (80000000L)
#define GetSystemClock()            (SYS_CLOCK)
#define GetInstructionClock()       (SYS_CLOCK)


/* KALMAN FILTER DEFINES AND CONSTANTS */
#define USE_MADGWICK 1
#define USE_MAHONY 2

#define FILTERING_ALG USE_MADGWICK
#define SAMPLE_FREQ 100.0f  // 1 / 10 ms

// KALMAN: using macros for the functions so main program is easier to read
#if (FILTERING_ALG == USE_MADGWICK)
// Macros for madgwick
typedef KALMAN_STATE_MADGWICK K_state;
#define mKInit(p_k) Kalman_MadgwickInit(p_k)
#define mKUpdate(p_imu, p_k, samp) Kalman_MadgwickUpdate(p_imu, p_k, samp)

#elif (FILTERING_ALG == USE_MAHONY)
// Macros for Mahony
typedef KALMAN_STATE_MAHONY K_state;
#define mKInit(p_k) Kalman_MahonyInit(p_k);
#define mKUpdate(p_imu, p_k, samp) Kalman_MahonyUpdate(p_imu, p_k, samp)
#endif
// KALMAN: Gain constants
#define K_BETA_DEF (KALMAN_DEFAULT_BETADEF)
#define K_TWOKPDEF (KALMAN_DEFAULT_TWOKPDEF)
#define K_TWOKIDEF (KALMAN_DEFAULT_TWOKIDEF)


/* IMU DEFINES AND CONSTANTS */
// How many IMUs we are using
#define N_IMUS 1
// I2C bus frequency to communicate with IMUs
#define I2C_CLOCK 400000
// IMU: acceleration and gyroscope settings being used
#define ACC_RANG ACCEL_RANGE_2G         // accelerometer range
#define ACC_BW ACCEL_BW_100             // accelerometer bandwidth
#define GYR_DLPF GYRO_DLPF_LPF_20HZ     // gyro digital low pass filter setting
#define GYR_SAMP_DIV 9                  // sample divider being used
#define GYR_POW_SEL GYRO_PWR_MGM_CLK_SEL_X  // gyro clock selector
// IMU: calibration settings
#define CALIBRATE_SAMPLES 256
#define CALIBRATE_DELAY ((UINT)((1 / SAMPLE_FREQ) * 1000))
// IMU: I2C addresses of each device
#define IMU0_ACC_ADDR ACCEL_DEFAULT_ADDR
#define IMU0_GYRO_ADDR GYRO_DEFAULT_ADDR
#define IMU1_ACC_ADDR ACCEL_DEFAULT_ADDR
#define IMU2_GYRO_ADDR GYRO_DEFAULT_ADDR
#define IMU2_ACC_ADDR ACCEL_DEFAULT_ADDR
#define IMU2_GYRO_ADDR GYRO_DEFAULT_ADDR


/* BATTERY MONITOR DEFINES AND CONSTANTS */
// *not currently supported
#undef BATTERY_MONITOR_AVAILABLE  // just incase, that way battery functions are not called


/* XBEE DEFINES AND CONSTANTS */
#define UART_PORT_XBEE UART1
#define UPDATE_XBEE_THRESH 500
typedef struct TRANSMIT_PACKAGE {
  UINT8 n_bytes;
  imu_id id;
  QUATERNION q;
} TRANSMIT_PACKAGE;

// XBee transmission variable
TRANSMIT_PACKAGE package;
 


/* LCD DEFINES AND CONSTANTS */
#define N_INPUTS_LCD 11
// LCD: various settings
#define CURSOR_VAL LCD_CURSOR_OFF
#define CURSOR_BLINK LCD_CURSOR_BLINK_OFF

#define DISP_THRESH_LCD 500
#define TEMP_UPDATE_THRESH 500
// LCD: Simple structure for organizing LCD_DATA
typedef struct port_pin_pair {
    unsigned int bitnum;
    IoPortId port_id;
} PP_PAIR;
// LCD: pins/ports
// rs, rw,  en, d0-d7
 // TODO fix these for the actual project
PP_PAIR lcd_pairs[N_INPUTS_LCD] = {
    {BIT_1, IOPORT_D}, \
    {BIT_5, IOPORT_D}, \
    {BIT_6, IOPORT_D}, \
    {BIT_7, IOPORT_D}, \
    {BIT_0, IOPORT_F}, \
    {BIT_1, IOPORT_F}, \
    {BIT_0, IOPORT_E}, \
    {BIT_1, IOPORT_E}, \
    {BIT_2, IOPORT_E}, \
    {BIT_3, IOPORT_E}, \
    {BIT_4, IOPORT_E}};
// LCD: display messages
#define START_MESSAGE "Swish Sleeve"
#define INIT_MESSAGE "Initializing"
#define XBEE_INIT_MESSAGE "XBee"
#define IMU_FILTERS_INIT_MESSAGE "IMUs/Filters"
#define CALIB_MESSAGE_A "Hold arms still"
#define CALIB_MESSAGE_B "Calibrating"
#define READY_MESSAGE "System Ready\nAcquiring data..."
#define BATT_PRE "Battery: "
#define RSSI_PRE "RSSI: "
#define TEMP_PRE "Avg temp: "
// LCD: Success messages
#define SUCC_IMU_MESSAGE "IMUs were\nsuccessful"
// LCD: Error or invalid display messages
#define ERROR_IMU_MESSAGE "ERR: IMU Failure"
#define BATT_INVAL "NO BATT. SIGNAL"
#define RSS_INVAL "NO RSSI. SIGNAL"


/* BUTTON DEFINES AND CONSTANTS */    
// BUTTON: Pins and ports used for button reset FIXME: might need to be changed for diagram/working
#define BUTTON1_A_PORT IOPORT_G
#define BUTTON1_A_PIN BIT_3
#define BUTTON1_B_PORT IOPORT_G
#define BUTTON1_B_PIN BIT_2


/* TIMER1 DEFINES AND CONSTANTS */
// TIMER1: Settings for interrupt, 10 ms interrupt
#define TIM1_SETTINGS (T1_ON | T1_SOURCE_INT | T1_PS_1_256)
#define TIM1_PERIOD 3124  // Done by hand and tested for 10 ms, SYS_CLOCK / PBFREQ / TIM1_PS / 1000 / (1 / 10 ms)
volatile BOOL UPDATE = FALSE;     // flag for update that timer will change


// SwishSleeve and utility functions
void UpdateAndSendPackages(imu_t *imus, K_state *states, IMU_RESULT *imu_res);
inline void ButtonConfig();
BOOL Button1Check();
// Timer interrupt functions
void ConfigTimer1Intrs();
#define SetTimer1Intrs(on) mT1IntEnable(on) // simple macro expansion to the mT1IntEnable, {on} should be TRUE or FALSE
void RecalibrateIMUs(imu_t *const imus, const int rssiPercentage);
// LCD utility functions
void Send2LineDisplay(char *line_1, char *line_2, const unsigned char bottomLineStartOffset);
void DetectIMUErrorTrap(IMU_RESULT *imu_results);
// Updating status functions, for either battery or if not supported, just use average temperature
#ifdef BATTERY_MONITOR_AVAILABLE
void UpdateLCDStatus(const int signalPercent, const int batteryPercent);
#else
void UpdateLCDStatus(const int signalPercent, const int avgTemperature);
#endif

/* GLOBAL VARIABLES USED BY THE PROGRAM */
#if (N_IMUS == 1)
const I2C_MODULE i2cs[N_IMUS] = {I2C1};
#elif (N_IMUS == 2)
const I2C_MODULE i2cs[N_IMUS] = {I2C1, I2C2};
#elif (N_IMUS == 3)
const I2C_MODULE i2cs[N_IMUS] = {I2C1, I2C2, I2C3};
#endif

/**************************************************************************************************
  Title: 
    Swish Sleeve Main Application
    
  Version: 
    0.2
    
  Filename: 
    swishsleeve-0.2.c
    
  Author(s): 
    mkobit
    
  Purpose of Program: 
    Main program for the Swish Sleeve application
    
  How to build: 
    delay.c
    imu.c - itg3200.c, adxl345.c, i2c_shared.c
    *battery_monitor.c  *NOT CURRENTLY SUPPORTED
    kalman.c
    lcd_16x2.c - delay.c
    math_helpers.c
    xbee.c
    
  Update History: 
    
    
**************************************************/
/*** MAIN SWISH SLEEVE PROGRAM ***/
int main() {
  int i;
  K_state states[N_IMUS];
  imu_t imus[N_IMUS];
  IMU_RESULT imu_res[N_IMUS];   // Result variables to keep track of statuses from libraries
  unsigned int pbFreq;          // Actual peripheral bus frequency
    // Counters for updating IMUs in cyclic order, LCD, battery monitor, and possibly(?) FIXME the RSSI
  int signalPercent = 0;

  // Counters for updating IMUs in cyclic order, LCD, battery monitor, and possibly(?) FIXME the RSSI
  int lastUpdateImu = 0;        // Determine which IMU to update first, start with IMU 0
  unsigned int lcd_update_counter = 0;
  unsigned int signal_strength_update_counter = 0;
#ifdef BATTERY_MONITOR_AVAILABLE
  unsigned int bmon_update_counter = 0;
  unsigned int bmonPercent = 0;
#else
  unsigned int temperature_update_counter = 0;
  int temperature = 0;
#endif

#ifdef BATTERY_MONITOR_AVAILABLE
  int batteryPercent = 0;
#else
  int avgTemperature = 0;
#endif

 // All packages are same size
  package.n_bytes = (UINT8) sizeof(TRANSMIT_PACKAGE);
  

  // Initialize Delay module first
  DelayInit(GetSystemClock());
  // Delay for components to boot up
  DelayS(1);

  // Initialize LCD module
  LcdInit(lcd_pairs[0].bitnum, lcd_pairs[0].port_id, lcd_pairs[1].bitnum, lcd_pairs[1].port_id, lcd_pairs[2].bitnum, lcd_pairs[2].port_id, lcd_pairs[3].bitnum, lcd_pairs[3].port_id, lcd_pairs[4].bitnum, lcd_pairs[4].port_id, lcd_pairs[5].bitnum, lcd_pairs[5].port_id, lcd_pairs[6].bitnum, lcd_pairs[6].port_id, lcd_pairs[7].bitnum, lcd_pairs[7].port_id, lcd_pairs[8].bitnum, lcd_pairs[8].port_id, lcd_pairs[9].bitnum, lcd_pairs[9].port_id, lcd_pairs[10].bitnum, lcd_pairs[10].port_id, LCD_DOTS_5x8);
  LcdInstrSetDisplayMode(LCD_DISPLAY_ON, CURSOR_VAL, CURSOR_BLINK);

  // Display initializing message
  Send2LineDisplay(INIT_MESSAGE, START_MESSAGE, 0);
  DelayS(2);
  
  // Initialization for each IMU and filters
  Send2LineDisplay(INIT_MESSAGE, IMU_FILTERS_INIT_MESSAGE, 0);
#if (N_IMUS == 3)
  imu_res[2] = ImuInit(&imus[2], i2cs[2], pbFreq, I2C_CLOCK, IMU2_ACC_ADDR, ACC_RANG, ACC_BW, IMU2_GYRO_ADDR, GYR_DLPF, GYR_SAMP_DIV, GYR_POW_SEL);
  ImuSetID(&imus[2], 2);
  mKInit(&states[2]);
#endif
#if (N_IMUS == 2)
  imu_res[1] = ImuInit(&imus[1], i2cs[1], pbFreq, I2C_CLOCK, IMU1_ACC_ADDR, ACC_RANG, ACC_BW, IMU1_GYRO_ADDR, GYR_DLPF, GYR_SAMP_DIV, GYR_POW_SEL);
  ImuSetID(&imus[1], 1);
  mKInit(&states[1]);
#endif
  imu_res[0] = ImuInit(&imus[0], i2cs[0], pbFreq, I2C_CLOCK, IMU0_ACC_ADDR, ACC_RANG, ACC_BW, IMU0_GYRO_ADDR, GYR_DLPF, GYR_SAMP_DIV, GYR_POW_SEL);
  mKInit(&states[0]);
  ImuSetID(&imus[0], 0);
  DelayS(1);
  
  // Make sure initializations are correct
  DetectIMUErrorTrap(imu_res);

  // Configure settings for XBee
  Send2LineDisplay(INIT_MESSAGE, XBEE_INIT_MESSAGE, 0);
  XBeeConfigure(UART_PORT_XBEE, pbFreq, XBEE_BAUDRATE);
  DelayS(1);

  // Configure reset button
  ButtonConfig();

    // IMU calibrations and also a trap for IMUs that fail/succeed
  Send2LineDisplay(CALIB_MESSAGE_A, CALIB_MESSAGE_B, 0);

#if (N_IMUS == 3)
  imu_res[2] = ImuCalibrate(&imus[2], TRUE, TRUE, CALIBRATE_SAMPLES, CALIBRATE_DELAY);
#endif
#if (N_IMUS == 2)
  imu_res[1] = ImuCalibrate(&imus[0], TRUE, TRUE, CALIBRATE_SAMPLES, CALIBRATE_DELAY);
#endif
  imu_res[0] = ImuCalibrate(&imus[0], TRUE, TRUE, CALIBRATE_SAMPLES, CALIBRATE_DELAY);
  DelayS(1);

  // Make sure calibrations are correct
  DetectIMUErrorTrap(imu_res);

  // Configure interrupts for the system as multi vector
  INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);

  // Set timer for interrupts based on how often IMUUpdate and KalmanUpdate occur
  ConfigTimer1Intrs();

  // System ready message
  LcdClearAndDisplayData(READY_MESSAGE);
  DelayS(3);

  // Display system status
#ifdef BATTERY_MONITOR_AVAILABLE
  // Get signal strength and battery percentage
  UpdateLCDStatus(signalPercent, batteryPercent);
#else
  // Get signal strength and average temperature
  // TODO capture signal strength
  for (i = 0; i < N_IMUS; i++) {
    avgTemperature += (int) ImuGetGyroTemp(&imus[i]);
  }
  UpdateLCDStatus(signalPercent, avgTemperature / N_IMUS);
#endif

  // Enable interrupts
  INTEnableInterrupts();

  // Clear and enable Timer1 interrupts
  WriteTimer1(0);
  SetTimer1Intrs(TRUE);

  while (1) {
    while (!UPDATE) {}

    if (Button1Check()) {
      // Recalibrate if button was pressed and released
      RecalibrateIMUs(imus, signalPercent);
      lcd_update_counter = 0;
      signal_strength_update_counter = 0;
#ifdef BATTERY_MONITOR_AVAILABLE
      bmon_update_counter = 0;
#else
      temperature_update_counter = 0;
#endif
      continue;
    } else {
      // Update counters for display
      signal_strength_update_counter++;
#ifdef BATTERY_MONITOR_AVAILABLE
      bmon_update_counter++;
#else
      temperature_update_counter++;
#endif
    }

    // Update the IMUs
    for (i = 0; i < N_IMUS; i++) {
      // Update in a different order each time for fairness
      imu_res[i] = ImuUpdate(&imus[(i + lastUpdateImu) % N_IMUS]);
    }
    // Change which IMU gets updated first
    lastUpdateImu = (lastUpdateImu + 1) % N_IMUS;

    // Kalman filter the data
    for (i = 0; i < N_IMUS; i++) {
      mKUpdate(&imus[i], &states[i], SAMPLE_FREQ);
    }

    // Update RSSI if at threshhold
    // TODO
    if (signal_strength_update_counter == UPDATE_XBEE_THRESH) {
      signal_strength_update_counter = 0;
      //FIXME signalPercent = XBeeCaptureSignalStrenth();
    }

#ifdef BATTERY_MONITOR_AVAILABLE
    // NOT SUPPORTED
    if (bmon_update_counter == )
#else
    if (temperature_update_counter == TEMP_UPDATE_THRESH) {
      temperature_update_counter = 0;
      temperature = 0;
      for (i = 0; i< N_IMUS; i++) {
        temperature += ImuGetGyroTemp(&imus[i]);
      }
      temperature /= N_IMUS;
    }
#endif

    // Update battery monitor at threshhold
    // *NOT BEING USED RIGHT NOW*

    // Output data via XBee{
    UpdateAndSendPackages(imus, states, imu_res);

    if (lcd_update_counter == DISP_THRESH_LCD) {
      lcd_update_counter == 0;
#ifdef BATTERY_MONITOR_AVAILABLE
      UpdateLCDStatus(signalPercent, bmonPercent);
#else
      UpdateLCDStatus(signalPercent, temperature);
#endif
    }
  }
  
  

  return 0;
}


/**************************************************************************************************
UTILITY FUNCTIONS USED BY SWISH SLEEVE PROGRAM
**************************************************************************************************/

/**************************************************************************************************
  Function:
    inline void ButtonConfig()

  Author(s):
    mkobit

  Summary:
    Configures the two #define-d pins and ports to be used as inputs for a button

  Description:
    Same as summary

  Preconditions:
    Pins/ports are not being used for any other module

  Parameters:
    void

  Returns:
    void

  Example:
    <code>
    ButtonConfig()
    </code>

  Conditions at Exit:
    Both pins and ports configured as digital inputs

**************************************************************************************************/
inline void ButtonConfig() {
  PORTSetPinsDigitalIn(BUTTON1_A_PORT, BUTTON1_A_PIN);
  PORTSetPinsDigitalIn(BUTTON1_B_PORT, BUTTON1_B_PIN);
}

/**************************************************************************************************
  Function:
    BOOL Button1Check()

  Author(s):
    mkobit

  Summary:
    Determines if button was pressed and then released

  Description:
    Maintains a state of if the button was down, and determines if the button was released afterwards

  Preconditions:
    ButtonConfig

  Parameters:
    void

  Returns:
    BOOL result - TRUE if button pushed and released, FALSE if otherwords

  Example:
    <code>
    BOOL buttonReleased = Button1Check()
    </code>

  Conditions at Exit:
    Static variable (BOOL down) is TRUE if the state of the button is down, else FALSE

**************************************************************************************************/
BOOL Button1Check() {
  static BOOL down = FALSE;
  BOOL b1, b2;
  BOOL res = FALSE;;

  b1 = PORTReadBits(BUTTON1_A_PORT, BUTTON1_A_PIN) ? TRUE : FALSE;
  b2 = PORTReadBits(BUTTON1_B_PORT, BUTTON1_B_PIN) ? TRUE : FALSE;

  if (down && (b1 && !b2)) {
    // Button relased after it was pushed down
    res = TRUE;
  }
  down = !b1 && b2;

  return (res);
}

/**************************************************************************************************
  Function:
    void ConfigTimer1Intrs()

  Author(s):
    mkobit

  Summary:
    Configures the parameters of Timer 1 and opens it

  Description:
    Same as summary

  Preconditions:
    Timer 1 not being used

  Parameters:
    void

  Returns:
    void

  Example:
    <code>
    ConfigTimer1Intrs()
    </code>

  Conditions at Exit:
    Timer 1 open and configured

**************************************************************************************************/
void ConfigTimer1Intrs() {
  // Configure timers for our target update rate
  ConfigIntTimer1(T1_INT_OFF | T1_INT_PRIOR_1);
  OpenTimer1(TIM1_SETTINGS, TIM1_PERIOD);
}

/**************************************************************************************************
  Function:
    void __ISR(_TIMER_1_VECTOR, ipl1) Timer1IntrHandler()

  Author(s):
    mkobit

  Summary:
    Handler for timer 1 interrupts

  Description:
    Maps the interrupt vector to this handler and sets the volatile flag UPDATE that means the main loop should continue updating

  Preconditions:
    Timer 1 interrupts configured and enabled

  Parameters:
    void

  Returns:
    void

  Example:
    N/A

  Conditions at Exit:
    Timer 1 interrupts are cleared, UPDATE flag set to TRUE

**************************************************************************************************/
void __ISR(_TIMER_1_VECTOR, ipl1) Timer1IntrHandler() {
  // Clear interrupt flag
  mT1ClearIntFlag();
  // Handle timer 1 interrupt
  UPDATE = TRUE;
}

/**************************************************************************************************
  Function:
    void Send2LineDisplay(char *line_1, char *line_2, const unsigned char bottomLineStartOffset)

  Author(s):
    mkobit

  Summary:
    Displays two lines on LCD with the offset on the second line

  Description:
    Uses LCD library to clear and display two lines on it

  Preconditions:
    LCD configured

  Parameters:
    char *line_1 - string to display on first line
    char *line_2 - string to display on second line
    const unsigned char bottomLineStartOffset - offset to display the second string on

  Returns:
    void

  Example:
    <code>
    Send2LineDisplay("Hello!", "There!", 6)
    </code>

  Conditions at Exit:
    LCD has the 2 lines displayed

**************************************************************************************************/
void Send2LineDisplay(char *line_1, char *line_2, const unsigned char bottomLineStartOffset) {
  LcdInstrClearDisplay();
  LcdDisplayData(line_1);
  LcdInstrSetDDRAMAddress(LCD_LINES_2 + bottomLineStartOffset);
  LcdDisplayData(line_2);
}

/**************************************************************************************************
  Function:
    void DetectIMUErrorTrap(IMU_RESULT *imu_results)

  Author(s):
    mkobit

  Summary:
    If any IMU function results in an error, will be held here forever

  Description:
    Checks IMU results and if there was a failure, simply stalls here and displays an error message to the user

  Preconditions:
    (imu_results) filled in with valid results

  Parameters:
    IMU_RESULT *imu_results - results of IMU functions

  Returns:
    void

  Example:
    <code>
    DetectIMUErrorTrap(imu_res)
    </code>

  Conditions at Exit:
    None

**************************************************************************************************/
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

/**************************************************************************************************
  Function:
    void RecalibrateIMUs(imu_t *const imus)

  Author(s):
    mkobit

  Summary:
    Recalibrates IMUs, stall in loop if any recalibration fails

  Description:
    Loops through however many IMUs there are and calibrated

  Preconditions:
    IMUs initialized

  Parameters:
    imu_t *const imus - pointers to the IMUs that need to be calibrated

  Returns:
    void

  Example:
    <code>
    RecalibrateIMUs(p_imus)
    </code>

  Conditions at Exit:
    void

**************************************************************************************************/
void RecalibrateIMUs(imu_t *const imus, const int rssiPercentage) {
  int i;
  IMU_RESULT res[N_IMUS];
  UINT intrs;
  int temp = 0;

  // Turn off timer interrupts and recalibrate
  intrs = INTDisableInterrupts();

  // Display calibrating message
  Send2LineDisplay(CALIB_MESSAGE_A, CALIB_MESSAGE_B, 0);

  for (i = 0; i < N_IMUS; i++) {
    res[i] = ImuCalibrate(&imus[i], TRUE, TRUE, CALIBRATE_SAMPLES, CALIBRATE_DELAY);
    temp += (int) ImuGetGyroTemp(&imus[i]);
  }

  // Trap here if an error occurs during calibration
  DetectIMUErrorTrap(res);

  // Display a new message
#ifdef BATTERY_MONITOR_AVAILABLE
  // Battery if available
  UpdateLCDStatus(rssiPercentage, batteryPercentage);
#else
  // Otherwise, average IMU temperature
  UpdateLCDStatus(rssiPercentage, temp / N_IMUS);
#endif

  // Clear Timer1 and enable interrupts
  WriteTimer1(0);
  INTRestoreInterrupts(intrs);
}

/**************************************************************************************************
  Function:
    *void UpdateLCDStatus(const int signalPercent, const int batteryPercent)
    *void UpdateLCDStatus(const int signalPercent, const int avgTemperature)

  Author(s):
    mkobit

  Summary:
    Updates the LCD display screen

  Description:
    Displays a 2 line status message on the LCD, depending on which status is #define-d at the top. If battery messages are defined, will display that.
    If they are not, the average temperature of the IMUs will be displayed

  Preconditions:
    LCD initialized

  Parameters:
    const int signalPercent - integer representing signal strength percentage of maximum strength
    *const int batteryPercent - percentage of battery life average temperatures of the IMUs
    *const int avgTemperature - average temperature of IMUs

  Returns:
    void

  Example:
  #ifdef BATTERY_MONITOR_AVAILABLE
    <code>
    UpdateLCDStatus(signalPercent, batteryPercent)
    </code>
  #else
    <code>
    UpdateLCDStatus(signalPercent, averageTemperature)
    </code>
  #endif

  Conditions at Exit:
    LCD has a new 2-line message on it

  Notes:
    Anything marked with a * denotes that it depends if battery monitor is #define-d or not

**************************************************************************************************/
#ifdef BATTERY_MONITOR_AVAILABLE
void UpdateLCDStatus(const int signalPercent, const int batteryPercent) {
#else
void UpdateLCDStatus(const int signalPercent, const int avgTemperature) {
#endif
  char line1[40];
  char line2[40];
  char percentageval[5];
#ifndef BATTERY_MONITOR_AVAILABLE
  static int imuTemp = -1;
  const static char tempEnd[3] = {176, 'C', '\0'}; // \degrees C
#endif

  // Display signal message
  if (signalPercent <= 0) {
    // invalid signal strength, copy to buffer
    strcpy(line1, RSS_INVAL);
    // Append newline to data for battery message
  } else {
    strcpy(line1, RSSI_PRE);
    // Format battery string
    itoa(percentageval, signalPercent, 10);
    strcat(line1, percentageval);
    strcat(line1, "%");
  }

#ifdef BATTERY_MONITOR_AVAILABLE
  // Display battery message
  if (paramPercent <= 0) {
    // Concat RSSI error message
    strcpy(line2, RSS_INVAL);
  } else {
    strcpy(line2, RSSI_PRE);
    // Format battery string
    itoa(percentageval, paramPercent, 10);
    strcat(line2, percentageval);
    strcat(line2, "%");
  }
#else
  //Display temperature message
  if (imuTemp <= 0) {
    // Use first reading, double to avoid average reducing it
    imuTemp = avgTemperature * 2;
  }
  
  // Use average
  imuTemp = (imuTemp + avgTemperature) / 2;
  strcpy(line2, TEMP_PRE);
  itoa(percentageval, avgTemperature, 10);
  strcat(line2, percentageval);
  strcat(line2, tempEnd);
#endif

  // Clear and display this 2 line message
  Send2LineDisplay(line1, line2, 0);
}

/**************************************************************************************************
  Function:
    void UpdateAndSendPackages(imu_t *imus, K_state *states, IMU_RESULT *imu_res)

  Author(s):
    mkobit

  Summary:
    Sends the data of each IMU via XBee using the global PACKAGE variable

  Description:
    Same as summary

  Preconditions:
    IMUs updated
    XBee configured

  Parameters:
    imu_t *imus - IMUs in the system
    K_state *states - Kalman filter states
    IMU_RESULT *imu_res - results of IMU updates

  Returns:
    void

  Example:
    <code>
    </code>

  Conditions at Exit:
    Package data and send it via XBee

**************************************************************************************************/
void UpdateAndSendPackages(imu_t *imus, K_state *states, IMU_RESULT *imu_res) {
  // Use the global package variable
  int i;
  static QUATERNION *const package_qp = &package.q;
  QUATERNION *thisqp;
  for (i = 0; i < N_IMUS; i++) {
    if (imu_res[i] == IMU_SUCCESS) {
      thisqp = &states[i].q;
      package.id = ImuGetId(&imus[i]);
      package_qp->q0 = thisqp->q0;
      package_qp->q1 = thisqp->q1;
      package_qp->q2 = thisqp->q2;
      package_qp->q3 = thisqp->q3;
      XBeeSendDataBuffer(UART_PORT_XBEE, (char *) &package, package.n_bytes);
    }
  }
}