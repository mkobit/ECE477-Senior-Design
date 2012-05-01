/*
 * File:   XBEE.h
 * Author: team7
 *
 * Created on April 25, 2012, 10:49 PM
 */

#ifndef XBEE_H
#define XBEE_H

//TODO: USER CODE

#define XBEE_BAUDRATE 57600
#define XBEE_TIMERPERIOD 320
#define XBEE_N_CAPTURES 5
#define CLEAR_VT "\033[2J"

#define XBEE_IOPORT_IC IOPORT_D
#define XBEE_IOPIN_IC BIT_8


#define XbeeOpenTimer(settings, period) OpenTimer3(settings, period)
#define XbeeOpenCapture(settings) OpenCapture1(settings)
#define XbeeCaptureReady() mIC1CaptureReady()
#define XbeeReadCapture() mIC1ReadCapture()
#define XbeeCaptureClose() CloseCapture1()

// IC settings
#define XBEE_TIMER_SETTINGS T3_ON | T3_PS_1_256
#define XBEE_IC_FALL_SETTINGS IC_TIMER3_SRC | IC_EVERY_FALL_EDGE | IC_ON
#define XBEE_IC_RISE_SETTINGS IC_TIMER3_SRC | IC_EVERY_RISE_EDGE | IC_ON

void XBeeSendDataBuffer(UART_MODULE,const char *buffer, UINT32 size);
void XBeeConfigure(UART_MODULE , UINT32 freq, UINT32 baudrate);
int XBeeCaptureSignalStrength(void);
void XBeeCaptureSignalStrength2(void);

#endif
