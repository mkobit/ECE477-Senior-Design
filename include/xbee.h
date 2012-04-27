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
#define Time3Period 3124
#define CLEAR_VT "\033[2J"



void XBeeSendDataBuffer(UART_MODULE,const char *buffer, UINT32 size);
void XBeeConfigure(UART_MODULE , UINT32 freq, UINT32 baudrate);
void XBeeCaptureSignalStrenth(void);
VOID XBeeCaptureSignalStrenth2(void);

#endif
