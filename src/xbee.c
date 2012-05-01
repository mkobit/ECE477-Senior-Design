#include <plib.h>
#include "xbee.h"
#include "delay.h"

// *****************************************************************************
// void UARTTxBuffer(char *buffer, UINT32 size)
// *****************************************************************************
void XBeeSendDataBuffer(UART_MODULE id, const char *buffer, UINT32 size)
{
  int i;

  for(i= 0; i < 6;i++){

    while(!UARTTransmitterIsReady(id));

    if(i == 5){
      UARTSendDataByte(id, (char)size); 
    }
    else{
      UARTSendDataByte(id, ('a'+i));
    }
  }

  while(size)
  {
    while(!UARTTransmitterIsReady(id));

    UARTSendDataByte(id, *buffer);

    buffer++;
    size--;
  }

  while(!UARTTransmissionHasCompleted(id));
}

/*
 Argument:
 *
 *id:  the uart module needed
 *frequency: PeripheralClock Frequency
 *Baud Rate: Desired Baudrate
 *

 */


void XBeeConfigure(UART_MODULE id, UINT32 freq, UINT32 baudrate ){
  UARTConfigure(id, UART_ENABLE_PINS_TX_RX_ONLY);
  //UARTSetFifoMode(id, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
  UARTSetLineControl(id, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
  UARTSetDataRate(id, freq, baudrate);
  UARTEnable(id, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

   // Setup Timer 3
  XbeeOpenTimer(XBEE_TIMER_SETTINGS, XBEE_TIMERPERIOD);
}



int XBeeCaptureSignalStrength()
{
  unsigned int RiseEdge1, RiseEdge2, FallEdge1;
  float duty_cycle, duty_temp;
  duty_cycle = 0;
  duty_temp = 0;
  int i;
  int scaled;
  unsigned int timeout;

  // Enable Input Capture Module 1
  // - Capture Every edge
  // - Use Timer 3 source
  // - Capture rising edge first

  // Wait for Capture events

  //Now Read the captured timer value
  for (i = 0; i < XBEE_N_CAPTURES; i++)
  {
    RiseEdge1 = 0;
    FallEdge1 = 0;
    RiseEdge2 = 0;
    timeout = 0;
    WriteTimer3(0);
    
    XbeeOpenCapture(XBEE_IC_RISE_SETTINGS);
    while(!XbeeCaptureReady()) {
      timeout++;
      if (timeout == XBEE_TIMEOUT) {
        XbeeCaptureClose();
        PORTSetPinsDigitalIn(XBEE_IOPORT_IC, XBEE_IOPIN_IC);
        RiseEdge1 = PORTReadBits(XBEE_IOPORT_IC, XBEE_IOPIN_IC);
        PORTResetPins(XBEE_IOPORT_IC, XBEE_IOPIN_IC);
        if (RiseEdge1) {
          // high value, 100%
          return 100;
        } else {
          return 0;
        }
      }
    }
    RiseEdge1 = XbeeReadCapture();
    
    XbeeCaptureClose();
    XbeeOpenCapture(XBEE_IC_FALL_SETTINGS);
    
    timeout = 0;
    while(!XbeeCaptureReady()) {
      timeout++;
      if (timeout == XBEE_TIMEOUT) {
        XbeeCaptureClose();
        PORTSetPinsDigitalIn(XBEE_IOPORT_IC, XBEE_IOPIN_IC);
        RiseEdge1 = PORTReadBits(XBEE_IOPORT_IC, XBEE_IOPIN_IC);
        PORTResetPins(XBEE_IOPORT_IC, XBEE_IOPIN_IC);
        if (RiseEdge1) {
          // high value, 100%
          return 100;
        } else {
          return 0;
        }
      }
    }
    FallEdge1 = XbeeReadCapture();

    XbeeCaptureClose();
    XbeeOpenCapture(XBEE_IC_RISE_SETTINGS);
    
    timeout = 0;
    while(!XbeeCaptureReady()) {
      timeout++;
      if (timeout == XBEE_TIMEOUT) {
        XbeeCaptureClose();
        PORTSetPinsDigitalIn(XBEE_IOPORT_IC, XBEE_IOPIN_IC);
        RiseEdge1 = PORTReadBits(XBEE_IOPORT_IC, XBEE_IOPIN_IC);
        PORTResetPins(XBEE_IOPORT_IC, XBEE_IOPIN_IC);
        if (RiseEdge1) {
          // high value, 100%
          duty_cycle += 100;
        } else {
          duty_cycle += 0;
        }
      }
    }
    
    RiseEdge2 = XbeeReadCapture();
    XbeeCaptureClose();
    if (RiseEdge2 - RiseEdge1 != 0) {
      duty_temp = (float)(FallEdge1 - RiseEdge1) / (float)(RiseEdge2 - RiseEdge1);
    } else {
      i--;
      continue;
    }


    if ((FallEdge1 > RiseEdge1) && (RiseEdge2 > FallEdge1) && duty_temp <= 1.0f && duty_temp >= 0.0f){
      //0.97 is the scale factor
      duty_cycle += duty_temp;
      //printf("Duty Cycle: %f\n", 0.97*duty_cycle);
    }
    else if (duty_temp < .005f) {
      duty_cycle += 0.0f;
    }
  }

  scaled = (int)((duty_cycle / XBEE_N_CAPTURES) * 100);
  return scaled;
}



void XBeeCaptureSignalStrength2()
{
  unsigned int RiseEdge1, RiseEdge2, FallEdge1;
  float duty_cycle, duty_temp;
  int count;
  count = 0;
  duty_cycle = 0;
  duty_temp = 0;
  //Clear interrupt flag
  mIC1ClearIntFlag();

  // Setup Timer 3
  OpenTimer3(T3_ON | T1_PS_1_256, XBEE_TIMERPERIOD);

  // Enable Input Capture Module 1
  // - Capture Every edge
  // - Enable capture interrupts
  // - Use Timer 3 source
  // - Capture rising edge first
  XbeeOpenCapture( IC_EVERY_EDGE | IC_INT_1CAPTURE | IC_TIMER3_SRC | IC_FEDGE_RISE | IC_ON );

// Wait for Capture events

  //TODO Change Timer Period
  //Now Read the captured timer value
  while( 1 )
  {

    while(!XbeeCaptureReady());
    if( XbeeCaptureReady() ){
          RiseEdge1 = XbeeReadCapture();
    }
    //while(!XbeeCaptureReady());
    if( XbeeCaptureReady() ){
          FallEdge1 = XbeeReadCapture();
    }
    //while(!XbeeCaptureReady());
    if( XbeeCaptureReady() ){
      RiseEdge2 = XbeeReadCapture();
    }
    duty_temp = (float)(FallEdge1 - RiseEdge1) / (float)(RiseEdge2 - RiseEdge1);


    if ((FallEdge1 > RiseEdge1) && (RiseEdge2 > FallEdge1)){

      duty_cycle = (duty_cycle+duty_temp)/2;
      //printf("Duty Cycle: %f\n", duty_cycle);
    }
    else if(duty_temp == 1.0){
      //printf("Duty Cycle: %d\n", duty_temp);
    }
    DelayMs(30);
  }
  XbeeCaptureClose();
  CloseTimer3();
}