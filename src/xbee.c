#include <plib.h>
#include "xbee.h"
#include "delay.h"

// *****************************************************************************
// void UARTTxBuffer(char *buffer, UINT32 size)
// *****************************************************************************
void XBeeSendDataBuffer(UART_MODULE id, const char *buffer, UINT32 size)
{
  int i;

  for(i= 0; i <6;i++){

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
}



int XBeeCaptureSignalStrength()
{
  unsigned int RiseEdge1, RiseEdge2, FallEdge1;
  float duty_cycle, duty_temp;
  int count;
  count = 0;
  duty_cycle = 0;
  duty_temp = 0;
  int i;
  //Clear interrupt flag
  mIC1ClearIntFlag();

  // Setup Timer 3
   OpenTimer3(T3_ON | T1_PS_1_256, Time3Period);

  // Enable Input Capture Module 1
  // - Capture Every edge
  // - Enable capture interrupts
  // - Use Timer 3 source
  // - Capture rising edge first
  OpenCapture1( IC_EVERY_EDGE | IC_INT_1CAPTURE | IC_TIMER3_SRC | IC_FEDGE_RISE | IC_ON );

  // Wait for Capture events

  //TODO Change Timer Period
  //Now Read the captured timer value
  for (i = 0; i < 5; i++)
  {
    OpenCapture1( IC_EVERY_EDGE | IC_INT_1CAPTURE | IC_TIMER3_SRC | IC_FEDGE_RISE | IC_ON );
    while(!mIC1CaptureReady());
    if( mIC1CaptureReady() ){
          RiseEdge1 = mIC1ReadCapture();
    }
    //while(!mIC1CaptureReady());
    if( mIC1CaptureReady() ){
          FallEdge1 = mIC1ReadCapture();

    }
    //while(!mIC1CaptureReady());
    if( mIC1CaptureReady() ){
      RiseEdge2 = mIC1ReadCapture();
    }
    duty_temp = (float)(FallEdge1 - RiseEdge1) / (float)(RiseEdge2 - RiseEdge1);


    if ((FallEdge1 > RiseEdge1) && (RiseEdge2 > FallEdge1)){
      //0.97 is the scale factor
      duty_cycle = 0.97*(duty_cycle+duty_temp)/2;
      //printf("Duty Cycle: %f\n", 0.97*duty_cycle);
    }
    else{
      //printf("Duty Cycle: %d\n", duty_temp);
      //If there are no edges then it means 100% signal strength, RSSI output is always high
      duty_cycle = 1.0;
    }
    DelayUs(200);
    CloseCapture1();
  }
  CloseCapture1();
  CloseTimer3();

  return (int)(duty_cycle*100);
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
  OpenTimer3(T3_ON | T1_PS_1_256, Time3Period);

  // Enable Input Capture Module 1
  // - Capture Every edge
  // - Enable capture interrupts
  // - Use Timer 3 source
  // - Capture rising edge first
  OpenCapture1( IC_EVERY_EDGE | IC_INT_1CAPTURE | IC_TIMER3_SRC | IC_FEDGE_RISE | IC_ON );

// Wait for Capture events

  //TODO Change Timer Period
  //Now Read the captured timer value
  while( 1 )
  {

    while(!mIC1CaptureReady());
    if( mIC1CaptureReady() ){
          RiseEdge1 = mIC1ReadCapture();
    }
    //while(!mIC1CaptureReady());
    if( mIC1CaptureReady() ){
          FallEdge1 = mIC1ReadCapture();
    }
    //while(!mIC1CaptureReady());
    if( mIC1CaptureReady() ){
      RiseEdge2 = mIC1ReadCapture();
    }
    duty_temp = (float)(FallEdge1 - RiseEdge1) / (float)(RiseEdge2 - RiseEdge1);


    if ((FallEdge1 > RiseEdge1) && (RiseEdge2 > FallEdge1)){

      duty_cycle = (duty_cycle+duty_temp)/2;
      printf("Duty Cycle: %f\n", duty_cycle);
    }
    else if(duty_temp == 1.0){
      printf("Duty Cycle: %d\n", duty_temp);
    }
    DelayMs(30);
  }
  CloseCapture1();
  CloseTimer3();
}