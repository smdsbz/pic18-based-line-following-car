//////////////////////////////////////////////////////////////////////
//                                                                  //
//      AUTHOR:     Your Name                                       //
//      DATE:       xx/07/09                                        //
//      REVISION:   1.0                                             //
//      DEVICE:     PIC18F2525                                      //
//      FILENAME:   LED_ON.c                                        //
//                                                                  //
/////// DESCRIPTION //////////////////////////////////////////////////
// Turn on LED on RA1 on SW1 press                                  //
//////////////////////////////////////////////////////////////////////

#include <p18F2525.h>
#include <delays.h>
#include <timers.h>
#include <pwm.h>
#include <adc.h>
// #include <usart.h>
// #include <stdlib.h>

//********** Configuration bits **********//
#pragma config OSC      = INTIO7    //Internal oscillator, output to pin 10
#pragma config FCMEN    = OFF       //Fail-safe clock monitor disabled
#pragma config IESO = OFF       //Int.-ext. oscillator switch over disabled
#pragma config PWRT = OFF       //Power-up timer disabled
#pragma config BOREN    = OFF       //Brown-out reset disabled
#pragma config WDT      = OFF       //Watch-dog timer disabled
#pragma config MCLRE    = ON        //Master clear enabled
#pragma config LPT1OSC  = OFF       //T1 oscillator disabled
#pragma config PBADEN   = OFF       //Port B analogue-digital disabled on reset
#pragma config STVREN   = OFF       //Stack overflow reset disabled
#pragma config LVP      = OFF       //Low voltage programming disabled
#pragma config XINST    = OFF       //XINST disabled
#pragma config DEBUG    = OFF       //Background debugger disabled

#pragma config CP0      = OFF       //Code protection disabled
#pragma config CP1      = OFF
#pragma config CP2      = OFF

#pragma config WRT0 = OFF       //Write protection disabled
#pragma config WRT1 = OFF
#pragma config WRT2 = OFF

#pragma config WRTB = OFF       //Boot block write protection
#pragma config WRTC = OFF       //Config reg write protection
#pragma config WRTD = OFF       //Data EEPROM write protection

#pragma config EBTR0    = OFF       //Table read write protection
#pragma config EBTR1    = OFF
#pragma config EBTR2    = OFF

#pragma config EBTRB    = OFF       //Boot block table write protection

#pragma config CPB      = OFF       //Boot block code protection disabled
#pragma config CPD      = OFF       //Data EEPROM code protection disabled


// Definitions /////////////////////////

#define LED         LATCbits.LATC0
#define LED0        LATCbits.LATC0
#define LED1        LATCbits.LATC3

#define MOTOR       LATCbits.LATC0
#define FBCTL       TODO
#define FBOUT       TODO

// goto switches
#define SW1         PORTBbits.RB0
#define SW2         PORTBbits.RB1
#define SW3         PORTBbits.RB2

// power / steering controller
#define POWBK       LATCbits.LATC1
#define POWFT       LATCbits.LATC6
#define CTL         LATAbits.LATA0
#define FORBAK      LATAbits.LATA6
#define STEER       LATCbits.LATC5

#define GO          PORTBbits.RB0

// speed controller
#define SPDL0       PORTBbits.RB0
#define SPDL1       PORTBbits.RB6

// human-readable logic values
#define ON          1
#define OFF         0
#define FORWARD     1
#define BACKWARD    0
#define LEFT        0
#define RIGHT       1
#define TRUE        1
#define FALSE       0


// Code /////////////////////////
#pragma code

// simple LED with switches
#if 0
void main(void) {
  TRISA = 0x00;
  TRISB = 0x01;
  TRISC = 0x00;

  LED0 = OFF;

  while (1) {
    if (SW1 == 0) {
      LED0 = ON;
    } else {
      LED0 = OFF;
    }
  }  // while (1)

}
#endif


// flashing LEDs
#if 0
void main(void) {
  TRISA = 0x00;
  TRISB = 0x01;
  TRISC = 0x00;

  LED0 = OFF;
  LED1 = OFF;

  while (1) {
    LED1 = !LED1;
    LED0 = !LED0; Delay1KTCYx(125 * 1e3);
    LED0 = !LED0; Delay1KTCYx(125 * 1e3);
    LED0 = !LED0; Delay1KTCYx(125 * 1e3);
    LED0 = !LED0; Delay1KTCYx(125 * 1e3);
    LED0 = !LED0; Delay1KTCYx(125 * 1e3);
  }

}
#endif


// powering the motor
#if 0
void main(void) {
  TRISA = 0x00;
  TRISB = 0x01;
  TRISC = 0x00;

  while (1) {
    if (SW1 == 0) {
      MOTOR = ON;
      Delay1KTCYx(250 * 1e3);
      MOTOR = OFF;
    } else {
      MOTOR = OFF;
    }
  }

}
#endif


// going either ways - controlled by PIC
#if 0
void main(void) {
  TRISA = 0x00;     // 0000 0000
  TRISB = 0x01;     // 0000 0001
  TRISC = 0x00;     // 0000 0000

  while (1) {
    if (SW1 == 0) {
      // drive forward for 2s
      CTL = FORWARD; POW = ON;
      Delay1KTCYx(2 * 500 * 1e3);
      // stop for 1s
      POW = OFF;
      Delay1KTCYx(1 * 500 * 1e3);
      // forward for 4s
      POW = ON;
      Delay1KTCYx(4 * 500 * 1e3);
      // stop for 1s
      POW = OFF;
      Delay1KTCYx(1 * 500 * 1e3);
      // backwards for 3s
      CTL = BACKWARD; POW = ON;
      Delay1KTCYx(3 * 500 * 1e3);
    }
    // stop
    POW = OFF;
  }

}
#endif


// going either ways - controlled by button
#if 0
void main(void) {
  TRISA = 0x00;     // 0000 0000
  TRISB = 0x41;     // 0100 0001
  TRISC = 0x00;     // 0000 0000

  while (1) {

    if (SW2 ^ SW1) { POW = ON; }
    else { POW = OFF; }

    if (SW1) {
      CTL = FORWARD;
    } else {
      CTL = BACKWARD;
    }

  }
}
#endif


// control motor speed via PWM
#if 0
void main(void) {
  TRISA = 0x00;     // 0000 0000
  TRISB = 0x41;     // 0100 0001
  TRISC = 0x00;     // 0000 0000

  POW = ON; CTL = FORWARD;
  OpenPWM2(0xff);
  SetDCPWM2(0x0ff);

  while (1) {
    if (SPDL1 & SPDL0) {
      SetDCPWM2(0x0ff);
    } else if (SPDL1 & !SPDL0) {
      SetDCPWM2(0x1ff);
    } else if (!SPDL1 & SPDL0) {
      SetDCPWM2(0x2ff);
    } else if (!SPDL1 & !SPDL0) {
      SetDCPWM2(0x3ff);
    }
  }

}
#endif


// basic steering
#if 0
void main(void) {
  TRISA = 0x00;     // 0000 0000
  TRISB = 0x07;     // 0000 0111
  TRISC = 0x00;     // 0000 0000

  while (1) {

    if (SW2 ^ SW1) {    // steer and run
      if (!SW1) { STEER = LEFT; }
      else { STEER = RIGHT; }
      POWBK = ON; POWFT = ON;
    } else {    // go straight or stop
      POWFT = OFF;
      if (!SW1) { POWBK = ON; }
      else { POWBK = OFF; }
    }

    // forward / backward
    if (!SW3) { FORBAK = BACKWARD; }
    else { FORBAK = FORWARD; }

  }

}
#endif


// ADC driver
#if 0
int result, busy;

void main(void) {
  TRISA = 0b00000001;
  TRISB = 0x00;     // 0000 0000
  TRISC = 0x00;     // 0000 0000

  //LED = ON;

  OpenADC(
      ADC_FOSC_32       &
      ADC_RIGHT_JUST    &
      ADC_12_TAD,
      ANCHL           &
      ADC_REF_VDD_VSS   &
      ADC_INT_OFF,
      ADC_1ANA
  );

  while (1) {

    while (!BusyADC()) {
      if (ReadADC() > 0x1ff) {
        LED = ON;
      } else {
        LED = OFF;
      }
      ConvertADC();
    }

  }

}
#endif


// Final Project (Line-Following Meth.)
#if 0
#define ANCHL   ADC_CH2
#define ANCHR   ADC_CH1
int started = 0;
unsigned char snsr_chan = ANCHL;
unsigned snsr_val = 0b00;   // sensor state values

unsigned char tmp;

unsigned char
cmpr_val(int val) {
  switch (snsr_chan) {
    case ANCHL: {     // left sensor
      if (val > 0x07) { return 0b1; }
      return 0b0;
    }
    case ANCHR: {     // right sensor
      if (val > 0x07) { return 0b1; }
      return 0b0;
    }
  }
  // won't reach here!
  return 0b0;
}

unsigned char
start_read_snsr(void) {
  SetChanADC(snsr_chan);
  if (!BusyADC()) {
    ConvertADC();
    return 1;
  } else {
    return 0;
  }
}

unsigned char
set_snsr_chan(void) {
  switch (snsr_chan) {
    case ANCHL: {
      snsr_chan = ANCHR;
      break;
    }
    case ANCHR: {
      snsr_chan = ANCHL;
      break;
    }
  }
  return snsr_chan;
}

void
set_snsr_val(void) {
  tmp = cmpr_val(ReadADC());
  switch (snsr_chan) {
    case ANCHL: {     // left sensor - low on dark
      snsr_val = (snsr_val & 0b01) | (tmp << 1);
      break;
    }
    case ANCHR: {     // right sensor - high on dark
      // NOTE: converted to low on dark, keep mnemonic!
      snsr_val = (snsr_val & 0b10) | (tmp);
      break;
    }
  }
  return;
}

void
set_steering(void) {
  switch (snsr_val) {
    case 0b00: {
      POWFT = OFF;
      break;
    }
    case 0b10: {
      STEER = RIGHT;
      POWFT = ON;
      break;
    }
    case 0b01: {
      STEER = LEFT;
      POWFT = ON;
      break;
    }
    case 0b11: { break; }
  }
}

void main(void) {
  TRISA = 0b00000110;   // RA1, RA2 analog input
  TRISB = 0b00000001;   // RB0 GO switch
  TRISC = 0b00000000;

  OpenPWM2(0xff); SetDCPWM2(0x001);
  POWBK = OFF; POWFT = OFF;

  OpenADC(
      ADC_FOSC_32       &
      ADC_RIGHT_JUST    &
      ADC_12_TAD,
      ANCHR             &   // input chanel, will be modified
      ADC_REF_VDD_VSS   &
      ADC_INT_OFF,
      ADC_3ANA
  );

  while (1) {

    // - init POWs
    if (!GO) { SetDCPWM2(0x1ef); }

    // - read sensor vals
    if (!BusyADC()) {
      // -- write state to ram
      set_snsr_val();
      // -- switch channel
      set_snsr_chan();
      // -- read next
      start_read_snsr();
    }

    // - steer and go
    set_steering();

    // DEBUG: check state with voltage meter
    if (snsr_val & 0b01) { LATBbits.LATB5 = 1; }
    else { LATBbits.LATB5 = 0; }
    if (snsr_val & 0b10) { LATBbits.LATB4 = 1; }
    else { LATBbits.LATB4 = 0; }

  }     // endwhile

}
#endif


// Final Project (Hard Program Meth.)
#if 1
int done = TRUE;

void main(void) {
  TRISA = 0b00000110;   // RA1, RA2 analog input
  TRISB = 0b00000001;   // RB0 GO switch
  TRISC = 0b00000000;

  FORBAK = FORWARD;

  OpenPWM2(0xff);
  SetDCPWM2(0x000);

  while (1) {

    if (!GO) { done = FALSE; }

    if (!done) {

      POWFT = OFF;

      SetDCPWM2(0x0ff); Delay1KTCYx(2 * 1e6);
      SetDCPWM2(0x1ff); Delay1KTCYx(2 * 1e6);
      SetDCPWM2(0x2ff); Delay1KTCYx(1 * 1e6);
      SetDCPWM2(0x3ff); Delay1KTCYx(1 * 1e6);
      Delay1KTCYx(4 * 1e6);

      // DRIFT!
      STEER = LEFT;
      POWFT = ON;
      Delay1KTCYx(4 * 1e5);
      STEER = RIGHT;
      Delay1KTCYx(1 * 1e5);
      POWFT = OFF;

      Delay1KTCYx(1 * 1e6);

      SetDCPWM2(0x000);

      done = TRUE;
    
    }   // endwhile

  }

}
#endif

