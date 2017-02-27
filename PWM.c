/*
 * File:   pwm.c
 * Author: Michael Ding
 *
 * Created on July 14, 2016, 10:25 AM
 */


#include <xc.h>
#include "PWM.h"
//#include <./delay_100ms.h>
//<editor-fold defaultstate="collapsed" desc=" CONFIG BITS ">

// CONFIG1H
#pragma config OSC = INTIO7    // Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)
//</editor-fold>

#define _XTAL_FREQ 32000000
#define TMR2PRESCALE 16

//<editor-fold defaultstate="collapsed" desc="Variable Defs">
long freq; // Selected PWM Frequency
unsigned int i = 0; // Keep track of current PWM duty cycle
//</editor-fold>



void delay_10ms(unsigned char n) { 
         while (n-- != 0) { 
             __delay_ms(10); 
         } 
     }

int PWM_Max_Duty()
// Returns the register values to be set for 100% duty cycle, this is dependent
// on the PWM frequency and oscillator frequency
// https://electrosome.com/pwm-pic-microcontroller-mplab-xc8/
{
  return(_XTAL_FREQ/(freq*TMR2PRESCALE));
}
  
void set_PWM_freq(long fre)
// Sets PR2 register to match the frequency desired
// See datasheet pg 149, equation 16-1
// https://electrosome.com/pwm-pic-microcontroller-mplab-xc8/
{
  PR2 = (_XTAL_FREQ/(fre*4*TMR2PRESCALE)) - 1;
  freq = fre;
}

void set_PWM1_duty(unsigned int duty)
// Sets the duty cycle of PWM1, from 1024 to 0
// See datasheet pg 150, equation 16-1
// https://electrosome.com/pwm-pic-microcontroller-mplab-xc8/
{
  if(duty<1024)
  {
    duty = ((float)duty/1023)*PWM_Max_Duty();
    CCP1X = duty & 2; // Set the 2 lest significant bit in CCP1CON register
    CCP1Y = duty & 1;
    CCPR1L = duty>>2; // Set rest of the duty cycle bits in CCPR1L
  }
    
}

void set_PWM2_duty(unsigned int duty)
// Sets the duty cycle of PWM2, from 1024 to 0
// See datasheet pg 150, equation 16-1
// https://electrosome.com/pwm-pic-microcontroller-mplab-xc8/
{
  if(duty<1024)
  {
    duty = ((float)duty/1023)*PWM_Max_Duty();
    CCP2X = duty & 2; // Set the 2 lest significant bit in CCP2CON register
    CCP2Y = duty & 1;
    CCPR2L = duty>>2; // Set rest of the duty cycle bits in CCPR2L
  }
}

void PWM1_Start()
// START PWM1 OUTPUT, PWM1 have enhanced features, see datasheet
{
  //Configure CCP1CON, single output mode, all active high
  P1M1 = 0;
  P1M0 = 0;
  CCP1M3 = 1;
  CCP1M2 = 1;
  CCP1M1 = 0;
  CCP1M0 = 0;
  
  //Configure prescale values for Timer2, according to TMR2PRESCALAR
  #if TMR2PRESCALAR == 1
    T2CKPS0 = 0;
    T2CKPS1 = 0;
  #elif TMR2PRESCALAR == 4
    T2CKPS0 = 1;
    T2CKPS1 = 0;
  #elif TMR2PRESCALAR == 16
    T2CKPS0 = 1;
    T2CKPS1 = 1;
  #endif

  // Enable timer 2
  TMR2ON = 1;
  
  // Enable PWM output pins
  TRISCbits.TRISC2 = 0;
}

void PWM2_Start()
// START PWM2 OUTPUT
{
  //Configure CCP2CON, enter PWM mode
  CCP1M3 = 1;
  CCP1M2 = 1;
  
  //Configure prescale values for Timer2, according to TMR2PRESCALAR
  #if TMR2PRESCALAR == 1
    T2CKPS0 = 0;
    T2CKPS1 = 0;
  #elif TMR2PRESCALAR == 4
    T2CKPS0 = 1;
    T2CKPS1 = 0;
  #elif TMR2PRESCALAR == 16
    T2CKPS0 = 1;
    T2CKPS1 = 1;
  #endif

  // Enable timer 2
  TMR2ON = 1;
  
  // Enable PWM output pins
  TRISCbits.TRISC1 = 0;
}

void PWM1_Stop()
// Stop PWM1 output
{
  CCP1M3 = 0;
  CCP1M2 = 0;
  CCP1M1 = 0;
  CCP1M0 = 0;
}

void PWM2_Stop()
{
  CCP2M3 = 0;
  CCP2M2 = 0;
  CCP2M1 = 0;
  CCP2M0 = 0;
}
/*
void main(void) {
    
    OSCCON = 0xF0;  //8MHz
    
    TRISA = 0xFF; // Set Port A as all input
    TRISB = 0xFF; 
    TRISC = 0x00;
    TRISD = 0x00; //All output mode for LCD
    TRISE = 0x00;    

    LATA = 0x00;
    LATB = 0x00; 
    LATC = 0x00;
    LATD = 0x00;
    LATE = 0x00;
    
    // Set internal oscillator to run at 8 MHZ
    OSCCON = OSCCON | 0b01110000; 
    // Enable PLL for the internal oscillator, Processor now runs at 32MHZ
    OSCTUNEbits.PLLEN = 1; 
     
    // Configure PWM frequency, 50khz
    // Scale for pwm is not linear, see datasheet pg150
    set_PWM_freq (3100);
    
    PWM1_Start();
    
    // Infinite loop, cycle state on each pin
    TRISC = 0x11110001;
    set_PWM1_duty(512);
    TRISB = 0x00;
    i=920;
    while(1){
        // LATCbits.LATC3 = 1 - LATCbits.LATC3 ; // This line of close flashes RC3 every iteration of the loop
        LATCbits.LATC5 = 1;
        LATCbits.LATC6 = 0;
        set_PWM1_duty(i);
//        LATB = i>>2;
//        if(i <= 1020)
//          i= i+10;
//        else
//            i = 0;
        //delay_10ms(30); 
   }
  }
  
  */