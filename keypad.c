/*
 * File:   main.c
 * Author: True Administrator
 *
 * Created on July 18, 2016, 12:11 PM
 */


#include <xc.h>
#include <stdio.h>
#include "configBits.h"
#include "constants.h"
#include "lcd.h"


const char keys[] = "123A456B789C*0#D"; 

void main(void) {
    TRISC = 0x00;
    TRISD = 0x00;   //All output mode
    TRISB = 0xFF;   //All input mode
    LATB = 0x00; 
    LATC = 0x00;
    ADCON0 = 0x00;  //Disable ADC
    ADCON1 = 0xFF;  //Set PORTB to be digital instead of analog default  
    initLCD();
    
    while(1){
        while(PORTBbits.RB1 == 0){ 
            // RB1 is the interrupt pin, so if there is no key pressed, RB1 will be 0
            // the PIC will wait and do nothing until a key press is signaled
        }
        unsigned char keypress = (PORTB & 0xF0)>>4; // Read the 4 bit character code
        while(PORTBbits.RB1 == 1){
            // Wait until the key has been released
        }
        Nop();  //Apply breakpoint here because of compiler optimizations
        Nop();
        unsigned char temp = keys[keypress];
        putch(temp);   // Push the character to be displayed on the LCD
    }
    
    return;
}
