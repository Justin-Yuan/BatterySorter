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

#define __delay_1s() for(char i=0;i<100;i++){__delay_ms(10);}
#define __lcd_newline() lcdInst(0b11000000);
#define __lcd_clear() lcdInst(0x01);
#define __lcd_home() lcdInst(0b10000000);


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
    INT1IE = 1;
    ei();           //Enable all interrupts
    
    while(1){
        di();     //Prevent LCD transmission from being corrupted midway
        printf("Chocolate? ");
        __lcd_home();
        ei();   // XC8 default routine, enable all interrupt
        __delay_1s();
        __delay_1s();
        __delay_1s();
        __delay_1s();
        di();   // XC8 default routine, disable all interrupt
        printf("CHOCOLATE?!");
        __lcd_home();
        ei();
        __delay_1s();
        __delay_1s();
    }
    
    return;
}

void interrupt keypressed(void) {
    if(INT1IF){
        __lcd_newline();
        unsigned char keypress = (PORTB & 0xF0) >> 4;
        putch(keys[keypress]);
        __lcd_home();
        INT1IF = 0;     //Clear flag bit
    }
}


