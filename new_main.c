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
#include "I2C.h"
#include "macros.h"
#include "helpers.h"


// global variables 
unsigned int is_active = 1; // top level flag 
unsigned int started = 0;   // starting flag 
unsigned int ended = 0;     //ending flag
unsigned int quit = 0;      // quit info display page 

unsigned int elapsed_time = 0;

// battery info
unsigned int total_num = 0;
unsigned int AA_num = 0;
unsigned int C_num = 0;
unsigned int Nine_num = 0;
unsigned int Drain_num = 0;

// page info
unsigned int page = 0;


// main function 
void main(void) {

    // <editor-fold defaultstate="collapsed" desc=" STARTUP SEQUENCE ">
    
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
    
    ADCON0 = 0x00;  //Disable ADC
    ADCON1 = 0xFF;  //Set PORTB to be digital instead of analog default  
    
    nRBPU = 0;
    INT1IE = 1;
    ei();           //Enable all interrupts

    //</editor-fold>
    
    //variables for storing logs 

    while(1) {

        // initializations and prompt to start
        initLCD();
        di();       
        printf("press * to start");
        __lcd_nextline();
        printf("press # for");
        ei();

        while(page == HOME) {

        }

        while(page == TIME) {

        }

        while(page == AA_BAT) {

        }

        while() {

        }

        while() {

        }


    }
    
}



