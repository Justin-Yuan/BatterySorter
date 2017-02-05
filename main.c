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

// function declarations 
void set_time(void);
void current_time(unsigned char* );
void print_message(unsigned char );
int calculate_elapsed_time(unsigned char* );

// global constant variables
const char keys[] = "123A456B789C*0#D"; 
const char happynewyear[7] = {  0x00, //45 Seconds 
                            0x00, //59 Minutes
                            0x00, //24 hour mode, set to 23:00
                            0x02, //Saturday 
                            0x6, //31st
                            0x02, //December
                            0x17};//2016

// global variables 
unsigned int is_active = 1; // top level flag 
unsigned int started = 0;   // starting flag 
unsigned int ended = 0;     //ending flag 
unsigned int elapsed_time = 0;
unsigned int total_num = 0;
unsigned int AA_num = 0;
unsigned int C_num = 0;
unsigned int Nine_num = 0;

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
    
    initLCD();
    __lcd_home();
    printf("press * to start");
    
    unsigned char time[7];
    I2C_Master_Init(10000); //Initialize I2C Master with 100KHz clock
    di(); // Disable all interrupts

    
    while(1){
        // while(PORTBbits.RB1 == 0){ 
        //     // RB1 is the interrupt pin, so if there is no key pressed, RB1 will be 0
        //     // the PIC will wait and do nothing until a key press is signaled
        // }
        // unsigned char keypress = (PORTB & 0xF0)>>4; // Read the 4 bit character code
        // while(PORTBbits.RB1 == 1){
        //     // Wait until the key has been released
        // }
        // Nop();  //Apply breakpoint here because of compiler optimizations
        // Nop();
        // unsigned char temp = keys[keypress];
        // putch(temp);   // Push the character to be displayed on the LCD

        di();     //Prevent LCD transmission from being corrupted midway
        printf("Chocolate? ");
        __lcd_home();
        ei();   // XC8 default routine, enable all interrupt
        __delay_1s();
        __delay_1s();
        __delay_1s();
        __delay_1s();


       current_time(time);
        
    
    }
    
    return;
}

/*######################################################################################*/

void interrupt keypressed(void) {
    if(INT1IF){
        __lcd_newline();
        unsigned char keypress = (PORTB & 0xF0) >> 4;
        putch(keys[keypress]);
        __lcd_home();
        INT1IF = 0;     //Clear flag bit
    }
}

void set_time(void) {
    I2C_Master_Start(); //Start condition
    I2C_Master_Write(0b11010000); //7 bit RTC address + Write
    I2C_Master_Write(0x00); //Set memory pointer to seconds
    for(char i=0; i<7; i++){
        I2C_Master_Write(happynewyear[i]);
    }    
    I2C_Master_Stop(); //Stop condition
}

void current_time(unsigned char* time) {
        //Reset RTC memory pointer 
        I2C_Master_Start(); //Start condition
        I2C_Master_Write(0b11010000); //7 bit RTC address + Write
        I2C_Master_Write(0x00); //Set memory pointer to seconds
        I2C_Master_Stop(); //Stop condition

        //Read Current Time
        I2C_Master_Start();
        I2C_Master_Write(0b11010001); //7 bit RTC address + Read
        for(unsigned char i=0;i<0x06;i++){
            time[i] = I2C_Master_Read(1);
        }
        time[6] = I2C_Master_Read(0);       //Final Read without ack
        I2C_Master_Stop();
        __lcd_home();
        printf("%02x/%02x/%02x", time[6],time[5],time[4]);    //Print date in YY/MM/DD
        __lcd_newline();
        printf("%02x:%02x:%02x", time[2],time[1],time[0]);    //HH:MM:SS
        // __delay_1s();
}

void print_message(unsigned char temp) {
    if (temp == '*') {      // start sorting
        printf("start in 5s ...");
        for(int i = 0; i < 5; i++) { __delay_1s(); }
        __lcd_clear();
        started = 1;
        set_time();
    } 
    else if (temp == '#') { // emergency stop 
        is_active = 0;
    }
    else if (temp == '0') { // home page
        __lcd_clear();
        __lcd_home();
        printf("<4>-time  <5>-total");
        __lcd_newline();
        printf("<7>-AA  <8>-C  <9>-9V");
    }
    else if (temp == '4') {
        __lcd_clear();
        __lcd_home();
        printf("elapsed time: %d", elapsed_time);
        __lcd_newline();
        printf("<0> HOME");
    }
    else if (temp == '5') {
        __lcd_clear();
        __lcd_home();
        printf("num of total: %d", total_num);
        __lcd_newline();
        printf("<0> HOME");
    }
    else if (temp == '7') {
        __lcd_clear();
        __lcd_home();
        printf("num of AA: %d", AA_num);
        __lcd_newline();
        printf("<0> HOME");
    }
    else if (temp == '8') {
        __lcd_clear();
        __lcd_home();
        printf("num of C: %d", C_num);
        __lcd_newline();
        printf("<0> HOME");
    }
    else if (temp == '9') {
        __lcd_clear();
        __lcd_home();
        printf("num of 9V: %d", Nine_num);
        __lcd_newline();
        printf("<0> HOME");
    }

}

int calculate_elapsed_time(unsigned char* time) {
    return (__bcd_to_num(time[0] + 60*__bcd_to_num(time[1]));
}
