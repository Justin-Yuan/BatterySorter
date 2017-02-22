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
unsigned int is_active = 1; // run sorting operation

unsigned int started = 0;   // starting flag 
unsigned int ended = 0;     //ending flag
unsigned int quit = 0;      // quit info display page 


// battery info
unsigned int total_num = 0;
unsigned int AA_num = 0;
unsigned int C_num = 0;
unsigned int Nine_num = 0;
unsigned int Drain_num = 0;

// menu info
unsigned int menu = HOME;

// time info
unsigned int elapsed_time = 0;
unsigned int is_wait = 0;

// log info
unsigned int log = 0;

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
    

    // initializations and prompt to start
    di();       
    initLCD();
    ei();


    // main execution loop
    while(1) {

        if(is_active) {
            LATC = LATC | 0b00000010; //RC1 = 1 , free keypad pins
            set_time();

            while(total_num < 15 && !is_wait) {



                current_time(time);
                elapsed_time = calculate_elapsed_time(time);
                __lcd_clear();
                __lcd_home();
                printf("  %3d", elapsed_time);
                __delay_ms(300);
                termination(elapsed_time);
            }

            LATC = LATC && 0b11111101; // RC1 = 0 enable keypad 
            is_active = !is_active; // reset the operation flag 
        } else if (log != 0) {

            log = 0;    //reset log flag 
        } else {

            while(menu == HOME && !is_active) {
                di();
                __lcd_clear();
                __lcd_home();
                printf("press * to run");
                __lcd_newline();
                printf("<< 7 DATA 9 >>");
                ei();
            }

            while(menu == TIME) {
                di();
                __lcd_clear();
                __lcd_home();
                printf("elapsed time: %d", elapsed_time);
                __lcd_newline();
                printf("<< 7 DATA 9 >>");
                ei();
            }

            while(menu == TOTAL_BAT) {
                di();
                __lcd_clear();
                __lcd_home();
                printf("total: %d", total_num);
                __lcd_newline();
                printf("<< 7 DATA 9 >>");
                ei();
            }

            while(menu == AA_BAT) {
                di();
                __lcd_clear();
                __lcd_home();
                printf("AA sorted: %d", AA_num);
                __lcd_newline();
                printf("<< 7 DATA 9 >>");
                ei();
            }

            while(menu == C_BAT) {
                di();
                __lcd_clear();
                __lcd_home();
                printf("C sorted: %d", C_num);
                __lcd_newline();
                printf("<< 7 DATA 9 >>");
                ei();
            }

            while(menu == NINE_BAT) {
                di();
                __lcd_clear();
                __lcd_home();
                printf("9V sorted: %d", Nine_num);
                __lcd_newline();
                printf("<< 7 DATA 9 >>");
                ei();
            }

            while(menu == DRAIN_BAT) {
                di();
                __lcd_clear();
                __lcd_home();
                printf("Drained: %d", Drain_num);
                __lcd_newline();
                printf("<< 7 DATA 9 >>");
                ei();
            }

            while(menu == HISTORY) {
                di();
                __lcd_clear();
                __lcd_home();
                printf("op #: 1 2 3");
                __lcd_newline();
                printf("<< 7 DATA 9 >>");
                ei();
            }

        }   // end of the if(is_active)-else
    }   // end of the while(1) loop
    return ;   
}


/**
 * [set_time description]
 */
void set_time(void) {
    I2C_Master_Start(); //Start condition
    I2C_Master_Write(0b11010000); //7 bit RTC address + Write
    I2C_Master_Write(0x00); //Set memory pointer to seconds
    for(char i=0; i<7; i++){
        I2C_Master_Write(happynewyear[i]);
    }    
    I2C_Master_Stop(); //Stop condition
}

/**
 * [keypressed description]
 * @return  [description]
 */
void interrupt keypressed(void) {
    if(INT1IF){
        // __lcd_newline();
        unsigned char keypress = (PORTB & 0xF0) >> 4;
        selectMenu(keys[keypress]);
        INT1IF = 0;     //Clear flag bit
    }
}

/**
 * switch between different menus 
 * @param temp : key pressed 
 */
void selectMenu(unsigned char temp) {
    extern unsigned int menu;
    extern unsigned int log;
    extern unsigned int is_active;
    switch(menu) {
        case HOME:
            if(temp == '*') {
                is_active = 1;
            } else {
                menu = (temp == '7')? TIME : AA_BAT;
            }
            break;
        case TIME:
            menu = (temp == '7')? HISTORY : HOME; 
            break;
        case TOTAL_BAT:
            menu = (temp == '7')? HOME : AA_BAT; 
            break;
        case AA_BAT:
            menu = (temp == '7')? TOTAL_BAT : C_BAT; 
            break;
        case C_BAT:
            menu = (temp == '7')? AA_BAT : NINE_BAT; 
            break;
        case NINE_BAT:
            menu = (temp == '7')? C_BAT : DRAIN_BAT; 
            break;
        case DRAIN_BAT:
            menu = (temp == '7')? NINE_BAT : HISTORY; 
            break;
        case HISTORY:
            if(temp == '1') {
                log = 1;
            } else if (temp == '2') {
                log = 2;
            } else if (temp == '3') {
                log = 3;
            } else {
                menu = (temp == '')? DRAIN_BAT : TIME; 
            }
            break;
        default:
            printf("error");
    } 
}

/**
 * [current_time description]
 * @param time [description]
 */
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
        __lcd_clear();
        __lcd_home();
        // printf("%02x/%02x/%02x", time[6],time[5],time[4]);    //Print date in YY/MM/DD
        __lcd_newline();
        // printf("%02x:%02x:%02x", time[2],time[1],time[0]);    //HH:MM:SS
        // __delay_1s();
        __delay_ms(300);
}

/**
 * [calculate_elapsed_time description]
 * @param  time [description]
 * @return      [description]
 */
int calculate_elapsed_time(unsigned char* time) {
    return (__bcd_to_num(time[0]) + 60*__bcd_to_num(time[1]));
}

/**
 * [termination description]
 * @param time_now [description]
 */
void termination(unsigned int time_now) {
    if (time_now > 70) { extern unsigned int is_wait; is_wait = 1; }
}

