/*
 * File:   main.c
 * Author: True Administrator
 *
 * Created on July 18, 2016, 12:11 PM
 */

#include "helpers.h"

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
        select_menu(keys[keypress]);
        INT1IF = 0;     //Clear flag bit
    }
}

/**
 * switch between different menus 
 * @param temp : key pressed 
 */
void select_menu(unsigned char temp) {
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

