/*
 * File:   main.c
 * Author: True Administrator
 *
 * Created on July 18, 2016, 12:11 PM
 */

#include "helpers.h"

void interrupt keypressed(void) {
    if(INT1IF){
        __lcd_newline();
        unsigned char keypress = (PORTB & 0xF0) >> 4;
        print_message(keys[keypress]);
        //__lcd_home();
        //printf("sddd");
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
        __lcd_clear();
        __lcd_home();
        printf("%02x/%02x/%02x", time[6],time[5],time[4]);    //Print date in YY/MM/DD
        __lcd_newline();
        printf("%02x:%02x:%02x", time[2],time[1],time[0]);    //HH:MM:SS
        // __delay_1s();
        __delay_ms(300);
}

void print_message(unsigned char temp) {
    if (temp == '*') {      // start sorting
        printf("start in 5s ...");
        for(unsigned int i = 0; i < 5; i++) { __delay_1s(); }
        __lcd_clear();
        extern unsigned int started;
        started = 1;
        printf("%d", started);
        //set_time();
    } 
    else if (temp == 'D') { // emergency stop
        extern unsigned int is_active;
        is_active = 0;
    }
    else if (temp == '#') { // emergency stop
        extern unsigned int quit;
        quit = 1;
    }
    else if (temp == '0') { // home page
        __lcd_clear();
        __lcd_home();
        printf("<5>time <6>total");
        __lcd_newline();
        printf("<4>NEXT <#>QUIT");
    }
    else if (temp == '4') { // home page continued
        __lcd_clear();
        __lcd_home();
        printf("<7>AA <8>C <9>9V");
        __lcd_newline();
        printf("<C>drain <B>BACK");
    }
    else if (temp == 'B') { // home page
        __lcd_clear();
        __lcd_home();
        printf("<5>time <6>total");
        __lcd_newline();
        printf("<4>NEXT <#>QUIT");
    }
    else if (temp == '5') { // elapsed time info
        __lcd_clear();
        __lcd_home();
        //extern int elapsed_time;
        printf("elapsed time: %d", elapsed_time);
        __lcd_newline();
        printf("<0>HOME <#>QUIT");
    }
    else if (temp == '6') { // total number sorted info 
        __lcd_clear();
        __lcd_home();
        printf("num of total: %d", total_num);
        __lcd_newline();
        printf("<0>HOME <#>QUIT");
    }   
    else if (temp == '7') { // number of AA orted info
        __lcd_clear();
        __lcd_home();
        printf("num of AA: %d", AA_num);
        __lcd_newline();
        printf("<0>HOME <#>QUIT");
    }
    else if (temp == '8') { // number of C sorted info 
        __lcd_clear();
        __lcd_home();
        printf("num of C: %d", C_num);
        __lcd_newline();
        printf("<0>HOME <#>QUIT");
    }
    else if (temp == '9') { // number of 9V sorted info 
        __lcd_clear();
        __lcd_home();
        printf("num of 9V: %d", Nine_num);
        __lcd_newline();
        printf("<0>HOME <#>QUIT");
    }
     else if (temp == 'C') { // number of 9V sorted info 
        __lcd_clear();
        __lcd_home();
        printf("num of drain: %d", Drain_num);
        __lcd_newline();
        printf("<0>HOME <#>QUIT");
    }
    else {                  // default 
        printf("");
    }

}

int calculate_elapsed_time(unsigned char* time) {
    return (__bcd_to_num(time[0]) + 60*__bcd_to_num(time[1]));
}

void termination(unsigned int time_now) {
    if (time_now > 10) { extern unsigned int ended; ended = 1; }
}

