/*
 * File:   main.c
 * Author: True Administrator
 *
 * Created on July 18, 2016, 12:11 PM
 */

// page selection 
#define HOME 0
#define TIME 1
#define TOTAL_BAT 2
#define AA_BAT 3
#define C_BAT 4
#define NINE_BAT 5
#define DRAIN_BAT 6
#define HISTORY 7

// global constant variables
const char keys[] = "123A456B789C*0#D"; 
const char happynewyear[7] = {  0x00, //45 Seconds 
                            0x00, //59 Minutes
                            0x00, //24 hour mode, set to 23:00
                            0x02, //Saturday 
                            0x6, //31st
                            0x02, //December
                            0x17};//2016


// function declarations 
void set_time(void);
void current_time(unsigned char*);
void select_menu(unsigned char);
int calculate_elapsed_time(unsigned char*);
void termination(unsigned int);


