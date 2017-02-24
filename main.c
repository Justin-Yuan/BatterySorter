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

//#include "eeprom_routines.h"
//#include "helpers.h"
//#include "PWM.h"

// page selection 
#define HOME 0
#define TIME 1
#define TOTAL_BAT 2
#define AA_BAT 3
#define C_BAT 4
#define NINE_BAT 5
#define DRAIN_BAT 6
#define HISTORY 7
#define logA 8
#define logB 9
#define logC 10

// function declarations 
void set_time(void);
void current_time(unsigned char*);
void select_menu(unsigned char);
int calculate_elapsed_time(unsigned char*);
void termination(unsigned int);
void readADC(char);
void cleanCount(void);
int Eeprom_ReadByte(int);
void Eeprom_WriteByte(int, int);

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
unsigned int is_active = 0; // run sorting operation

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
    
    ADCON0 = 0x00;  //Disable ADC
    ADCON1 = 0x1B;  //AN0 to AN3 used as analog input
    //CVRCON = 0x00; // Disable CCP reference voltage output
    CMCONbits.CIS = 0;
    ADFM = 1;
    
    ei();           //Enable all interrupts
    //</editor-fold>
    
    
    // initializations and prompt to start
    di();
    I2C_Master_Init(10000); //Initialize I2C Master with 100KHz clock
    initLCD();
    unsigned int address = 0;
    ei();


    // main execution loop
    while(1) {

        if(is_active) {
            LATCbits.LATC0 = 1; //RC0 = 1 , free keypad pins
            unsigned char time[7];
            elapsed_time = 0;
            cleanCount();
            set_time();

            unsigned int is_battery = 1;
            
/*
            OSCCON = 0xF0;  //8MHz
            // Set internal oscillator to run at 8 MHZ
            OSCCON = OSCCON | 0b01110000; 
            // Enable PLL for the internal oscillator, Processor now runs at 32MHZ
            OSCTUNEbits.PLLEN = 1; 

            set_PWM_freq (3100);
    
            PWM1_Start();

            TRISC = 0x11110001;
*/
            while(total_num < 15 && !is_wait) {
/*
                // rotate wheel & move conveyor belt
                LATCbits.LATC5 = 1;
                LATCbits.LATC6 = 0;
                set_PWM1_duty(512);

                // infrared sensor read input 
                is_battery = LATAbits.LATA1;
                __lcd_clear();
                __lcd_home();
                printf(" %d ", is_battery);
                __lcd_newline();

                
*/
                is_battery = PORTAbits.RA4 ;
                
                //readADC(2);
                //__delay_1s();
                
                if(!is_battery) {
                   // sorting logic  
                    readADC(2);
                    // bin selection


                    // push battery
                    
                    // testing 
                    AA_num++;
                    C_num += 2;
                    Nine_num += 3;
                    Drain_num += 4;
                }
                
                current_time(time);
                elapsed_time = calculate_elapsed_time(time);
                printf(" %3d %d %x %x", elapsed_time, is_battery, ADRESH, ADRESL);
                __delay_ms(300);
                termination(elapsed_time);
            }
            
            // EEPROM logging 
            Eeprom_WriteByte(address++, AA_num);
            Eeprom_WriteByte(address++, C_num);
            Eeprom_WriteByte(address++, Nine_num);
            Eeprom_WriteByte(address++, Drain_num);
            Eeprom_WriteByte(address++, elapsed_time);

            LATCbits.LATC0 = 0; // RC1 = 0 enable keypad 
            is_wait = !is_wait;
            is_active = !is_active; // reset the operation flag 
        } else if (log != 0) {
            
            while(menu == logA) {
                di();
                __lcd_home();
                printf("AA:%d C:%d 9V:%d", AA_num, C_num, Nine_num);
                __lcd_newline();
                printf("D:%d time:%d 8>>", Drain_num, elapsed_time);
                ei();
            }
            
            while(menu == logB) {
                di();
                __lcd_home();
                printf("AA:%d C:%d 9V:%d", AA_num, C_num, Nine_num);
                __lcd_newline();
                printf("D:%d time:%d 8>>", Drain_num, elapsed_time);
                ei();
            }
            
            while(menu == logC) {
                di();
                __lcd_home();
                printf("AA:%d C:%d 9V:%d", AA_num, C_num, Nine_num);
                __lcd_newline();
                printf("D:%d time:%d 8>>", Drain_num, elapsed_time);
                ei();
            }
            
        } else {

            while(menu == HOME && !is_active) {
                di();
                __lcd_home();
                printf("press * to run");
                __lcd_newline();
                printf("<< 7 DATA 9 >>");
                ei();
            }

            while(menu == TIME) {
                di();
                __lcd_home();
                printf("elapsed time: %d", elapsed_time);
                __lcd_newline();
                printf("<< 7 DATA 9 >>");
                ei();
            }

            while(menu == TOTAL_BAT) {
                di();
                __lcd_home();
                printf("total: %d", total_num);
                __lcd_newline();
                printf("<< 7 DATA 9 >>");
                ei();
            }

            while(menu == AA_BAT) {
                di();
                __lcd_home();
                printf("AA sorted: %d", AA_num);
                __lcd_newline();
                printf("<< 7 DATA 9 >>");
                ei();
            }

            while(menu == C_BAT) {
                di();
                __lcd_home();
                printf("C sorted: %d", C_num);
                __lcd_newline();
                printf("<< 7 DATA 9 >>");
                ei();
            }

            while(menu == NINE_BAT) {
                di();
                __lcd_home();
                printf("9V sorted: %d", Nine_num);
                __lcd_newline();
                printf("<< 7 DATA 9 >>");
                ei();
            }

            while(menu == DRAIN_BAT) {
                di();
                __lcd_home();
                printf("Drained: %d", Drain_num);
                __lcd_newline();
                printf("<< 7 DATA 9 >>");
                ei();
            }

            while(menu == HISTORY) {
                di();
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
        __lcd_clear();
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
                is_active = !is_active;
            } else {
                if(temp == '7'){menu = TIME;} else if(temp == '9'){menu = TOTAL_BAT;}
            }
            break;
        case TIME:
            if(temp == '7'){menu = HISTORY;} else if(temp == '9'){menu = HOME;}
            break;
        case TOTAL_BAT:
            if(temp == '7'){menu = HOME;} else if(temp == '9'){menu = AA_BAT;}
            break;
        case AA_BAT:
            if(temp == '7'){menu = TOTAL_BAT;} else if(temp == '9'){menu = C_BAT;}
            break;
        case C_BAT:
            if(temp == '7'){menu = AA_BAT;} else if(temp == '9'){menu = NINE_BAT;}
            break;
        case NINE_BAT:
            if(temp == '7'){menu = C_BAT;} else if(temp == '9'){menu = DRAIN_BAT;}
            break;
        case DRAIN_BAT:
            if(temp == '7'){menu = NINE_BAT;} else if(temp == '9'){menu = HISTORY;}
            break;
        case HISTORY:
            if(temp == '1') {
                log = 1;
                menu = logA;
            } else if (temp == '2') {
                log = 2;
                menu = logB;
            } else if (temp == '3') {
                log = 3;
                menu = logC;
            } else {
                if(temp == '7'){menu = DRAIN_BAT;} else if(temp == '9'){menu = TIME;}
            }
            break;
        case logA:
        case logB:
        case logC:
            if(temp == '8') {
                menu = HISTORY;
                log = 0;
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
    if (time_now > 50) { extern unsigned int is_wait; is_wait = 1; }
}

/**
 * 
 * @param channel
 */
void readADC(char channel) {
    // Select A2D channel to read
    ADCON0 = ((channel <<2));
    ADON = 1;
    ADCON0bits.GO = 1;
   while(ADCON0bits.GO_NOT_DONE){__delay_ms(5);}    
}

/**
 * 
 */
void cleanCount(void) {
    extern unsigned int AA_num, C_num, Nine_num, Drain_num, elapsed_time;
    AA_num = 0;
    C_num = 0;
    Nine_num = 0;
    Drain_num = 0;
    elapsed_time = 0;
}

/**
 * 
 * @param address
 * @return 
 */
int Eeprom_ReadByte(int address) {
    // Set address registers
    EEADRH = (address >> 8);
    EEADR = address;

    EECON1bits.EEPGD = 0;       // Select EEPROM Data Memory
    EECON1bits.CFGS = 0;        // Access flash/EEPROM NOT config. registers
    EECON1bits.RD = 1;          // Start a read cycle

    // A read should only take one cycle, and then the hardware will clear
    // the RD bit
    while(EECON1bits.RD == 1);

    return EEDATA;              // Return data
}

/**
 * 
 * @param address
 * @param data
 */
void Eeprom_WriteByte(int address, int data) {    
    // Set address registers
    EEADRH = (address >> 8);
    EEADR = address;

    EEDATA = data;          // Write data we want to write to SFR
    EECON1bits.EEPGD = 0;   // Select EEPROM data memory
    EECON1bits.CFGS = 0;    // Access flash/EEPROM NOT config. registers
    EECON1bits.WREN = 1;    // Enable writing of EEPROM (this is disabled again after the write completes)

    // The next three lines of code perform the required operations to
    // initiate a EEPROM write
    EECON2 = 0x55;          // Part of required sequence for write to internal EEPROM
    EECON2 = 0xAA;          // Part of required sequence for write to internal EEPROM
    EECON1bits.WR = 1;      // Part of required sequence for write to internal EEPROM

    // Loop until write operation is complete
    while(PIR2bits.EEIF == 0)
    {
        continue;   // Do nothing, are just waiting
    }

    PIR2bits.EEIF = 0;      //Clearing EEIF bit (this MUST be cleared in software after each write)
    EECON1bits.WREN = 0;    // Disable write (for safety, it is re-enabled next time a EEPROM write is performed)
}