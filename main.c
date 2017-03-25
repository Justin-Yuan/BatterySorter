/*
 * File:   main.c
 * Author: True Administrator
 *
 * Created on July 18, 2016, 12:11 PM
 */

#include <xc.h>
#include <stdio.h>
#include <stdint.h>
#include "configBits.h"
#include "constants.h"
#include "lcd.h"
#include "I2C.h"
#include "macros.h"

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
#define STANDBY 11
#define TEST 12
#define RETREAT 13
#define PUSH 14

// function declarations 
void set_time(void);
void current_time(unsigned char*);
void select_menu(unsigned char);
int calculate_elapsed_time(unsigned char*);
int is_termination(unsigned int);

void readADC(char);
float convert_voltage();
float average_voltage(char, unsigned int);
float convert_to_valid_voltage(float);
float get_voltage(char, unsigned int);

void clean_count(void);
void clean_motor_status();

uint8_t Eeprom_ReadByte(uint16_t);
void Eeprom_WriteByte(uint16_t, uint8_t);
uint16_t next_address(uint16_t);
void show_log(uint16_t);
// <TODO>
void pulse_delay(unsigned int);
void set_servo_duration(char, char, unsigned int);
void move_stepper1(unsigned int);
void move_stepper2(unsigned int);
void clear_stepper(char);
void set_stepper_duration(char, unsigned int);
void move_stepper_angle(char, float, char);
void clear_dc(char);
void move_dc1();
void move_dc2(char);
void move_servo(unsigned int);
void move_stepper(unsigned int, char);


// battery testing stage
unsigned int regulate_signal(unsigned int);
unsigned int ir(unsigned int, unsigned int); 
unsigned int microswitch(unsigned int);
void rotate_bin(char direction, char degree);
unsigned int test_battery();
void move_bin(unsigned int);
void push_battery(unsigned int);
void move_probes(char, unsigned int);

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

// battery info
unsigned int total_num = 0;
unsigned int AA_num = 0;
unsigned int C_num = 0;
unsigned int Nine_num = 0;
unsigned int Drain_num = 0;
unsigned int previous_type = DRAIN_BAT;

// menu info
unsigned int menu = HOME;

// time info
unsigned int elapsed_time = 0;
unsigned int is_wait = 0;

// log info
unsigned int log = 0;

// pusher info 
unsigned int is_up = 0;

// dc info
unsigned int dc_count = 0;
unsigned int dc_duration_low = 1;
unsigned int dc_duration_high = 20;
unsigned int stop_count = 0;
unsigned int stop_inactive = 600;
unsigned int stop_active = 1200;

// servo duration/count info
unsigned int servo1_active = 0;
unsigned int servo1_high = 1;
unsigned int servo1_low = 0;
unsigned int servo2_active = 0;
unsigned int servo2_high = 1;
unsigned int servo2_low = 0;

unsigned int servo1_high_duration = 3;
unsigned int servo1_low_duration = 37;
unsigned int servo2_high_duration = 3;
unsigned int servo2_low_duration = 37;

//stepper degree counter 
unsigned int stepper1_active = 0;   // standby: 0, positive spin: 1, negative spin: 2
unsigned int stepper1_duration_count = 0;     
unsigned int stepper2_active = 0;
unsigned int stepper2_duration_count = 0;

unsigned int stepper1_duration = 5000;    // pusher operation time 5s 
unsigned int stepper2_duration = 100;

// main function 
void main(void) {
    // <editor-fold defaultstate="collapsed" desc=" STARTUP SEQUENCE ">
    TRISA = 0x04; // Set Port A as all input
    TRISB = 0xFF; 
    TRISC = 0x00;
    TRISD = 0x00; //All output mode for LCD
    TRISE = 0x07;    

    LATA = 0x00;
    LATB = 0x00; 
    LATC = 0x00;
    LATD = 0x00;
    LATE = 0x00;
    
    nRBPU = 0;
    INT1IE = 1;	    //enable external interrupt for keypad
    INT1IF = 0;	    //turn off external interrupt flag
    
    ADCON0 = 0x00;  //Disable ADC
    ADCON1 = 0x0B;  //AN0 to AN3 used as analog input
    CVRCON = 0x00; // Disable CCP reference voltage output
    CMCONbits.CIS = 0;
    ADFM = 1;
    TRISA = 0x04;

    GIE = 1;        //globally enable interrupt
    PEIE = 1;       //enable peripheral interrupt
    TMR0IE = 1;     //enable TMR0 overflow interrupt
    TMR0IF = 0;	    //turn off TMR0 overflow interrupt flag
    TMR1IE = 1;     //enable TMR1 overflow interrupt
    TMR1IF = 0;     //turn off TMR1 overflow interrupt flag
    TMR3IE = 1;     //enable TMR2 overflow interrupt
    TMR3IF = 0;     //turn off TMR2 overflow interrupt flag

    // intialize timers for interrupt 
    // timer 0 
    T0CON = 0b00000111;
    TMR0 = 55770;
    // timer 1
    T1CON = 0b10000000;
    TMR1 = 65285;
    // timer 2
    T2CON = 0b10000000;
    TMR2 = 53035;
    // timer 3
    T3CON = 0b10000000;
    TMR3 = 63785; //60535;  // need to determine the third timer, 65535-0.005*10000000/4
    
    ei();           //Enable all interrupts
    //</editor-fold>

    
    // initializations and prompt to start
    di();
    //I2C_Master_Init(10000); //Initialize I2C Master with 100KHz clock
    initLCD();
    uint16_t address = 0;
    ei();


    // main execution loop
    while(1) {

        if(is_active) {
            // LATCbits.LATC6 = 1; //RC6 = 1 , free keypad pins
            // unsigned char time[7];
            elapsed_time = 1;
            clean_count();
           // set_time();

            unsigned int is_battery_bottom = 0;
            unsigned int is_battery_up = 0;
            unsigned int is_hit = 1;
            unsigned int result = 0;
            float voltage = 0;

            // turn on background timers 
            T0CONbits.TMR0ON = 1; 
            T2CONbits.TMR2ON = 1;    
            T3CONbits.TMR3ON = 1;
            // turn on control timer
            T1CONbits.TMR1ON = 1; 
            
            // operation loop 
            while(total_num < 15 && !is_wait) {
        
                // LATCbits.LATC0 = 1;
                // LATCbits.LATC1 = 1;

                // testing 
                // is_battery =  1;
                // real logic: ir input 
                is_battery_bottom = ir(1, 5);
                is_battery_up = ir(2, 5);
                is_hit = microswitch(5);

                // TESTING
               LATAbits.LATA3 = 1;
               LATAbits.LATA5 = 1;

/**********************************************************************************/

                // if(is_battery_bottom && !is_battery_up) {
                //     // pusher up 
                //     while(!is_battery_up) {
                //         move_stepper(1, 1);  consider changing to an certain angle 
                //         is_battery_up = ir(2, 5);
                //     }
                // }
                // if(is_battery_bottom && is_battery_up) {
                //     // probes down 
                //     move_probes(RETREAT);
                //     move_probes(TEST);

                //     // sorting logic
                //     result = test_battery();

                //     // bin selection
                //     move_bin(result);

                //     // push battery
                //     move_probes(TEST);
                //     move_probes(RETREAT);
                //     move_probes(PUSH);
                //     move_probes(RETREAT);
                    
                //     // pusher down
                    // while(!is_hit) {
                    //     move_stepper(2, 1);  consider changing to an angle too 
                    //     is_hit = microswitch(5);
                    // }
                // }

/**********************************************************************************/
                // Testing

                // while (ir(1, 5)) { 
                //     // move_stepper_angle(1, 180, 1);
                //     set_servo_duration(1, 1, 12);
                //     set_servo_duration(1, 0, 188);
                //     move_servo(1);
                // }

                // while(!ir(1,5)) {
                //     set_servo_duration(1, 1, 13);
                //     set_servo_duration(1, 0, 187);
                // }
                // move_servo(0);

                // while (ir(2, 5)) { 
                //     // move_stepper_angle(1, 180, 1);
                //     set_servo_duration(1, 1, 3);
                //     set_servo_duration(1, 0, 37);
                //     move_servo(1);
                // }
                // move_servo(0);

                // while (microswitch(5)) {
                //     set_servo_duration(1, 1, 2);
                //     set_servo_duration(1, 0, 38);
                //     move_servo(1);
                // }
                // move_servo(0);

                // if (is_battery_up) {
                //     for(unsigned int i = 0; i < 20; i++) {
                //         move_probes(RETREAT, 300);
                //     }
                //     for(unsigned int i = 0; i < 120; i++) {
                //         move_probes(TEST, 300);
                //     }
                //     for(unsigned int i = 0; i < 20; i++) {
                //         move_probes(RETREAT, 300);
                //     }
                //     for(unsigned int i = 0; i < 20; i++) {
                //         move_probes(PUSH, 300);
                //     }
                //     for(unsigned int i = 0; i < 50; i++) {
                //         move_probes(RETREAT, 300);
                //     }
                //     // if (stepper1_active == 0) {
                //     //     move_stepper(1, 1);
                //     //     move_stepper(2, 2);
                //     // }
                // } 

                // if (is_battery_up) {
                //     // move_probes(TEST, 300);
                //     if (stepper1_active == 0) {
                //         move_stepper_angle(1, 90, 0);
                //         // move_stepper(2, 1);
                //     }
                    // if(stepper2_active == 0) {
                    //     // move_stepper_angle(2, 90, 0);
                    //     move_stepper(2, 2);
                    // }
                
//////////////
                if (is_battery_bottom && !is_battery_up && !is_up) {
                    // move_stepper(2, 2);
                    // move_stepper(2, 1);
                    move_stepper_angle(2, 11000, 1);
                    //move_stepper(2, 2);
                   // is_battery_bottom = ir(1, 5);
                    //is_battery_up = ir(2, 5);
                }
                
                // clear_stepper(2);
                if (is_battery_up) {
                    //for(unsigned int i = 0; i < 1; i++) {
                        //move_probes(TEST, 300);
                        __delay_ms(2000);
                    //}
                    for(unsigned int i = 0; i < 20; i++) {
                        move_probes(RETREAT, 300);
                    }
                    for(unsigned int i = 0; i < 100; i++) {
                        move_probes(TEST, 300);
                        //result = test_battery();
                        //printf("%d", result);
                        // __lcd_clear();
                        // __lcd_home();
                        // voltage = get_voltage(2, 10);
                        // printf("%f", voltage);
                        // result = test_battery();
                        // move_bin(result);
                    }

                    result = test_battery();
                    printf("%d", result);
                    // rotate_bin(0, 2);
                    move_bin(result);
                    for(unsigned int i = 0; i < 1000; i++) {
                        __lcd_clear();
                        __lcd_home();
                        printf("%d", result);
                    }

                    
                    // for(unsigned int i = 0; i < 30; i++) {
                    //     move_stepper_angle(1, 100, 0);
                    // }

                    for(unsigned int i = 0; i < 20; i++) {
                        move_probes(RETREAT, 300);
                    }
                    for(unsigned int i = 0; i < 20; i++) {
                        move_probes(PUSH, 300);
                    }
                    for(unsigned int i = 0; i < 50; i++) {
                        move_probes(RETREAT, 300);
                    }
                    is_up = 1;
                }
                if (is_battery_bottom && is_up) {
                    move_stepper_angle(2, 10800, 0);
                }
 //////////////               
                // result = test_battery();
                // move_bin(result);

                // if (is_hit) {
                //     move_probes(PUSH, 300);
                //     move_stepper(0, 1);
                //     move_stepper(0, 2);
                // }
                voltage = get_voltage(2, 10);
//                current_time(time);
//                elapsed_time = calculate_elapsed_time(time);
//                printf(" %3d %d %x %x %f", elapsed_time, is_battery, ADRESH, ADRESL, convert_voltage(ADRESH, ADRESL));
//                __delay_ms(300);
//                termination(elapsed_time);
                __lcd_clear();
                __lcd_home();
                printf("%d %f", elapsed_time, voltage);
                __lcd_newline();
                // printf("%d  %d %d", is_battery_bottom, is_battery_up, is_hit);
                printf("%d %d %d %d %d",is_battery_bottom, is_battery_up, is_hit, result, previous_type);
/**********************************************************************************/
                __delay_ms(300);
                is_wait = is_termination(elapsed_time);         
            }

            // TESTING
            LATAbits.LATA3 = 0;
            LATAbits.LATA5 = 0;

            // EEPROM logging 
            Eeprom_WriteByte(address, AA_num);
            address = next_address(address);
            Eeprom_WriteByte(address, C_num);
            address = next_address(address);
            Eeprom_WriteByte(address, Nine_num);
            address = next_address(address);
            Eeprom_WriteByte(address, Drain_num);
            address = next_address(address);
            Eeprom_WriteByte(address, elapsed_time);
            address = next_address(address);

           // LATCbits.LATC0 = 0; // RC1 = 0 enable keypad     TODO, change pin assignment 
 
            is_wait = !is_wait;
            is_active = !is_active; // reset the operation flag

            T0CONbits.TMR0ON = 0;   //turn off timers
            T2CONbits.TMR2ON = 0;
            T3CONbits.TMR3ON = 0;
            T1CONbits.TMR1ON = 0; 
        } else if (log != 0) {
            
            while(menu == logA) {
                di();
                show_log(0);
                ei();
            }
            
            while(menu == logB) {
                di();
                show_log(40);
                ei();
            }
            
            while(menu == logC) {
                di();
                show_log(80);
                ei();
            }
            
        } else {

            while(menu == HOME && !is_active) {
                di();
                __lcd_home();
                printf("press 5 to run");
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

/******************************************************************************************************/

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
//void interrupt keypressed(void) {
//    if(INT1IF){
//        __lcd_clear();
//        unsigned char keypress = (PORTB & 0xF0) >> 4;
//        select_menu(keys[keypress]);
//        INT1IF = 0;     //Clear flag bit
//    }
//}

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
            if(temp == '5') {
                is_active = !is_active;
                T0CONbits.TMR0ON = 1; //turn on TMR0 
                T1CONbits.TMR1ON = 1; //TMR1 off when not driving steppers

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

/******************************************************************************************************/
/* old timer codes */

/**
 * [current_time description]
 * @param time [description]
 */
// void current_time(unsigned char* time) {
//         //Reset RTC memory pointer 
//         I2C_Master_Start(); //Start condition
//         I2C_Master_Write(0b11010000); //7 bit RTC address + Write
//         I2C_Master_Write(0x00); //Set memory pointer to seconds
//         I2C_Master_Stop(); //Stop condition

//         //Read Current Time
//         I2C_Master_Start();
//         I2C_Master_Write(0b11010001); //7 bit RTC address + Read
//         for(unsigned char i=0;i<0x06;i++){
//             time[i] = I2C_Master_Read(1);
//         }
//         time[6] = I2C_Master_Read(0);       //Final Read without ack
//         I2C_Master_Stop();
//         __lcd_clear();
//         __lcd_home();
//         // printf("%02x/%02x/%02x", time[6],time[5],time[4]);    //Print date in YY/MM/DD
//         __lcd_newline();
//         // printf("%02x:%02x:%02x", time[2],time[1],time[0]);    //HH:MM:SS
//         // __delay_1s();
//         __delay_ms(300);
// }

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
int is_termination(unsigned int time_now) {
    // if (time_now > 30) { extern unsigned int is_wait; is_wait = 1; }
    if (time_now > 100) { 
        return 1; 
    } else { 
        return 0; 
    }

    // extern unsigned int total_num;
    // return (time_now >= 180 || total_num >= 15)? 1 : 0;
}

/******************************************************************************************************/
/* voltage reading codes */

void readADC(char channel) {
    // Select A2D channel to read
    ADCON0 = ((channel <<2));
    ADON = 1;
    ADCON0bits.GO = 1;
    while(ADCON0bits.GO_NOT_DONE){__delay_ms(5);}    
}

float convert_voltage() { //(unsigned int high, unsigned int low) {
    // highest reading is 3ff or 1023
    int reading = ADRESH*16*16 + ADRESL;
    //printf("111111111111");
    return (float)(reading * 5.0 / 1023);
    // return ((ADRESH<<8)+ADRESL);
    // return __bcd_to_num(ADRESL)
}

float average_voltage(char channel, unsigned int average_span) {
    float vol = 0.0;
    for(unsigned int i = 0; i < average_span; i++) {
        // readADC(channel);
        vol += convert_voltage();
        //printf("000000");
        // __delay_ms(5);
    }
    return vol/average_span;
}

float convert_to_valid_voltage(float raw_voltage) {
    return ((float)((int)(raw_voltage * 1000)))/1000.0;
}

float get_voltage(char channel, unsigned int average_span) {
    readADC(channel);
    return convert_to_valid_voltage(average_voltage(channel, average_span));
} 

/******************************************************************************************************/
/* clean-up codes */

void clean_count(void) {
    extern unsigned int AA_num, C_num, Nine_num, Drain_num, elapsed_time;
    AA_num = 0;
    C_num = 0;
    Nine_num = 0;
    Drain_num = 0;
    elapsed_time = 0;
}

void clean_motor_status() {
    LATA = 0x00;
    LATB = 0x00; 
    LATC = 0x00;
    LATD = 0x00;
    LATE = 0x00;
}

/******************************************************************************************************/
/* EEPROM storage codes */

uint8_t Eeprom_ReadByte(uint16_t address) {
    // Set address registers
    EEADRH = (uint8_t)(address >> 8);
    EEADR = (uint8_t)address;

    EECON1bits.EEPGD = 0;       // Select EEPROM Data Memory
    EECON1bits.CFGS = 0;        // Access flash/EEPROM NOT config. registers
    EECON1bits.RD = 1;          // Start a read cycle

    // A read should only take one cycle, and then the hardware will clear
    // the RD bit
    while(EECON1bits.RD == 1);

    return EEDATA;              // Return data
}


void Eeprom_WriteByte(uint16_t address, uint8_t data) {    
    // Set address registers
    EEADRH = (uint8_t)(address >> 8);
    EEADR = (uint8_t)address;

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


uint16_t next_address(uint16_t address) {
    return address + 8;
}


void show_log(uint16_t log_address) {
    // read in log address and start fetching historical data
    uint16_t address = log_address;
    __lcd_home();
    
    unsigned int AA_num = Eeprom_ReadByte(address);
    address = next_address(address);
    unsigned int C_num = Eeprom_ReadByte(address);
    address = next_address(address);
    unsigned int Nine_num = Eeprom_ReadByte(address);
    address = next_address(address);
    unsigned int Drain_num = Eeprom_ReadByte(address);
    address = next_address(address);
    unsigned int elapsed_time = Eeprom_ReadByte(address);
    
    printf("AA:%d C:%d 9V:%d", AA_num, C_num, Nine_num);
    __lcd_newline();
    printf("D:%d time:%d 8>>", Drain_num, elapsed_time);
}

/******************************************************************************************************/
/* interrupt codes */

void interrupt ISR(void) {
    extern unsigned int is_active; 
    extern int motorstep;
    extern unsigned int elapsed_time;
    
    // interrupt for keypad input 
    if(INT1IF) {
        __lcd_clear();
        unsigned char keypress = (PORTB & 0xF0) >> 4;
        select_menu(keys[keypress]);
        INT1IF = 0;     //Clear flag bit
    }

    // interrupt for elapsed time counter 
    if(is_active && TMR0IF) {
        TMR0IF = 0;     //clear interrupt bit 
        TMR0 = 55770;   //reset timer 0 count 
        elapsed_time++;
    }
    
    // interrupt for servo controls 
    if(is_active && TMR1IF) {
        TMR1IF = 0;     //clear interrupt bit 
        TMR1 = 65285;   //reset timer 1 count
        // driving two servo motors for the two probes
        if (servo1_active) {    // white wire 
            if (servo1_low == 0) {
                if (servo1_high < servo1_high_duration) {
                    servo1_high++;
                    LATCbits.LATC6 = 1;
                } else {
                    servo1_high = 0;
                    servo1_low = 1;
                }
            } else if (servo1_high == 0){
                if (servo1_low < servo1_low_duration) {
                    servo1_low++;
                    LATCbits.LATC6 = 0;
                } else {
                    servo1_high = 1;
                    servo1_low = 0;
                }
            }
        }

        if (servo2_active) {    // blue wire 
            if (servo2_low == 0) {
                if (servo2_high < servo2_high_duration) {
                    servo2_high++;
                    LATCbits.LATC7 = 1;
                } else {
                    servo2_high = 0;
                    servo2_low = 1;
                }
            } else if (servo2_high == 0){
                if (servo2_low < servo2_low_duration) {
                    servo2_low++;
                    LATCbits.LATC7 = 0;
                } else {
                    servo2_high = 1;
                    servo2_low = 0;
                }
            }
        }  

        // if(servo1_active && servo2_active) {
        //     // Step 1: close 
        //     while() {
        //     }

            // Step 2: kick 

        //    Step 3: resume 
        // }  
    }

    // interrupt for background dc motors 
    if(is_active && TMR2IF) {
        TMR2IF = 0;     //clear interrupt bit 
        TMR2 = 53035;   //reset timer 2 count
        // driving two DC motors for the wheel and conveyor belt    
        if (stop_count < stop_inactive) {
            if (dc_count < dc_duration_low) {
                clear_dc(1);
                dc_count++;
            } else if (dc_count < dc_duration_high){
                move_dc1();
                dc_count++;
            } else {
                dc_count = 0;
            }
            stop_count++;
        } else if (stop_count < stop_active) {
            clear_dc(1);
            stop_count++;
        } else {
            stop_count = 0;
        }
        // move_dc1();
        move_dc2(1);
    }

    // interrupt for stepper control 
    if(is_active && TMR3IF) {
        TMR3IF = 0;     //clear interrupt bit 
        TMR3 = 63785; //60535;   // reset timer 3 count
        // driving two stepper motors for the pusher and the bin plate 
        // move_stepper1(1);
        // move_stepper2(0);
        // clear_stepper(1);
        // clear_stepper(2);

        if (stepper1_active == 1) {
            if(stepper1_duration_count < stepper1_duration) {
                move_stepper1(1);
                stepper1_duration_count++;
            } else {
                stepper1_duration_count = 0;
                stepper1_active = 0;
            }
        } else if (stepper1_active == 2) {
            if(stepper1_duration_count < stepper1_duration) {
                move_stepper1(0);
                stepper1_duration_count++;
            } else {
                stepper1_duration_count = 0;
                stepper1_active = 0;
            }
        } else {
            clear_stepper(1);
        }

        if (stepper2_active == 1) {
            if(stepper2_duration_count < stepper2_duration) {
                move_stepper2(1);
                stepper2_duration_count++;
            } else {
                stepper2_duration_count = 0;
                stepper2_active = 0;
            }
        } else if (stepper2_active == 2) {
            if(stepper2_duration_count < stepper2_duration) {
                move_stepper2(0);
                stepper2_duration_count++;
            } else {
                stepper2_duration_count = 0;
                stepper2_active = 0;
            }
        } else {
            clear_stepper(2);
        }
    }
}   // end of the interrupt ISR   

/******************************************************************************************************/
/* motor control codes */

void pulse_delay(unsigned int pulse) {
    int numDelayPerTenSec;
    numDelayPerTenSec = (int)(pulse / 10);
    for(int i = 0; i < numDelayPerTenSec; i++) {
        __delay_us(10);
    }
}

void set_servo_duration(char channel, char highlow, unsigned int duration) {
    extern unsigned int servo1_high_duration, servo1_low_duration;
    if (channel == 1) {
        if (highlow == 1) {
            servo1_high_duration = duration;
        } else {
            servo1_low_duration = duration;
        }
    } else if (channel == 2) {
        if (highlow == 1) {
            servo2_high_duration = duration;
        } else {
            servo2_low_duration = duration;
        }
    }
}


void move_stepper1(unsigned int direction) {  // wheel, stepper 1 
    LATCbits.LATC3 = !LATCbits.LATC3;
    LATCbits.LATC4 = direction;
}


void move_stepper2(unsigned int direction) {   // pusher, stepper 2
    LATCbits.LATC0 = !LATCbits.LATC0;
    LATCbits.LATC1 = direction;
}

void clear_stepper(char channel) {
    extern unsigned int stepper1_active, stepper2_active;
    if (channel == 1) {
        stepper1_active = 0;
        LATCbits.LATC3 = 0;
        LATCbits.LATC4 = 0;
    } else if (channel == 2) {
        stepper2_active = 0;
        LATCbits.LATC0 = 0;
        LATCbits.LATC1 = 0;
    }
}


void set_stepper_duration(char channel, unsigned int duration) {
    extern unsigned int stepper1_duration, stepper2_duration;
    if (channel == 1) {
        stepper1_duration = duration;
    } else if ( channel == 2) {
        stepper2_duration = duration;
    }
} 


void move_stepper_angle(char channel, float angle, char direction) {
    unsigned int duration = angle;  // a function of angle and stepper specs 
    unsigned int command = ((direction == 1)? 1 : 2);
    set_stepper_duration(channel, duration);
    move_stepper(command, channel);
} 


void clear_dc(char channel) {
    if (channel == 1) {
        LATCbits.LATC2 = 0;
    } else if (channel == 2) {
        LATCbits.LATC5 = 0; 
        LATDbits.LATD0 = 0;
    }
}

void move_dc1() {
    // RC0, 1, 2 are used for the first DC motor, no pwm control for now
    LATCbits.LATC2 = 1; // !LATCbits.LATC2;
}


void move_dc2(char direction) {
    // RC3, 4, 5 are used for the second DC motor, no pwm control for now
    if (direction == 1) {
        LATCbits.LATC5 = 1; // !LATCbits.LATC5;
        LATDbits.LATD0 = 0;
    } else {
        LATCbits.LATC5 = 0; 
        LATDbits.LATD0 = 1;
    }
}


void move_servo(unsigned int command) {
    extern unsigned int servo1_active, servo2_active;
    if(command == 1) {  // start servo 
        servo1_active = 1;
        servo2_active = 1;
    } else {            // stop servo 
        servo1_active = 0;
        servo2_active = 0;
    }
}

void move_stepper(unsigned int command, char channel) {
    extern unsigned int stepper1_active, stepper2_active;
    if (channel == 1) {
        if (command == 1) {
            stepper1_active = 1;
        } else if (command == 2) {
            stepper1_active = 2;
        } else {
            stepper1_active = 0;
        }
    } else if (channel == 2) {
        if (command == 1) {
            stepper2_active = 1;
        } else if (command == 2) {
            stepper2_active = 2;
        } else {
            stepper2_active = 0;
        }
    }
}

/******************************************************************************************************/
/* battery testing codes */

unsigned int regulate_signal(unsigned int raw_signal) {
    return raw_signal ? 0 : 1;
}


unsigned int ir(unsigned int channel, unsigned int average_span) {
    if (channel == 1) {
        for(unsigned int i = 0; i < average_span; i++) {
            if (PORTEbits.RE0) {
                return 0;
            }
        }
        return 1;
    } else if (channel == 2) {
        for(unsigned int i = 0; i < average_span; i++) {
            if (PORTEbits.RE1) {
                return 0;
            }
        }
        return 1;
    } else {
        return 0;   // default value, 
    }
}


unsigned int microswitch(unsigned int average_span) {
    for(unsigned int i = 0; i < average_span; i++) {
        if (PORTEbits.RE2) {
            return 1;
        }
    }
    return 0;
}


void rotate_bin(char direction, char degree) {
    // degree 1 is 90, degree 2 is 180; direction is 1 and 0
    if (degree == 1) {
        if (direction == 0) {
            for(unsigned int i = 0; i < 30; i++) {
                move_stepper_angle(1, 100, 0);
            }
        } else {
            for(unsigned int i = 0; i < 30; i++) {
                move_stepper_angle(1, 100, 1);
            }
        }
    } else {
        for(unsigned int i = 0; i < 70; i++) {
            move_stepper_angle(1, 100, 0);
        }
    }
}


float max_voltage(unsigned int duration) {
    float current = get_voltage(2, 50);
    float temp = 0;
    for (unsigned int i = 0; i < duration; i++) {
        temp = get_voltage(2, 50);
        current = ((temp > current)? temp : current);
    }
    return current;
}


unsigned int test_battery() {
    // AA_BAT 3
    // C_BAT 4
    // NINE_BAT 5
    // DRAIN_BAT 6

    extern unsigned int AA_num, C_num, Nine_num, Drain_num, total_num;
    float D = 0;    // voltage reading D
    
    // Step 1: reset relays, open 1, 2, 3, 4, 5, 6
    LATAbits.LATA3 = 0;   // relay 1
    LATDbits.LATD1 = 0;   // relay 2
    LATAbits.LATA5 = 0;   // relay 3
    LATAbits.LATA4 = 0;   // relay 4
    LATAbits.LATA1 = 0;   // relay 5
    LATAbits.LATA0 = 0;   // relay 6

    // Step 2: close relay 1 and 3, check D, if voltage -> 9V, open 1 and 3
    LATAbits.LATA3 = 1;
    LATAbits.LATA5 = 1;

    for (unsigned int i = 0; i < 20; i++) {
        D = max_voltage(10);  // get_voltage(2, 50);
        if (D > 3.82) {
            Nine_num++;
            total_num++;
            return NINE_BAT;
        } else if (D > 0.01) {
            Drain_num++;
            total_num++;
            return DRAIN_BAT;
        }
    }
    LATAbits.LATA3 = 0;
    LATAbits.LATA5 = 0;

    // Step 3: close relay 4 and 6, check D, if voltage -> 9V, open 4 and 6
    LATAbits.LATA0 = 1;
    LATAbits.LATA4 = 1;
    for (unsigned int i = 0; i < 20; i++) {
        D = max_voltage(10);  // get_voltage(2, 50);
        if (D > 3.82) {
            Nine_num++;
            total_num++;
            return NINE_BAT;
        } else if (D > 0.01) {
            Drain_num++;
            total_num++;
            return DRAIN_BAT;
        }
    }
    LATAbits.LATA4 = 0;
    LATAbits.LATA0 = 0;

    // Step 4: close relay 3 and 5, check D, if voltage -> C, open 3 and 5 
    LATAbits.LATA5 = 1;
    LATAbits.LATA1 = 1;
    for (unsigned int i = 0; i < 20; i++) {
        D = max_voltage(10);  // get_voltage(2, 50);
        if (D > 0.63) {
            C_num++;
            total_num++;
            return C_BAT;
        } else if (D > 0.01) {
            Drain_num++;
            total_num++;
            return DRAIN_BAT;
        }
    }
    LATAbits.LATA5 = 0;
    LATAbits.LATA1 = 0;

    // Step 5: close relay 2 and 6, check D, if voltage -> AA, open 2 and 6 
    LATDbits.LATD1 = 1;
    LATAbits.LATA0 = 1;
    for (unsigned int i = 0; i < 20; i++) {
        D = max_voltage(10);  // get_voltage(2, 50);
        if (D > 0.63) {
            AA_num++;
            total_num++;
            return AA_BAT;
        } else if (D > 0.01) {
            Drain_num++;
            total_num++;
            return DRAIN_BAT;
        }
    }
    LATDbits.LATD1 = 0;
    LATAbits.LATA0 = 0;

    // Default case 
    Drain_num++;
    total_num++;
    return DRAIN_BAT;
}


void move_bin(unsigned int bat_type) {
    extern unsigned int previous_type;
    switch(bat_type) {
        case AA_BAT:
            if (previous_type == DRAIN_BAT) {
                // move_stepper_angle(1, 90, 0);
                rotate_bin(0, 1);
            } else if (previous_type == C_BAT) {
                // move_stepper_angle(1, 90, 1);
                rotate_bin(1, 1);
            } else if (previous_type == NINE_BAT) {
                // move_stepper_angle(1, 180, 0);
                rotate_bin(0, 2);
            }
            break;
        case C_BAT: 
            if (previous_type == DRAIN_BAT) {
                // move_stepper_angle(1, 180, 0);
                rotate_bin(0, 2);
            } else if (previous_type == AA_BAT) {
                // move_stepper_angle(1, 90, 0);
                rotate_bin(0, 1);
            } else if (previous_type == NINE_BAT) {
                // move_stepper_angle(1, 90, 1);
                rotate_bin(1, 1);
            }
            break;
        case NINE_BAT:
            if (previous_type == DRAIN_BAT) {
                // move_stepper_angle(1, 90, 1);
                rotate_bin(1, 1);
            } else if (previous_type == C_BAT) {
                // move_stepper_angle(1, 90, 0);
                rotate_bin(0, 1);
            } else if (previous_type == AA_BAT) {
                // move_stepper_angle(1, 180, 0);
                rotate_bin(0, 2);
            }
            break;
        case DRAIN_BAT:
            if (previous_type == AA_BAT) {
                // move_stepper_angle(1, 90, 1);
                rotate_bin(1, 1);
            } else if (previous_type == C_BAT) {
                // move_stepper_angle(1, 180, 0);
                rotate_bin(0, 2);
            } else if (previous_type == NINE_BAT) {
                // move_stepper_angle(1, 90, 0);
                rotate_bin(0, 1);
            }
            break;
        default:    // default should be drained battery due to a higher probability
            break; 
    }
    previous_type = bat_type;
}


void move_probes(char status, unsigned int speed) {
    unsigned int i = 0;
    // angle 1: around 60 degrees, 2, 38
    // angle 2: around 90 degrees, 3, 37
    // angle 3: around 120 degrees, 4 36
    if (status == RETREAT) {
        while (i < speed) {
            i++;
            set_servo_duration(1, 1, 10);
            set_servo_duration(1, 0, 190);
            set_servo_duration(2, 1, 17);
            set_servo_duration(2, 0, 183);
            move_servo(1);
        }
        move_servo(0);
    } else if (status == TEST) {
        while (i < speed) {
            i++;
            set_servo_duration(1, 1, 16);
            set_servo_duration(1, 0, 184);
            set_servo_duration(2, 1, 9);
            set_servo_duration(2, 0, 191);
            move_servo(1);
        }
        move_servo(0);
    } else if (status == PUSH) {
        while (i < speed) {
            i++;
            set_servo_duration(1, 1, 17);
            set_servo_duration(1, 0, 183);
            set_servo_duration(2, 1, 17);
            set_servo_duration(2, 0, 183);
            move_servo(1);
        }
        move_servo(0);
    }
}
