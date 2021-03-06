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
#define logD 11
#define STANDBY 12
#define TEST 13
#define RETREAT 14
#define PUSH 15

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
void clean_up();

uint8_t Eeprom_ReadByte(uint16_t);
void Eeprom_WriteByte(uint16_t, uint8_t);
uint16_t next_address(uint16_t);
void show_log(uint16_t);
// <TODO>
void pulse_delay(unsigned int);
void set_servo_duration(char, char, unsigned int);
void move_stepper1(unsigned int);
void move_stepper2(unsigned int);
// void move_wheel(char, char);
void move_wheel(char);
void move_pusher(char);
void clear_stepper(char);
void moveServoRetreat();
void moveServoTest();
void moveServoPush();
void set_stepper_duration(char, unsigned int);
void move_stepper_angle(char, float, char);
void set_dc(char, char);
void clear_dc(char);
void move_dc1();
void move_dc2(char);
void set_belt_direction(char);
void move_servo(unsigned int);
void move_stepper(unsigned int, char);

//PC interface
int getHundreds(unsigned int);
int getTens(unsigned int);
int getOnes(unsigned int);
char getChar(int);
int constructHeader(char*, int);
void constructNum(char*, int);
void constructElapsedTime(char*, int);
void constructStartTime(char*, char*);
void logPCNum(char*, int, char*);
void logPCTime(char*, char*, char*, char*);

// battery testing stage
unsigned int regulate_signal(unsigned int);
unsigned int ir(unsigned int, unsigned int); 
unsigned int laserBlocked(unsigned int);
void rotate_bin(char direction, char degree);
unsigned int test_battery();
void move_bin(unsigned int);
void push_battery(unsigned int);
void move_probes(char, unsigned int);

// global constant variables
const char keys[] = "123A456B789C*0#D"; 
const char happynewyear[7] = {  0x00, //00 Seconds 
                            0x00, //00 Minutes
                            0x19, //24 hour mode, set to 19:00
                            0x02, //Monday 
                            0x10, //10st
                            0x04, //April
                            0x17};//2017

// global variables 
unsigned int is_active = 0; // run sorting operation
char start_time[] = "0000000";

// PC interface info
int length;
char* header;
char num[3];
char time_header[16] = "elapsed time is ";
char run_time[3];
char start_time_header[23] = " seconds, started from "; 
char started_time[19] = "  /  /     :  :  \n\n";

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
uint8_t next_block = 0;
uint16_t address = 1;


// pusher info 
unsigned int test_done = 0;

// dc info
unsigned int dc1_active = 1;
unsigned int dc2_active = 1;
unsigned int dc_count = 0;
unsigned int dc_duration_low = 40;
unsigned int dc_duration_high = 60;
unsigned int stop_count = 0;
unsigned int stop_inactive = 2000;
unsigned int stop_active = 8000;
unsigned int forward = 1;
unsigned int laser_counter = 0;
unsigned int bin_inhibitor = 1;

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

unsigned int stepper1_duration = 200; //5000    
unsigned int stepper2_duration = 100;

unsigned int mode = 1;
unsigned int is_blocked = 0;


// main function 
void main(void) {
    // // Testing Stage 1
    // // Eeprom_WriteByte(0, 1);

    // // Testing Stage 2
    // unsigned int ttt;
    // while(1) {
    //     ttt = Eeprom_ReadByte(0);
    //     printf("%d", ttt);
    // }

    // <editor-fold defaultstate="collapsed" desc=" STARTUP SEQUENCE ">
    TRISA = 0x04; // Set Port A as all input
    TRISB = 0xFF; 
    TRISC = 0x00;
    TRISD = 0x00; //All output mode for LCD
    TRISE = 0x05;    

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

    // while(1){
    //     LATCbits.LATC2 = 1;
    //     __delay_ms(300);
    //     LATCbits.LATC2 = 0;
    //     __delay_ms(300);

    //     // LATCbits.LATC1 = 0;
    //     // // LATCbits.LATC0 = 1;
    //     //  for(unsigned int i = 0; i < 21600; i++) {
    //     //     LATCbits.LATC0 = 1;
    //     //     __delay_us(90);
    //     //     LATCbits.LATC0 = 0;
    //     //     __delay_us(90);
    //     // }
    //     // __delay_ms(100);
    //     // LATCbits.LATC1 = 1;
    //     // for(unsigned int i = 0; i < 21600; i++) {
    //     //     LATCbits.LATC0 = 1;
    //     //     __delay_us(90);
    //     //     LATCbits.LATC0 = 0;
    //     //     __delay_us(90);
    //     // }
    //     // __delay_ms(100);
    // }

    GIE = 1;        //globally enable interrupt
    PEIE = 1;       //enable peripheral interrupt
    TMR0IE = 1;     //enable TMR0 overflow interrupt
    TMR0IF = 0;	    //turn off TMR0 overflow interrupt flag
    TMR1IE = 1;     //enable TMR1 overflow interrupt
    TMR1IF = 0;     //turn off TMR1 overflow interrupt flag
    TMR3IE = 1;     //enable TMR2 overflow interrupt
    TMR3IF = 0;     //turn off TMR2 overflow interrupt flag
    TMR2IE = 1;     //enable TMR2 overflow interrupt
    TMR2IF = 0;     //turn off TMR2 overflow interrupt flag
    // intialize timers for interrupt 
    // timer 0 
    T0CON = 0b00000111;
    TMR0 = 55770;
    // timer 1
    T1CON = 0b10000000;
    TMR1 = 65285;
    // timer 2
    T2CON = 0b10000000;
    TMR2 = 63035;   //53035;
    // timer 3
    T3CON = 0b10000000;
    TMR3 = 63785;//63535;//53035;//63785; //60535;  // need to determine the third timer, 65535-0.005*10000000/4
    
    ei();           //Enable all interrupts
    //</editor-fold>

    
    // initializations and prompt to start
    di();
    I2C_Master_Init(10000); //Initialize I2C Master with 100KHz clock
    initLCD();
    // uint16_t address = 1;
    ei();

    set_time();

    // main execution loop
    while(1) {

        if(is_active) {
            // LATCbits.LATC6 = 1;//RC6 = 1 , free keypad pins
            unsigned char time[7];

            // address indicator 1, 2, 3, 4
            address = Eeprom_ReadByte(0);
            if(address > 4) {
                address = 1;
            }
            next_block = address + 1;
            next_block = (next_block > 4) ? 1 : next_block;
            Eeprom_WriteByte(0, next_block);
            address = (address - 1) * 96 + 1;
   
            current_time(time);
            for(unsigned int i = 0; i < 7; i++) {
                start_time[i] = time[i];
                Eeprom_WriteByte(address, time[i]);
                address = next_address(address);
            }

            elapsed_time = 0;
            clean_count();

            unsigned int is_battery_bottom = 0;
            unsigned int is_battery_up = 0;
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

                // real logic: ir input 
                is_battery_bottom = ir(1, 1);
                // is_battery_up = ir(1, 1);
                is_blocked = laserBlocked(1);
                
////////////// //////////////    //////////////    //////////////    //////////////    //////////////   

                // if (!is_battery_bottom) {   
                //     // reset test_done flag: standby-bottom 0-done 0; pushing-bottom 1-done 0; declining-bottom 1-done 1 
                //     test_done = 0;
                // }    

                if (is_battery_bottom) { // && !is_battery_up) { // && !test_done) {
                    __delay_ms(30);
                    
                    set_dc(1, 0);
                    clear_dc(1);
                    set_dc(2, 0);
                    clear_dc(2);
                    move_pusher(0);

                    // set_dc(1, 0);
                    // clear_dc(1);
                    // set_dc(2, 0);
                    // clear_dc(2);

                    test_done = 1;
                }

                // is_battery_up = ir(1, 1);
                // if (is_battery_up && !test_done) {
                //     test_done = 1;
                // }
                if (test_done) {
                    // __delay_ms(1000);
                    for(unsigned int i = 0; i < 20; i++) {
                        move_probes(RETREAT, 100);
                    }

                    // for(unsigned int i = 0; i < 50; i++) {     // previous bound 100
                    //     move_probes(TEST, 300);
                    // }

                    set_servo_duration(1, 1, 16);
                    set_servo_duration(1, 0, 184);
                    set_servo_duration(2, 1, 8);
                    set_servo_duration(2, 0, 192);
                    move_servo(1);

                    __delay_ms(200);
                    result = test_battery();

                    // __delay_ms(200);
                    move_bin(result);

                    // __delay_ms(200);
                    move_servo(0);
                    __delay_ms(30);

                    for(unsigned int i = 0; i < 20; i++) {
                        move_probes(RETREAT, 100);           
                    }
                    for(unsigned int i = 0; i < 20; i++) {
                        move_probes(PUSH, 100);
                    }
                    for(unsigned int i = 0; i < 20; i++) {
                        move_probes(RETREAT, 100);
                    }
                    // test_done = 1;
                }
                
                if (is_battery_bottom) {
                    // set_belt_direction(1);
                    // set_dc(1, 1);
                    // set_dc(2, 1);
                    move_pusher(1);
                    set_dc(1, 1);  //working 
                    set_dc(2, 1);
                }

                test_done = 0;
                
 ////////////// //////////////    //////////////    //////////////    //////////////    //////////////                  
                
               current_time(time);
//                elapsed_time = calculate_elapsed_time(time);
//                printf(" %3d %d %x %x %f", elapsed_time, is_battery, ADRESH, ADRESL, convert_voltage(ADRESH, ADRESL));
//                __delay_ms(300);
                __lcd_clear();
                __lcd_home();
                // printf("%d %d", elapsed_time, result);
                // printf("%d %f", elapsed_time, voltage);
                printf("%02x/%02x/%02x", time[6],time[5],time[4]);
                // printf("%02x:%02x:%02x", start_time[2],start_time[1],start_time[0]);
                __lcd_newline();
                // __delay_ms(300);
                printf("%02x:%02x:%02x", time[2],time[1],time[0]);
                // printf("%d  %d %d", is_battery_bottom, is_battery_up, is_hit);
                // printf("%d %d %d %d", elapsed_time, is_battery_bottom, is_blocked, result);
/**********************************************************************************/
                __delay_ms(30);
                is_wait = is_termination(elapsed_time);         
            }

            T0CONbits.TMR0ON = 0;   //turn off timers 
            T2CONbits.TMR2ON = 0;
            T3CONbits.TMR3ON = 0;
            T1CONbits.TMR1ON = 0; 
            clean_up();

            // AA_num += 1;
            // C_num += 2;
            // Nine_num += 3;
            // Drain_num += 4;

            total_num = AA_num + C_num + Nine_num + Drain_num;

            // EEPROM logging 
            Eeprom_WriteByte(address, AA_num);
            address = next_address(address);
            Eeprom_WriteByte(address, C_num);
            address = next_address(address);
            Eeprom_WriteByte(address, Nine_num);
            address = next_address(address);
            Eeprom_WriteByte(address, Drain_num);
            address = next_address(address);
            Eeprom_WriteByte(address, elapsed_time-1);
            address = next_address(address);

            // PC interface output
            length = constructHeader(header, AA_BAT);
            constructNum(num, AA_num);
            logPCNum(header, length, num);
            length = constructHeader(header, C_BAT);
            constructNum(num, C_num);
            logPCNum(header, length, num);
            length = constructHeader(header, NINE_BAT);
            constructNum(num, Nine_num);
            logPCNum(header, length, num);
            length = constructHeader(header, DRAIN_BAT);
            constructNum(num, Drain_num);
            logPCNum(header, length, num);
            length = constructHeader(header, 100);
            constructNum(num, AA_num + C_num + Nine_num + Drain_num);
            logPCNum(header, length, num);

            constructElapsedTime(run_time, elapsed_time-1);
            constructStartTime(started_time, start_time);
            logPCTime(time_header, run_time, start_time_header, started_time);

           // LATCbits.LATC0 = 0; // RC1 = 0 enable keypad     TODO, change pin assignment 
 
            is_wait = !is_wait;
            is_active = !is_active; // reset the operation flag
        } else if (log != 0) {
            
            while(menu == logA) {
                di();
                show_log(1);
                ei();
            }
            
            while(menu == logB) {
                di();
                show_log(97);   //40);
                ei();
            }
            
            while(menu == logC) {
                di();
                show_log(193); //80);
                ei();
            }

            while(menu == logD) {
                di();
                show_log(289);
                ei();
            }
            
        } else {

            while(menu == HOME && !is_active) {
                di();
                // __lcd_clear();
                __lcd_home();
                printf("press 5 to run    ");
                __lcd_newline();
                printf("<< 7 DATA 9 >>");
                ei();
            }

            while(menu == TIME) {
                di();
                //  __lcd_clear();
                __lcd_home();
                if(elapsed_time == 0) {
                    printf("elapsed time:%d", elapsed_time);
                } else {
                    printf("elapsed time:%d", elapsed_time - 1);
                }
                __lcd_newline();
                printf("<< 7 DATA 9 >>");
                ei();
            }

            while(menu == TOTAL_BAT) {
                di();
                //  __lcd_clear();
                __lcd_home();
                printf("total: %d", total_num);
                __lcd_newline();
                printf("<< 7 DATA 9 >>");
                ei();
            }

            while(menu == AA_BAT) {
                di();
                //  __lcd_clear();
                __lcd_home();
                printf("AA sorted: %d", AA_num);
                __lcd_newline();
                printf("<< 7 DATA 9 >>");
                ei();
            }

            while(menu == C_BAT) {
                di();
                //  __lcd_clear();
                __lcd_home();
                printf("C sorted: %d", C_num);
                __lcd_newline();
                printf("<< 7 DATA 9 >>");
                ei();
            }

            while(menu == NINE_BAT) {
                di();
                //  __lcd_clear();
                __lcd_home();
                printf("9V sorted: %d", Nine_num);
                __lcd_newline();
                printf("<< 7 DATA 9 >>");
                ei();
            }

            while(menu == DRAIN_BAT) {
                di();
                //  __lcd_clear();
                __lcd_home();
                printf("Drained: %d", Drain_num);
                __lcd_newline();
                printf("<< 7 DATA 9 >>");
                ei();
            }

            while(menu == HISTORY) {
                di();
                //  __lcd_clear();
                __lcd_home();
                printf("run:1 2 3 4");
                __lcd_newline();
                printf("<< 7 DATA 9 >>");
                ei();
            }
        }   // end of the if(is_active)-else
    }   // end of the while(1) loop
    return ;   
}

/******************************************************************************************************/

void set_time(void) {
    I2C_Master_Start(); //Start condition
    I2C_Master_Write(0b11010000); //7 bit RTC address + Write
    I2C_Master_Write(0x00); //Set memory pointer to seconds
    for(char i=0; i<7; i++){
        I2C_Master_Write(happynewyear[i]);
    }    
    I2C_Master_Stop(); //Stop condition
}

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
            } else if (temp == '4'){
                log = 4;
                menu = logD;
            } else {
                if(temp == '7'){menu = DRAIN_BAT;} else if(temp == '9'){menu = TIME;}
            }
            break;
        case logA:
        case logB:
        case logC:
        case logD:
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
        // __lcd_clear();
        // __lcd_home();
        // printf("%02x/%02x/%02x", time[6],time[5],time[4]);    //Print date in YY/MM/DD
        // __lcd_newline();
        // printf("%02x:%02x:%02x", time[2],time[1],time[0]);    //HH:MM:SS
        // __delay_1s();
        __delay_ms(300);
}

int calculate_elapsed_time(unsigned char* time) {
        return (__bcd_to_num(time[0]) + 60*__bcd_to_num(time[1]));
}

int is_termination(unsigned int time_now) {
    // if (time_now > 30) { extern unsigned int is_wait; is_wait = 1; }
    // if (time_now > 5) { 
    //     return 1; 
    // } else { 
    //     return 0; 
    // }

    extern unsigned int total_num;
    return (time_now >= 180 || total_num >= 15)? 1 : 0;
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
    return (float)(reading * 5.0 / 1023);
}

float average_voltage(char channel, unsigned int average_span) {
    float vol = 0.0;
    for(unsigned int i = 0; i < average_span; i++) {
        vol += convert_voltage();
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
    extern unsigned int AA_num, C_num, Nine_num, Drain_num, total_num, elapsed_time;
    AA_num = 0;
    C_num = 0;
    Nine_num = 0;
    Drain_num = 0;
    total_num = 0;
    elapsed_time = 0;
}

void clean_motor_status() {
    LATA = 0x00;
    LATB = 0x00; 
    LATC = 0x00;
    LATD = 0x00;
    LATE = 0x00;
}

void clean_up() {
    LATC = 0x00;

    LATAbits.LATA0 = 0;
    LATAbits.LATA1 = 0;
    LATAbits.LATA3 = 0;
    LATAbits.LATA4 = 0;
    LATAbits.LATA5 = 0;
    LATDbits.LATD1 = 0;
    
    LATEbits.LATE1 = 0;
}

/******************************************************************************************************/
/* PC interface code */

int getHundreds(unsigned int num) {
    if(num > 99) { return (int)(num / 100); }
    return 0;
}

int getTens(unsigned int num) {
    if(num > 9) { return (int)(num / 10); }
    return 0;
}

int getOnes(unsigned int num) {
    return num % 10;
}

char getChar(int num) {
    return num + '0'; 
}

int constructHeader(char* headerr, int type) {
    extern char* header;
    if (type == AA_BAT) {
        header = "number of AA is ";
        return 16;
    } else if (type == C_BAT) {
        header = "number of C is ";
        return 15;
    } else if (type == NINE_BAT) {
        header = "number of 9V is ";
        return 16;
    } else if (type == DRAIN_BAT) {
        header = "number of Drained is ";
        return 21;
    } else {
        header = "total number is ";
        return 16;
    }
}

void constructNum(char* num, int x) {
    num[0] = getChar(getTens(x));
    num[1] = getChar(getOnes(x));
    num[2] = '\n';
}

void constructElapsedTime(char* run_time, int e_time) {
    run_time[0] = getChar(getHundreds(e_time));
    run_time[1] = getChar(getTens(e_time));
    run_time[2] = getChar(getOnes(e_time));
}

void constructStartTime(char* started_time, char* s_time) {
    started_time[0] = getChar(getTens( __bcd_to_num(s_time[6]) ));
    started_time[1] = getChar(getOnes( __bcd_to_num(s_time[6]) ));
    started_time[3] = getChar(getTens( __bcd_to_num(s_time[5]) ));
    started_time[4] = getChar(getOnes( __bcd_to_num(s_time[5]) ));
    started_time[6] = getChar(getTens( __bcd_to_num(s_time[4]) ));
    started_time[7] = getChar(getOnes( __bcd_to_num(s_time[4]) ));
    started_time[9] = getChar(getTens( __bcd_to_num(s_time[2]) ));
    started_time[10] = getChar(getOnes( __bcd_to_num(s_time[2]) ));
    started_time[12] = getChar(getTens( __bcd_to_num(s_time[1]) ));
    started_time[13] = getChar(getOnes( __bcd_to_num(s_time[1]) ));
    started_time[15] = getChar(getTens( __bcd_to_num(s_time[0]) ));
    started_time[16] = getChar(getOnes( __bcd_to_num(s_time[0]) ));
}

void logPCNum(char* header, int length, char* num) {
    for(unsigned int i = 0; i < length; i++) {
        I2C_Master_Start(); //Start condition
        I2C_Master_Write(0b00010000); //7 bit RTC address + Write
        I2C_Master_Write(header[i]); //7 bit RTC address + Write
        I2C_Master_Stop();
    }
    for(unsigned int i = 0; i < 3; i++) {
        I2C_Master_Start(); //Start condition
        I2C_Master_Write(0b00010000); //7 bit RTC address + Write
        I2C_Master_Write(num[i]); //7 bit RTC address + Write
        I2C_Master_Stop();
    }
}

void logPCTime(char* time_header, char* run_time, char* start_time_header, char* started_time) {
    for(unsigned int i = 0; i < 16; i++) {
        I2C_Master_Start(); //Start condition
        I2C_Master_Write(0b00010000); //7 bit RTC address + Write
        I2C_Master_Write(time_header[i]); //7 bit RTC address + Write
        I2C_Master_Stop();
    }
    for(unsigned int i = 0; i < 3; i++) {
        I2C_Master_Start(); //Start condition
        I2C_Master_Write(0b00010000); //7 bit RTC address + Write
        I2C_Master_Write(run_time[i]); //7 bit RTC address + Write
        I2C_Master_Stop();
    }
    for(unsigned int i = 0; i < 23; i++) {
        I2C_Master_Start(); //Start condition
        I2C_Master_Write(0b00010000); //7 bit RTC address + Write
        I2C_Master_Write(start_time_header[i]); //7 bit RTC address + Write
        I2C_Master_Stop();
    }
    for(unsigned int i = 0; i < 19; i++) {
        I2C_Master_Start(); //Start condition
        I2C_Master_Write(0b00010000); //7 bit RTC address + Write
        I2C_Master_Write(started_time[i]); //7 bit RTC address + Write
        I2C_Master_Stop();
    }
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
    uint16_t address = log_address + 56;
    __lcd_home();
    
    unsigned int AA_num = Eeprom_ReadByte(address);
    if(AA_num > 15) {
        AA_num = 0;
    }
    address = next_address(address);
    unsigned int C_num = Eeprom_ReadByte(address);
    if(C_num > 15) {
        C_num = 0;
    }
    address = next_address(address);
    unsigned int Nine_num = Eeprom_ReadByte(address);
    if(Nine_num > 15) {
        Nine_num = 0;
    }
    address = next_address(address);
    unsigned int Drain_num = Eeprom_ReadByte(address);
    if(Drain_num > 15) {
        Drain_num = 0;
    }
    address = next_address(address);
    unsigned int elapsed_time = Eeprom_ReadByte(address);
    if(elapsed_time > 181) {
        elapsed_time = 0;
    }
    
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
        if (servo1_active) {    // pink wire left 2
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

        if (servo2_active) {    // white wire right  1
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

    }

    // interrupt for background dc motors 
    if(is_active && TMR2IF) {
        TMR2IF = 0;     //clear interrupt bit 
        TMR2 = 63035;   //53035;   //reset timer 2 count

        is_blocked = laserBlocked(1);
        if(is_blocked) {
            set_dc(1, 0);
            clear_dc(1);
            // bin_inhibitor = !bin_inhibitor;
        }

        // if(laser_counter < 70000 && !bin_inhibitor) {
        //     laser_counter++;
        // } else if (laser_counter >= 40000){
        //     laser_counter = 0;
        //     bin_inhibitor = !bin_inhibitor;
        // }

        // // driving two DC motors for the wheel and conveyor belt    
        // if (dc1_active) {
        //     if (stop_count < stop_inactive) {
        //         if (dc_count < dc_duration_low) {
        //             clear_dc(1);
        //             dc_count++;
        //         } else if (dc_count < dc_duration_high){
        //             move_dc1();
        //             dc_count++;
        //         } else {
        //             dc_count = 0;
        //         }
        //         stop_count++;
        //     } else if (stop_count < stop_active) {
        //         clear_dc(1);
        //         stop_count++;
        //     } else {
        //         stop_count = 0;
        //     }
        // }

        if (dc1_active) { //} || bin_inhibitor) { //) { // || bin_inhibitor) {   // 10ms out of 1000ms
            if (stop_count < stop_inactive) {
                if (dc_count < dc_duration_low) {
                    clear_dc(1);
                    dc_count++;
                } else if (dc_count < dc_duration_high){
                    // if (bin_inhibitor) {
                    //     move_dc1();
                    // } else {
                    //     clear_dc(1);
                    // }
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
        }

        // move_dc1();
        if (dc2_active) {
            if(forward) {
                move_dc2(1);
            }
            else {
                move_dc2(0);
            }
        }
     }

    // interrupt for stepper control 
    if(is_active && TMR3IF) {
        TMR3IF = 0;     //clear interrupt bit 
        TMR3 = 63785;//63535; ///*63785;*/ 53035;//60535;   // reset timer 3 count
        // driving two stepper motors for the pusher and the bin plate 
        // move_stepper1(1);
        // move_stepper2(0);
        // clear_stepper(1);
        // clear_stepper(2);

        // if (stepper1_active == 1) {
        //     if(stepper1_duration_count < stepper1_duration) {
        //         move_stepper1(1);
        //         stepper1_duration_count++;
        //     } else {
        //         stepper1_duration_count = 0;
        //         stepper1_active = 0;
        //     }
        // } else if (stepper1_active == 2) {
        //     if(stepper1_duration_count < stepper1_duration) {
        //         move_stepper1(0);
        //         stepper1_duration_count++;
        //     } else {
        //         stepper1_duration_count = 0;
        //         stepper1_active = 0;
        //     }
        // } else {
        //     clear_stepper(1);
        // }

        // if (stepper2_active == 1) {
        //     if(stepper2_duration_count < stepper2_duration) {
        //         move_stepper2(1);
        //         stepper2_duration_count++;
        //     } else {
        //         stepper2_duration_count = 0;
        //         stepper2_active = 0;
        //     }
        // } else if (stepper2_active == 2) {
        //     if(stepper2_duration_count < stepper2_duration) {
        //         move_stepper2(0);
        //         stepper2_duration_count++;
        //     } else {
        //         stepper2_duration_count = 0;
        //         stepper2_active = 0;
        //     }
        // } else {
        //     clear_stepper(2);
        // }
    }
}   // end of the interrupt ISR   

/******************************************************************************************************/
/* motor control codes */

// void pulse_delay(unsigned int pulse) {
//     int numDelayPerTenSec;
//     numDelayPerTenSec = (int)(pulse / 10);
//     for(int i = 0; i < numDelayPerTenSec; i++) {
//         __delay_us(10);
//     }
// }

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
    LATCbits.LATC2 = !LATCbits.LATC2;
    // LATCbits.LATC4 = direction;
}


void move_stepper2(unsigned int direction) {   // pusher, stepper 2
    LATCbits.LATC0 = !LATCbits.LATC0;
    LATCbits.LATC1 = direction;
}


// void move_wheel(char direction, char mode) {
//     LATCbits.LATC4 = direction;
//     // mode 1 is 90 degrees, mode 2 is 180 degrees, and 3 is 270 degrees 
//     if (mode == 1) {
//         for(unsigned int i = 0; i < 50; i++) {
//             LATCbits.LATC2 = 1;
//             __delay_ms(5);
//             LATCbits.LATC2 = 0;
//             __delay_ms(5);
//         }
//     } else if (mode == 2){
//         for(unsigned int i = 0; i < 100; i++) {
//             LATCbits.LATC2 = 1;
//             __delay_ms(5);
//             LATCbits.LATC2 = 0;
//             __delay_ms(5);
//         }
//     } else if (mode == 3) {
//         for(unsigned int i = 0; i < 150; i++) {
//             LATCbits.LATC2 = 1;
//             __delay_ms(5);
//             LATCbits.LATC2 = 0;
//             __delay_ms(5);
//         }
//     } else {
//         LATCbits.LATC2 = 0;
//     }
// }

void move_wheel(char mode) {
    // LATCbits.LATC4 = direction;
    // mode 1 is 90 degrees, mode 2 is 180 degrees, and 3 is 270 degrees 
    if (mode == 1) {
        for(unsigned int i = 0; i < 50; i++) {
            LATCbits.LATC2 = 1;
            __delay_ms(4);
            LATCbits.LATC2 = 0;
            __delay_ms(4);
        }
    } else if (mode == 2){
        for(unsigned int i = 0; i < 100; i++) {
            LATCbits.LATC2 = 1;
            __delay_ms(4);
            LATCbits.LATC2 = 0;
            __delay_ms(4);
        }
    } else if (mode == 3) {
        for(unsigned int i = 0; i < 150; i++) {
            LATCbits.LATC2 = 1;
            __delay_ms(4);
            LATCbits.LATC2 = 0;
            __delay_ms(4);
        }
    } else {
        LATCbits.LATC2 = 0;
    }
}

void move_pusher(char direction) {
    LATCbits.LATC1 = direction;
    // mode 1 is 90 degrees, mode 2 is 180 degrees
    // set_dc(2, 0); // turn off belt
    // clear_dc(2);
    for(unsigned int i = 0; i < 21600; i++) {
        LATCbits.LATC0 = 1;
        __delay_us(50);
        LATCbits.LATC0 = !LATCbits.LATC0;
        __delay_us(50);
        // if(i == 15000) {
        //     set_dc(2, 1); // turn on belt
        //     set_belt_direction(direction);
        // }
    }
}

void clear_stepper(char channel) {
    extern unsigned int stepper1_active, stepper2_active;
    if (channel == 1) {
        stepper1_active = 0;
        LATCbits.LATC2 = 0;
        // LATCbits.LATC4 = 0;
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
    unsigned int duration = 2*angle;  // a function of angle and stepper specs 
    unsigned int command = ((direction == 1)? 1 : 2);
    set_stepper_duration(channel, duration);
    move_stepper(command, channel);
} 


void set_dc(char channel, char active) {
    extern unsigned int dc1_active, dc2_active;
    if (channel == 1) {
        dc1_active = active;
    } else {
        dc2_active = active;
    }
}


void clear_dc(char channel) {
    if (channel == 1) {
        LATEbits.LATE1 = 0;
    } else if (channel == 2) {
        LATCbits.LATC5 = 0; 
        LATDbits.LATD0 = 0;
    }
}

void move_dc1() {
    // RC0, 1, 2 are used for the first DC motor, no pwm control for now
    LATEbits.LATE1 = 1;
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

void set_belt_direction(char direction) {
    extern unsigned int forward;
    forward = direction;
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

unsigned int ir(unsigned int channel, unsigned int average_span) {
    if (channel == 1) {
        for(unsigned int i = 0; i < average_span; i++) {
            if (PORTEbits.RE0) {
                return 0;
            }
        }
        return 1;
    // } else if (channel == 2) {
    //     for(unsigned int i = 0; i < average_span; i++) {
    //         if (PORTEbits.RE1) {
    //             return 0;
    //         }
    //     }
    //     return 1;
    } else {
        return 0;   // default value, 
    }
}

unsigned int laserBlocked(unsigned int average_span) {  // not blocking: high, blocking: low;
    for(unsigned int i = 0; i < average_span; i++) {
        if (!PORTEbits.RE2) {
            return 1;
        }
    }
    return 0;
}

float max_voltage(unsigned int duration) {
    float current = get_voltage(2, 10);
    float temp = 0;
    for (unsigned int i = 0; i < duration; i++) {
        temp = get_voltage(2, 50);
        current = ((temp > current)? temp : current);
    }
    return current;
}

void move_bin(unsigned int bat_type) {
    extern unsigned int previous_type;
    switch(previous_type) {
        case AA_BAT:
            if (bat_type == DRAIN_BAT) {
                // move_wheel(0, 1);
                move_wheel(1);
            } else if (bat_type == C_BAT) {
                // move_wheel(1, 1);
                move_wheel(3);
            } else if (bat_type == NINE_BAT) {
                // move_wheel(0, 2);
                move_wheel(2);
            } else {
                // move_wheel(0, 0);
                move_wheel(0);
            }
            break;
        case C_BAT: 
            if (bat_type == DRAIN_BAT) {
                // move_wheel(0, 2);
                move_wheel(2);
            } else if (bat_type == AA_BAT) {
                // move_wheel(0, 1);
                move_wheel(1);
            } else if (bat_type == NINE_BAT) {
                // move_wheel(1, 1);
                move_wheel(3);
            } else {
                // move_wheel(0, 0);
                move_wheel(0);
            }
            break;
        case NINE_BAT:
            if (bat_type == DRAIN_BAT) {
                // move_wheel(1, 1);
                move_wheel(3);
            } else if (bat_type == C_BAT) {
                // move_wheel(0, 1);
                move_wheel(1);
            } else if (bat_type == AA_BAT) {
                // move_wheel(0, 2);
                move_wheel(2);
            } else {
                // move_wheel(0, 0);
                move_wheel(0);
            }
            break;
        case DRAIN_BAT:
            if (bat_type == AA_BAT) {
                // move_wheel(1, 1);
                move_wheel(3);
            } else if (bat_type == C_BAT) {
                // move_wheel(0, 2);
                move_wheel(2);
            } else if (bat_type == NINE_BAT) {
                // move_wheel(0, 1);
                move_wheel(1);
            } else {
                // move_wheel(0, 0);
                move_wheel(0);
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
            set_servo_duration(1, 1, 9);
            set_servo_duration(1, 0, 191);
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
            set_servo_duration(2, 1, 8);
            set_servo_duration(2, 0, 192);
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

unsigned int test_battery() {
    // AA_BAT 3
    // C_BAT 4
    // NINE_BAT 5
    // DRAIN_BAT 6

    extern unsigned int AA_num, C_num, Nine_num, Drain_num, total_num;
    float D = 0;    // voltage reading D
    float temp = 0;
    
    // Step 1: reset relays, open 1, 2, 3, 4, 5, 6
    LATAbits.LATA3 = 0;   // relay 1
    LATDbits.LATD1 = 0;   // relay 2
    LATAbits.LATA5 = 0;   // relay 3
    LATAbits.LATA4 = 0;   // relay 4
    LATAbits.LATA1 = 0;   // relay 5
    LATAbits.LATA0 = 0;   // relay 6

    __delay_ms(30);

    // Step 2: close relay 1 and 3, check D, if voltage -> 9V, open 1 and 3
    LATAbits.LATA3 = 1;
    LATAbits.LATA5 = 1;
    __delay_ms(30);
    for (unsigned int i = 0; i < 5; i++) {
        readADC(2);
        temp = convert_voltage(); // max_voltage(10);  // get_voltage(2, 50);
        D = (temp>D)? temp:D;
    }
    LATAbits.LATA3 = 0;
    LATAbits.LATA5 = 0;
    if (D > 3.0) {
        Nine_num++;
        total_num++;
        return NINE_BAT;
    }
    D = 0;
    __delay_ms(30);

    // Step 3: close relay 4 and 6, check D, if voltage -> 9V, open 4 and 6
    LATAbits.LATA0 = 1;
    LATAbits.LATA4 = 1;
    __delay_ms(30);
    for (unsigned int i = 0; i < 5; i++) {
        readADC(2);
        temp = convert_voltage(); //max_voltage(10);  // get_voltage(2, 50);
        D = (temp>D)? temp:D;
    }
    LATAbits.LATA4 = 0;
    LATAbits.LATA0 = 0;
    if (D > 3.0) {
        Nine_num++;
        total_num++;
        return NINE_BAT;
    } 
    D = 0;
    __delay_ms(30);

    // Step 4: close relay 3 and 5, check D, if voltage -> C, open 3 and 5 
    LATAbits.LATA5 = 1;
    LATAbits.LATA1 = 1;
    __delay_ms(30);
    for (unsigned int i = 0; i < 5; i++) {
        readADC(2);
        temp = convert_voltage(); //max_voltage(10);  // get_voltage(2, 50);
        D = (temp>D)? temp:D;
    }
    LATAbits.LATA5 = 0;
    LATAbits.LATA1 = 0;
    if (D > 0.20) {
        C_num++;
        total_num++;
        return C_BAT;
    }
    D = 0;
    __delay_ms(30);

    // Step 5: close relay 2 and 6, check D, if voltage -> AA, open 2 and 6 
    LATDbits.LATD1 = 1;
    LATAbits.LATA0 = 1;
    __delay_ms(30);
    for (unsigned int i = 0; i < 5; i++) {
        readADC(2);
        temp = convert_voltage(); //max_voltage(10);  // get_voltage(2, 50);
        D = (temp>D)? temp:D;
    }
    LATDbits.LATD1 = 0;
    LATAbits.LATA0 = 0;
    if (D > 0.20) {
        AA_num++;
        total_num++;
        return AA_BAT;
    }
    D = 0;
    __delay_ms(30);

    // Default case 
    Drain_num++;
    total_num++;
    return DRAIN_BAT;
}