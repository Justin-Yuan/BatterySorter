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

// function declarations 
void set_time(void);
void current_time(unsigned char*);
void select_menu(unsigned char);
int calculate_elapsed_time(unsigned char*);
int is_termination(unsigned int);

void readADC(char);
float convert_voltage();
float average_voltage(unsigned int);
float convert_to_valid_voltage(float);
float get_voltage(char, unsigned int);

void clean_count(void);
uint8_t Eeprom_ReadByte(uint16_t);
void Eeprom_WriteByte(uint16_t, uint8_t);
uint16_t next_address(uint16_t);
void show_log(uint16_t);
// <TODO>
void pulse_delay(unsigned int pulse);
void move_stepper1();
void move_stepper_angle(unsigned int, unsigned int);
void move_servo1(unsigned int);
void move_servo2(unsigned int);
void move_dc1();
void move_dc1();
//</TODO>
unsigned int regulate_signal(unsigned int);
unsigned int ir(unsigned int, unsigned int); 
unsigned int microswitch(unsigned int);
unsigned int test_battery();


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

int motorstep = 0;


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
    INT1IE = 1;	    //enable external interrupt for keypad
    INT1IF = 0;	    //turn off external interrupt flag
    
    ADCON0 = 0x00;  //Disable ADC
    ADCON1 = 0x0B;  //AN0 to AN3 used as analog input
    CVRCON = 0x00; // Disable CCP reference voltage output
    CMCONbits.CIS = 0;
    ADFM = 1;

    GIE = 1;        //globally enable interrupt
    PEIE = 1;       //enable peripheral interrupt
    TMR0IE = 1;     //enable TMR0 overflow interrupt
    TMR0IF = 0;	    //turn off TMR0 overflow interrupt flag
    TMR1IE = 1;     //enable TMR1 overflow interrupt
    TMR1IF = 0;     //turn off TMR1 overflow interrupt flag
    TMR2IE = 1;     //enable TMR2 overflow interrupt
    TMR2IF = 0;     //turn off TMR2 overflow interrupt flag

    // intialize timers for interrupt 
    // timer 0 
    T0CON = 0b00000111;
    TMR0 = 55770;
    // timer 1
    T1CON = 0b10000000;
    TMR1 = 58035;
    // timer 2
    T2CON = 0b10000000;
    //TMR2 = ;  // need to determine the third timer 
    
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
            LATCbits.LATC6 = 1; //RC6 = 1 , free keypad pins
            // unsigned char time[7];
            elapsed_time = 1;
            clean_count();
           // set_time();

            unsigned int is_battery = 1;
            unsigned int is_hit = 0;
            T0CONbits.TMR0ON = 1;   //turn on timers
            T1CONbits.TMR1ON = 1; 
            float voltage = 0;
            
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
            
//            for(unsigned int i = 0; i < 4; i++) {
//                move_servo(100,5);
//            }
            
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
                //move_stepper(100,5);
                
                //LATCbits.LATC0 = 1;
                
                //voltage = get_voltage(2, 50);
                //__delay_1s();

               // LATCbits.LATC0 = 1;
                // LATCbits.LATC1 = 1;


                
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
                
//                current_time(time);
//                elapsed_time = calculate_elapsed_time(time);
//                printf(" %3d %d %x %x %f", elapsed_time, is_battery, ADRESH, ADRESL, convert_voltage(ADRESH, ADRESL));
//                __delay_ms(300);
//                termination(elapsed_time);
                __lcd_clear();
                __lcd_home();
                //printf("%d", elapsed_time);
                //printf("  %f", voltage);
                __lcd_newline();
                //printf("%d", voltage > 0.3);
                __delay_ms(10);
                is_wait = is_termination(elapsed_time);
            
            }

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

            LATCbits.LATC0 = 0; // RC1 = 0 enable keypad     TODO, change pin assignment 
            
            // testing 
            //LATCbits.LATC0 = 0;
            //LATCbits.LATC1 = 0;

            is_wait = !is_wait;
            is_active = !is_active; // reset the operation flag

            T0CONbits.TMR0ON = 0;   //turn off timers
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
int is_termination(unsigned int time_now) {
    // if (time_now > 30) { extern unsigned int is_wait; is_wait = 1; }
    if (time_now > 30) { return 1; }
    else { return 0; }
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
    // return ((ADRESH<<8)+ADRESL);
    // return __bcd_to_num(ADRESL)
}

float average_voltage(unsigned int average_span) {
    float vol = 0;
    for(unsigned int i = 0; i < average_span; i++) {
        vol += convert_voltage();
        // __delay_ms(5);
    }
    return vol/average_span;
}

float convert_to_valid_voltage(float raw_voltage) {
    return ((float)((int)(raw_voltage * 1000)))/1000.0;
}

float get_voltage(char channel, unsigned int average_span) {
    readADC(channel);
    return convert_to_valid_voltage(average_voltage(average_span));
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
    
    // interrupt for background operations 
    if(is_active && TMR1IF) {
        TMR1IF = 0;     //clear interrupt bit 
        TMR1 = 58035;   //reset timer 1 count

//        if(motorstep < 2000) {
//            move_servo1(motorstep);
//            motorstep = motorstep + 20;
//        } else {
//            motorstep = 1000;
//        }
        
        // if(motorstep < 2000) {
           // move_stepper1();
            // motorstep++;
        // }
        
        //move_servo1(motorstep);
        //printf("ppp");
        
        // move_stepper1();
        // __delay_ms(10);
        // move_stepper_angle(1, 100);
         move_dc1();
        // move_dc2();
    }
}   

/******************************************************************************************************/
/* motor control codes */

void pulse_delay(unsigned int pulse) {
    int numDelayPerTenSec;
    numDelayPerTenSec = (int)(pulse / 10);
    for(int i = 0; i < numDelayPerTenSec; i++) {
        __delay_us(10);
    }
}


void move_stepper1() {  // 200 steps per revolution <-> 1 step is 1.8 degree 
   // for(unsigned int i = 0; i < duration; i++) {
   //     LATCbits.LATC0 = 1;     // speed output pin 
   //     LATCbits.LATC1 = 1;     // direction output pin 
   //     __delay_us(500);
   //     LATCbits.LATC0 = 0;     // 
   //     __delay_us(500);
   //     __delay_ms(10);   
   // }
   LATEbits.LATE0 = 1;     // speed output pin 
   LATEbits.LATE1 = 1;     // direction output pin 
   __delay_us(500);
   LATEbits.LATE0 = 0;     // 
   __delay_us(500);
   //__delay_ms(5);
}

//void move_stepper_angle(unsigned int stepper_id, unsigned int angle) {
//    unsigned int duration = angle;
//    for(unsigned int i = 0; i < duration; i++) {
//        if(stepper_id == 1) {
//            move_stepper1();
//            __delay_ms(5);
//        } 
//    }
//} 


void move_servo1(unsigned int pulse) {
    // RC6 is used for the first servo motor, 5-10% duty cycle(900-2100us pulse width)
    LATCbits.LATC6 = 1;
    pulse_delay(pulse);
    LATCbits.LATC6 = 0;
    __delay_ms(3);
}


void move_servo2(unsigned int pulse) {
    // RC7 is used for the second servo motor, 5-10% duty cycle(900-2100us pulse width)
    LATCbits.LATC7 = 1;
    pulse_delay(pulse);
    LATCbits.LATC7 = 0;
    __delay_ms(20);
}


void move_dc1() {
    // RC0, 1, 2 are used for the first DC motor, no pwm control for now
    LATCbits.LATC2 = 1;
    LATCbits.LATC0 = 1;
    LATCbits.LATC1 = 0;
    __delay_ms(5);
    LATCbits.LATC2 = 0;
    __delay_ms(5);
}


void move_dc2() {
    // RC3, 4, 5 are used for the second DC motor, no pwm control for now
    LATCbits.LATC5 = 1;
    LATCbits.LATC3 = 1;
    LATCbits.LATC4 = 0;
    __delay_ms(5);
    LATCbits.LATC5 = 0;
    __delay_ms(5);
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
            return 0;
        }
    }
    return 1;
}


unsigned int test_battery() {
    // AA_BAT 3
    // C_BAT 4
    // NINE_BAT 5
    // DRAIN_BAT 6

    extern unsigned int AA_num, C_num, Nine_num, Drain_num, total_num;
    float D = 0;    // voltage reading D
    
    // Step 1: reset relays, open 1, 2, 3, 4, 5, 6
    TRISA = 0b11110011;
    LATAbits.LATA7 = 0;   // relay 1
    LATAbits.LATA6 = 0;   // relay 2
    LATAbits.LATA5 = 0;   // relay 3
    LATAbits.LATA4 = 0;   // relay 4
    LATAbits.LATA1 = 0;   // relay 5
    LATAbits.LATA0 = 0;   // relay 6

    // Step 2: close relay 1 and 3, check D, if voltage -> 9V, open 1 and 3
    LATAbits.LATA7 = 1;
    LATAbits.LATA5 = 1;
    D = get_voltage(2, 50);
    if (D > 3.82) {
        Nine_num++;
        total_num++;
        return NINE_BAT;
    } else if (D > 0.10) {
        Drain_num++;
        total_num++;
        return DRAIN_BAT;
    }
    LATAbits.LATA7 = 0;
    LATAbits.LATA5 = 0;

    // Step 3: close relay 4 and 6, check D, if voltage -> 9V, open 4 and 6
    LATAbits.LATA0 = 1;
    LATAbits.LATA4 = 1;
    D = get_voltage(2, 50);
    if (D > 3.82) {
        Nine_num++;
        total_num++;
        return NINE_BAT;
    } else if (D > 0.10) {
        Drain_num++;
        total_num++;
        return DRAIN_BAT;
    }
    LATAbits.LATA4 = 0;
    LATAbits.LATA0 = 0;

    // Step 4: close relay 3 and 5, check D, if voltage -> C, open 3 and 5 
    LATAbits.LATA5 = 1;
    LATAbits.LATA1 = 1;
    D = get_voltage(2, 50);
    if (D > 0.63) {
        C_num++;
        total_num++;
        return C_BAT;
    } else if (D > 0.10) {
        Drain_num++;
        total_num++;
        return DRAIN_BAT;
    }
    LATAbits.LATA5 = 0;
    LATAbits.LATA1 = 0;

    // Step 5: close relay 2 and 6, check D, if voltage -> AA, open 2 and 6 
    LATAbits.LATA6 = 1;
    LATAbits.LATA0 = 1;
    D = convert_voltage(2, 50);
    if (D > 0.63) {
        AA_num++;
        total_num++;
        return AA_BAT;
    } else if (D > 0.10) {
        Drain_num++;
        total_num++;
        return DRAIN_BAT;
    }
    LATAbits.LATA6 = 0;
    LATAbits.LATA0 = 0;
}


// void push_battery(unsigned int signal) {
    
// }


// void move_bin(unsigned int bat_type) {
//     switch(bat_type) {
//         case AA_BAT:

//             break;
//         case C_BAT: 

//             break;
//         case NINE_BAT:

//             break;
//         case DRAIN_BAT:

//             break;
//         default:    // default should be drained battery due to a higher probability

//             break; 
//     }
// }

