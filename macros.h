/* 
 * File:   macros.h
 * Author: Administrator
 *
 * Created on August 17, 2016, 2:42 PM
 */

#ifndef MACROS_H
#define	MACROS_H

#define __delay_1s() for(char i=0;i<100;i++){__delay_ms(10);}
#define __lcd_newline() lcdInst(0b11000000);
#define __lcd_clear() { lcdInst(0b00000001); __delay_ms(15); }
#define __lcd_home() lcdInst(0b10000000);
#define __bcd_to_num(num) (num & 0x0F) + ((num & 0xF0)>>4)*10


#endif	/* MACROS_H */

