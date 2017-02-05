/*
 * File:   I2C.c
 * Author: Administrator
 *
 * Created on August 4, 2016, 3:22 PM
 */


#include <xc.h>
#include "I2C.h"
#include "configBits.h"

void I2C_Master_Init(const unsigned long c)
{
  // See Datasheet pg171, I2C mode configuration
  SSPSTAT = 0b00000000;
  SSPCON1 = 0b00101000;
  SSPCON2 = 0b00000000;
  SSPADD = (_XTAL_FREQ/(4*c))-1;
  TRISC3 = 1;        //Setting as input as given in datasheet
  TRISC4 = 1;        //Setting as input as given in datasheet
}

void I2C_Master_Wait()
{
  while ((SSPSTAT & 0x04) || (SSPCON2 & 0x1F));
}

void I2C_Master_Start()
{
  I2C_Master_Wait();
  SEN = 1;
}

void I2C_Master_RepeatedStart()
{
  I2C_Master_Wait();
  RSEN = 1;
}

void I2C_Master_Stop()
{
  I2C_Master_Wait();
  PEN = 1;
}

void I2C_Master_Write(unsigned d)
{
  I2C_Master_Wait();
  SSPBUF = d;
}

unsigned char I2C_Master_Read(unsigned char a)
{
  unsigned char temp;
  I2C_Master_Wait();
  RCEN = 1;
  I2C_Master_Wait();
  temp = SSPBUF;
  I2C_Master_Wait();
  ACKDT = (a)?0:1;
  ACKEN = 1;
  return temp;
}

void delay_10ms(unsigned char n) { 
    while (n-- != 0) { 
        __delay_ms(5); 
    } 
}