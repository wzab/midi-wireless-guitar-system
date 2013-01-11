/*
  The code below is published as PUBLIC DOMAIN as Wojciech M. Zabolotny
  ( wzab<at>ise.pw.edu.pl ) 2011.01.11
  No warranty of any kind is provided! Use at your own risk!
*/ 

#ifndef _RFM70_H_
#define _RFM70_H_

/* Includes: */
#include <ctype.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "spi_atmega32u4.h"

#define PORT_CS PORTB
#define DDR_CS DDRB
#define PIN_CS 0
#define PORT_CE PORTE
#define DDR_CE DDRE
#define PIN_CE 6

__attribute__ ((always_inline))
static inline void set_cs(uint8_t val)
{
  if(val)
    PORT_CS |= (1<<PIN_CS);
  else 
    PORT_CS &= ~(1<<PIN_CS);
}

__attribute__ ((always_inline))
static inline void set_ce(uint8_t val)
{
  if(val)
    PORT_CE |= (1<<PIN_CE);
  else 
    PORT_CE &= ~(1<<PIN_CE);
}

static inline void rfm70_hw_setup(void)
{
  DDR_CS |= (1<<PIN_CS);
  DDR_CE |= (1<<PIN_CE);
}

__attribute__ ((always_inline))
static inline uint8_t read_reg(uint8_t reg) 
{
  uint8_t res;
  set_cs(0);
  SPI_Transfer(reg);
  res=SPI_Transfer(0);
  set_cs(1);
  return res;
}

__attribute__ ((always_inline))
static inline void write_reg(uint8_t reg, uint8_t val)
{
  set_cs(0);
  SPI_Transfer(reg | 0x20);
  SPI_Transfer(val);
  set_cs(1);  
}

uint8_t read_reg(uint8_t reg);
void write_pcmd(uint8_t * cmd, uint8_t len);
void rfm70_init(void);
void rfm70_init1(void);
void rfm70_init2(void);
void switch_to_tx_mode(void);
void switch_to_rx_mode(void);
void show_error(uint8_t msg);
void set_channel(uint8_t cnum);
#endif
