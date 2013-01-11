#ifndef _RFM70_H_
#define _RFM70_H_

#define F_CPU 8000000


/* Includes: */
#include <ctype.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "usart_spi_atmega88.h"

#define PORT_CS PORTD
#define DDR_CS DDRD
#define PIN_CS 3
#define PORT_CE PORTD
#define DDR_CE DDRD
#define PIN_CE 2

static inline void set_cs(uint8_t val)
{
  if(val)
    PORT_CS |= (1<<PIN_CS);
  else 
    PORT_CS &= ~(1<<PIN_CS);
}

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

static inline uint8_t read_reg(uint8_t reg)
{
  uint8_t res;
  set_cs(0);
  USART_SPI_Transfer(reg);
  res=USART_SPI_Transfer(0);
  set_cs(1);
  return res;
}

void write_reg(uint8_t reg, uint8_t val);
uint8_t read_reg(uint8_t reg);
void write_pcmd(uint8_t const * cmd, uint8_t len);
void rfm70_init(uint8_t channel);
void switch_to_tx_mode(void);
void switch_to_rx_mode(void);
void show_error(uint8_t msg);
void set_channel(uint8_t cnum);
#endif
