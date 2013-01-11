/*
  The code below is published as PUBLIC DOMAIN as Wojciech M. Zabolotny
  ( wzab<at>ise.pw.edu.pl ) 2011.01.11
  No warranty of any kind is provided! Use at your own risk!
*/ 

#ifndef _SPI_ATMEGA88_H_
#define _SPI_ATMEGA88_H_
  #define DDR_SPI_SCK DDRB
  #define PINS_SPI_SCK PINB
  #define PIN_NR_SPI_SCK 5

  #define DDR_SPI_SS DDRB
  #define PINS_SPI_SS PINB
  #define PIN_NR_SPI_SS 2

  #define DDR_SPI_MOSI DDRB
  #define PINS_SPI_MOSI PINB
  #define PIN_NR_SPI_MOSI 3
#endif
