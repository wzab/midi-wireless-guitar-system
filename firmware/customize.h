/*
  The code below is published as PUBLIC DOMAIN as Wojciech M. Zabolotny
  ( wzab<at>ise.pw.edu.pl ) 2011.01.11
  No warranty of any kind is provided! Use at your own risk!
*/ 
/*
  When you compile the firmware for another set of Rx/Tx, you should set the set number
  here (0x00-0xff) and the initial channel (0x00-0x52)
 */
#ifndef _CUSTOMIZE_H_
#define _CUSTOMIZE_H_
#define SET_NUMBER 0x30
#define INITIAL_CHANNEL 0x50
/* Uncomment the line below to emulate 48kHz sampling for Windows. */
#define WGS_EMULATE_48KHZ 1 
#endif
