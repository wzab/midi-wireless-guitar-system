/*
  This file implements the transmitter part of the wireless guitar system
  written by Wojciech M. Zabolotny ( wzab<at>ise.pw.edu.pl )
  Copyright (C) Wojciech Zabolotny, 2011
  Please treat my modifications as PUBLIC DOMAIN, however if you
  use them, please mention it in documentation of your software
  and/or device (this is a suggestion, not the obligation).

  I do not provide any warranty on my modifications!
  You may use it only on your own risk!
*/

#include "rfm70.h"
#include "../customize.h"
#include "abuf.h"
#include "spi_atmega88.h"
#include "byte_select.h"

volatile uint32_t abuf[ABUF_LEN];
volatile uint8_t abuf_wptr = 0;
volatile uint8_t abuf_rptr = 0;
volatile uint8_t abuf_overrun = 0;

//Overrun LED location
#define PORT_OVR PORTD
#define PINS_OVR PIND
#define DDR_OVR DDRD
#define PIN_NR_OVR 5

//Switches allocation
#define PORT_SW1 PORTD
#define PINS_SW1 PIND
#define DDR_SW1  DDRD
#define PIN_NR_SW1 7

#define PORT_SW2 PORTD
#define PINS_SW2 PIND
#define DDR_SW2  DDRD
#define PIN_NR_SW2 6

#define PORT_SW3 PORTB
#define PINS_SW3 PINB
#define DDR_SW3  DDRB
#define PIN_NR_SW3 1

#define PORT_SW4 PORTB
#define PINS_SW4 PINB
#define DDR_SW4  DDRB
#define PIN_NR_SW4 4

#define PORT_C1 PORTC
#define PINS_C1 PINC
#define DDR_C1  DDRC
#define PIN_NR_C1 4

#define PORT_C2 PORTC
#define PINS_C2 PINC
#define DDR_C2  DDRC
#define PIN_NR_C2 3

static void init_switches(void)
{
  //Set input direction
  DDR_SW1 &= ~(1<<PIN_NR_SW1);
  //Activate pull-up
  PORT_SW1 |= (1<<PIN_NR_SW1);
  //Set input direction
  DDR_SW2 &= ~(1<<PIN_NR_SW2);
  //Activate pull-up
  PORT_SW2 |= (1<<PIN_NR_SW2);
  //Set input direction
  DDR_SW3 &= ~(1<<PIN_NR_SW3);
  //Activate pull-up
  PORT_SW3 |= (1<<PIN_NR_SW3);
  //Set input direction
  DDR_SW4 &= ~(1<<PIN_NR_SW4);
  //Activate pull-up
  PORT_SW4 |= (1<<PIN_NR_SW4);
}

// Switch deglitching routines
// We need really fast code, so to keep it maintaineable,
// I implement it with macro instead of loop
#define proc_switch(condition,state,bit_mask)		\
  if(condition) {					\
    /* switch released */				\
    if (state > (255-SWITCH_THRESHOLD)) {		\
      /* switch was pressed and now is released */	\
      state --;						\
      /* but still report it as pressed */		\
      res |= bit_mask;					\
    } else {						\
      /* switch was released long enough */		\
      state = 0;					\
    }							\
  } else {						\
    /* switch pressed */				\
    if (state < SWITCH_THRESHOLD) {			\
      /* switch was released but now is pressed */	\
      state++;						\
    } else {						\
      /* switch was pressed long enough */		\
      state = 255 ;					\
      /* report it as pressed */			\
      res |= bit_mask ;					\
    }							\
  }

#define SWITCH_THRESHOLD (10)
uint8_t switch_state[4]={0,0,0,0};
      
static inline uint8_t read_switches(void)
{
  uint8_t res = 0;
  proc_switch(PINS_SW1 & (1<<PIN_NR_SW1),switch_state[0],1);
  proc_switch(PINS_SW2 & (1<<PIN_NR_SW2),switch_state[1],2);
  proc_switch(PINS_SW3 & (1<<PIN_NR_SW3),switch_state[2],4);
  proc_switch(PINS_SW4 & (1<<PIN_NR_SW4),switch_state[3],8);
  return res;
}

// Configuration switch deglitching routines
// We need really fast code, so to keep it maintaineable,
// I implement it with macro instead of loop
#define proc_cfg_switch(condition,state,action_pressed,action_released)	\
  if(condition) {							\
    /* switch released */						\
    if (state > (255-SWITCH_THRESHOLD)) {				\
      /* switch was pressed and now is released */			\
      state --;								\
    } else {								\
      /* switch was released long enough */				\
      /* trigger the action when switch was pressed and is released */	\
      if(state>128) action_released ;					\
      state = 0;							\
    }									\
  } else {								\
    /* switch pressed */						\
    if (state < SWITCH_THRESHOLD) {					\
      /* switch was released but now is pressed */			\
      state++;								\
    } else {								\
      /* switch was pressed long enough */				\
      /* trigger the action when switch was released and is pressed */	\
      if(state<128) action_pressed;					\
      state = 255 ;							\
    }									\
  }

/* Functions below are used to change channel up/down
   We have 83 channels 0-82. However most interferences (e.g. WiFi)
   occupy broader bandwidth covering a few consecutive channels.
   Therefore we change channel number not by 1, but by 8.
   Because 83 and 8 are relative primes (well in fact 83 is prime),
   we may select each channel from 0 to 82.
*/
#define MAX_CHANNEL 82
#define CHANNEL_STEP 8
uint8_t rf_channel = INITIAL_CHANNEL;
uint8_t swcfg_state0 = 0;
uint8_t swcfg_state1 = 0;

static inline void channel_up(void)
{
  set_ce(0);
  rf_channel += CHANNEL_STEP;
  if(rf_channel>MAX_CHANNEL) rf_channel -= (MAX_CHANNEL+1); 
  set_channel(rf_channel);
  set_ce(1);
}

static inline void channel_down(void)
{
  set_ce(0);
  rf_channel -= CHANNEL_STEP;
  if(rf_channel>MAX_CHANNEL) rf_channel += (MAX_CHANNEL+1); // Yes, it is not
  // a mistake! We perform calculations modulo MAX_CHANNEL+1, 
  // and rf_channel is uint8_t, so if after subtraction of CHANNEL_STEP
  // we get something above MAX_CHANNEL, it means that result was negative,
  // (but it is placed in uint8_t so its slightly below 255).
  // Therefore we need to correct the result by adding MAX_CHANNEL+1!
  set_channel(rf_channel);
  set_ce(1);
}

static inline void init_cfg_switches(void)
{
  //Set input direction
  DDR_C1 &= ~(1<<PIN_NR_C1);
  //Activate pull-up
  PORT_C1 |= (1<<PIN_NR_C1);
  //Set input direction
  DDR_C2 &= ~(1<<PIN_NR_C2);
  //Activate pull-up
  PORT_C2 |= (1<<PIN_NR_C2);
}

static inline void check_cfg_switches(void)
{
  proc_cfg_switch(PINS_C1 & (1<<PIN_NR_C1),swcfg_state0,channel_up(), {});
  proc_cfg_switch(PINS_C2 & (1<<PIN_NR_C2),swcfg_state1,channel_down(), {});
}

#define NR_OF_ADC_CHANS 4
volatile uint8_t adc_val[NR_OF_ADC_CHANS];
volatile uint8_t switches;
uint8_t adc_step=0;
const uint8_t adc_chan[NR_OF_ADC_CHANS]={0,1,2,5}; //Numbers of scanned channels

// ADC Interrupt routine
// It works, but should be rewritten in assembly,
// or should be running with active interupts!
ISR(ADC_vect,ISR_NOBLOCK)
{
  adc_val[adc_step++] = ADCH;
  if(adc_step==NR_OF_ADC_CHANS)
    adc_step = 0;
  ADMUX = (0 << REFS1) | (1 << REFS0) | (1 << ADLAR) |
    adc_chan[adc_step];
}
// ADC Setup routine
static void ADC_Init(void)
{
  uint8_t i;
  //Disable used digital inputs for used analog inputs
  for(i=0;i<NR_OF_ADC_CHANS;i++)
    DIDR0 |= (1 << adc_chan[i]);
  ADMUX = (0 << REFS1) | (1 << REFS0) | (1 << ADLAR) | 
    (adc_chan[0] << MUX0);
  ADCSRB = (0<<ACME) & (0 << ADTS0) ; //Free runnning mode
  ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | 
    (1 << ADIF) | (1 << ADIE) |
    (7 << ADPS0) ; //The lowest speed!
}

static inline uint8_t abuf_is_empty(void)
{
  if (abuf_rptr == abuf_wptr) return 0xff;
  return 0;
}

static inline void abuf_put(uint32_t t)
{
  uint8_t new_wptr=(abuf_wptr+1) & (ABUF_LEN-1);
  if (new_wptr==abuf_rptr) abuf_overrun=1;
  else {
    abuf[abuf_wptr] = t;
    abuf_wptr = new_wptr;
  }
}
static inline int32_t abuf_get(void)
{
  uint32_t res = abuf[abuf_rptr];
  cli();
  abuf_rptr = (abuf_rptr+1) & (ABUF_LEN-1);
  sei();
  return (int32_t) res; // for CS5344 (or another ADC with left-justified I2S-like format)
  //return (int32_t) (res + res); // for CS5343 (or another ADC with I2S format
}

/* Initialization of the SPI for Cirrus CS5343/4 */
void SPI_Slave_Init(void)
{
  /* Set all SPI related pins as inputs */
  DDR_SPI_SCK &= ~(1<<PIN_NR_SPI_SCK);
  DDR_SPI_SS &= ~(1<<PIN_NR_SPI_SS);
  DDR_SPI_MOSI &= ~(1<<PIN_NR_SPI_MOSI);
  /* Enable SPI */
  SPCR = (1<<SPE);
  /* Enable SPI interrupts */
  SPCR |= (1<<SPIE);
}

char SPI_SlaveReceive(void)
{
  /* Wait for reception complete */
  while(!(SPSR & (1<<SPIF)))
    ;
  /* Return Data Register */
  return SPDR;
}

volatile uint8_t count_led_ovr = 0;

inline void init_led_ovr(void)
{
  DDR_OVR |= (1<<PIN_NR_OVR);
  PORT_OVR |= (1<<PIN_NR_OVR);
}

ISR(TIMER1_COMPA_vect, ISR_NOBLOCK)
{
  uint8_t tmp = count_led_ovr;
  if(tmp) {
    tmp--;
    if(tmp == 0) {
      PORT_OVR |= (1<<PIN_NR_OVR);
    }
    count_led_ovr = tmp;
  }
}

static inline void start_timer1(uint16_t val)
{
  OCR1A = val;
  TCCR1A = 0;
  TCCR1B = (0<<WGM13) | (1 << WGM12) | (1 << CS10);
  TIMSK1 = (1<<OCIE1A);
}

void show_error(uint8_t msg)
{
  uint8_t mask;
  PORT_C2 &= ~(1<<PIN_NR_C2); //Write 0 (we simulate the open-drain output)
  while(1) {
    mask=0x80;
    while(mask) {
      if(mask & msg)
	DDR_C2  |= (1<<PIN_NR_C2); //Activate the output - the LED or beeper is on
      else
	DDR_C2  &= ~(1<<PIN_NR_C2); //Deactivate the output - the LED or beeper is off
      _delay_ms(200);
      mask >>= 1;
    }
    DDR_C2 &= ~(1<<PIN_NR_C2);
    _delay_ms(400);
  }
}


uint8_t packet_nr=0;
static PROGMEM const uint8_t cmd_switch_cfg[]={0x50,0x53};
/* Main routine */
int main(void)
{
  //Initialize the hardware - RFM70 and SPI
  uint8_t b1,b2;
  MCUSR &= ~(1 << WDRF);
  wdt_disable();
  clock_prescale_set(clock_div_1);
  USART_SPI_Init(0);
  rfm70_hw_setup();
  _delay_ms(500);
  set_ce(1);
  set_cs(1);
  b1=read_reg(0x07);
  write_pcmd(cmd_switch_cfg,sizeof(cmd_switch_cfg));
  b2=read_reg(0x07);
  if ((b1 ^ b2) != 0x80) show_error(0xae);
  rfm70_init(rf_channel);
  switch_to_tx_mode();
  init_switches();
  init_cfg_switches();
  ADC_Init();
  abuf_rptr = 0;
  abuf_wptr = 0;
  abuf_overrun = 0;
  SPI_Slave_Init();
  init_led_ovr();
  start_timer1(40000); // 200Hz timer
  sei();
  {
    //As we seem to have problems with the overrun LED,
    //Let's trigger it once to check wether it is working correctly
    PORT_OVR &= ~(1<<PIN_NR_OVR);
    count_led_ovr = 200; // 1 second!
  }
  //now in the loop transmit the data received from the CS5343
  packet_nr = 0;
  //We transmit all the information in packets with different numbers
  while(1) {
    uint8_t i;
    if(abuf_overrun) {
      PORT_OVR &= ~(1<<PIN_NR_OVR);
      abuf_overrun = 0;
      count_led_ovr = 100; //The LED should be on for 500 ms
    }
    //Check the status of the transmitter
    b1=read_reg(0x07);
    if(b1 & 0x10) {
      //Maximum number of retransmissions occured!
      //set_ce(0);
      write_reg(0x07,0x10); //clear interrupt
      //set_ce(1); //retry transfer!
      //Whe we have a problem with transmission, then we may never
      //get to the code servicing packet N*16+7 which checks
      //for channel change buttons.
      //Therefore we check them here as well!
      check_cfg_switches();
    }
    //Write data only if transmitter is ready!
    if((b1 & 0x01) == 0) {
      set_cs(0);
      USART_SPI_Fast_Transfer(0xa0); //Start of the packet!
      //Now transmit the packet specific information
      USART_SPI_Fast_Transfer(packet_nr);
      //Now transmit 10 ADC words
      for(i=0;i<6;i++) {
	int32_t smp1,smp2;
	// We should perform the regular downsampling, but I'm afraid,
	// that AVR's throughput may be too small for implementation
	// of the antialiasing filter
	// Therefore I simple take the mean value of two consecutive samples
	while(abuf_is_empty()) {}; 
	smp1 = abuf_get(); 
	while(abuf_is_empty()) {};
	smp1 = (smp1>>1) + (abuf_get()>>1); 
	while(abuf_is_empty()) {};
	smp2 = abuf_get(); 
	while(abuf_is_empty()) {};
	smp2 = (smp2>>1)+(abuf_get()>>1);
	//Now pack two 20-bit samples into 5 bytes
	//First transmit whole bytes
	USART_SPI_Fast_Transfer(uint32_b3(smp1));
	USART_SPI_Fast_Transfer(uint32_b2(smp1));
	USART_SPI_Fast_Transfer(uint32_b3(smp2));
	USART_SPI_Fast_Transfer(uint32_b2(smp2));
	//Now transmit two nibbles
	//After compilation check the generated code! It shouls use the "swap" instruction
	//to manipulate nibbles!!!
	USART_SPI_Fast_Transfer(
				(uint32_b1(smp1) & 0xf0) | //Upper nibble of 3rd byte of smp1
				((uint32_b1(smp2) & 0xf0) >> 4) //Upper nibble of 3rd byte of smp2
				);
      }
      switch(packet_nr & 0x07) {
      case 0:
	USART_SPI_Fast_Transfer(0x01);
	break;
      case 1:
	USART_SPI_Fast_Transfer(read_switches()); //Scan switches every 8th packet!
	break;
      case 2:
	USART_SPI_Fast_Transfer(adc_val[0]);
	break;
      case 3:
	USART_SPI_Fast_Transfer(adc_val[1]);
	break;
      case 4:
	USART_SPI_Fast_Transfer(adc_val[2]);
	break;
      case 5:
	USART_SPI_Fast_Transfer(adc_val[3]);
	break;
      case 6:
	USART_SPI_Fast_Transfer(0);
	break;
      case 7:
	USART_SPI_Fast_Transfer(0);
	break;
      }
      // Now end until the data are really transmitted!
      USART_SPI_End_Fast_Transfer();
      set_cs(1);
      //set_ce(0);
      if((packet_nr & 0x07)==0x07)  check_cfg_switches(); //We check for channel change request every 8th packet!
      packet_nr++;
    } 
  }
}
