/*
  The code below is based on the PIC demo code for RFM70
  provided by HOPE RF.
  I (Wojciech M. Zabolotny wzab<at>ise.pw.edu.pl) have modified it
  for AVR.
  All my modifications are PUBLIC DOMAIN.
 */
#include "rfm70.h"
#include "../customize.h"
static PROGMEM const uint8_t set1[][4]={
  {0x40, 0x4b, 0x01, 0xe2},
  {0xc0, 0x4b, 0x00, 0x00},
  {0xd0, 0xfc, 0x8c, 0x02},
  {0x99, 0x00, 0x39, 0x41},
  {0xd9, 0x9e, 0x86, 0x0b},// #?? d9 czy f9?
  {0x24, 0x06, 0x7f, 0xa6},
  {0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00},
  {0x00, 0x00, 0x00, 0x00},
  {0x00, 0x12, 0x73, 0x00},
  {0x36, 0xB4, 0x80, 0x00},
};
static PROGMEM const uint8_t set1_14[]={0x41,0x20,0x08,0x04,0x81,0x20,0xCF,0xF7,0xFE,0xFF,0xFF};
static PROGMEM const uint8_t set0[][2]={
  {0,0x7F}, //mask RX_DR\TX_DS\MAX_RT (we do not use IRQ!),Enable CRC ,2byte,POWER UP,PRX
  {1,0x1}, //Enable AACK only in pipe 0! Was:{1,0x3F}, //Enable auto acknowledgement data pipe5\4\3\2\1\0
  {2,0x01}, //Enable only RX pipe 0 //Enable RX Addresses pipe5\4\3\2\1\0
  {3,0x01}, //RX/TX address field width 3 bytes
  {4,0x0f}, //up to 15 retransmissions 250,A5(Bs delay! Was: {4,0xff}, //auto retransmission dalay (4000us),auto retransmission count(15)
  {5,INITIAL_CHANNEL}, //(5,0x17), #32 channel
  {6,0x3f}, //6,0x17), #air data rate-1M,out power 0dbm,setup LNA gain
  {7,0x07}, //
  {8,0x00}, //
  {9,0x00}, //
  {12,0xc3},// only LSB Receive address data pipe 2, MSB bytes is equal to RX_ADDR_P1[39:8]
  {13,0xc4},// only LSB Receive address data pipe 3, MSB bytes is equal to RX_ADDR_P1[39:8]
  {14,0xc5},// only LSB Receive address data pipe 4, MSB bytes is equal to RX_ADDR_P1[39:8]
  {15,0xc6},// only LSB Receive address data pipe 5, MSB bytes is equal to RX_ADDR_P1[39:8]
  {17,0x20},// Number of bytes in RX payload in data pipe0(32 byte) 
  {18,0x20},// Number of bytes in RX payload in data pipe1(32 byte)
  {19,0x20},// Number of bytes in RX payload in data pipe2(32 byte)
  {20,0x20},// Number of bytes in RX payload in data pipe3(32 byte)
  {21,0x20},// Number of bytes in RX payload in data pipe4(32 byte)
  {22,0x20},// Number of bytes in RX payload in data pipe5(32 byte)
  {23,0x00},// fifo status
  {28,0x00},// No dynamic payload length! {28,0x3F},// Enable dynamic payload length data pipe5\4\3\2\1\0
  {29,0x07},// Enables Dynamic Payload Length,Enables Payload with ACK,Enables the W_TX_PAYLOAD_NOACK command 
};
static PROGMEM const uint8_t cmd_activate[]={0x50,0x73};
static PROGMEM const uint8_t cmd_tog1[]={0xd9 | 0x06, 0x9e, 0x86, 0x0b}; //assosciated with set1[4]!
static PROGMEM const uint8_t cmd_tog2[]={0xd9 & ~0x06, 0x9e, 0x86, 0x0b};
static PROGMEM const uint8_t cmd_flush_rx[]={0xe2,0x00};
static PROGMEM const uint8_t cmd_flush_tx[]={0xe1,0x00};
static PROGMEM const uint8_t cmd_switch_cfg[]={0x50,0x53};

static PROGMEM const uint8_t adr0[]={SET_NUMBER,'W','G'};
//static PROGMEM uint8_t adr1[]={SET_NUMBER,'W','G'};

uint8_t USART_SPI_Transfer(uint8_t c);


void write_pcmd(uint8_t const * cmd, uint8_t len)
{
  set_cs(1);
  set_cs(0);
  while(len--) {
    USART_SPI_Transfer(pgm_read_byte(cmd++));
  };
  set_cs(1);
}

void write_reg(uint8_t reg, uint8_t val)
{
  set_cs(1);
  set_cs(0);
  USART_SPI_Transfer(reg | 0x20);
  USART_SPI_Transfer(val);
  set_cs(1);  
}

void switch_cfg(uint8_t cnum)
{
  uint8_t tmp = read_reg(0x07) & 0x80;
  if(cnum) {
    if(!tmp)
      write_pcmd(cmd_switch_cfg,sizeof(cmd_switch_cfg));
  } else {
    if(tmp)
      write_pcmd(cmd_switch_cfg,sizeof(cmd_switch_cfg));
  }
}

void switch_to_rx_mode(void)
{
  uint8_t val;
  write_pcmd(cmd_flush_rx,sizeof(cmd_flush_rx));
  val = read_reg(0x07);
  write_reg(0x07,val);
  set_ce(0);
  val=read_reg(0x00);
  val |= 0x01;
  write_reg(0x00,val);
  set_ce(1);
}

void switch_to_tx_mode(void)
{
  uint8_t val;
  write_pcmd(cmd_flush_tx,sizeof(cmd_flush_tx));
  set_ce(0);
  val=read_reg(0x00);
  val &= ~0x01;
  write_reg(0x00,val);
  set_ce(1);
}

void set_channel(uint8_t cnum)
{
  write_reg(5, cnum);
}


void write_reg_pbuf(uint8_t reg, uint8_t const * buf, uint8_t len)
{
  set_cs(1);
  set_cs(0);
  USART_SPI_Transfer(reg | 0x20);
  while(len--)
    USART_SPI_Transfer(pgm_read_byte(buf++));
  set_cs(1);
}

uint8_t send_packet(uint8_t * data, uint8_t len)
{
   uint8_t status;
   switch_to_tx_mode();
   status = read_reg(0x17); //FIFO_STATUS
   if (status & 0x20) return 0xff; //Error?
   set_cs(0);
   USART_SPI_Transfer(0xb0);
   while(len--) {
     USART_SPI_Transfer(*(data++));
   }
   set_cs(1);
   return 0;
}

void rfm70_init(uint8_t channel)
{
  uint8_t i;
  switch_cfg(0);
  for(i=0;i<20;i++) {
    write_reg(pgm_read_byte(&set0[i][0]),pgm_read_byte(&set0[i][1]));
    }
  write_reg_pbuf(10,adr0,sizeof(adr0));
  //write_reg_pbuf(11,adr1,sizeof(adr1)); // PIPE 1 not used!
  write_reg_pbuf(16,adr0,sizeof(adr0));
  set_channel(channel);
  if(!read_reg(29))
    write_pcmd(cmd_activate,sizeof(cmd_activate));
  write_reg(pgm_read_byte(&set0[22][0]),pgm_read_byte(&set0[22][1]));
  write_reg(pgm_read_byte(&set0[21][0]),pgm_read_byte(&set0[21][1]));
  switch_cfg(1);
  for(i=0;i<14;i++) {
    write_reg_pbuf(i,set1[i],sizeof(set1[i]));
    }
  write_reg_pbuf(14,set1_14,sizeof(set1_14));
  write_reg_pbuf(4,cmd_tog1,sizeof(cmd_tog1));
  write_reg_pbuf(4,cmd_tog2,sizeof(cmd_tog2));
  //delay 50 ms
  _delay_ms(50);
  switch_cfg(0);
  switch_to_rx_mode();
}


