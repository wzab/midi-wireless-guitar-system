/*
  The code below is published as PUBLIC DOMAIN as Wojciech M. Zabolotny
  ( wzab<at>ise.pw.edu.pl ) 2011.01.11
  No warranty of any kind is provided! Use at your own risk!
*/ 

static inline uint8_t SPI_Transfer(uint8_t c)
{
  /* Wait for empty transmit buffer */
  while ( !( UCSR1A & (1<<UDRE1)) );
  /* Put data into buffer, sends the data */
  UDR1 = c;
  /* Wait for data to be received */
  while ( !(UCSR1A & (1<<RXC1)) );
  /* Get and return received data from buffer */
  return UDR1;
}
