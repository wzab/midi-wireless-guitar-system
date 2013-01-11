/*
  The code below is published as PUBLIC DOMAIN as Wojciech M. Zabolotny
  ( wzab<at>ise.pw.edu.pl ) 2011.01.11
  No warranty of any kind is provided! Use at your own risk!
*/ 

static inline uint8_t USART_SPI_Transfer(uint8_t c)
{
  const uint8_t status_mask=((1<<RXC0) | (1<<TXC0));
  /* Wait for empty transmit buffer */
  while ( !( UCSR0A & (1<<UDRE0)) ){};
  /* Put data into buffer, sends the data */
  cli();
  UDR0 = c;
  UCSR0A = (1<<TXC0); //Clear the completed transfer flag!
  sei();
  /* Wait for data to be received and transmission completed 
     Please note, that it is not enough to wait for availability
     of received data (RXC0), if you set SS high right after
     SS is set, you'll experience SPI frame error in the RFM70!
     I've lost quite a long time trying to debug this problem!!!
  */
  while ( (UCSR0A & status_mask) != status_mask ){};
  /* Get and return received data from buffer */
  return UDR0;
}

static inline void USART_SPI_Fast_Transfer(uint8_t c)
{
  /* Wait for empty transmit buffer */
  while ( !( UCSR0A & (1<<UDRE0)) );
  /* Put data into buffer, sends the data */
  UDR0 = c;
}

static inline uint8_t USART_SPI_End_Fast_Transfer(void)
{
  uint8_t dummy;
  const uint8_t status_mask=((1<<RXC0) | (1<<TXC0));
  /* Wait for data to be received and transmission completed 
     Please note, that it is not enough to wait for availability
     of received data (RXC0), if you set SS high right after
     SS is set, you'll experience SPI frame error in the RFM70!
     I've lost quite a long time trying to debug this problem!!!
  */
  while ( (UCSR0A & status_mask) != status_mask ){};
  /* Flush received data from buffer */
  while (UCSR0A & (1<<RXC0)) dummy=UDR0;
}

static inline void USART_SPI_Init( unsigned int baud )
{
  UBRR0 = 0;
  /* Setting the XCKn port pin as output, enables master mode. */
  DDRD |= (1 << 4); //  XCK1_DDR |= (1<<XCK1);
  /* Set MSPI mode of operation and SPI data mode 0. */
  UCSR0C = (1<<UMSEL01)|(1<<UMSEL00)|(0<<UCPHA0)|(0<<UCPOL0);
  /* Enable receiver and transmitter. */
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);
  /* Set baud rate. */
  /* IMPORTANT: The Baud Rate must be set after the transmitter is enabled
   */
  UBRR0 = baud;
}

