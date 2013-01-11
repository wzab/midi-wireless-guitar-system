/*
  The code below is published as PUBLIC DOMAIN as Wojciech M. Zabolotny
  ( wzab<at>ise.pw.edu.pl ) 2011.01.11
  No warranty of any kind is provided! Use at your own risk!
*/ 

static inline void SPI_MasterInit(void)
{
  /* Set SS , MOSI and SCK output, all others input */
  DDRB |= (1<<0)|(1<<2)|(1<<1);
  /* Enable SPI, Master, set clock rate fck/2 */
  SPSR = (1<<SPI2X);
  SPCR = (1<<SPE)|(1<<MSTR)|(0<<SPR0);
}

static inline uint8_t SPI_Transfer(uint8_t cData)
{
  /* Start transmission */
  SPDR = cData;
  /* Wait for transmission complete */
  while(!(SPSR & (1<<SPIF)))
    {};
  return SPDR;
}


