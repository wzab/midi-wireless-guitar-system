#FUSEH = 0xdd
#FUSEL = 0xbf
avrdude -c usbasp -B 3 -p atmega88 -U hfuse:w:0xdd:m -U lfuse:w:0xbf:m 
