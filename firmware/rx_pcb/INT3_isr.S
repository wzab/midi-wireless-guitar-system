#include <avr/io.h>				;required for register definitions
#include "abuf.h"
     .data
;  .section .noinit
smp_b: .byte 0,0,0,0
smp_mask: .byte 0	
					
    .extern abuf_rptr		; buffer read pointer  (uint8_t)
    .extern abuf_wptr	        ; buffer write pointer (uint8_t)
    .extern abuf		; buffer (vector of uint32_t with length ABUF_LEN)
    .extern abuf_overrun        ; buffer overrun flag (uint8_t)
	.text
	.macro SPI_Transfer_r0 
	out	SPDR,r0
loop0:	sbis    SPSR,SPIF
	rjmp	loop0
	in	r0,SPDR	
	.endm	
	.global INT3_vect                
INT3_vect:
	 push    r0
	 in      r0, 0x3f        
	 push	r0
	 push	r30
	 push   r31
	;;  end of prolog
	cbi	EIMSK,INT3
	sbi	EIFR,INTF0
	ldi     r31,0x02
	in	r30,PORTD
	eor     r30,r31
	out	PORTD,r30

int3_rcv_loop:	
	;;  write 0x40 to STATUS register
	ldi	r0,0x07
	cbi	PORT_CS,PIN_CS
	SPI_Transfer_r0
	ldi	r0,0x40
	SPI_Transfer_r0
	sbi	PORT_CS,PIN_CS
	;;  read the 0x17 register
	ldi	r0,0x17
	cbi	PORT_CS,PIN_CS
	SPI_Transfer_r0
	ldi	r0,0x00
	SPI_Transfer_r0
	sbi	PORT_CS,PIN_CS
	;; Check if bit 1 is set
	sbrc	r0,0
	rjmp	exit_loop
	;; Read the payload length
exit_loop:	
	cli
	sbi	EIMSK,INT3
	;;  epilog - repeated multiple times to save clocks on jumps
	pop     r31
	pop	r30
	 pop     r0
         out     0x3f, r0
         pop     r0
         reti	