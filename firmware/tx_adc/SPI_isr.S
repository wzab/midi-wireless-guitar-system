;;  The code below is written and published as PUBLIC DOMAIN
;;  by Wojciech M. Zabolotny ( wzab<at>ise.pw.edu.pl ) 2011.01.11
;;  No warranty of any kind is provided! Use at your own risk! 
#include <avr/io.h>				;required for register definitions
#include "abuf.h"
#include "spi_atmega88.h"
		
     .data
;  .section .noinit
smp_b: .byte 0,0,0,0
smp_mask: .byte 0	
					
    .extern abuf_rptr		; buffer read pointer  (uint8_t)
    .extern abuf_wptr	        ; buffer write pointer (uint8_t)
    .extern abuf		; buffer (vector of uint32_t with length ABUF_LEN)
    .extern abuf_overrun        ; buffer overrun flag (uint8_t)
	.text
	.global SPI_STC_vect                
SPI_STC_vect:	
	 push    r0
	 in      r0, 0x3f        
	 push r0
	;;  end of prolog
	 in	r0,_SFR_IO_ADDR(SPSR) ;  read status to clear interrupt
	 sbic	_SFR_IO_ADDR(PINS_SPI_SS),PIN_NR_SPI_SS ;  Test the SS input
	 rjmp	end_of_frame	;  If SS is high, this is end of data in our channel
	 lds	r0,smp_mask
	 sbrc	r0,0
	 rjmp	byte_1st
	 sbrc   r0,1
	 rjmp   byte_2nd
	 sbrc   r0,2
	 rjmp   byte_3rd
	 sbrc   r0,3
	 rjmp   byte_4th
	;; It should not happen, but we should be prepared for nonstandard MCLK/SCLK ratio
	 in	r0,_SFR_IO_ADDR(SPDR)
	;;  epilog - repeated multiple times to save clocks on jumps
	 pop     r0
         out     0x3f, r0
         pop     r0
         reti	
byte_1st:
	;; update the byte mask
	 lsl	r0
	 sts    smp_mask,r0	
	;; store the data
	 in	r0,_SFR_IO_ADDR(SPDR)
	 sts    smp_b,r0
	;;  epilog - repeated multiple times to save clocks on jumps
	 pop     r0
         out     0x3f, r0
         pop     r0
         reti
byte_2nd:
	;; update the byte mask
	 lsl	r0
	 sts    smp_mask,r0	
	;; store the data
	 in	r0,_SFR_IO_ADDR(SPDR)
	 sts    smp_b+1,r0
	;;  epilog - repeated multiple times to save clocks on jumps
	 pop     r0
         out     0x3f, r0
         pop     r0
         reti
byte_3rd:
	;; update the byte mask
	 lsl	r0
	 sts    smp_mask,r0	
	;; store the data
	 in	r0,_SFR_IO_ADDR(SPDR)
	 sts    smp_b+2,r0
	;;  epilog - repeated multiple times to save clocks on jumps
	 pop     r0
         out     0x3f, r0
         pop     r0
         reti
byte_4th:
	;; update the byte mask
	 lsl	r0
	 sts    smp_mask,r0	
	;; store the data
	 in	r0,_SFR_IO_ADDR(SPDR)
	 sts    smp_b+3,r0
	;;  epilog - repeated multiple times to save clocks on jumps
	 pop     r0
         out     0x3f, r0
         pop     r0
         reti
end_of_frame:
	;; Now, at the end of frame we can perform more
	;; sophisticated processing
	push	r30
	push    r31
	in	r30,_SFR_IO_ADDR(SPDR)
	;; first, verify that it is not the 4th byte
	 lds	r0,smp_mask
	 sbrs	r0,4
	 rjmp	not_4th_byte
	;; update the byte mask
	 lsl	r0
	 sts    smp_mask,r0	
	;; store the data
	 sts    smp_b+3,r30
not_4th_byte:		
	;; set mask for next words
	 ldi    r30,1
	 sts    smp_mask,r30
	;; Now copy all bytes to the buffer
	;; Check if there is a free space
	 lds	r30,abuf_wptr
	 lds    r0,abuf_rptr
	 mov	r31,r30
	 inc	r31
	 andi	r31, (ABUF_LEN-1)
	 cp	r0,r31
	 breq	no_space
	;; There is free space in the buffer - copy the data
	 sts    abuf_wptr,r31	; Store new abuf_wptr
	 ldi	r31,0
	;; Multiply Z by 4
	 add	r30,r30
	 adc	r31,r31
	 add	r30,r30
	 adc	r31,r31
	;; Find the address in abuf
	 subi   r30,lo8(-(abuf))
	 sbci   r31,hi8(-(abuf))
	 lds	r0,smp_b+3
	 st    Z+,r0		
	 lds	r0,smp_b+2
	 st    Z+,r0
	 lds	r0,smp_b+1	
	 st    Z+,r0
	 lds	r0,smp_b
	 st    Z+,r0
exit_isr_2:
	;; exit isr when r31,r30 and r0 were saved
	;; extended epilog
	 pop	r31
	 pop	r30
	 pop     r0
         out     0x3f, r0
         pop     r0
         reti
no_space:	
	ldi r30,0xff
	sts abuf_overrun,r30
	;; this is anyway emergency situation and sampling was disturbed
	;; so we can loose some cycles for rjmp ;-)
	;; I do not provide another copy of epilog, using rjmp instead...
	rjmp exit_isr_2 
