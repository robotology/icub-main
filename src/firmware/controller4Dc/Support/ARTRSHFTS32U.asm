;----------------------------------------------------------------------
;  Metrowerks Embedded Runtime Support 1998 May 
;
;	rtshift.asm
;	
; 		Copyright © 1998 Metrowerks, Inc.
; 		All rights reserved.
;
; 	Routines
; 	--------
; 	Fixed and integer 32 bit shifts (with and without REP instruction)
;	Algorithms taken from "DSP 56800 Family Manual" section 8.3
;  
;----------------------------------------------------------------------
;

	section		rtlib
	
	global	ARTRSHFTS32U
	global	ARTRSHFTFXS32U

	org	p:
ARTRSHFTS32U: 
ARTRSHFTFXS32U: 
	move	y0,x0
    andc    #$001f,x0       ; perform a mod 32 shift.
	tstw	x0
	beq		done			; if high bit of A is set, lsrr A,0 will put -1 in A2
    cmp     #16,x0          ; if shift count is more than 16 
    blt     lt16_2			; do the shift in two steps: 
    move    a1,a0           ; shift A 16-bits right 
   	move	a2,a1
   	beq		done
lt16_2:
	move	a0,y0
	move	a1,y1
    lsrr    y0,x0,A         ; logically shift 16 the LSB
	move	a1,a0
	move	a2,a1			; clear a1
	asrac	y1,x0,A			; Add MSB shifted.  
done:
    rts

	ENDSEC

	end
