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
	
	global	ARTLSHFTS32U
	global	ARTLSHFTFXS32U

	org	p:
ARTLSHFTS32U:
ARTLSHFTFXS32U:	
	andc	#$001f,y0	; perform a mod 32 shift.
	tstw	y0
	beq		done
	cmp		#16,y0		; if shift count is more than 16
	blt		lt16	    ; do the shift in two steps:
toobig:				
	move	a0,A		; shift A 16-bits left
	asll	a1,y0,A		; shift the remaining bits 
    rts
lt16:	
	do		y0,done
	asl		A
done:
	rts

	ENDSEC

	end
