;----------------------------------------------------------------------
;  Metrowerks Embedded Runtime Support 1998 May 
;
;	rtdiv32.asm
;	
; 		Copyright © 1998 Metrowerks, Inc.
; 		All rights reserved.
;
; 	Routines
; 	--------
; 	32 bit signed and unsigned integer divide
;  
;----------------------------------------------------------------------
 
	section	rtlib
	include	"asmdef.h"

    global  ARTDIVS32UZ

    org p:
    
ARTDIVS32UZ:
	move	x:(sp-2),b		; get the divisor off the stack
	move	b1,y1			; save the sign bit in a temp
	move	x:(sp-3),b0
	abs		B
	lea		(sp)+			; push the absolute value of divisor
	move	b0,x:(sp)+
	move	b1,x:(sp)
	eor		a1,y1			; get result sign
	move	y1,x:mr0		; save it in mr0
	abs		A				; dividend

	jsr		ARTDIVU32UZ		; call the unsigned version
    lea		(sp-2)     
	
	tstw	x:mr0			; Fix the result
	bge		Done
	neg		A
Done:
    rts    
    endsec
 
    end
