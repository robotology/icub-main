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

    global  ARTDIVU32UZ

    org p:
    
ARTDIVU32UZ:
    clr     B           ; clear register for resulting quotient
;	bfset	#$0100,omr	; make sure the CC bit is set
	
	move	a0,y0
	move	a1,y1		; move A to y1:y0
	move	x:(sp-2),a
	move	x:(sp-3),a0	; load divisor into A
    bfclr   #1,sr       ; clear the carry bit

	move	#32,LC		; I don't know why I can't use immediate #32
	do		LC,endloop
	rol		y0
	rol		y1
	bcc		notthisbit
	asl		B
	bfset	#1,b0
	bra		around
notthisbit:
	asl		B
around:
	sub		A,B			; If carry is clear, sub was OK.	 	
	bcc		nottoobig	; otherwise, restore B
	add		A,B
nottoobig:
	bfchg	#1,sr		; reverse cc bit 
	nop
	nop
endloop:
	rol		y0
	rol		y1
	move	y1,A		; move result to A
	move	y0,a0

	IF REV_B
;	bfclr	#$0100,omr	; set CC setting to 36 bits
	ENDIF

	rts
    
    endsec
 
    end
