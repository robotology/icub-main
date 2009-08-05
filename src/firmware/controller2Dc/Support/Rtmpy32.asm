;----------------------------------------------------------------------
;  Metrowerks Embedded Runtime Support 1998 May 
;
;	rtmpy32.asm
;	
; 		Copyright © 1998 Metrowerks, Inc.
; 		All rights reserved.
;
; 	Routines
; 	--------
; 	32 bit signed and unsigned multiply (they're the same)
;  
;----------------------------------------------------------------------
;
	section	rtlib

	global	ARTMPYS32U
	global	ARTMPYU32U
	org p:

ARTMPYS32U:
ARTMPYU32U:
    move    a1,y1   	; save first parameter (A) in y
    move    a0,y0
    move    X:(sp-3),x0
        
    move    #0,A
    tstw    x0      	; is low half of second parameter negative?
    bge     positive
    move    y0,a1   	; bit 15 of x0 is set, so need to eventually add
                        ; in y0*2^15, moving it to a1 is equivalent
                        ; to y0*2^16. The "asr" following the "macsu"
                        ; really makes this y0*2^15.
        
    lsl     x0      	; bit 15 is set, so clear it since this is the
    lsr     x0      	; signed operand in the "macsu" below.
        
positive:
    macsu   x0,y0,A
    asr     A
        
                        ; start CB * 2^16 (C is signed, B is unsigned)
                        ; C is currently in memory, B is in x0
                        ; can do this as a regular 16x16 integer multiply
                        ; since the upper 16-bits are going to be discarded
                        ; anyways!
    move    X:(sp-2),x0
    impy    y0,x0,y0
    add     y0,A    	; this adds y0 zero-extended with 16-LSBs to A,
                        ; so the 2^16 is for free!!
        
                        ; start AD * 2^16 (A is signed, D is unsigned)
                        ; A is currently in y1, D is in memory
                        ; can do this as a regular 16x16 integer multiply
                        ; since the upper 16-bits are going to be discarded
                        ; anyways!
    move    X:(Sp-3),y0 
    impy    y1,y0,y0
    add     y0,A    	; this adds x0 zero-extended with 16-LSBs to A,
                        ; so the 2^16 is for free!!
        
                        ; need to correctly sign-extend A
    move    a0,x0
    move    a1,A
    move    x0,a0
        
    rts

	endsec

	end

