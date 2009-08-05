;----------------------------------------------------------------------
;  Metrowerks Embedded Runtime Support 1998 May 
;
;	rtrem32.asm
;	
; 		Copyright © 1998 Metrowerks, Inc.
; 		All rights reserved.
;
; 	Routines
; 	--------
; 
;  
;----------------------------------------------------------------------
;
 
	section		rtlib
    global  ARTREMU32Z 
    org p:
 
    include "asmdef.h"

ARTREMU32Z: 
 
    move    x:(sp-2),B     ; push divisor on stack
    move    x:(sp-3),b0
    lea     (sp)+
    move    b0,x:(sp)+
    move    b1,x:(sp)
    jsr     ARTDIVU32UZ    ; call the unsigned version
	pop	
	pop     
    tfr     B,A
    rts

    endsec
 
    end
