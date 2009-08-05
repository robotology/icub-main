;=============================================================
;=== FILE: isrutilities.asm
;===
;=== Copyright (c)1998 Metrowerks, Inc.  All rights reserved.
;=============================================================
; Recommended tab stop = 8.

;===============================================================================
; SECTION: the floating point code
	SECTION	fp_engine
	OPT	CC
	GLOBAL	ARTU16_TO_F32ISR
	GLOBAL	ARTS16_TO_F32ISR
	GLOBAL	ARTU32_TO_F32ISR
	GLOBAL	ARTS32_TO_F32ISR

	include "Fp568d.h"
	

;===============================================================================
; FUNCTION: ARTU16_TO_F32ISR
; DESCRIPTION: ISR wrapper for ARTU16_TO_F32
; INPUT: none
; OUTPUT: none
;
ARTU16_TO_F32ISR: 
;
; The result is in A, so do not save/restore A

	lea		(SP)+
	
	move	Y0,X:(SP)+
	move	X0,X:(SP)+
	move    Y1,X:(SP)+
	move    R0,X:(SP)+
	move	B0,X:(SP)+
	move	B1,X:(SP)+
	move    B2,X:(SP)
	

    jsr     ARTU16_TO_F32

	pop		B2
	pop		B1
	pop		B0
	pop		R0
	pop		Y1
	pop		X0
	pop		Y0
    rts

;===============================================================================
; FUNCTION: ARTS16_TO_F32ISR
; DESCRIPTION: ISR wrapper for ARTS16_TO_F32
; INPUT: none
; OUTPUT: none
;
ARTS16_TO_F32ISR: 
;
; The result is in A, so do not save/restore A
;
	lea		(SP)+
	
	move	Y0,X:(SP)+
	move	X0,X:(SP)+
	move    Y1,X:(SP)+
	move    R0,X:(SP)+
	move	B0,X:(SP)+
	move	B1,X:(SP)+
	move    B2,X:(SP)

    jsr     ARTS16_TO_F32

	pop		B2
	pop		B1
	pop		B0
	pop		R0
	pop		Y1
	pop		X0
	pop		Y0
    rts

;===============================================================================
; FUNCTION: ARTU32_TO_F32ISR
; DESCRIPTION: ISR wrapper for ARTU32_TO_F32
; INPUT: none
; OUTPUT: none
;
ARTU32_TO_F32ISR: 
;
; The result is in Y0, so do not save/restore Y0
;
; The result is in A, so do not save/restore A

	lea		(SP)+
	
	move	Y0,X:(SP)+
	move	X0,X:(SP)+
	move    Y1,X:(SP)+
	move    R0,X:(SP)+
	move	B0,X:(SP)+
	move	B1,X:(SP)+
	move    B2,X:(SP)


    jsr     ARTU32_TO_F32

	pop		B2
	pop		B1
	pop		B0
	pop		R0
	pop		Y1
	pop		X0
	pop		Y0
    rts

;===============================================================================
; FUNCTION: ARTS32_TO_F32ISR
; DESCRIPTION: ISR wrapper for ARTS32_TO_F32
; INPUT: none
; OUTPUT: none
;
ARTS32_TO_F32ISR: 
;
; The result is in Y0 so do not save/restore Y0
;

	lea		(SP)+
	
	move	Y0,X:(SP)+
	move	X0,X:(SP)+
	move    Y1,X:(SP)+
	move    R0,X:(SP)+
	move	B0,X:(SP)+
	move	B1,X:(SP)+
	move    B2,X:(SP)


    jsr     ARTS32_TO_F32

	pop		B2
	pop		B1
	pop		B0
	pop		R0
	pop		Y1
	pop		X0
	pop		Y0
    rts

    endsec
 
    end
