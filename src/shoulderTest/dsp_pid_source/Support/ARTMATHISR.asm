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
	GLOBAL	ARTADDF32UISR
	GLOBAL  ARTSUBF32UISR
	GLOBAL	ARTCMPF32ISR
	GLOBAL	ARTCMPEF32ISR
	GLOBAL  ARTMPYF32UISR
	GLOBAL	ARTDIVF32UZISR

	include "Fp568d.h"
	

;===============================================================================
; FUNCTION: ARTADDF32UISR
; DESCRIPTION: ISR wrapper for ARTADDF32U
; INPUT: none
; OUTPUT: none
;
ARTADDF32UISR: 
;
; The result is in A, so do not save/restore A
; b,x0,r0,y0,y1	

	lea		(SP)+
	
	move	Y0,X:(SP)+
	move	X0,X:(SP)+
	move	R0,X:(SP)+
	move    Y1,X:(SP)+
	move	B0,X:(SP)+
	move	B1,X:(SP)+
	move    B2,X:(SP)

    jsr     ARTADDF32U

	pop		B2
	pop		B1
	pop		B0
	pop		Y1
	pop		R0
	pop		X0
	pop		Y0
    rts

;===============================================================================
; FUNCTION: ARTSUBF32UISR
; DESCRIPTION: ISR wrapper for ARTSUBF32U
; INPUT: none
; OUTPUT: none
;
ARTSUBF32UISR: 
;
; The result is in A, so do not save/restore A
; b,x0,r0,y0,y1	

	lea		(SP)+
	
	move	Y0,X:(SP)+
	move	X0,X:(SP)+
	move	R0,X:(SP)+
	move    Y1,X:(SP)+
	move	B0,X:(SP)+
	move	B1,X:(SP)+
	move    B2,X:(SP)

    jsr     ARTSUBF32U

	pop		B2
	pop		B1
	pop		B0
	pop		Y1
	pop	 	R0
	pop		X0
	pop		Y0
    rts

;===============================================================================
; FUNCTION: ARTCMPF32ISR
; DESCRIPTION: ISR wrapper for ARTCMPF32
; INPUT: none
; OUTPUT: none
;
ARTCMPF32ISR: 
;
; The result is in CC field, it is okay to save/restore Y0
; b,x0,r0,y0,y1	

	lea		(SP)+
	
	move	X0,X:(SP)+
	move    Y1,X:(SP)+
	move	Y0,X:(SP)+
	move	A0,X:(SP)+
	move	A1,X:(SP)+
	move    A2,X:(SP)+
	move	B0,X:(SP)+
	move	B1,X:(SP)+
	move    B2,X:(SP)

    jsr     ARTCMPF32

	pop		B2
	pop		B1
	pop		B0
	pop		A2
	pop		A1
	pop		A0
	pop		Y0
	pop		Y1
	pop		X0
    rts

;===============================================================================
; FUNCTION: ARTCMPEF32ISR
; DESCRIPTION: ISR wrapper for ARTCMPEF32
; INPUT: none
; OUTPUT: none
;
ARTCMPEF32ISR: 
;
; The result is in CC field, it is okay to save/restore Y0

	lea		(SP)+
	
	move	X0,X:(SP)+
	move    Y1,X:(SP)+
	move	Y0,X:(SP)+
	move	A0,X:(SP)+
	move	A1,X:(SP)+
	move    A2,X:(SP)+
	move	B0,X:(SP)+
	move	B1,X:(SP)+
	move    B2,X:(SP)

    jsr     ARTCMPEF32

	pop		B2
	pop		B1
	pop		B0
	pop		A2
	pop		A1
	pop		A0
	pop		Y0
	pop		Y1
	pop		X0
    rts


;===============================================================================
; FUNCTION: ARTMPYF32UISR
; DESCRIPTION: ISR wrapper for ARTMPYF32UISR
; INPUT: none
; OUTPUT: none
;
ARTMPYF32UISR: 
;
; The result is in A so do not save/restore A
;
	lea		(SP)+
	
	move	X0,X:(SP)+
	move    Y1,X:(SP)+
	move	Y0,X:(SP)+
	move	R0,X:(SP)+
	move    R1,X:(SP)+
	move	B0,X:(SP)+
	move	B1,X:(SP)+
	move    B2,X:(SP)

    jsr     ARTMPYF32U

	pop		B2
	pop		B1
	pop		B0
	pop		R1
	pop		R0
	pop		Y0
	pop		Y1
	pop		X0
    rts


;===============================================================================
; FUNCTION: ARTDIVF32UZISR
; DESCRIPTION: ISR wrapper for ARTDIVF32UZ
; INPUT: none
; OUTPUT: none
;
ARTDIVF32UZISR: 
;
; The result is in A so do not save/restore A
; b,x0,r0,y0,y1	

	lea		(SP)+
	
	move	X0,X:(SP)+
	move    Y1,X:(SP)+
	move	Y0,X:(SP)+
	move	R0,X:(SP)+
	move	B0,X:(SP)+
	move	B1,X:(SP)+
	move    B2,X:(SP)

    jsr     ARTDIVF32UZ

	pop		B2
	pop		B1
	pop		B0
	pop		R0
	pop		Y0
	pop		Y1
	pop		X0
    rts


    endsec
 
    end
