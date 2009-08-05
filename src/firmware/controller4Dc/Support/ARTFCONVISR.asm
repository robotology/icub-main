;=============================================================
;=== FILE: ARTCONVISR.asm
;===
;=== Copyright (c)1998 Metrowerks, Inc.  All rights reserved.
;=============================================================
; Recommended tab stop = 8.

;===============================================================================
; SECTION: the floating point code
	SECTION	fp_engine
	OPT	CC
	GLOBAL	ARTF32_TO_U16UISR
	GLOBAL	ARTF32_TO_S16UISR
	GLOBAL	ARTF32_TO_U32UISR
	GLOBAL	ARTF32_TO_S32UISR

	include "Fp568d.h"
	

;===============================================================================
; FUNCTION: ARTF32_TO_U16UISR
; DESCRIPTION: ISR wrapper for ARTF32_TO_U16U
; INPUT: none
; OUTPUT: none
;
ARTF32_TO_U16UISR: 
;
; The result is in Y0,so do not save/restore Y0
; b,x0,r0,y0,y1	

	lea		(SP)+
	
	move	Y1,X:(SP)+
	move	A0,X:(SP)+
	move	A1,X:(SP)+
	move    A2,X:(SP)

    jsr     ARTF32_TO_U16U

	pop		A2
	pop		A1
	pop		A0
	pop		Y1
    rts

;===============================================================================
; FUNCTION: ARTF32_TO_S16UISR
; DESCRIPTION: ISR wrapper for ARTF32_TO_S16U
; INPUT: none
; OUTPUT: none
;
ARTF32_TO_S16UISR
;
; The result is in Y0, so do not save/restore Y0
; b,x0,r0,y0,y1	

	lea		(SP)+
	
	move	Y1,X:(SP)+
	move	A0,X:(SP)+
	move	A1,X:(SP)+
	move    A2,X:(SP)

    jsr     ARTF32_TO_S16U

	pop		A2
	pop		A1
	pop		A0
	pop		Y1
    rts

;===============================================================================
; FUNCTION: ARTF32_TO_U32UISR
; DESCRIPTION: ISR wrapper for ARTF32_TO_U32U
; INPUT: none
; OUTPUT: none
;
ARTF32_TO_U32UISR
;
; The result is in A, so do not save/restore A
; b,x0,r0,y0,y1	

	lea		(SP)+
	
	move	X0,X:(SP)+
	move    Y1,X:(SP)+
	move    Y0,X:(SP)+
	move	B0,X:(SP)+
	move	B1,X:(SP)+
	move    B2,X:(SP)

    jsr     ARTF32_TO_U32U

	pop		B2
	pop		B1
	pop		B0
	pop		Y0
	pop		Y1
	pop		X0
    rts


;===============================================================================
; FUNCTION: ARTF32_TO_S32UISR
; DESCRIPTION: ISR wrapper for ARTF32_TO_S32U
; INPUT: none
; OUTPUT: none
;
ARTF32_TO_S32UISR
;
; The result is in A so do not save/restore A
; b,x0,r0,y0,y1	

	lea		(SP)+
	
	move	X0,X:(SP)+
	move    Y1,X:(SP)+
	move	Y0,X:(SP)+
	move	B0,X:(SP)+
	move	B1,X:(SP)+
	move    B2,X:(SP)

    jsr     ARTF32_TO_S32U

	pop		B2
	pop		B1
	pop		B0
	pop		Y0
	pop		Y1
	pop		X0
    rts


    endsec
 
    end
