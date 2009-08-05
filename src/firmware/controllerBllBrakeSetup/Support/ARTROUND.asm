;=============================================================
;=== FILE: fp568d.s
;===
;=== Copyright (c)1998 Metrowerks, Inc.  All rights reserved.
;=============================================================
; Recommended tab stop = 8.

;===============================================================================
; SECTION: the floating point code
	SECTION	fp_engine
	OPT	CC
	GLOBAL	FARTROUND
	GLOBAL	FARTGETFPSCR
	GLOBAL	FARTSETFPSCR
	
	include "Fp568d.h"


;===============================================================================
; FUNCTION: FARTROUND, ARTSETFPSCR
; DESCRIPTION: Set the rounding mode, according to the code in x; set the fpscr.
; INPUT: y0 = 2-bit code.
; OUTPUT: y0
;
FARTROUND:
	move	x:FPE_state,y1
	bfclr	#$03,y1			; remove old mode
	bfclr	#$fffc,y0		; strip extraneous bits
	or	y1,y0
FARTSETFPSCR:
	move	y0,x:FPE_state
	rts
	
;===============================================================================
; FUNCTION: FARTGETFPSCR
; DESCRIPTION: Return the fpscr in y0.
; INPUT: none
; OUTPUT: y0 = fpscr
;
FARTGETFPSCR:
	move	x:FPE_state,y0
	rts

	ENDSEC