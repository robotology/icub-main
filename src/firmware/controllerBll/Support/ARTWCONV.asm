;=============================================================
;=== FILE: ARTWCONV.asm
;===
;=== Copyright (c)1998 Metrowerks, Inc.  All rights reserved.
;=============================================================
; Recommended tab stop = 8.


;===============================================================================
; SECTION: the floating point code
	SECTION	fp_engine
	OPT	CC
	
	include "Fp568d.h"
	
	GLOBAL	ARTU16_TO_F32
	GLOBAL	ARTS16_TO_F32
	GLOBAL	ARTU32_TO_F32
	GLOBAL	ARTS32_TO_F32


;===============================================================================
; FUNCTION: ARTU16_TO_F32, ARTU32_TO_F32
; DESCRIPTION: Convert unsigned word to float.
; INPUT: y0 = unsigned word
; OUTPUT: a = float
; OPTIMIZATION: Uses signed conversion, with faked positive sign.
;
ARTU16_TO_F32:
	move	y0,a
	move	#15,y1
	bra	utoX_common
ARTU32_TO_F32:
	move	#31,y1
utoX_common:
	lea     (SP+SIZE_TEMPS) ; reserve stack space for all temps
	NORMALIZE_OMR
	move	#0,y0
	move	y0,x:(sp-rsign)		; positive sign
	bra	wtoX_common

;===============================================================================
; FUNCTION: ARTS16_TO_F32, ARTS32_TO_F32
; DESCRIPTION: Convert signed word to float.
; INPUT: y0 = signed word
; OUTPUT: a = float
; CASES: Zero maps to +0. Large values are rounded according to the current mode.
;
ARTS16_TO_F32:
	move	y0,a
	move	#15,y1
	bra	itoX_common
ARTS32_TO_F32:
	move	#31,y1
itoX_common:
	lea     (SP+SIZE_TEMPS) ; reserve stack space for all temps
	NORMALIZE_OMR
	move	a1,x:(sp-rsign)
	abs	a			; ********** check extension ***********
	; fall through to wtoX_common
	
;===============================================================================
; TERMINAL: wtoX_common  (word to floating conversion, common code)
; DESCRIPTION: Normalize integer in a and coerce to destination. If dest
;	is wide (future), be sure to widen the integer operand suitably.
; INPUT: a = unsigned integer, rsign = sign, y1 = unbiased exp
; OUTPUT: Convert to float (or double) destination using coercions.
;
wtoX_common:
	; Normalize the operand in a and set the exponent to the
	; unbiased exp.  Then branch to appropriate coercion routine.
	; SPECIAL CASE: Zero is treated specially.
	add	#$7f,y1			; bias exponent
	tstw	a1			; is high half 0?
	blt	wtoX_done		; already normal
	bne	wtoX_leftward		; go shift < 16 bits
	tstw	a0
	beq	y_34_rts		; return with 0
	; Fall through with 16+ bits of shift
	move	a0,a			; shift and clear low half
	sub	#16,y1
	tstw	a1
	blt	wtoX_done		; exactly 16 bits needed
wtoX_leftward:
	sub	#1,y1
	asl	a
	tstw	a1
	bgt	wtoX_leftward
wtoX_done:
	jmp	ARTf_coerce

y_34_rts:
	RESTORE_OMR
	lea     (SP-SIZE_TEMPS)       ; pop temp stack space
	rts
	
	ENDSEC