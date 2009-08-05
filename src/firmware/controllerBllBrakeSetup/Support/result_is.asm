;=============================================================
;=== FILE: result_is.asm
;===
;=== Copyright (c)1998 Metrowerks, Inc.  All rights reserved.
;=============================================================
; Recommended tab stop = 8.

;===============================================================================
; SECTION: the floating point code
	SECTION	fp_engine
	OPT	CC
	GLOBAL  ARTf_result_is_zero
	GLOBAL  ARTf_result_is_INF
	GLOBAL  ARTf_y_is_NaN
	GLOBAL  ARTf_result_is_Invalid
	GLOBAL  ARTf_x_and_y_are_NaN
	GLOBAL  ARTf_x_is_NaN
	
	include "Fp568d.h"





;===============================================================================
; TERMINAL: f_result_is_zero
; DESCRIPTION: a gets 0 with sign from rsign
; OUTPUT: a
;
ARTf_result_is_zero:
	move	x:(sp-rsign),a
	bfclr	#$7fff,a1
	RESTORE_OMR
	lea     (SP-SIZE_TEMPS)       ; pop temp stack space
	rts


;===============================================================================
; TERMINAL: f_result_is_INF
; DESCRIPTION: a gets INF with sign from rsign
; OUTPUT: a
;
ARTf_result_is_INF:
	move	x:(sp-rsign),a
	move	#$ff00,a1
	asr	a
	RESTORE_OMR
	lea     (SP-SIZE_TEMPS)       ; pop temp stack space
	rts
	

;===============================================================================
; TERMINAL: f_y_is_NaN
; DESCRIPTION: Know y is a NaN and other operand isn't.  If y is signaling,
;	quiet it and signal invalid.  Else just return it.
; INPUT: y in memory
; OPTIMIZATION: Use shared code.
;
ARTf_y_is_NaN:
	move	x:(sp-yhi),a
	move	x:(sp-ylo),a0
	bra	f_NaNs_common


;===============================================================================
; TERMINAL: f_result_is_Invalid
; DESCRIPTION: a gets +NaN
; OUTPUT: a
;
ARTf_result_is_Invalid:
	move	x:(sp-rsign),a
	move	#$ff80,a1
	asr	a
f_inv_common:
	SetInvalid
	RESTORE_OMR
	lea     (SP-SIZE_TEMPS)       ; pop temp stack space
	rts
	

;===============================================================================
; TERMINAL: f_x_and_y_are_NaN
; DESCRIPTION: Both are NaNs.  If x is signaling, go signal.  Ditto for y.
;	Else return x with no signal.
; INPUT: x bits in b, y bits in a.
; TRICK: Use fact that NaN bits are delivered without implicit bit,
;	with the quiet bit in bit #31.  Q=1, S=0, so try for the
;	signal.
;
ARTf_x_and_y_are_NaN:
	tstw	b1			; check x first
	bge	ARTf_x_is_NaN		; it's signaling
	tstw	a1
	bge	ARTf_y_is_NaN		; use y if it's signaling
	; ...else fall through to f_x_is_NaN
	

;===============================================================================
; TERMINAL: f_x_is_NaN
; DESCRIPTION: Know x is a NaN and other operand isn't.  If x is signaling,
;	return canonical NaN and signal invalid.  Else just return it.
; INPUT: x saved in low mem registers.
;
ARTf_x_is_NaN:
	; x has bits seee eeee eSff ffff ffff ffff ffff ffff,
	; where S is the signaling bit.  If bit is clear, set and signal.
	move	x:(sp-xhi),a
	move	x:(sp-xlo),a0
f_NaNs_common:
	brclr	#$0040,a1,f_inv_common	; quiet bit clear? if yes, signaling
	RESTORE_OMR
	lea     (SP-SIZE_TEMPS)       ; pop temp stack space
	rts
	



	
	ENDSEC