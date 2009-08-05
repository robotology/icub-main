;=============================================================
;=== FILE: Fp568d.h
;===
;=== Copyright (c)1998 Metrowerks, Inc.  All rights reserved.
;=============================================================
; Recommended tab stop = 8.

			XREF	FPE_state

UseRep		EQU	1
	
	
; xhi/xlo save the x operand (passed in a) in two-op functions, which
;	is handy for NaN handling, etc.
; yhi/ylo save the y operand (passed in b) in two-op functions, which
;	is handy for NaN handling, etc.
; rexp is the result exponent
; rsign is the result sign (compute as xor during unpack)
; yflip is 0 for add, 0x8000 for sub, to indicate a flip of y's sign
; omr contains value of original OMR register on entry to routine
xhi	        EQU 0
xlo	        EQU 1
yhi	        EQU 2
ylo	        EQU 3
yflip	    EQU 4
rsign	    EQU 5
rexp	    EQU 6
saved_omr   EQU 7

SIZE_TEMPS  EQU 8

;offsets for the unpack routines.
xhi_unp	    EQU	2
xlo_unp	    EQU	3
yhi_unp	    EQU	4
ylo_unp	    EQU	5
yflip_unp	EQU	6
rsign_unp	EQU	7
rexp_unp	EQU	8


INVALID		EQU	$0040
OVERFLOW	EQU	$0010
UNDERFLOW	EQU	$0008
DIVBYZERO	EQU	$0020
INEXACT		EQU	$0004

TONEAREST	EQU	0
TOWARDZERO	EQU	1
UPWARD		EQU	2
DOWNWARD	EQU	3
BITDIRRND	EQU	2
BITDOWNORCHOP		EQU	1

LFPState	MACRO
	move	x:FPE_state,x0
	ENDM
	
SetInvalid	MACRO
	bfset	#INVALID,x:FPE_state
	ENDM
	
SetInexact	MACRO
	bfset	#INEXACT,x:FPE_state
	ENDM
	
SetOverflow	MACRO
	bfset	#(OVERFLOW+INEXACT),x:FPE_state
	ENDM
	
SetUnderflow	MACRO
	bfset	#(UNDERFLOW+INEXACT),x:FPE_state
	ENDM
	
SetDivByZero	MACRO
	bfset	#DIVBYZERO,x:FPE_state
	ENDM
	
StFPState	MACRO
	move	x0,x:FPE_state
	ENDM
	
NORMALIZE_OMR MACRO
	move	OMR,x:(SP-saved_omr)   ; save original OMR state
	bfset 	#$0100,OMR             ; set CC bit
	bfclr 	#$0030,OMR             ; clear R and SA bits
	ENDM

RESTORE_OMR MACRO
	move    x:(SP-saved_omr),OMR   ; restore original OMR state
	ENDM

