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
	GLOBAL	ARTF32_TO_U16U
	GLOBAL	ARTF32_TO_S16U
	GLOBAL	ARTF32_TO_U32U
	GLOBAL	ARTF32_TO_S32U
	
	include "Fp568d.h"


;===============================================================================
; A NOTE ABOUT CONVERSIONS TO INTEGRAL FORMATS
;=============================================
;
; Conversions from float to the 16- and 32-bit signed and unsigned
; integral formats follow a number of conventions.
; 1) A negative number converts to unsigned 0, without signaling an exception.
; 2) Otherwise, any fractional part is truncated, regardless of rounding mode.
; 3) If a nonzero fraction is truncated, and the value doesn't overflow,
;	inexact is set.
; 4) If a value overflows the integer destination, invalid (not overflow) is set
;	and the result is set to the most negative value for signed destinations
;	and to the biggest value for unsigned destinations.
; Conversions from float and double can share common terminal code.
; Bits 0x018 of y1 are set to indicate:
;	0x008 -- signed destination (versus unsigned)
;	0x004 -- long destination (32-bit), versus short (16-bit)
;	0x003 -- bits from unpack routine
; To use the f_unpack mechanism, they are placed in the word as is, not needing
; to be shifted left 2 bits.  (We let the routine normalize subnorms.)
DST_SIGNED	EQU	8
DST_LONG	EQU	4
DST_UNSIGNED	EQU	0
DST_SHORT	EQU	0

;===============================================================================
; FUNCTION: ARTF32_TO_S32U
; DESCRIPTION: Convert float to long integer.
; INPUT: a = float
; OUTPUT: a = long
;
ARTF32_TO_S32U:
	move	#(DST_SIGNED+DST_LONG),x0
	bra	f_toX_common

;===============================================================================
; FUNCTION: ARTF32_TO_U32U
; DESCRIPTION: Convert float to unsigned long integer.
; INPUT: a = float
; OUTPUT: a = unsigned long long
;
ARTF32_TO_U32U:
	move	#(DST_UNSIGNED+DST_LONG),x0
	bra	f_toX_common
	
;===============================================================================
; FUNCTION: ARTF32_TO_U16U
; DESCRIPTION: Convert float to unsigned short.
; INPUT: a = float
; OUTPUT: y0 = unsigned int
;
ARTF32_TO_U16U:
	move	#(DST_UNSIGNED+DST_SHORT),x0
	bra	f_toX_common

;===============================================================================
; FUNCTION: ARTF32_TO_S16U
; DESCRIPTION: Convert float to integer.
; INPUT: a = float
; OUTPUT: y0 = int
;
ARTF32_TO_S16U:
	move	#(DST_SIGNED+DST_SHORT),x0
	; Fall through to f_toX_common
	
f_toX_common:
	lea     (SP+SIZE_TEMPS) ; reserve stack space for all temps
	NORMALIZE_OMR
	move	a1,x:(sp-rsign)		; result sign, if DST_SIGNED
	jsr	ARTf_unpackA		; return with x0 set to code
	move	x0,y0			; BUG trashes dispatch register
	jsr	dispatch_y0
	; OPTIMIZATION: save x0 for later tests.
	
f_toX_top:
	bra	go_f_tou		; unsigned int cases, #
	bra	i_result_is_zero	;	zero
	bra	u_result_from_inf	;	INF
	bra	u_result_is_max		;	NaN
	
	bra	go_f_toul		; unsigned long long cases, #
	bra	l_result_is_zero	;	zero
	bra	ul_result_from_inf 	;	INF
	bra	ul_result_is_max	;	NaN

	bra	f_toi			; signed int cases, #
	bra	i_result_is_zero	;	zero
	bra	i_result_is_max		;	INF
	bra	i_result_is_neg_max	;	NaN

	bra	go_f_tol		; signed long long cases, #
	bra	l_result_is_zero	;	zero
	bra	l_result_is_max		;	INF
	bra	l_result_is_neg_max	;	NaN
	
dispatch_y0:
	add	x:(sp-1),y0
	move	y0,x:(sp-1)
	rts

	; Compensate for branch range.
go_f_toul
	;jmp	f_toul -- same address
go_f_tol
	jmp	f_tol
go_f_tou:
	jmp	f_tou
	
;===============================================================================
; TERMINAL: u_result_from_inf
; DESCRIPTION: Unsigned result from INF, -INF-->0, +INF-->max
; INPUT: rsign
; OUTPUT: Return with y0 = 0 or max positive.
;
u_result_from_inf:
	; If it's -INF, return 0, else return max with Invalid.
	tstw	x:(sp-rsign)
	bge	u_result_is_max
	; Fall through to i_result_is_zero...

;===============================================================================
; TERMINAL: i_result_is_zero
; DESCRIPTION: Force an int zero return value.
; INPUT: 
; OUTPUT: Return with y0 = 0.
;
i_result_is_zero:
	move	#0,y0
	RESTORE_OMR
	lea     (SP-SIZE_TEMPS)       ; pop temp stack space
	rts
	
;===============================================================================
; TERMINAL: short_result_is_max, u_result_is_max, i_result_is_neg_max,
;		i_result_is_max
; DESCRIPTION: Signed or unsigned max result, with Invalid signal.
; INPUT: x0 = original unpack code = bits 0...0qrst
;	where q = DST_SIGNED, r = DST_LONG, st = x info
;	rsign = sign
; OUTPUT: Return with y0 = max positive (signed or unsigned) or max negative
;	and signal Invalid.
;
i_result_is_neg_max:
	move	#$ffff,x:(sp-rsign)			; fake neg sign
i_result_is_max:
	move	#$8000,y0			; tentative huge neg.
	tstw	x:(sp-rsign)
	blt	toX_exit_Invalid
	dec	y0				; back to $7fff
	bra	toX_exit_Invalid

; Get here by computing huge value. Distinguish signed and not.
short_result_is_max:
	brset	#DST_SIGNED,x0,i_result_is_max
	; Fall through to u_result_is_max.
u_result_is_max:
	move	#$ffff,y0
toX_exit_Invalid:
	SetInvalid
	RESTORE_OMR
	lea     (SP-SIZE_TEMPS)       ; pop temp stack space
	rts
	
;===============================================================================
; TERMINAL: l_result_is_zero
; DESCRIPTION: Force an int zero return value.
; INPUT: 
; OUTPUT: Return with a = 0.
;
l_result_is_zero:
	clr	a
	RESTORE_OMR
	lea     (SP-SIZE_TEMPS)       ; pop temp stack space
	rts

;===============================================================================
; TERMINAL: ul_result_from_inf
; DESCRIPTION: Unsigned result from INF, -INF-->0, +INF-->max
; INPUT: rsign
; OUTPUT: Return with a = 0 or max positive.
;
ul_result_from_inf:
	tstw	x:(sp-rsign)
	blt	l_result_is_zero
	bra	ul_result_is_max

;===============================================================================
; TERMINAL: l_result_is_neg_max, l_result_is_max,
;		long_result_is_max, ul_result_is_max
; DESCRIPTION: Signed or unsigned long long max result, with Invalid signal.
; INPUT: x0 = original unpack code = bits 0...0qrst
;	where q = DST_SIGNED, r = DST_LONG, st = x info
;	rsign.15 = sign
; OUTPUT: Return with a = max positive (signed or unsigned) or max
;	value.
;
l_result_is_neg_max:
	move	#$ffff,x:(sp-rsign)		; fake neg sign in msb word
l_result_is_max:
	move	#$8000,a		; a = 8000 0000
	tstw	x:(sp-rsign)
	blt	toX_exit_Invalid	; yes, it's negative
	dec	a			; a = 7fff 0000
	move	#-1,a0			; a = 7fff ffff
	bra	toX_exit_Invalid
	
; Get here by computing a large value from a number.
long_result_is_max:
	brset	#DST_SIGNED,x0,l_result_is_max
	; Fall through to...
ul_result_is_max:
	move	#-1,a
	move	a1,a0
	bra	toX_exit_Invalid


;===============================================================================
; TERMINAL: f_toi, f_tou
; DESCRIPTION: Convert unpacked float to a 16-bit integer.
; INPUT: x0 = original unpack code = bits 0...0qrst
;	where q = DST_SIGNED, r = DST_LONG, st = x info
;	rsign.15 = sign
; OUTPUT: Return with y0 set to value.
;
f_toi:
f_tou:
	; Convert an unpacked float to one of the short types.
	; Unbias the exponent and place the binary point to the right
	; of a1. That is, subtract 127+15 = 142, from the biased exp.
	sub	#142,y1
	brset	#DST_SIGNED,x0,y_6
	tstw	x:(sp-rsign)
	blt	i_result_is_zero	; neg to unsigned is 0
y_6:
	tstw	y1			; exp > 0 means oflow
	bgt	short_result_is_max
	beq	to_short_check_oflow	; tricky cases
	; Negative exponent gives number of right shifts.
	move	y1,b
	neg	b
	cmp	#16,b			; will we shift all the way
	blt	to_short_right
	
	; Shift all the way to 0, truncating fraction bits.
	move	a1,y1
	move	#0,a1
	bra	to_short_right_common
	
to_short_right:
	; Shift a1 rightward into a, collecting all round bits for
	; inexact test.
	move	b,y0			; shift count
	move	a0,y1			; low round bits
	lsrr	a1,y0,a
to_short_right_common:
	move	a0,y0			; higher round bits
	or	y0,y1
	bra	to_short_finale
	
to_short_check_oflow:
	; a1 = tentative value, with leading bit of 1.
	; a0 = rounding bits, if any
	; rsign = sign
	; x0 = result code
	; Any result with leading 0 is within range regardless.
	; Any 32-bit result is fine for unsigned destinations.
	; Otherwise, the only huge magnitude allowed to be negative
	; is 0x80000000, so test for that one tricky case.
	move	a0,y1			; round bits
	brclr	#DST_SIGNED,x0,to_short_finale	; unsigned big numbers pass
	tstw	x:(sp-rsign)			; if positive, must overflow
	jge	i_result_is_max
	; Signed & negative, so must be 0x8000...
	tfr	a,b
	add	a,b			; 32-bit result is 0 only if
					; operands were 0x80000000
	; The book doesn't say so, but add sets CC based on 32 bits.
	;;;tst	b			; 32-bit value zero means OK
	jne	i_result_is_max		; too big if any lower bits are set
	; Fall through to to_short_finale with a = 0x80000000 = result
	
to_short_finale:
	; a1 = value in range
	; y1 = rounding bits
	; rsign = sign
	; x0 = conversion codes
	brclr	#DST_SIGNED,x0,to_short_check	; unsigned
	tstw	x:(sp-rsign)
	bge	to_short_check
	move	#0,a0
	neg	a			; negate result
to_short_check:
	tstw	y1
	beq	y_8			; any round bits?
to_short_finale_inexact:
	SetInexact
y_8:
	move	a1,y0
	RESTORE_OMR
	lea     (SP-SIZE_TEMPS)       ; pop temp stack space
	rts
	


;===============================================================================
; TERMINAL: f_tol, f_toul
; DESCRIPTION: Convert unpacked float to a 32-bit integer.
; INPUT: x0 = original unpack code = bits 0...0qrst
;	where q = DST_SIGNED, r = DST_LONG, st = x info
;	rsign.15 = sign
; OUTPUT: Return with a (long) set to value.
;
f_tol:
f_toul:
	; Convert an unpacked float to one of the long types.
	; Unbias the exponent and place the binary point to the right
	; of a1. That is, subtract 127+31 = 158, from the biased exp.
	sub	#158,y1
	brset	#DST_SIGNED,x0,y_16
	tstw	x:(sp-rsign)
	jlt	l_result_is_zero	; neg to unsigned is 0
y_16:
	tstw	y1			; exp > 0 means oflow
	bgt	long_result_is_max
	beq	to_long_check_oflow	; tricky cases
	; Negative exponent gives number of right shifts.
	move	y1,b
	neg	b
	cmp	#32,b			; will we shift all the way
	blt	to_long_right
	
	; Shift all the way to 0, truncating fraction bits.
	move	a1,y1
	move	a0,y0			; higher round bits
	or	y0,y1
	clr	a
	bra	to_long_finale
	
to_long_right:
	; Shift a1 rightward into a, collecting all round bits for
	; inexact test.  Three cases arise:
	; CASE  under16: b < 16, so shift is less than a word
	;	exactly16: b == 16, so easy shift
	;	over16: b > 16 so shift into low word
	move	b,y0			; shift count
	cmp	#16,y0
	bgt	over16
	blt	under16
;exactly16:
	move	a0,y1			; round bits
	move	a1,a0
	move	#0,a1
	bra	to_long_finale
over16:
	; Save the lowest bits for rounding. Then shift the high sixteen
	; bits by (shift-16).  Finally, do the last 16-bit shift.
	sub	#16,y0			; diminish by a word
	move	a0,y1			; low round bits
	lsrr	a1,y0,a			; a has aligned bits, left one word
	move	a0,y0			; higher round bits
	or	y0,y1
	move	a1,a0			; now shift the other 16 bits
	move	#0,a1
	bra	to_long_finale
under16:
	; Trickiest case, shifting the 32-bit value by under 16 bits.
	; Use b to shift low half, then shift and accumulate.
	move	a0,y1			; low bits, for shifting
	lsrr	y1,y0,b			; b1 = low bits, b0 = rounding
	move	b0,y1			; all the round bits
	move	b1,b0			; lowest sig bits
	move	#0,b1
	lsrac	a1,y0,b
	tfr	b,a
	bra	to_long_finale
	
to_long_check_oflow:
	; a1 = tentative value, with leading bit of 1.
	; no rounding bits
	; rsign = sign
	; x0 = result code
	; Any result with leading 0 is within range regardless.
	; Any 32-bit result is fine for unsigned destinations.
	; Otherwise, the only huge magnitude allowed to be negative
	; is 0x80000000, so test for that one tricky case.
	move	#0,y1			; round bits
	brclr	#DST_SIGNED,x0,y_18	; unsigned big numbers pass
	tstw	x:(sp-rsign)			; if positive, must overflow
	jge	l_result_is_max
	; Signed & negative, so must be 0x8000...
	tfr	a,b
	add	a,b			; 32-bit result is 0 only if
					; operands were 0x80000000
	;;;;tst	b			; 32-bit value zero means OK
	jne	l_result_is_max		; too big if any lower bits are set
	; Fall through to to_long_finale with a = 0x80000000 = result
	
to_long_finale:
	; a = value in range
	; y1 = rounding bits
	; rsign = sign
	; x0 = conversion codes
	brclr	#DST_SIGNED,x0,to_long_check	; unsigned
	tstw	x:(sp-rsign)
	bge	to_long_check
	neg	a			; negate result
to_long_check:
	tstw	y1
	beq	y_18			; any round bits?
	SetInexact
y_18:	
	RESTORE_OMR
	lea     (SP-SIZE_TEMPS)       ; pop temp stack space
	rts
	
	ENDSEC