;=============================================================
;=== FILE: fcoerce.asm
;===
;=== Copyright (c)1998 Metrowerks, Inc.  All rights reserved.
;=============================================================
; Recommended tab stop = 8.

;===============================================================================
; SECTION: the floating point code
	SECTION	fp_engine
	OPT	CC
	GLOBAL	ARTf_coerce
	
	include "Fp568d.h"


;===============================================================================
; TERMINAL: f_coerce
; DESCRIPTION: Coerce a value to float format.
; INPUT: a = bits, y1 = exponent, x:(sp-rsign) = sign.
; OUTPUT: a
;
ARTf_coerce:
	tstw	y1
	jle	f_tiny_result
	bftstl	#$0ff,a0		; C clear if any nonzero rounding bits
	bcc	f_round
f_checkoflow:
	cmp	#$0ff,y1		; huge exp?
	jge	f_oflow
f_pack:
	; a = bits, y1 = exp, x:rsign has sign bit
	move	#8,x0			; shift count
	move	a1,r0			; r0 = rrss
	move	a0,y0			; y0 = tt00
	lsrr	y0,x0,a			; a  = 00tt 0000
	move	a1,a0			; a  = 00tt 00tt
	move	a2,a1			; a  = 0000 00tt
	move	r0,y0
	lsrac	y0,x0,a			; a  = 00rr sstt
	; Clear implicit, and if it's already zero, decrement the exp.
	bfclr	#$0080,a1		; knock off lead bit
	bcs	f_pack_norm		; C set when lead bit is 0
	decw	y1			; decrement exp (to 0)
f_pack_norm:
	move	x:(sp-rsign),b		; b  = xxxx xyyyy... bits, where
					; x is the sign bit
	lsll	y1,x0,y1		; align exp left
	move	y1,b1			; b = xxxx eeee eeee 0000... bits
	asr	b			; b = xxxx xeee eeee e000... bits
	add	b,a			; add = or, when bits are exclusive
	RESTORE_OMR
	lea     (SP-SIZE_TEMPS)       ; pop temp stack space
	rts
	
;===============================================================================
; TERMINAL: f_round
; DESCRIPTION: Round an inexact value and continue with coercion.
; INPUT: a = bits normalized to 0xffffff00
;	y1 = biased, repaired exp
;	rsign.15 = result sign
; OUTPUT: rounded significand with junk low bits, Inexact set
;
f_round:
	SetInexact
f_tiny_resume:
	; Get back here with a tiny, inexact result.  Underflow and inexact
	; have been signaled in that case.
	brset	#BITDIRRND,x:FPE_state,f_round_dirrnd
	brset	#BITDOWNORCHOP,x:FPE_state,f_checkoflow
	; round to nearest...
	brclr	#$0080,a0,f_checkoflow	; done if round bit clear
	bftstl	#$007f,a0		; C clear if any set
	bcc	f_roundup
	; get here in half-way case
	brclr	#$0100,a0,f_checkoflow	; already even, so no round
f_roundup:
	move	#0,a2			; clear carry-out bits
	clr	b
	move	#$0100,b0		; increment unit
	add	b,a
	brclr	#1,a2,f_checkoflow	; look for carry-out
	asr	a			; realign
	add	#1,y1
	bra	f_checkoflow

f_round_dirrnd:
	brset	#BITDOWNORCHOP,x:FPE_state,f_round_downward
;f_round_upward:
	tstw	x:(sp-rsign)
	bge	f_roundup
	bra	f_checkoflow
f_round_downward:
	tstw	x:(sp-rsign)
	blt	f_roundup
	bra	f_checkoflow

	
;===============================================================================
; TERMINAL: f_tiny_result
; DESCRIPTION: Subnormalize a result and return to rounding...
; INPUT: a = bits normalized to 0xffffff00, with rounding bits in 0x0...0ff
;	y1 = biased, underflowed exp
;	rsign = result sign
; OUTPUT: subnormalized signif. and exp=1 to continue at f_tiny_resume if
;	inexact and at f_pack if exact.  Note, it's OK to be tiny and exact;
;	no underflow occurs.
f_tiny_result:
	; 1 - exp = number of bits to shift right in order to achieve
	; exp of 1 with unnormalized value.
	; If shift > 24, then every bit is sticky.
	; Else shift requisite amount.
	; In any case, the exponent goes to the minimum, 1.
	move	#1,x0
	sub	y1,x0			; x0 = 1 - exp = # bits to shift
	cmp	#25,x0			; 25 < x0 means minuscule
	bgt	f_very_tiny_result
	
	; Shift fr right by x0, with bits shifted off or-ed into sticky.
	move	#0,y0			; accumulate stickies into b
	move	y0,y1			; (don't need y1 for exp here)
	clr	b
	do	x0,sub_loop_end
	asr	a			; C <-- sticky bit
	adc	y,b
sub_loop_end:
	; b0 is at most 25, if all bits are one
	move	a0,y0
	move	b0,y1
	or	y0,y1
	move	y1,a0
	move	#1,y1			; set minimum exponent
	; CASES:
	;	Exact --> no underflow or inexact; just stuff subnormal
	;	Rounds up to min normal --> be sure biased exp is 1
	;	Rounds to subnormal --> inexact and underflow
	bftstl	#$00ff,a0		; sets C if exact
	jlo	f_pack
	bra	f_set_uflow

f_very_tiny_result:
	; Value underflows to 0 or tiny subnormal.  Latter arises
	; depending on rounding.
	clr	a
	move	#1,a0			; barest sticky
	move	a0,y1			; min exponent
f_set_uflow:
	SetUnderflow
	bra	f_tiny_resume		; go round and pack result


;===============================================================================
; TERMINAL: f_oflow
; DESCRIPTION: Stuff INF or largest result, according to rounding mode. Signal
;	overflow and inexact.
;
f_oflow:
	; Know result is too big. Signal, and respond according to the rounding
	; mode.
	SetOverflow
	; CASES:
	; nearest, pos & upward, neg & downward --> INF
	; chop, pos & downward, neg & upward --> HUGE
	brset	#BITDIRRND,x:FPE_state,f_oflow_dirrnd
	brset	#BITDOWNORCHOP,x:FPE_state,f_result_is_huge
f_go_inf:
	jmp	ARTf_result_is_INF


;===============================================================================
; TERMINAL: f_oflow_dirrnd
; DESCRIPTION: Check mode and sign to decide huge or INF.
;
f_oflow_dirrnd:
	brset	#BITDOWNORCHOP,x:FPE_state,f_oflow_down
	tstw	x:(sp-rsign)
	bge	f_go_inf
	bra	f_result_is_huge
f_oflow_down:
	tstw	x:(sp-rsign)
	blt	f_go_inf
	; Fall through to huge...
	

;===============================================================================
; TERMINAL: f_result_is_huge
; DESCRIPTION: Stuff huge with sign from rsign.
; Want the value 0x7f7fffff.  Get it by adding -1 to 0x7f800000.
;
f_result_is_huge:
	move	x:(sp-rsign),a
	move	#$feff,a1
	move	#$ffff,a0
	asr	a
	RESTORE_OMR
	lea     (SP-SIZE_TEMPS)       ; pop temp stack space
	rts


	
	ENDSEC