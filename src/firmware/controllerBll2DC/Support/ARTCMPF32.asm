;=============================================================
;=== FILE: ARTCMPF32.asm
;===
;=== Copyright (c)1998 Metrowerks, Inc.  All rights reserved.
;=============================================================
; Recommended tab stop = 8.

;===============================================================================
; SECTION: the floating point code
	SECTION	fp_engine
	OPT	    CC
	GLOBAL	ARTCMPF32
	GLOBAL	ARTCMPEF32
	
	include "Fp568d.h"


;===============================================================================
; FUNCTION: FARTCMPF32, FARTCMPEF32
; DESCRIPTION: Comparison functions, all described here because of commonality.
;
; INPUT: x in a, y at sp-[2, 3]
; OUTPUT: y0 = result
;	FARTCMPF32  --> 0-eq, 1-lt, 2-gt, 3-un (invalid if an SNaN)
;	FARTCMPEF32 --> 0-eq, 1-lt, 2-gt, 3-un (invalid if unordered)
;	In both cases, the condition codes are set as follows
;
;    | <  =  >  ?  <-- IEEE relation discovered, '?' means 'unordered'
;  --+------------
;  C | 1  0  0  0  <-- setting of condition codes on exit from function
;  Z | 0  1  0  0
;  N | 1  0  0  0
;  V | 0  0  0  1
;
; Here's how to use the CCs to implement C-style branches:
;
;         |  neg.  |  "fp"  |                          |
;  rel_op | rel_op | branch | DSP568 branch (with CC)  | op
; --------+--------+-----------------------------------+------------
;    !=   |   ==   |  fbeq  |  beq  (Z = 1)            | FARTCMPF32
;    ==   |   !=   |  fbne  |  bne  (Z = 0)            | FARTCMPF32
;    <    |  ">=?" |  fbgeu |  bcc  (C = 0)            | FARTCMPEF32
;    <=   |  ">?"  |  fbgu  |  bgt  (Z + (V + N) = 0)  | FARTCMPEF32
;    >    |  "<=?" |  fbleu |  ble  (Z + (V + N) = 1)  | FARTCMPEF32
;    >=   |  "<?"  |  fblu  |  blt  ((V + N) = 1)      | ARTCMPEF32
;
;Note that equal/not-equal comparisons never raise invalid on unordered,
;but the other four C relationals do raise invalid on unordered.  Here
;is how to simply branch on the six C relational operators:
;    
;         |  "fp"  |
;  rel_op | branch | DSP568 branch (with CC)
; --------+-----------------------------------
;    ==   |  fbeq  |  beq  (Z = 1)
;    !=   |  fbne  |  bne  (Z = 0)
;    <    |  fblt  |  blo  (C = 1)
;    <=   |  fble  |  bls  (C + Z = 1)
;    >    |  fbgt  |  bgt  (Z + (N xor V) = 0)
;    >=   |  fbge  |  bge  ((N xor V) = 0)
;
;Finally, here is the table of the other 8 relationals, excluding the
;trivial cases branch-on-true and branch-on-false.  Four of these
;cases arose in the flipped conditionals above.  The statement
;"bxx skip" means that the condition is not true, and that the
;second conditional branch should be skipped.  A pair of
;branches like "bcs ; beq" has the target of the "fp" branch
;as the target of both branches.
;
;         |  "fp"  |
;  rel_op | branch | DSP568 branch (with CC)
; --------+-----------------------------------
;  ">=?"  |  fbgeu |  bcc  (C = 0)
;  ">?"   |  fbgu  |  bgt  (Z + (V + N) = 0)
;  "<=?"  |  fbleu |  ble  (Z + (V + N) = 1)
;  "<?"   |  fblu  |  blt  ((V + N) = 1)
;  "?"    |  fbu   |  bgt skip; bcc
;  "<=>"  |  fbleg |  bcs ; beq
;  "=?"   |  fbeu  |  bcs skip; ble
;  "<>"   |  fbgl  |  bcs ; bgt
;
;What this shows is that with careful use of the integer condition
;codes on a machine supporting signed and unsigned arithmetic, it's
;possible to satisfy the 4-way IEEE floating point branches with zero
;extra overhead.
;
; INCIDENTALLY: un --> unordered; NaNs are unordered with EVERYTHING,
; or --> ordered; all pairs of numbers, finite or infinite, are ordered
; as lt, eq, or gt.
;
; IMPLEMENTATION: This routine descends from a line of float/double
; routines that include boolean functions for testing ==, !=, etc.
; individually.
; The code uses a dispatch table, based on the sum
;	<operation code> + <relation>
; In this case there are just two codes, for the vanilla and the
; exceptional comparison.  Because DSP568 is a word-oriented machine,
; the <relation> is encoded in bis 0x0003, and the <operation code>
; in bit 0x004 -- all in y1.
; Historically, one grand dispatch has 
; taken care of myriad cases, including setting of Invalid on unordered.
; The comparison codes are 0-eq, 1-lt, 2-gt, 3-un, as with the result.
; The idea is to compare the values as integers, converting the sign-magnitude
; representation to 2's-complement.  To catch unordered cases, watch compare
; against -INF and +INF as 2's-complement integers.  Also, look for 
; signaling NaNs to force a signal, regardless of the type of comparison.
;
; BUG WORKAROUND: 32-bit cmp or accumulators doesn't work, so use a pair
; of comparisons: signed for high parts and, if those are equal, UNSIGNED
; for low parts.  This works because of the nature of 2's-complement
; representation.
;
OP_CMP	EQU $000	; was $040
OP_CMPE	EQU $004	; was $048

ARTCMPEF32:
	move	#OP_CMPE,y1
	bra	f_compare
ARTCMPF32:
	move	#OP_CMP,y1
	;bra	f_compare
	; Fall through to f_compare
	
f_compare:
	; y1 = operation code
	; a = x operand
	; (sp-2) = y operand
	; Change signed-magnitude representation to 2's-complement to get
	; basic comparison.  Then watch for NaNs.
	move	x:(sp-2),b		; high 16 bits and sign extension
	move	x:(sp-3),b0		; low 16 bits
	lea     (SP+SIZE_TEMPS) ; reserve stack space for all temps
	NORMALIZE_OMR
	tstw	a1			; is sign bit set?
	bge	y_12			; ge ==> already nonnegative
	asl	a
	move	#0,a2			; clear sign bits
	asr	a			; realign
	neg	a
y_12:
	tstw	b1
	bge	y_13
	asl	b
	move	#0,b2
	asr	b
	neg	b
y_13:
	; Here's the big comparison x vs y.  After the comparison, check
	; the lower against -NaN and the biggest against NaN.  Be sure
	; to watch for BOTH being less than -NaN...
	; Compare in two halves, signed high and unsigned low.
	move	a1,x0
	cmp	b1,x0		; check high parts
	blt	f_bounds_less
	bgt	f_bounds_greater
	move	a0,x0
	move	b0,y0
	cmp	y0,x0		; b vs. a, low parts
	bcs	f_bounds_less	; carry set means "lower"
	beq	f_bounds_check	; they're equal
	; Fall through to f_bounds_greater.
f_bounds_greater:
	add	#1,y1
f_bounds_less:
	add	#1,y1			; code = 4 if greater than	
f_bounds_check:
	; y1 = code.
	; Now check whether one or the other is a NaN by first restoring
	; to positive form and then checking against QNaN and SNaN.
	; Quiet NaNs look bigger than 7fc00000, and signaling NaNs
	; lie between 7f800000 (inf) and 7fc00000, exclusive.
	; TRICK: logically OR the unordered code into result, which is
	; now based on initial compare.
	; TRICK: Lead frac bit marks quiet NaN.  If a value has
	; magnitude >= 0x7fc00000 then it's quiet.
	move	#$7f80,x0		; true +INF
	
	tst	a			; is the a operand negative
	bge	y_133
	neg	a
y_133:
	cmp	a1,x0
	bgt	check_fy		; a-b < 0 means a is at most huge
	blt	f_cmp_x_NaN
	tstw	a0			; a0 0 if INF
	beq	check_fy
f_cmp_x_NaN:
	move	#$7fc0,x0		; make smallest QNaN
	cmp	a1,x0			; a - QNaN
	bgt	f_compare_un_inv	; a-b < 0 means a is an SNaN, so done
	bfset	#3,y1			; mark the result unordered
	; Fall through to check_fy in case it's signaling.
check_fy:
	move	#$7f80,x0
	tst	b
	bge	y_135
	neg	b
y_135:
	cmp	b1,x0
	bgt	compare_dispatch	; a-b < 0 means a at most huge
	blt	f_cmp_y_NaN
	tstw	b0			; 0 if INF
	beq	compare_dispatch
f_cmp_y_NaN:
	move	#$7fc0,x0
	cmp	b1,x0			; a - QNaN
	ble	f_compare_un		; a-b >= 0 means a is QNaN
	; Fall through to f_compare_un_inv.
	
f_compare_un_inv:
	SetInvalid
f_compare_un:
	bfset	#3,y1			; mark the result unordered
	; Fall through to compare_dispatch	


;===============================================================================
; TERMINAL: compare_dispatch
; DESCRIPTION: Return result from one of many comparison routines.
; INPUT: y1 = dispatch code = (opcode * 8) + (relation * 2), where the codes
;	are defined at the start of FPE_cmp.
; OUTPUT: result in y0
;
compare_dispatch:
	jsr	cmp_do_dispatch
	
cmp_dispatch_top:
	bra	cmp_equal 	; fcmp (ARTCMPF32)
	bra	cmp_less 
	bra	cmp_greater
	bra	cmp_unordered 

	bra	cmp_equal	; fcmpe (ARTFCMPEF32)
	bra	cmp_less 
	bra	cmp_greater
	bra	cmp_unordered_invalid 

cmp_do_dispatch:
	add	x:(sp-1),y1		; bump table address
	move	y1,x:(sp-1)
	rts				; ...into branch
	

;===============================================================================
; TERMINAL: cmp_false_invalid, cmp_false, cmp_true, cmp_equal, cmp_less
;	cmp_false_invalid
; DESCRIPTION: Deliver the result of a comparison in y0.
;	cmp_false -- set y0 to 0
;	cmp_true -- leave y0 as 1
;	cmp_equal -- set y0 to 0.
; INPUT: y0 = 0
; OUTPUT: y0 = result, with CC set
;
cmp_false_invalid:
	SetInvalid
cmp_equal:
cmp_false:
	clr	y0
	tstw	y0		; set Z in CC
	RESTORE_OMR
	lea     (SP-SIZE_TEMPS)       ; pop temp stack space
	rts
cmp_less:
cmp_true:
cmp_done:
	move	#1,y0
	cmp	#<2,y0		; set C and N
	RESTORE_OMR
	lea     (SP-SIZE_TEMPS)       ; pop temp stack space
	rts


;===============================================================================
; TERMINAL: cmp_greater
; DESCRIPTION: Deliver the result greater than.
; OUTPUT: y0 = 2
;
cmp_greater:
	move	#2,y0
	cmp	#<1,y0		; clear all CC bits
	RESTORE_OMR
	lea     (SP-SIZE_TEMPS)       ; pop temp stack space
	rts


;===============================================================================
; TERMINAL: cmp_unordered, cmp_unordered_invalid
; DESCRIPTION: Deliver the result unordered.
; OUTPUT: y0 = 3 with V set and all other bits clear
;
cmp_unordered_invalid:
	SetInvalid
cmp_unordered:
	move	#3,y0
	cmp	#<1,y0		; clear all CC bits
	bfset	#2,sr		; set V bit
	RESTORE_OMR
	lea     (SP-SIZE_TEMPS)       ; pop temp stack space
	rts

	
	

	ENDSEC