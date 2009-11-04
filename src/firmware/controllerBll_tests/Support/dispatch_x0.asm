;=============================================================
;=== FILE: dispatch_x0.asm
;===
;=== Copyright (c)1998 Metrowerks, Inc.  All rights reserved.
;=============================================================
; Recommended tab stop = 8.

;===============================================================================
; SECTION: the floating point code
	SECTION	fp_engine
	OPT	CC
	GLOBAL	ARTdispatch_x0
	

; used in floating point library jump tables
	
ARTdispatch_x0:
	; TRICK: Avoid mysterious "add x0,x:(sp-1)", which
	; seems to malfunction.  It trashes x0 anyway (!)
	; and fools the debugger when stepping.
	add		x:(sp-1),x0
	move	x0,x:(sp-1)
	rts

	
	ENDSEC