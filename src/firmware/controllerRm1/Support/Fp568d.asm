;=============================================================
;=== FILE: Fp568d.asm
;===
;=== Copyright (c)1998 Metrowerks, Inc.  All rights reserved.
;=============================================================
; Recommended tab stop = 8.

	
;===============================================================================
; GLOBAL DATA: fpe_state -- one word of state per process, indicating the
;	prevailing rouding mode and holding the sticky exception flags.
;
	SECTION	fp_state GLOBAL
	
	GLOBAL  FPE_state
	
	
	ORG	X:
FPE_state:
	DC	$0000
	ENDSEC
