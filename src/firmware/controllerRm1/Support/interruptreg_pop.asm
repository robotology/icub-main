
				section	rtlib
				GLOBAL INTERRUPTREGPOP
				include "asmdef.h"
				ORG	P:
INTERRUPTREGPOP:
; 
; Pop ALL registers from the stack in reverse   
; order from the routine archPushAllRegisters
;
				
		; 
		; To return, we must simulate the original 
		; jsr after popping all registers saved
		;
		; We use the stack space
		; from the call to archPushAllRegisters
		; to store the PC/SR for the RTS instruction 
		;
				pop     Y1
				move    Y1,X:(SP-31)
				pop     Y1
				move    Y1,X:(SP-31)
				pop     Y1
				move    Y1,X:<mr7
				pop     Y1
				move    Y1,X:<mr6
				pop     Y1
				move    Y1,X:<mr5
				pop     Y1
				move    Y1,X:<mr4
				pop     Y1
				move    Y1,X:<mr3
				pop     Y1
				move    Y1,X:<mr2
				pop     Y1
				move    Y1,X:<mr1
				move    X:(SP),Y1    ; leave SP+1 for hardware stack restoration
				move    Y1,X:<mr0

		;
		; restore hardware stack
		;
				move    HWS,LA
				move    HWS,LA
				brclr   #-32768,X:(SP-2),SkipHWS1				
				move    X:(SP-1),HWS
SkipHWS1:
				brclr   #-32768,X:(SP-4),SkipHWS2
				move    X:(SP-3),HWS
SkipHWS2:
				lea     (SP-5)
		;
		; restore all saved registers
		;
				pop     LC
				pop     M01
				pop     LA
				pop     OMR
				pop     R3
				pop     R2
				pop     R1
				pop     R0
				pop     B2
				pop     B1
				pop     B0
				pop     A2
				pop     A1
				pop     A0
				pop     Y1
				pop     Y0
				pop     X0
				pop     N
				rts     

				ENDSEC
				END
