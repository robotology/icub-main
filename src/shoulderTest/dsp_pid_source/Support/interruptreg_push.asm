
				SECTION rtlib
				GLOBAL INTERRUPTREGPUSH
				include "asmdef.h"
				ORG	P:
; 
; Push ALL registers onto the stack EXCEPT for the following registers:
;
;     PC           => assumed to be global
;     SR           => assumed to be global or previously saved by interrupt mechanism
;     IPR          => assumed to be global
;     SP           => assumed to be global
;     0x38 - 0x3F  => Permanent register file used by CW
;

				
INTERRUPTREGPUSH:
				lea     (SP)+
				move    N,X:(SP)+
				move    X0,X:(SP)+
				move    Y0,X:(SP)+
				move    Y1,X:(SP)+
				move    A0,X:(SP)+
				move    A1,X:(SP)+
				move    A2,X:(SP)+
				move    B0,X:(SP)+
				move    B1,X:(SP)+
				move    B2,X:(SP)+
				move    R0,X:(SP)+
				move    R1,X:(SP)+
				move    R2,X:(SP)+
				move    R3,X:(SP)+
				move    OMR,X:(SP)+
				move    LA,X:(SP)+
				move    M01,X:(SP)+
				move    LC,X:(SP)+
		;
		; save hardware stack
		;
				move    SR,X:(SP)+
				move    HWS,X:(SP)+
				move    SR,X:(SP)+
				move    HWS,X:(SP)+
	
		;
		; Save temporary register file at 0x30 - 0x37 used by compiler
		;
				move    X:<mr0,Y1
				move    Y1,X:(SP)+
				move    X:<mr1,Y1
				move    Y1,X:(SP)+
				move    X:<mr2,Y1
				move    Y1,X:(SP)+
				move    X:<mr3,Y1
				move    Y1,X:(SP)+
				move    X:<mr4,Y1
				move    Y1,X:(SP)+
				move    X:<mr5,Y1
				move    Y1,X:(SP)+
				move    X:<mr6,Y1
				move    Y1,X:(SP)+
				move    X:<mr7,Y1
				move    Y1,X:(SP)+
		; 
		; 30 registers have been pushed on the stack.
		; To return, we must simulate the original jsr
		;
				move    X:(SP-32),Y1
				move    Y1,X:(SP)+
				move    SR,X:(SP)
				move 	X:(SP-28),Y1
				rts     
				endsec
				end