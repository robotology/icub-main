



; /* Metrowerks Standard Library
;  * Copyright © 1995-2002 Metrowerks Corporation.  All rights reserved.
;  *
;  * $Date: 2006/09/12 16:17:26 $
;  * $Revision: 1.1 $
; */


;----------------------------------------------------------------------
;
;       ARTDIVS16UZ.asm
;       
;
;       Routines
;       --------
;       16 bit signed integer divide (No REPs used)
;       INPUTS:
;         y0 dividend
;         y1 divisor
;       OUTPUT:
;         y0 <--- y0/y1    (16-bit signed result)
;         y1               (no changes)
;       Register Used:
;         B     assigned the value of the dividend
;
;       NOTE:
;         div - this instruction requires 36-bit correctly signed
;               positive dividend, and divisor must be 16-bit signed
;       
;
;  History:
;    2/2001     ef      conversion from V1 to V2 and optimization
;                       detection of divided by -1 is removed
;    2/2002     ef      remove divisor as 1
;                       unroll rep #8 into 8 DIV's
;                       use signed information on Y0
;                       make use on delay instruction RTSD
;                       avoid NOP debug padding by rearranging DIVs
;----------------------------------------------------------------------
;
       section         rtlib
       global          ARTDIVS16UZ

       org             p:
        
 ARTDIVS16UZ:
                  
;;;;;;;;;;;;;;;;; 16-bit Signed Divide ;;;;;;;;;;;;;;;;;;;;;
       asr16           y0,b            ; Copy dividend to b0 with sign extension
 
       abs             b               ; make dividend positive
       asl             b               ; Required for INTEGER Division 
                                       ; Required carry bit is cleard too!
       
;;;;;;;;;;;;;;;; 4-bit division ;;;;;;;;;;;;;;;; 
       div             y1,b            ; form quotient in b0
       div             y1,b
       div             y1,b
       div             y1,b
;;;;;;;;;;;;;;;; 4-bit division ;;;;;;;;;;;;;;;; 
       div             y1,b            ; form quotient in b0                     
       div             y1,b                                 
       div             y1,b
       div             y1,b                                 
;;;;;;;;;;;;;;;; 4-bit division ;;;;;;;;;;;;;;;; 
       div             y1,b            ; form quotient in b0
       div             y1,b
       div             y1,b
       div             y1,b
 
;;;;;;;;;;;;;;;; check sign of result ;;;;;;;;;;;;;;;; 
       eor.w           y1,y0           ; Result sign in N bit of SR
       bge             Quotient_pos    ; if quotient is positive, done 
       
Quotient_neg:
       div             y1,b            ; form quot in b0, 13th division
       div             y1,b            ; form quot in b0, 14th division
       div             y1,b            ; form quot in b0, 15th division
       
       rtsd                            ; return to caller after 3 instr
       div             y1,b            ; form quot in b0, 16th division                     
       neg             b               ; quot negative, change the sign 
       move.w          b0,y0           ; move return value into return register
       
Quotient_pos:
       div             y1,b            ; form quot in b0, 13th division
       div             y1,b            ; form quot in b0, 14th division
       
       rtsd                            ; return to caller after 3 instr
       div             y1,b            ; form quot in b0, 15th division                     
       div             y1,b            ; form quot in b0, 16th division                     
       move.w          b0,y0           ; move return value into return register

    endsec

    end
