



; /* Metrowerks Standard Library
;  * Copyright © 1995-2002 Metrowerks Corporation.  All rights reserved.
;  *
;  * $Date: 2006/09/13 11:30:38 $
;  * $Revision: 1.1 $
; */


;----------------------------------------------------------------------
;
;       ARTDIVU16UZ.asm
;       
;
;       Routines
;       --------
;       16 bit unsigned integer divide (No REPs used)
;       INPUTS:
;         y0 dividend
;         y1 divisor
;       OUTPUT:
;         y0 <--- y0/y1    (16-bit unsigned result)
;         y1               (no changes)
;       Register Used:
;         B     assigned the value of the dividend
;
;       NOTE:
;         div - this instruction requires 36-bit correctly signed
;               positive dividend, and divisor must be 16-bit signed.
;
;               When bit 15 of divisor is ON, no div's are necessary.
;               Bit 15 of divisor must be checked since it is an
;               unsigned number even when this bit is on. An alternate
;               route is used when bit 15 is 1, (High_bit_present)
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
       global          ARTDIVU16UZ

       org             p:

 ARTDIVU16UZ:
 
;;;;;;;;;;;;;;;;; 16-bit Unsigned Divide ;;;;;;;;;;;;;;;;;;;;;
       tst.w           y1              ; Check on high bit, should be zero
       blt             High_bit_present ; If bit 15 on, result is either 1 or 0
 
       lsr16           y0,b            ; Copy dividend to b0 no sign extension

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
       
;;;;;;;;;;;;;;; Complete DIVs, copy quotient and return 
       div             y1,b            ; form quot in b0, 13th div                     
       div             y1,b            ; form quot in b0, 14th div
       rtsd                            ; delay return after nxt 3 instr
       div             y1,b            ; form quot in b0, 15th div                     
       div             y1,b            ; form quot in b0, 16th div                     
       move.w          b0,y0           ; move result value into return register
       
;;;;;;;;;;;;;;; Result will either be 1 or 0
High_bit_present:
       clr.w           b               ; clear in case reslt is zero
       cmp.w           y1,y0           ; compare divisor y1 and dividend y0
       bcs             Reslt_is_zero   ; divisor is greater than dividend             
       
Reslt_is_one:
       inc.l           b               ; b0 to 1, dividend greater than divisor
                                       ; y0 > y1
Reslt_is_zero:
       rtsd                            ; delay return after nxt 3 instr
       move.w          b0,y0           ; move result value, b0 to y0, 
       nop                             ; filler
       nop                             ; filler
       
    endsec

    end
