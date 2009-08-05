
 



 
;  metrowerks sample code 





; define which clearBSS routine to assemble
; see alternative routine below


; use the software loop routine 
; if you have more than 8191 elements to zero

; use the hardware loop routine 
; if you have less than 8191 elements to zero
; 56800 loop LC register is 13-bits



; to handle the general case
; software loop is DSP56800 stationery default for __bssClear

    DEFINE  useSoftwareLoop   ''   





	section startup


; these variables are defined in the linker command file (LCF)

    XREF F_bss_size
    XREF F_bss_addr


	org	p:

	
	GLOBAL F__bssClear

;	SUBROUTINE "F__bssClear",F__bssClear,F__bssClearEND-F__bssClear


F__bssClear:
	
	
	
	
; if useSoftwareLoop is defined, assemble this code

    IF  @DEF('useSoftwareLoop')

    
    	
; clear bss with software loop	

    move    #F_bss_size,y0          ; set count    
	tstw    y0                      ; test count
    beq     end_bssClear            ; if zero count, then exit
 	
	move    #0,x0                   ; set x0 to zero
    move    #F_bss_addr,r1          ; set dest address
    nop                             
      
loop_bssClear:      
    move    x0,x:(r1)+              ; stash x0 value at address r1 
                                    ; and increment r1
    decw    y0                      ; decrement count and test
    bne     loop_bssClear           ; if not zero, continue loop      
end_bssClear:



    ELSE    ; assemble the following code




; clear bss with hardware loop	
    
    move    #0,x0                  ; set x0 to zero
    move    #F_bss_size,r2         ; set bss size
    move    #F_bss_addr,r1         ; dest address -- bss data start
    do      r2,end_bss_clear       ; do for r2 times
    move    x0,x:(r1)+             ; stash zero at address
    nop
end_bss_clear:



    ENDIF
    
	rts 

F__bssClearEND:

	endsec
	end




