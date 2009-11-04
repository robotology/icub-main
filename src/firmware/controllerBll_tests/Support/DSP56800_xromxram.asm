

 



;  metrowerks sample code 


; this __romCopy copies xROM to xRAM



; define which __romCopy routine to assemble
; see alternative routine below


; use the software loop routine 
; if you have more than 8191 elements to zero

; use the hardware loop routine 
; if you have less than 8191 elements to zero
; 56800 loop LC register is 13-bits



; to handle the general case
; DSP56800 stationery default clearBSS is software loop 

    DEFINE  useSoftwareLoop   ''   




	section startup
	
	
; these variables are defined in the linker command file (LCF)

    XREF F_rom_to_ram
    XREF F_data_size
    XREF F_data_RAM_addr
    XREF F_data_ROM_addr
    
    

	org	p:

	
	GLOBAL F__romCopy

;	SUBROUTINE "F__romCopy",F__romCopy,F__romCopyEND-F__romCopy


F__romCopy:

	

; optional check for xROM-xRAM copy request
; DSP56800 Stationery LCF sets variable F_rom_to_ram
; comment this test out if target's LCF is always set for xROM-xRAM 

	move	#F_rom_to_ram,x0        ; optional check
	tstw    x0                      
	beq     end_romCopy             ; if no xROM-to-xRAM, then exit



	

    IF  @DEF('useSoftwareLoop')
 
	
; xROM-to-xRAM software loop	
	    
    move    #F_data_size,y0         ; set count
 	tstw    y0                      ; optional zero count test
    beq     end_romCopy             ; if zero count, then exit
    
    move    #F_data_ROM_addr,r3     ; src address  -- ROM data start
    move    #F_data_RAM_addr,r1     ; dest address -- RAM data start
    nop
         
loop_romCopy:      
    move    x:(r3)+,x0              ; fetch value at x address r3
    move    x0,x:(r1)+              ; stash value at x address r1     
    dec     y0                      ; decrement count and test
    bne     loop_romCopy            ; if not zero, continue loop      




    ELSE    




; xROM-to-xRAM hardware loop	
    
    move    #F_data_size,r2         ; set count
 	                                ; hardware loop falls through if zero
    
    move    #F_data_ROM_addr,r3     ; src address  -- ROM data start
    move    #F_data_RAM_addr,r1     ; dest address -- RAM data start
    nop
    do      r2,end_romCopy          ; copy for r2 times
    move    x:(r3)+,x0              ; fetch value at address r3
    move    x0,x:(r1)+              ; stash value at address r1
    
 
 
    
    ENDIF
   
   
    
end_romCopy:
    rts 

F__romCopyEND:

	endsec	
	end


