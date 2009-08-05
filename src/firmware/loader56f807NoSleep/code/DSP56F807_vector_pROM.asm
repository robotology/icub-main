



; metrowerks sample code



	section rtlib
	org	p:
	
M56807_intRoutine:
	debug
	nop
	rti

;M56807_intDef:
;	debug
;	nop
;	rti
   endsec
    
	section interrupt_vectorsboot
	org	p:
	
	jmp F_EntryPoint     ; RESET                    ($00)
	jmp F_EntryPoint    	; COP Watchdog reset       ($02)
    endsec
	
	
	section interrupt_vectors
	org	p:
	jmp F_EntryPoint     ; RESET                    ($00)
	jmp F_EntryPoint    	; COP Watchdog reset       ($02)
	jmp M56807_intRoutine    ; reserved                 ($04)
	jmp M56807_intRoutine    ; illegal instruction      ($06)
	jmp M56807_intRoutine    ; Software interrupt       ($08)
	jmp M56807_intRoutine    ; hardware stack overflow  ($0A)
	jmp M56807_intRoutine    ; OnCE Trap		  		($0C)
	jmp M56807_intRoutine    ; reserved                 ($0E)
	jsr M56807_intRoutine	 ; external interrupt A     ($10)
	jmp M56807_intRoutine    ; external interrupt B     ($12)
	jmp M56807_intRoutine    ; reserved		            ($14)
	jmp M56807_intRoutine    ; boot flash interface     ($16)
	jmp M56807_intRoutine    ; program flash interface  ($18)
	jsr M56807_intRoutine     ; data flash interface     ($1A)
	jsr M56807_intRoutine    ; mscan transmitter ready  ($1C)
	jsr M56807_intRoutine    ; mscan receiver full      ($1E)
	jsr M56807_intRoutine  ; mscan error              ($20)
	jsr M56807_intRoutine ; mscan wakeup             ($22)
	jmp M56807_intRoutine    ; program flash interface 2($24)
	jmp M56807_intRoutine    ; GPIO E                   ($26)
	jmp M56807_intRoutine    ; GPIO D                   ($28)
	jmp M56807_intRoutine    ; reserved                 ($2A)
	jmp M56807_intRoutine    ; GPIO B                   ($2C)
	jmp M56807_intRoutine    ; GPIO A                   ($2E)
	jmp M56807_intRoutine    ; SPI transmitted empty    ($30)
	jmp M56807_intRoutine    ; SPI receiver full/error  ($32)
	jmp M56807_intRoutine    ; Quad decoder #1 home sw  ($34)
	jmp M56807_intRoutine    ; Quad decoder #1 idx pulse($36)
	jmp M56807_intRoutine    ; Quad decoder #0 home sw  ($38)
	jmp M56807_intRoutine    ; Quad decoder #0 idx pulse($3A)
	jmp M56807_intRoutine    ; Timer D Channel 0        ($3C)
	jmp M56807_intRoutine    ; Timer D Channel 1        ($3E)
	jmp M56807_intRoutine    ; Timer D Channel 2        ($40)
	jmp M56807_intRoutine    ; Timer D Channel 3        ($42)
	jmp M56807_intRoutine    ; Timer C Channel 0        ($44)
	jmp M56807_intRoutine    ; Timer C Channel 1        ($46)
	jmp M56807_intRoutine    ; Timer C Channel 2        ($48)
	jmp M56807_intRoutine    ; Timer C Channel 3        ($4a)
	jmp M56807_intRoutine    ; Timer B Channel 0        ($4c)
	jmp M56807_intRoutine    ; Timer B Channel 1        ($4e)
	jmp M56807_intRoutine    ; Timer B Channel 2        ($50)
	jmp M56807_intRoutine    ; Timer B Channel 3        ($52)
	jsr M56807_intRoutine    	 ; Timer A Channel 0        ($54)
	jmp M56807_intRoutine    ; Timer A Channel 1        ($56)
	jmp M56807_intRoutine    ; Timer A Channel 2        ($58)
	jmp M56807_intRoutine    ; Timer A Channel 3        ($5a)
	jmp M56807_intRoutine    ; SCI #1 Transmit complete ($5c)
	jmp M56807_intRoutine    ; SCI #1 transmitter ready ($5e)
	jmp M56807_intRoutine    ; SCI #1 receiver error    ($60)
	jmp M56807_intRoutine    ; SCI #1 receiver full     ($62)
	jmp M56807_intRoutine    ; SCI #0 Transmit complete ($64)
	jsr M56807_intRoutine     ; SCI #0 transmitter ready ($66)
	jsr M56807_intRoutine  ; SCI #0 receiver error    ($68)
	jsr M56807_intRoutine     ; SCI #0 receiver full     ($6a)
	jmp M56807_intRoutine    ; ADC B Conversion complete($6c)
	jmp M56807_intRoutine    ; ADC A Conversion complete($6e)
	jmp M56807_intRoutine    ; ADC B zero crossing/error($70)
	jmp M56807_intRoutine    ; ADC A zero crossing/error($72)
	jmp M56807_intRoutine    ; Reload PWM B 		    ($74)
	jmp M56807_intRoutine    ; Reload PWM A 		    ($76)
	jmp M56807_intRoutine    ; PWM B Fault	 		    ($78)
	jmp M56807_intRoutine    ; PWM A Fault	 		    ($7a)
	jmp M56807_intRoutine    ; PLL loss of lock		    ($7c)
	jmp M56807_intRoutine    ; low voltage detector	    ($7e)

	endsec	


	end






