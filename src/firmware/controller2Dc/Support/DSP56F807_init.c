

 

/*  metrowerks sample code  */




#include "DSP56F807_init.h"



extern _rom_to_ram;
extern _data_size;
extern _data_RAM_addr;
extern _data_ROM_addr;
extern _bss_size;
extern _bss_addr;



asm void init_M56807_()
{
	bfset	#_32bit_compares,omr    // debugger will override this 
	                                // if debugger option is on
	move	#-1,x0
	move	x0,m01                  // set the m reg to linear addressing
				
	move	hws,la                  // clear the hardware stack
	move	hws,la


// init registers

	move	#0,r1
	move	r1,x:IPR
	move	r1,x:COPCTL



// initialize compiler environment

CALLMAIN:                       

// setup stack
	move	#_stack_addr,r0		// get stack start address
	nop
	move	r0,x:<mr15	    	// set frame pointer to main stack top	
	move	r0,sp				// set stack pointer too
	move	#0,r1
	move	r1,x:(r0)
	
	

// setup the PLL (phase locked loop)
								
	move	#pllcr_init,x:PLLCR     // set lock detector on and choose core clock
	move	#plldb_init,x:PLLDB     // set to max freq
	move    #wait_lock,x0           // set x0 with timeout value
                                    // timeout handles simulator case
pll_test_lock:                      // loop until PLL is locked
                                    // or we reach timeout limit
    decw    x0                      // decrement our timeout value
    tstw    x0                      // test for zero
    beq     pll_timeout             // if timed-out, proceed anyway
	brclr	#pllsr_init,x:PLLSR,pll_test_lock   
pll_timeout:
                                    // pll locked                                          
	move	#pllcr_proceed,x:PLLCR  // set lock detector on, choose PLL clock
	move    x:PLLSR,x0              // clear pending clkgen interrupts
	move 	x0,x:PLLSR



// setup exception handler and interrupt levels

	move	M56807_int_Addr,r1	// exception handler address
	push	r1					// establish exception handler
    bfset   #$0100,sr           // enable all levels of interrupts
	bfclr 	#$0200,sr			// allow IPL 0 interrupts

	

// utilities  
    jsr __romCopy	            // perform any ROM-to-xRAM copy 
    jsr __bssClear	            // clear the BSS



// call main()
    
	move	#M56807_argc,y0		// pass parameters to main()
	move	#M56807_argv,r2
	move	#M56807_arge,r3
	jsr	 	main				// call the users program
;	jsr  	fflush
	debug
	rts
}






