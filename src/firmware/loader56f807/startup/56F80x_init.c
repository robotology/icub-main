/*   56F80x_init.c
	 initialize compiler environment
 */


#include "56F80x_init.h"


extern _xROM_to_xRAM;
extern _pROM_to_xRAM;
extern _Xbss_length;
extern _start_bss;
extern _StackAddr;
extern _Ldata_size;
extern _Ldata_ROM_addr;
extern _Ldata_RAM_addr;


asm void init_56800_()
{
	bfset	#_32bit_compares,omr    // debugger will override this
	                                // if debugger option is on
	move	#-1,x0
	move	x0,m01                  // set the m reg to linear addressing

	move	hws,la                  // clear the hardware stack
	move	hws,la

// initialize compiler environment

CALLMAIN:

// setup stack
	move	#_StackAddr,r0	 // get stack start address
	nop
	move	r0,x:<mr15	 // set frame pointer to main stack top
	move	r0,sp		 // set stack pointer too
	move	#0,r1
	move	r1,x:(r0)

// pROM-to-xRAM utility
    move    #_pROM_to_xRAM,r0    // check for the option
    tstw    r0
    beq     end_prom2xram        // if no pROM-to-xRAM, then exit
    move    #_Ldata_size,y0      // set data size
    tstw    y0                   // LNE
    beq     end_prom2xram        // LNE if no init data, then exit
    cmp     #8191,Y0
    bls     do_prom2xram
    debug			  // You have reached this because size of
			          // initialized data exceeded 2^13 size.
				  // Add code here to copy data above
                                  // 2^13 (or 8191).
do_prom2xram:
    move    #_Ldata_ROM_addr,r3   // src address -- pROM data start
    move    #_Ldata_RAM_addr,r1   // dest address -- xRAM data start
    do      y0,end_prom2xram      // copy for y0 times
    move    p:(r3)+,x0            // fetch value at address r3
    move    x0,x:(r1)+            // stash value at address r1
end_prom2xram:

// clear bss always
    move    #0,x0                   // set x0 to zero
    move    #_Xbss_length,y0        // set bss size
    tstw    y0                      // LNE
    beq     end_bss_clear           // LNE if no bss data, then exit
    cmp     #8191,Y0
    bls     do_bss_clear
    debug                           // You have reached this because size of
                                    // uninitialized data exceeded 2^13 size.
                                    // Add code here to clear bss data above
                                    // 2^13 (or 8191).
do_bss_clear:
    move    #_start_bss,r1          // dest address -- bss data start
    do      y0,end_bss_clear        // do for y0 times
    move    x0,x:(r1)+              // stash zero at address
    nop
end_bss_clear:
 

// call main()
	cmp     #1,Y1					//if y1==1 jump to mySleep
    jcc     sleepRoutine			
    jsr     main                    // else go directly to main()
sleepRoutine:
	jsr     mySleep

Finit_56800_END:
    debug
    bra    Finit_56800_END
}






