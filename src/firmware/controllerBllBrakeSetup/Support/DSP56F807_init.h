

 

/*  metrowerks sample code  */


#include "dsp56f807.h"
#include "DSP56F807_pll.h"
#include "DSP56F807_peripheral_regs.h"





#define _32bit_compares 0x0100  // for OMR -- non-development mode
#define M56807_argc 0           // main arg
#define mr15 $3F                // frame pointer



// defined in 56807_vector.asm
extern char *M56807_int_Addr;   // exception handler address
extern int  *M56807_argv;       // main arg
extern int  *M56807_arge;       // main arg


// defined in user code
extern main();


// defined in runtime lib
extern fflush();


// defined in linker command files
extern char *_stack_addr;



// defined in rom copy utility
void __romCopy();


// defined in bss clear utility
void __bssClear();





//prototype
void init_M56807_();

