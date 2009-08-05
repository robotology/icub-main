
#ifndef __flash_interfaceh__
#define __flash_interfaceh__

#include "dsp56f807.h"
#include "options.h"

/******************************************************/
// 
/******************************************************/
extern Int16 _flash_addr ;

/******************************************************/
// 
/******************************************************/

int  get_flash_addr (void);
byte writeToFlash   (word addr);
byte readFromFlash  (word addr);
byte resetFlash 	(word addr);

void flash_interface_init (byte b_type);

void set_additional_info ();
void get_additional_info ();

#ifdef DEBUG_SERIAL			
void dump_flash_memory (void);
#endif

#endif 
