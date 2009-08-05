
#ifndef __flash_interfaceh__
#define __flash_interfaceh__

#include "dsp56f807.h"

/******************************************************/
// 
/******************************************************/
extern Int16 _flash_addr ;

/******************************************************/
// 
/******************************************************/

int  get_flash_addr (void);
byte writeToFlash (word addr, byte board_ID);
byte readFromFlash (word addr, byte* board_ID);
void flash_interface_init (byte b_type);

#endif 
