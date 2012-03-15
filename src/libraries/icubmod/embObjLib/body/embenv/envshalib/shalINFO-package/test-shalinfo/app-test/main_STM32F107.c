
/* @file       iLoader_mcbstm32c.c
    @brief      This header file implements the iLoader process for a stm32f107 onto a keil board.
                In here we dont use the STM32 in an attempt to keep it slim.
    @author     marco.accame@iit.it
    @date       04/29/2009
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "hal.h"

#include "string.h"
                     
#include "stdlib.h"

#include "shalINFO.h"
#include "shalBASE.h"

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------
// empty-section

// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of extern variables, but better using _get(), _set() 
// --------------------------------------------------------------------------------------------------------------------
// empty-section

// --------------------------------------------------------------------------------------------------------------------
// - typedef with internal scope
// --------------------------------------------------------------------------------------------------------------------
// empty-section

// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------

#define LED         0x100

#define APPLADDR    0x8010000

// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

                    

// --------------------------------------------------------------------------------------------------------------------
// - definition of main 
// --------------------------------------------------------------------------------------------------------------------

int main(void) 
{
   extern const hal_params_cfg_t *hal_params_cfgMINE;
    uint32_t size04aligned;
    uint32_t *data32aligned = NULL;
 
    eEresult_t res;
    const shalinfo_macaddr_t *mac;
    const shalinfo_macaddr_t mmac = {0, 0, 0, 0, 0, 1};

    eEmoduleInfo_t *pv = NULL;

    // try to see if we have any request in goto. if it is shalIPCprocApplication 
    // or if we dont have any, then we try to jump to 0x8010000. if we cannot we stay 
    // in here forever at a wild led blink.
    // if it is shalIPCprocLoader, then we stay in here and blink  led at 1 sec rate
    // but after 10 seconds we try to jump to 0x8010000.


    hal_memory_getsize(hal_params_cfgMINE, &size04aligned); 

    if(0 != size04aligned)
    {
        data32aligned = (uint32_t*)calloc(size04aligned/4, sizeof(uint32_t));   
    }

    hal_initialise(hal_params_cfgMINE, data32aligned);

    hal_sys_systeminit();


    if(ee_res_NOK_generic == shalbase_isvalid())
    {
        // we are in here if we dont have a valid iSharIPC.
        for(;;);
    }

    shalbase_init(1);


    pv = shalinfo_moduleinfo_get();


    res = shalinfo_isvalid();

    if(ee_res_NOK_generic == res)
    {
        // we are in here if we dont have a valid iSharIPC.
        for(;;);
    }


    // we are in here only if the iSharINFO is ok

    // we init it. always init !!!
    shalinfo_init();

    // read the mac address.

    res = shalinfo_macaddr_get(&mac);
    res = shalinfo_macaddr_set(&mmac);
    res = shalinfo_macaddr_get(&mac);





    // just for .... 
    for(;;);  

}




// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------

