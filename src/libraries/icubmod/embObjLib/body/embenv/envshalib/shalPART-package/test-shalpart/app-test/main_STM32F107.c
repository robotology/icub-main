
/* @file       iLoader_mcbstm32c.c
    @brief      This header file implements the iLoader process for a stm32f107 onto a keil board.
                In here we dont use the STM32 in an attempt to keep it slim.
    @author     marco.accame@iit.it
    @date       04/29/2009
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------


#include "string.h"

#include "hal.h"

#include "stdlib.h"

#include "shalBASE.h"
#include "shalPART.h"

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

//union uParameter {
//  uint8_t     bByte;   //  8 bit unsigned
//  uint16_t    iInt16;  // 16 bit signed
//  uint32_t    iInt32;  // 32 bit signed
//};
//typedef union uParameter USR_tu_Para;

// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------

#define LED         0x100

#define APPLADDR    0x8010000

// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------


static void manage_error(void);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static volatile uint32_t msTicks;  

//inline uint32_t asasa(uint32_t a)
//{
//    return(a=2);
//}                          

// --------------------------------------------------------------------------------------------------------------------
// - definition of main 
// --------------------------------------------------------------------------------------------------------------------

#include "stdbool.h"

int main(void) 
{
    extern const hal_params_cfg_t *hal_params_cfgMINE;
    uint32_t size04aligned;
    uint32_t *data32aligned = NULL;
    uint32_t cnt = 0;


    bool asd = true;
 
    static eEresult_t res;

    static eEmoduleInfo_t *pv = NULL;

    static eEprocess_t proc;
    static eEprocess_t pp[] = {ee_procNone, ee_procNone, ee_procNone, ee_procNone, ee_procNone, 
                        ee_procNone, ee_procNone, ee_procNone, ee_procNone, ee_procNone};
    static const eEprocess_t *proctable = pp;
    static uint8_t size = 0;
    static const eEstorage_t strg = { .type = ee_strg_eeprom, .size = 32, .addr = 1024  };
    static const uint8_t cdata[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    static volatile uint8_t vdata[16] = {0};

    static eEmoduleInfo_t procinfo = 
    {
        .type               = ee_process,
        .signature          = ee_procApplication,
        .version            = {2, 2},
        .builddate          = {2010, 3, 8, 16, 57},
        .name               = "myappl.bin",
        .rom                = 
        {
            .addr           = 0x08000000,
            .size           = 1024*4
        },
        .ram                = 
        {
            .addr           = 0x20000000,
            .size           = 1024*4
        },
        .storage            = {ee_strg_none, 0, 0}
    };

    static const eEmoduleInfo_t *ppp;

    // try to see if we have any request in goto. if it is shalIPCprocApplication 
    // or if we dont have any, then we try to jump to 0x8010000. if we cannot we stay 
    // in here forever at a wild led blink.
    // if it is shalIPCprocLoader, then we stay in here and blink  led at 1 sec rate
    // but after 10 seconds we try to jump to 0x8010000.


//    if(true == asd)
//    {
//        SetSomething((USR_tu_Para) {asasa(0)});
//    }




    hal_memory_getsize(hal_params_cfgMINE, &size04aligned); 

    if(0 != size04aligned)
    {
        data32aligned = (uint32_t*)calloc(size04aligned/4, sizeof(uint32_t));   
    }

    hal_initialise(hal_params_cfgMINE, data32aligned);

    hal_sys_systeminit();


    if(ee_res_NOK_generic == shalbase_isvalid())
    {
        // we are in here if we dont have a valid one.
        manage_error();
    }
    else
    {
        shalbase_init(1);
    }


    // test the storage
    shalbase_storage_get((void*)&strg, (void*)vdata, 16);

    shalbase_storage_set(&strg, cdata, 16);

    shalbase_storage_get((void*)&strg, (void*)vdata, 16);



    if(ee_res_NOK_generic == shalpart_isvalid())
    {
        // we are in here if we dont have a valid one.
        manage_error();
    }
    else
    {
        shalpart_init();
    }

//    shalpart_reset(shalpart_reset_rawvals);

    // read the default proc address.

    res = shalpart_proc_def2run_get(&proc);


    shalpart_proc_add(ee_procApplication, &procinfo); 

    shalpart_proc_get(ee_procApplication, &ppp);

    procinfo.rom.size = 1024;

    shalpart_proc_set(ee_procApplication, &procinfo);

    res = shalpart_proc_allavailable_get(&proctable, &size);

    shalpart_proc_rem(ee_procApplication);


    res = shalpart_proc_allavailable_get(&proctable, &size);

    memcpy(pp, proctable, size);




    // just for .... 
    for(;;);  

}




// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static void manage_error(void)
{
    for(;;);
}

// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------





