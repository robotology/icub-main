
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

#include "osal.h"

#include "shalOSAL.h"

#if USESHALHAL
#include "shalHAL.h"
#endif

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

// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void init(void);
static void yotask(void *p);
static void mytask(void *p);

static void LED_init(void);
static void LED_toggle(void);
// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------


static osal_mutex_t *mtx = NULL;








// --------------------------------------------------------------------------------------------------------------------
// - definition of main 
// --------------------------------------------------------------------------------------------------------------------



int main(void) 
{
    extern const hal_params_cfg_t *hal_params_cfgMINE;
    extern const osal_params_cfg_t *osal_params_cfgMINE;
    uint32_t size04aligned;
    uint32_t *data32aligned = NULL;
    uint64_t *ram64data = NULL;
    uint32_t ram64size = 0;


    if(ee_res_NOK_generic == shalosal_isvalid())
    {
        for(;;);
    }

    shalosal_init(1);

#if USESHALHAL
    if(ee_res_NOK_generic == shalhal_isvalid())
    {
        for(;;);
    }

    shalhal_init(1);
#endif

    hal_memory_getsize(hal_params_cfgMINE, &size04aligned); 

    if(0 != size04aligned)
    {
        data32aligned = (uint32_t*)calloc(size04aligned/4, sizeof(uint32_t));   
    }

    hal_initialise(hal_params_cfgMINE, data32aligned);

    hal_sys_systeminit();


    osal_memory_getsize(osal_params_cfgMINE, &ram64size);

    ram64data = (uint64_t*)calloc(ram64size/8, sizeof(uint64_t));


    osal_initialise(osal_params_cfgMINE, ram64data);

  

    osal_start(init);                  



}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

static void init(void)
{
    mtx = osal_mutex_new();
    osal_task_new(mytask, (void*)0x12345678, 20, 128);
    osal_task_new(yotask, (void*)0x0, 30, 128);

}



static void mytask(void *p)
{
    static uint32_t mm = 0;
    static volatile osal_result_t res;
    static void *pp = NULL;

    p = p;


    pp = (void*)calloc(32, 1);

    LED_init();

    for(;;)
    {
        osal_system_task_wait(1000*1000);
        LED_toggle();

        res = osal_mutex_take(mtx, 0);
        mm ++;
        //osal_eventflag_set(0x00010000, sitsk, osal_callerTSK);
    }

}

static void yotask(void *p)
{
    static uint32_t nn = 0;

    p = p;

    for(;;)
    {
       osal_system_task_wait(500*1000);
        nn ++;
    }

}


static void LED_init(void)
{
    hal_gpio_init(hal_gpio_portE, hal_gpio_pin14, hal_gpio_dirOUT, hal_gpio_speed_2MHz);
}

static void LED_toggle(void)
{
    static uint8_t a = 0;
 
    hal_gpio_setval(hal_gpio_portE, hal_gpio_pin14, (hal_gpio_val_t)a);
    a = (0 == a) ? (1) : (0);
}



