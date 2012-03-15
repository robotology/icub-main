
/* @file       iLoader_mcbstm32c.c
    @brief      This header file implements the iLoader process for a stm32f107 onto a keil board.
                In here we dont use the STM32 in an attempt to keep it slim.
    @author     marco.accame@iit.it
    @date       04/29/2009
**/

// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------


#include "stm32.h"
#include "hal.h"
#include "hal_arch.h"

#include "string.h"
#include "stdlib.h"

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

#define APPLADDR    0x08020000



// --------------------------------------------------------------------------------------------------------------------
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------

static void Delay(uint32_t dlyTicks);
static void s_HW_LED_Config(void);
static void s_HW_SYSTICK_Config(void);
static void s_HW_LED_On(uint32_t led);
static void s_HW_LED_Off(uint32_t led);

static void s_HW_init(void);
void systickhandler(void);


static void manage_error(void);
static void stay_in_here_for_a_while(void);


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------

static volatile uint32_t msTicks;   

static uint32_t applAddress = 0;                         

// --------------------------------------------------------------------------------------------------------------------
// - definition of main 
// --------------------------------------------------------------------------------------------------------------------

int main(void) 
{
    static eEprocess_t pr;
    static eEresult_t res;
    static eEmoduleInfo_t *pv = NULL;
    static const eEstorage_t strg = { .type = ee_strg_eeprom, .size = 32, .addr = 2*1024  };
    static const uint8_t cdata[16] = {0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    static volatile uint8_t vdata[16];

    static eEmoduleInfo_t procinfo = 
    {
        .type               = ee_process,
        .signature          = ee_procApplication,
        .version            = {1, 1},
        .builddate          = {2010, 3, 8, 16, 57},
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
        .storage            = {ee_strg_none, 0, 0},
        .name               = "myappl.bin",
    };

    static eEboardInfo_t boardinfo =           // 56 BYTES (48 BYTES upto name[16])              
    {
        .type           = 1,
        .signature      = 2,
        .version        = {1, 1},
        .builddate      = {2011, 10, 17, 16, 52},         
        .rom            = {0x08000000, 256*1024},
        .ram            = {0x20000000, 64*1024},
        .storage        = {ee_strg_eeprom, 8*1024, 0},     
        .name           = "mcbstm32c",
        .uniqueid       = 0
    };

    static const eEboardInfo_t* brd = NULL; 

    // try to see if we have any request in goto. if it is shalIPCprocApplication 
    // or if we dont have any, then we try to jump to 0x8010000. if we cannot we stay 
    // in here forever at a wild led blink.
    // if it is shalIPCprocLoader, then we stay in here and blink  led at 1 sec rate
    // but after 10 seconds we try to jump to 0x8010000.


    s_HW_init();


    pv = shalbase_moduleinfo_get();



    if(ee_res_NOK_generic == shalbase_isvalid())
    {
        // we are in here if we dont have a valid shalBASE.
        manage_error();
    }


    // we are in here only if the shalBASE is ok

    // we init it. always init !!!
    shalbase_init(1);

    boardinfo.uniqueid = hal_arch_arm_uniqueid64_get();
    shalbase_boardinfo_synchronise(&boardinfo);

    shalbase_boardinfo_get(&brd);





    // test the storage
//    shalbase_storage_get(&strg, (void*)vdata, 16);
//
//    shalbase_storage_set(&strg, cdata, 16);
//
//    shalbase_storage_get(&strg, (void*)vdata, 16);


    // we see if tehre is a goto-process message
    res = shalbase_ipc_gotoproc_get(&pr);

    if((ee_res_NOK_generic == res) || (ee_procApplication == pr))
    {
        shalbase_ipc_gotoproc_clr();

        applAddress = APPLADDR;

        if(ee_res_OK == shalbase_system_canjump(applAddress))
        {
            shalbase_system_jumpnow(applAddress);   
        }
        else
        {
            // unfortunatley i cannot jump to the default applciation
            manage_error();
        }

    }
    else if(ee_procLoader == pr)
    {
        shalbase_ipc_gotoproc_clr();

        // stay in here and then restart after a while
        stay_in_here_for_a_while();
    }
    else
    {   
        // other iProcesses ....
        shalbase_ipc_gotoproc_clr();

        // i dont have otehr iprocesses...
        manage_error(); 
    }


    // just for .... 
    for(;;);  

}




// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------



static void Delay(uint32_t dlyTicks) 
{
    uint32_t curTicks;

    curTicks = msTicks;
    while((msTicks - curTicks) < dlyTicks);
}




void manage_error(void)
{
    s_HW_LED_Config(); 

    s_HW_SYSTICK_Config();

    while(1) 
    {
        s_HW_LED_On (LED);                            
        Delay (100);                                
        s_HW_LED_Off (LED);                            
        Delay (100);                               
    }

}


void stay_in_here_for_a_while(void)
{
    uint32_t cnt = 10;
    uint8_t data[8];
    uint8_t size = 0;
    eEresult_t res;

    uint8_t factor_on  = 1;
    uint8_t factor_off = 9;





    s_HW_SYSTICK_Config();

    s_HW_LED_Config(); 


    // i could see if anybody has given me any message on shared data.

    res = shalbase_ipc_volatiledata_get(data, &size, 8);

    if(ee_res_OK == res)
    {
        factor_on  = data[0];
        factor_off = data[1]; 
        
        shalbase_ipc_volatiledata_clr();  
    }

    while(1) 
    {
        if(0 == cnt)
        {
            // try to go to application.
            applAddress = APPLADDR;

            if(ee_res_OK == shalbase_system_canjump(applAddress))
            {
                shalbase_system_jumpnow(applAddress);   
            }
            else
            {
                manage_error();
            }

        }

        cnt --;

        s_HW_LED_On (LED);                            
        Delay (100*factor_on);                                
        s_HW_LED_Off (LED);                            
        Delay (100*factor_off);                               
    }

}


static void s_HW_init(void)
{
    extern const hal_cfg_t *hal_cfgMINE;
    uint32_t size04aligned;
    uint32_t *data32aligned = NULL;

    hal_base_memory_getsize(hal_cfgMINE, &size04aligned); 

    if(0 != size04aligned)
    {
        data32aligned = (uint32_t*)calloc(size04aligned/4, sizeof(uint32_t));   
    }

    hal_base_initialise(hal_cfgMINE, data32aligned);

    hal_sys_systeminit();


}

void systickhandler(void) 
{
    msTicks++;                      
}

static void s_HW_SYSTICK_Config(void)
{
    hal_sys_systick_sethandler(systickhandler, 1000, hal_int_priority00);
}


static void s_HW_LED_Config(void) 
{
    // assume port E. the LOADER_LED is led 8
    hal_gpio_init(hal_gpio_portE, hal_gpio_pin8, hal_gpio_dirOUT, hal_gpio_speed_2MHz);
}


static void s_HW_LED_On(uint32_t led) 
{
    hal_gpio_setval(hal_gpio_portE, hal_gpio_pin8, hal_gpio_valHIGH);
}


static void s_HW_LED_Off(uint32_t led) 
{
     hal_gpio_setval(hal_gpio_portE, hal_gpio_pin8, hal_gpio_valLOW);
}

// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------






