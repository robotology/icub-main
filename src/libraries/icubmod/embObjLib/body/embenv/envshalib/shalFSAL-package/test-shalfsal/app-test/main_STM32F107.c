
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

#if USESHALHAL
#include "shalOSAL.h"
#endif

#include "shalFSAL.h"

#include "fsal.h"

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

static void s_test_fsal(void);
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
    extern const fsal_params_cfg_t *fsal_params_cfgMINE;
    uint32_t size04aligned;
    uint32_t *data32aligned = NULL;
    uint64_t *ram64data = NULL;
    uint32_t ram64size = 0;
    uint32_t *ram32data = NULL;
    uint32_t ram32size = 0;

#if USESHALHAL
    if(ee_res_NOK_generic == shalhal_isvalid())
    {
        for(;;);
    }

    shalhal_init(1);
#endif

    if(ee_res_NOK_generic == shalfsal_isvalid())
    {
        for(;;);
    }

    shalfsal_init(1);



    hal_memory_getsize(hal_params_cfgMINE, &size04aligned); 

    if(0 != size04aligned)
    {
        data32aligned = (uint32_t*)calloc(size04aligned/4, sizeof(uint32_t));   
    }

    hal_initialise(hal_params_cfgMINE, data32aligned);

    hal_sys_systeminit();


    s_test_fsal();
      
#if USESHALOSAL

    if(ee_res_NOK_generic == shalosal_isvalid())
    {
        for(;;);
    }

    shalosal_init(1);

    osal_memory_getsize(osal_params_cfgMINE, &ram64size);

    ram64data = (uint64_t*)calloc(ram64size/8, sizeof(uint64_t));


    osal_initialise(osal_params_cfgMINE, ram64data);

  

    osal_start(init);                  

#endif


}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------

#if USESHALOSAL
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

#endif

uint8_t buf[512] =  { 0 };
uint8_t dat[512] = { 1 };
uint64_t signature[2] = {0x0011223344556677, 0x8899aabbccddeeff };

//#pragma O1
//#pragma Otime

static void s_test_fsal(void)
{
    static uint32_t value = 7;
    static char str[20] = {0};
    static FILE *fp = NULL;
    static uint8_t i = 0;
    static uint32_t tmpdat;

    static fsal_result_t rr;

    uint32_t len = 0;


  uint32_t ram32size;
  uint32_t *ram32data = NULL;
  extern const fsal_params_cfg_t *fsal_params_cfgMINE;


  fsal_memory_getsize(fsal_params_cfgMINE, &ram32size);
  ram32data = (uint32_t*)calloc(ram32size/4, sizeof(uint32_t));

    printf("before initialisation\n");

    rr = fsal_initialise(fsal_params_cfgMINE, ram32data);

    printf("initialisation done\n");

    printf("used: stdio_enable = %d and eflash_enable = %d\n",
           fsal_params_cfgMINE->stdio_enable, 
           fsal_params_cfgMINE->eflash_enable);


    if(0 == fsal_params_cfgMINE->eflash_enable)
    {
        printf("ok.... test finished and stdout works fine. bye.\n");
        return;
    }

    printf("formatting\n");


    fsal_format(fsal_drive_flash);


    // add the signature file with 16 bytes
        
    fp = fopen("sig.dat", "w");

    printf("signature.dat is open with fp = 0x%x\n", fp);
    len = fwrite(signature, 1, sizeof(signature), fp);
    printf("signature.dat is written with %d bytes\n", len);
    fclose(fp);
    fp = NULL;        
    

    printf("added signature.dat\n");
    
    fp = fopen("sig.dat", "r");
    if(NULL == fp)
    {
        printf("damn ... cannot reopen the sig.dat file\n");
        for(;;);
    }


    printf("reading signature.dat\n");
 
   
    for(i=0; i<4; i++)
    {
        fread(&tmpdat, 4, 1, fp);

        printf("read %x\n", tmpdat);

    }
    fclose(fp);
    fp = NULL;  


        printf("closed file signature.dat\n");


    fp = fopen("hi.txt", "w");
    fprintf(fp, "abcdefg %d\n", value);

    fclose(fp);
    fp = NULL;
    value = 0; 
    fp = fopen("hi.txt", "r");
    fscanf(fp, "%s %d", str, &value);
    fclose(fp);

    value = value;

    printf("test completed ... str is %s and value is %d\n", str, value);

}


