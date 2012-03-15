
/* @file       hal_cfg.c
	@brief      This file keeps internal implementation of the osal.
	@author     marco.accame@iit.it
    @date       11/27/2009
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "hal.h"
#include "stdlib.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "hal_cfg.h"


static void s_hal_cfg_on_fatalerror(uint32_t errorcode, const char * errormsg);


static const hal_params_cfg_t s_cfg = 
{   
    .cpu_family             = HAL_CPUFAM,
    .cpu_type               = HAL_CPUTYPE,
    .cpu_freq               = HAL_CPUFREQ,
    .sys_stacksize          = HAL_SYS_STACKSIZE,
    .sys_heapsize           = HAL_SYS_HEAPSIZE,
    .display_enable         = HAL_SPIDISPLAY_ENABLE,
    .eth_enable             = HAL_ETH_ENABLE,
    .eth_dmatxbuffer_num    = HAL_ETH_DMA_TX_BUF,
    .eth_dmarxbuffer_num    = HAL_ETH_DMA_RX_BUF,
    .can1_enable            = HAL_CAN1_ENABLE,
    .can1_inpbuffer_num     = HAL_CAN1_INPBUFFCAPACITY,
    .can1_outbuffer_num     = HAL_CAN1_OUTBUFFCAPACITY,
	.can1_baudrate          = HAL_CAN1_BAUDRATE,
    .can2_enable            = HAL_CAN2_ENABLE,
    .can2_inpbuffer_num     = HAL_CAN2_INPBUFFCAPACITY,
    .can2_outbuffer_num     = HAL_CAN2_OUTBUFFCAPACITY,
	.can2_baudrate          = HAL_CAN2_BAUDRATE,
	.spi1_enable            = HAL_SPI1_ENABLE,
	.spi1_baudrate          = HAL_SPI1_BAUDRATE,
	.spi2_enable            = HAL_SPI2_ENABLE,
	.spi2_baudrate          = HAL_SPI2_BAUDRATE,
	.spi3_enable            = HAL_SPI3_ENABLE,
	.spi3_baudrate          = HAL_SPI3_BAUDRATE,
    .extfn                  =
    {
        .usr_on_fatal_error                 = s_hal_cfg_on_fatalerror,
        .osal_system_scheduling_suspend     = NULL,
        .osal_system_scheduling_restart     = NULL
    }
    
};


extern const hal_params_cfg_t *hal_params_cfgMINE = &s_cfg;


#if USESHALOSAL
#else
void SysTick_Handler(void)
{
    hal_void_fp_void_t systickhandler = hal_sys_systick_gethandler();

    if(NULL != systickhandler)
    {
        systickhandler();
    }
}
#endif

static void s_hal_cfg_on_fatalerror(uint32_t errorcode, const char * errormsg)
{
    errorcode = errorcode;
    if(NULL != errormsg)
    {
//        hal_display_putstring(4, (uint8_t*)errormsg);
    }

    for(;;);

}







// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



