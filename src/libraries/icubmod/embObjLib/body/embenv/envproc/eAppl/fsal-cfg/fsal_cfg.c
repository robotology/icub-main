
/* @file       fsal_cfg.c
	@brief      This file keeps internal implementation of the fsal.
	@author     marco.accame@iit.it
    @date       11/27/2009
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "fsal.h"

#include "hal.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "fsal_cfg.h"


// defines

// static values

static const fsal_params_cfg_t s_cfg = 
{ 
    .glob_embfstype             = FSAL_GLOB_EMBFSTYPE,  
    .glob_cpufam                = FSAL_GLOB_CPUFAM,               // uint8_t         glob_cpufam;                                 
    .glob_maxopenablefiles      = FSAL_GLOB_FOPENMAX,             // uint8_t         glob_maxopenablefiles; 
    .glob_defdrive              = FSAL_GLOB_DEFDRIVE,             // uint8_t         glob_defdrive;
    .glob_sizedefragbuffer      = FSAL_GLOB_DEFRAGBUFFERSIZE,     // uint16_t        glob_sizedefragbuffer;

    .stdio_enable               = FSAL_USE_STDIO,                 // uint8_t         stdio_enable;

    .eram_enable                = FSAL_ERAM_ENABLE,             
    .eram_numsectors            = FSAL_ERAM_NUMSECTORS,                   
    .eram_sizekbsector          = FSAL_ERAM_SIZEKBSECTOR, 
                                   
    .eflash_enable              = FSAL_EFLASH_ENABLE,             // uint8_t         eflash_enable;  
    .eflash_numsectors          = FSAL_EFLASH_NUMSECTORS,         // uint8_t         eflash_numsectors;           
    .eflash_sizekbsector        = FSAL_EFLASH_SIZEKBSECTOR,       // uint8_t         eflash_sizekbsector; 
    .eflash_startaddress        = FSAL_EFLASH_STARTADDRESS,                            
    
    .eeprom_enable              = 0,
    .eeprom_numsectors          = 0,                 
    .eeprom_sizekbsector        = 0, 
    .eeprom_startaddress        = 0,

    .spiflash_enable            = 0,
    .spiflash_numsectors        = 0,                 
    .spiflash_sizekbsector      = 0, 
    .spiflash_startaddress      = 0,

    .extfn               =
    {
        .usr_on_fatal_error         = NULL,

        .osal_mutex_new             = NULL,
        .osal_mutex_take            = NULL,
        .osal_mutex_release         = NULL,
        .osal_param_tout_forever    = 0,

        .hal_stdio_init             = hal_sys_itm_init,
        .hal_stdio_getchar          = hal_sys_itm_getchar,
        .hal_stdio_putchar          = hal_sys_itm_putchar,

        .hal_flash_init             = (fsal_result_t (*)(void)) hal_flash_unlock,  // use unlock ...
        .hal_flash_erase            = (fsal_result_t (*)(uint32_t, uint32_t)) hal_flash_erase,
        .hal_flash_read             = NULL,
        .hal_flash_write            = (fsal_result_t (*)(uint32_t, uint32_t, void *)) hal_flash_write,

        .hal_eeprom_init            = NULL,
        .hal_eeprom_erase           = NULL,
        .hal_eeprom_read            = NULL,
        .hal_eeprom_write           = NULL,

        .hal_spiflash_init          = NULL,
        .hal_spiflash_erase         = NULL,
        .hal_spiflash_read          = NULL,
        .hal_spiflash_write         = NULL
    }         
};


extern const fsal_params_cfg_t *fsal_params_cfgMINE = &s_cfg;








// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



