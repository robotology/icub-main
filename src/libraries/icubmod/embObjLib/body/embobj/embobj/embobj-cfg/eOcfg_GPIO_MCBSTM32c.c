
/* @file       eOcfg_GPIO_MCBSTM32c.c
    @brief      This file implements internal implementation to the configuration of the GPIOs for the MCBSTM32 board
    @author     marco.accame@iit.it
    @date       09/01/2011
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "string.h"
#include "stdlib.h"
#include "EoCommon.h"
#include "EOiopin.h"



// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "eOcfg_GPIO_MCBSTM32c.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------
// empty-section 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
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
// - declaration of static functions
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition (and initialisation) of static variables
// --------------------------------------------------------------------------------------------------------------------


/* @var        s_inp_map
    @brief      It is an array which contains the gpio mapping for the simple input pins.
                Each entry must contain: {.id, .pos, .port, .val}, ...
                Last entry must be: {EOK_uint08dummy, eo_iopinposNONE, eo_iopinportNONE, eo_iopinvalNONE}
 **/
static const eOgpio_iopinmap_t s_inp_map[] =
{   
//    {   
//        EO_INIT(.id)    mcbstm32cInpBUTTON00,
//        EO_INIT(.port)  eo_iopinportA,                      
//        EO_INIT(.pos)   eo_iopinpos0,
//        EO_INIT(.val)   eo_iopinvalNONE
//    },
    {   // keep it last   
        EO_INIT(.id)    EOK_uint08dummy,
        EO_INIT(.port)  eo_iopinportNONE,                   
        EO_INIT(.pos)   eo_iopinposNONE,     
        EO_INIT(.val)   eo_iopinvalNONE  
    } 
}; 

/* @var        s_out_map
    @brief      It is an array which contains the gpio mapping for the simple output pins.
                Each entry must contain: {.id, .pos, .port, .val}, ...
                Last entry must be: {EOK_uint08dummy, eo_iopinposNONE, eo_iopinportNONE, eo_iopinvalNONE}
 **/
static const eOgpio_iopinmap_t s_out_map[] =
{   
    {      
        EO_INIT(.id)    iopinID_Out_mcbstm32c_LED_009,
        EO_INIT(.port)  eo_iopinportE,                   
        EO_INIT(.pos)   eo_iopinpos9,     
        EO_INIT(.val)   eo_iopinvalLOW  
    }, 
    {      
        EO_INIT(.id)    iopinID_Out_mcbstm32c_LED_010,
        EO_INIT(.port)  eo_iopinportE,                   
        EO_INIT(.pos)   eo_iopinpos10,     
        EO_INIT(.val)   eo_iopinvalLOW  
    }, 
    {   // keep it last   
        EO_INIT(.id)    EOK_uint08dummy,
        EO_INIT(.port)  eo_iopinportNONE,                   
        EO_INIT(.pos)   eo_iopinposNONE,     
        EO_INIT(.val)   eo_iopinvalNONE  
    } 
}; 

/* @var        s_mnginp_map
    @brief      It is an array which contains the gpio mapping for the managed input pins.
                Each entry must contain: {.id, .pos, .port, .val}, ...
                Last entry must be: {EOK_uint08dummy, eo_iopinposNONE, eo_iopinportNONE, eo_iopinvalNONE}
 **/
static const eOgpio_iopinmap_t s_mnginp_map[] =
{
    {      
        EO_INIT(.id)    iopinID_Iman_mcbstm32c_BUTTON_USER,
        EO_INIT(.port)  eo_iopinportB,                   
        EO_INIT(.pos)   eo_iopinpos7,     
        EO_INIT(.val)   eo_iopinvalNONE  
    },     
    {      
        EO_INIT(.id)    iopinID_Iman_mcbstm32c_BUTTON_TAMP,
        EO_INIT(.port)  eo_iopinportC,                   
        EO_INIT(.pos)   eo_iopinpos13,     
        EO_INIT(.val)   eo_iopinvalNONE  
    },    
    {      
        EO_INIT(.id)    iopinID_Iman_mcbstm32c_BUTTON_WKUP,
        EO_INIT(.port)  eo_iopinportA,                   
        EO_INIT(.pos)   eo_iopinpos0,     
        EO_INIT(.val)   eo_iopinvalNONE  
    }, 
    {   // keep it last   
        EO_INIT(.id)    EOK_uint08dummy,
        EO_INIT(.port)  eo_iopinportNONE,                   
        EO_INIT(.pos)   eo_iopinposNONE,     
        EO_INIT(.val)   eo_iopinvalNONE  
    } 
}; 

/* @var        s_mngout_map
    @brief      It is an array which contains the gpio mapping for the managed output pins.
                Each entry must contain: {.id, .pos, .port, .val}, ...
                Last entry must be: {EOK_uint08dummy, eo_iopinposNONE, eo_iopinportNONE, eo_iopinvalNONE}
 **/
static const eOgpio_iopinmap_t s_mngout_map[] =
{   
    {   
        EO_INIT(.id)    iopinID_Oman_mcbstm32c_LED_011,
        EO_INIT(.port)  eo_iopinportE,                   
        EO_INIT(.pos)   eo_iopinpos11,     
        EO_INIT(.val)   eo_iopinvalLOW  
    }, 
    {      
        EO_INIT(.id)    iopinID_Oman_mcbstm32c_LED_012,
        EO_INIT(.port)  eo_iopinportE,                   
        EO_INIT(.pos)   eo_iopinpos12,     
        EO_INIT(.val)   eo_iopinvalLOW  
    }, 
    {     
        EO_INIT(.id)    iopinID_Oman_mcbstm32c_LED_013,
        EO_INIT(.port)  eo_iopinportE,                   
        EO_INIT(.pos)   eo_iopinpos13,     
        EO_INIT(.val)   eo_iopinvalLOW  
    }, 
    {     
        EO_INIT(.id)    iopinID_Oman_mcbstm32c_LED_014,
        EO_INIT(.port)  eo_iopinportE,                   
        EO_INIT(.pos)   eo_iopinpos14,     
        EO_INIT(.val)   eo_iopinvalLOW  
    }, 
    {     
        EO_INIT(.id)    iopinID_Oman_mcbstm32c_LED_015,
        EO_INIT(.port)  eo_iopinportE,                   
        EO_INIT(.pos)   eo_iopinpos15,     
        EO_INIT(.val)   eo_iopinvalLOW  
    },                
    {   // keep it last   
        EO_INIT(.id)    EOK_uint08dummy,
        EO_INIT(.port)  eo_iopinportNONE,                   
        EO_INIT(.pos)   eo_iopinposNONE,     
        EO_INIT(.val)   eo_iopinvalNONE  
    } 
}; 


/* @enum        numbers_of_pins
    @brief      trick for some compilers which cannot initialise a struct with a static const uint8_t variable
 **/
enum numbers_of_pins    
{   
    s_ninp                          = sizeof(s_inp_map)/sizeof(eOgpio_iopinmap_t) - 1,
    s_nout                          = sizeof(s_out_map)/sizeof(eOgpio_iopinmap_t) - 1, 
    s_nmnginp                       = sizeof(s_mnginp_map)/sizeof(eOgpio_iopinmap_t) - 1, 
    s_nmngout                       = sizeof(s_mngout_map)/sizeof(eOgpio_iopinmap_t) - 1 
};




/* @var        s_theconfiguration
    @brief      It is the required configuration, which is the collection of previously defined 
                const variables.
    @details    Use the name s_theconfiguration.
 **/
static const eOgpio_cfg_t s_theconfiguration =    
{ 
	EO_INIT(.ninp)          s_ninp,
	EO_INIT(.nout)          s_nout,
	EO_INIT(.nmnginp)       s_nmnginp,
	EO_INIT(.nmngout)       s_nmngout,
	EO_INIT(.inp_map)       s_inp_map,
	EO_INIT(.out_map)       s_out_map,
	EO_INIT(.mnginp_map)    s_mnginp_map,
	EO_INIT(.mngout_map)    s_mngout_map
}; 



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------



extern const eOgpio_cfg_t * eo_cfg_gpio_mcbstm32c_Get(void)
{
    return(&s_theconfiguration);
}



// --------------------------------------------------------------------------------------------------------------------
// - definition of extern hidden functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section


// --------------------------------------------------------------------------------------------------------------------
// - definition of static functions 
// --------------------------------------------------------------------------------------------------------------------
// empty-section
 

// --------------------------------------------------------------------------------------------------------------------
// - end-of-file (leave a blank line after)
// --------------------------------------------------------------------------------------------------------------------



