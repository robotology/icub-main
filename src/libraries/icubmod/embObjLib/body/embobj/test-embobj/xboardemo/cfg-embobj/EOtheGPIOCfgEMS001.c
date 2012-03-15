
/** @file       EOtheGPIOCfgMCBSTM32x.c
    @brief      This file implements internal implementation to the configuration of the GPIOs for the MCBSTM32 board
    @author     marco.accame@iit.it
    @date       10/15/2009
**/


// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include "stdlib.h"
#include "EoCommon.h"
#include "string.h"
#include "EOVtheGPIOCfg_hid.h"
#include "hal.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern public interface
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheGPIOCfgEMS001.h"


// --------------------------------------------------------------------------------------------------------------------
// - declaration of extern hidden interface 
// --------------------------------------------------------------------------------------------------------------------

#include "EOtheGPIOCfgEMS001_hid.h" 


// --------------------------------------------------------------------------------------------------------------------
// - #define with internal scope
// --------------------------------------------------------------------------------------------------------------------

#define VCPP2008

#if defined(_KEILBOARDSTM32C_)
    #define hal_gpio_portX hal_gpio_portE
#elif defined(_KEILBOARDSTM32_)
    #define hal_gpio_portX hal_gpio_portB
#endif




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


/** @var        s_inp_map
    @brief      It is an array which contains the gpio mapping for the simple input pins.
                Each entry must contain: {ID, hal_gpio_pin_t, hal_gpio_port_t, hal_gpio_val_t}, ...
                Last entry must be: {EOK_uint08dummy, hal_gpio_pinNONE, hal_gpio_portNONE, hal_gpio_valNONE}
 **/
static const EOVtheGPIOCfg_map_t s_inp_map[] =
{   //  id                                      hal_gpio_port_t                       hal_gpio_pin_t        hal_gpio_val_t  
//  {   xbrdInpBUTTON00,                   hal_gpio_portA,                       hal_gpio_pin0,        hal_gpio_valNONE                 },
    {   EOK_uint08dummy,                                hal_gpio_portNONE,                    hal_gpio_pinNONE,     hal_gpio_valNONE   } 
}; 

/** @var        s_out_map
    @brief      It is an array which contains the gpio mapping for the simple output pins.
                Each entry must contain: {ID, hal_gpio_pin_t, hal_gpio_port_t}, ...
                Last entry must be: {255, hal_gpio_pinNONE, hal_gpio_portNONE}
 **/
static const EOVtheGPIOCfg_map_t s_out_map[] =
{   //  id                                      hal_gpio_port_t                       hal_gpio_pin_t        hal_gpio_val_t 
//    {   xbrdOutLED_009,                    hal_gpio_portX,                       hal_gpio_pin9,        hal_gpio_valLOW    },
//    {   xbrdOutLED_010,                    hal_gpio_portX,                       hal_gpio_pin10,       hal_gpio_valLOW    },
    {   EOK_uint08dummy,                                hal_gpio_portNONE,                    hal_gpio_pinNONE,     hal_gpio_valNONE   }
}; 

/** @var        s_mnginp_map
    @brief      It is an array which contains the gpio mapping for the managed input pins.
                Each entry must contain: {ID, hal_gpio_pin_t, hal_gpio_port_t}, ...
                Last entry must be: {255, hal_gpio_pinNONE, hal_gpio_portNONE}
 **/
static const EOVtheGPIOCfg_map_t s_mnginp_map[] =
{   //  id                                      hal_gpio_port_t                       hal_gpio_pin_t        hal_gpio_val_t 
//    {   xbrdItrigBUTTON_WKUP,              hal_gpio_portA,                       hal_gpio_pin0,        hal_gpio_valNONE   },
//    {   xbrdItrigBUTTON_TAMP,              hal_gpio_portC,                       hal_gpio_pin13,       hal_gpio_valNONE   },
    {   EOK_uint08dummy,                                hal_gpio_portNONE,                    hal_gpio_pinNONE,     hal_gpio_valNONE   }
}; 

/** @var        s_mngout_map
    @brief      It is an array which contains the gpio mapping for the managed output pins.
                Each entry must contain: {ID, hal_gpio_pin_t, hal_gpio_port_t}, ...
                Last entry must be: {255, hal_gpio_pinNONE, hal_gpio_portNONE}
 **/
static const EOVtheGPIOCfg_map_t s_mngout_map[] =
{   //  id                                      hal_gpio_port_t                       hal_gpio_pin_t        hal_gpio_val_t 
    {   xbrdwaveLED_ONE,                  hal_gpio_portE,                       hal_gpio_pin8,       hal_gpio_valHIGH    },                  
    {   xbrdwaveLED_TWO,                  hal_gpio_portE,                       hal_gpio_pin10,       hal_gpio_valHIGH    },
    {   xbrdwaveLED_THREE,                hal_gpio_portE,                       hal_gpio_pin12,       hal_gpio_valHIGH    },
    {   xbrdwaveLED_FOUR,                  hal_gpio_portE,                       hal_gpio_pin15,       hal_gpio_valHIGH    },
    {   EOK_uint08dummy,                        hal_gpio_portNONE,                    hal_gpio_pinNONE,     hal_gpio_valLOW    },
}; 




#ifdef VCPP2008   
enum    
{   // trick for visual c++ 2008, which cannot initialise a struct with a static const uint8_t variable
    s_ninp                          = sizeof(s_inp_map)/sizeof(EOVtheGPIOCfg_map_t) - 1,
    s_nout                          = sizeof(s_out_map)/sizeof(EOVtheGPIOCfg_map_t) - 1, 
    s_nmnginp                       = sizeof(s_mnginp_map)/sizeof(EOVtheGPIOCfg_map_t) - 1, 
    s_nmngout                       = sizeof(s_mngout_map)/sizeof(EOVtheGPIOCfg_map_t) - 1 
};
#else
// on armcc a struct can be correctly initialised by the following static const variables
static const uint8_t s_ninp         = sizeof(s_inp_map)/sizeof(EOVtheGPIOCfg_map_t) - 1;
static const uint8_t s_nout         = sizeof(s_out_map)/sizeof(EOVtheGPIOCfg_map_t) - 1; 
static const uint8_t s_nmnginp      = sizeof(s_mnginp_map)/sizeof(EOVtheGPIOCfg_map_t) - 1; 
static const uint8_t s_nmngout      = sizeof(s_mngout_map)/sizeof(EOVtheGPIOCfg_map_t) - 1; 
#endif



/** @var        s_theconfiguration
    @brief      It is the required configuration, which is the colelction of previously defined 
                const variables.
    @details    Use the name s_theconfiguration.
 **/
static const EOVtheGPIOCfg s_theconfiguration =    
{ 
	s_ninp,
	s_nout,
	s_nmnginp,
	s_nmngout,
	s_inp_map,
	s_out_map,
	s_mnginp_map,
	s_mngout_map
}; 


static const EOtheGPIOCfgEMS001 xxx =
{
    EOIDGPIOCONFIG,         // eoidentifier
    &s_theconfiguration     // theconfig
};

// --------------------------------------------------------------------------------------------------------------------
// - definition of extern public functions
// --------------------------------------------------------------------------------------------------------------------



extern const EOtheGPIOCfgEMS001 * eo_gpiocfg_ems001_GetHandle(void)
{
    return(&xxx);
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



