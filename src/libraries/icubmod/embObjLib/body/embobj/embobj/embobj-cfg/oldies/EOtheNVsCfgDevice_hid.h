
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHENVSCFGDEVICE_HID_H_
#define _EOTHENVSCFGDEVICE_HID_H_


/** @file       EOtheNVsCfgDevice_hid.h
    @brief      This header file implements hidden interface to the configuration of nvs for an example
    @author     marco.accame@iit.it
    @date       10/15/2009
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOtheNVs.h"
#include "EOVtheNVsCfg.h"


// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOtheNVsCfgDevice.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------

#define ARRAYSIZ                19

// - typedef with hidden scope ----------------------------------------------------------------------------------------

typedef __packed struct
{
	uint8_t			        doit;
	uint8_t			        doalsothat;
	uint32_t	            withperiodinmicrosecs;
} eOentity_global_cfg_t;

typedef __packed struct
{
	uint8_t			        acquisitionenabled;
	uint32_t                acquisitionperiod;
} eOentity_input_cfg_t;

typedef __packed struct
{
	uint8_t			        applicationenabled;
	uint32_t	            applicationmode;
} eOentity_output_cfg_t;

typedef __packed struct
{
	uint8_t			        input08value;
	uint64_t			    acquisitiontime;
    eOentity_input_cfg_t    inputconfiguration;
} eOentity_input08_t;

typedef __packed struct
{
	uint16_t			    input16value;
	uint64_t			    acquisitiontime;
    eOentity_input_cfg_t    inputconfiguration;
} eOentity_input16_t;

typedef __packed struct
{
	uint32_t			    input32value;
	uint64_t			    acquisitiontime;
    eOentity_input_cfg_t    inputconfiguration;
} eOentity_input32_t;


typedef __packed struct
{
	uint32_t			    outputvalue;
	uint64_t			    applicationtime;
    eOentity_output_cfg_t	outputconfiguration;
} eOentity_output_t;



// - definition of the hidden struct implementing the object ----------------------------------------------------------


/** @struct     EOtheNVsCfgDevice_hid
    @brief      Hidden definition. Implements private data used only internally by the 
                public or private (static) functions of the object and protected data
                used also by its derived objects.
 **/ 

struct EOtheNVsCfgDevice_hid 
{
    /** Derive a EOVtheGPIOCfg object by placing an object identifier for type EOVtheNVsCfg first       **/
    const uint32_t                  eoidentifier;
    /** And finally add a EOVtheNVsfg pointer                                                          **/
    const EOVtheNVsCfg * const      theconfig;
};


// - declaration of extern hidden functions ---------------------------------------------------------------------------
// empty-section


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



