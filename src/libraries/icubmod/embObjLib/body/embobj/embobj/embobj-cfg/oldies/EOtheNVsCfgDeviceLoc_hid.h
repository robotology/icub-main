
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHENVSCFGDEVICELOC_HID_H_
#define _EOTHENVSCFGDEVICELOC_HID_H_


/** @file       EOtheNVsCfgDeviceLoc_hid.h
    @brief      This header file implements hidden interface to the configuration of nvs for an example
    @author     marco.accame@iit.it
    @date       10/15/2009
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOtheNVs.h"
#include "EOVtheNVsCfg.h"
#include "EOvport_hid.h"

#include "EOtheNVsCfgDevice_hid.h"


// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOtheNVsCfgDeviceLoc.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------


// - typedef with hidden scope ----------------------------------------------------------------------------------------

typedef __packed struct
{
    EOvport                 vportROnly;
    EOvport                 vportWOnly;
	eOentity_global_cfg_t	globalconfiguration;
	eOentity_input08_t	    input08;
    eOentity_input16_t	    input16;
    eOentity_input32_t	    input32;
	eOentity_output_t	    output;
    uint8_t                 fixedarray[ARRAYSIZ];
    uint8_t                 varsizearray[ARRAYSIZ];
} eOdevice_Loc_t;

typedef struct
{
//    uint16_t   dev_vportROnly
    uint16_t   dev_vportROnly__cfg;
//    uint16_t   dev_vportROnly__dat
    
//    uint16_t   dev_vportWOnly
    uint16_t   dev_vportWOnly__cfg;
//    uint16_t   dev_vportWOnly__dat

//    uint16_t   device;
//
//    uint16_t   dev_globalconfiguration        s_offsets
    uint16_t   dev_globcfg__doit;
    uint16_t   dev_globcfg__doalsothat;
    uint16_t   dev_globcfg__withperiodinmicrosecs;
    
//    uint16_t   dev_input08
//    uint16_t   dev_input08__input08value
//    uint16_t   dev_input08__acquisitiontime
//    uint16_t   dev_input08_inputconfiguration
    uint16_t   dev_input08_inpcfg__acquisitionenabled;
    uint16_t   dev_input08_inpcfg__acquisitionperiod;
    
//    uint16_t   dev_input16
//    uint16_t   dev_input16__input16value
//    uint16_t   dev_input16__acquisitiontime
//    uint16_t   dev_input16_inputconfiguration
    uint16_t   dev_input16_inpcfg__acquisitionenabled;
    uint16_t   dev_input16_inpcfg__acquisitionperiod;
    
//    uint16_t   dev_input32
//    uint16_t   dev_input32__input32value
//    uint16_t   dev_input32__acquisitiontime
//    uint16_t   dev_input32_inputconfiguration
    uint16_t   dev_input32_inpcfg__acquisitionenabled;
    uint16_t   dev_input32_inpcfg__acquisitionperiod;
    
//    uint16_t   dev_output
//    uint16_t   dev_output__outputvalue
//    uint16_t   dev_output__applicationtime
//    uint16_t   dev_output_outputconfiguration
    uint16_t   dev_output_outcfg__applicationenabled;
    uint16_t   dev_output_outcfg__applicationmode;

//    uint16_t   dev__fixedarray
//    uint16_t   dev__varsizearray
    

} eOdevice_someoffsets_Loc_t;


// - definition of the hidden struct implementing the object ----------------------------------------------------------


extern const EOnetvarNode loc_device_thenetvarnodes[];

extern eOdevice_Loc_t loc_device_vol;

// - declaration of extern hidden functions ---------------------------------------------------------------------------
// empty-section


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



