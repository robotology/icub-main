
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHENVSCFGDEVICEREM00_HID_H_
#define _EOTHENVSCFGDEVICEREM00_HID_H_


/** @file       EOtheNVsCfgDeviceRem00_hid.h
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
 
#include "EOtheNVsCfgDeviceRem00.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------
// empty-section

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
} eOdevice_Rem00_t;



// - definition of the hidden struct implementing the object ----------------------------------------------------------
// empty-section

extern eOdevice_Rem00_t rem00_device_vol;       // used by the user of the smart node which wants to prepare data to send
extern eOdevice_Rem00_t rem00_device_rec;       // used by the smart node to store received data

extern const EOnetvarNode rem00_device_thenetvarnodes[];


// - declaration of extern hidden functions ---------------------------------------------------------------------------
// empty-section


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



