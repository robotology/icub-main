
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHENVSCFGMASTERLOC_HID_H_
#define _EOTHENVSCFGMASTERLOC_HID_H_


/** @file       EOtheNVsCfgMasterLoc_hid.h
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
 
#include "EOtheNVsCfgMasterLoc.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------


// - typedef with hidden scope ----------------------------------------------------------------------------------------


typedef __packed struct
{
	uint64_t			                macaddr;
	uint32_t			                ipaddr;
} eOmaster_loc_globalconstants_t;


typedef __packed struct
{
	uint8_t			                    doit;
	uint32_t			                doalsothat;
} eOmaster_loc_globalconfiguration_t;


typedef __packed struct
{
	eOmaster_loc_globalconstants_t      globalconstants;
	eOmaster_loc_globalconfiguration_t  globalconfiguration;
    uint8_t                             isactive;
} eOmaster_loc_t;


// offset from beginning of struct eOmaster_loc_t
#define master_loc_addrPER_root                     (0)


// - definition of the hidden struct implementing the object ----------------------------------------------------------


extern const EOnetvarNode master_loc_thenetvarnodes[];

extern eOmaster_loc_t master_loc_vol;

// - declaration of extern hidden functions ---------------------------------------------------------------------------
// empty-section


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



