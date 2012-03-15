
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHENVSCFGNODE00LOC_HID_H_
#define _EOTHENVSCFGNODE00LOC_HID_H_


/** @file       EOtheNVsCfgNode00Loc_hid.h
    @brief      This header file implements hidden interface to the configuration of nvs for an example
    @author     marco.accame@iit.it
    @date       10/15/2009
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOtheNVs.h"
#include "EOVtheNVsCfg.h"
#include "EOvport_hid.h"


// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOtheNVsCfgNode00Loc.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------


// - typedef with hidden scope ----------------------------------------------------------------------------------------


typedef __packed struct
{
	uint64_t			                macaddr;
	uint32_t			                ipaddr;
} eOnode00_loc_globalconstants_t;


typedef __packed struct
{
	uint8_t			                    acquireinput;
    uint32_t                            acquisitionperiod;
 	uint8_t			                    applyoutput;
    uint8_t                             signalvportro;
    uint32_t                            toipaddr;
    uint32_t                            withperiod;
} eOnode00_loc_globalconfiguration_t;


typedef __packed struct
{
	uint8_t			                    inputval;
    uint64_t                            acquisitiontime;
} eOnode00_loc_input_t;

typedef __packed struct
{
	uint8_t			                    outputval;
    uint8_t			                    lednumber;
} eOnode00_loc_output_t;

typedef __packed struct
{
    EOvport                             vportRO;
    EOvport                             vportWO;        
	eOnode00_loc_globalconstants_t      globalconstants;
	eOnode00_loc_globalconfiguration_t  globalconfiguration;
    eOnode00_loc_input_t                button;
    eOnode00_loc_output_t               led00;
    eOnode00_loc_output_t               led01;
    uint64_t                            timeoflife;
} eOnode00_loc_t;


// offset from beginning of struct eOnode00_loc_t
// do only for those NVs which are permanent and require to be written in eeprom
#define node00_loc_addrPER_root                         (0)

#define node00_loc_addrPER_vportRO__cfg                 (0)

#define node00_loc_addrPER_vportWO__cfg                 (sizeof(EOvport))

#define node00_loc_addrPER_globalconfiguration          (2*sizeof(EOvport)+sizeof(eOnode00_loc_globalconstants_t))
#define node00_loc_addrPER_globcfg__acquireinput        (node00_loc_addrPER_globalconfiguration+0)                           
#define node00_loc_addrPER_globcfg__acquisitionperiod   (node00_loc_addrPER_globcfg__acquireinput+1)                   
#define node00_loc_addrPER_globcfg__applyoutput         (node00_loc_addrPER_globcfg__acquisitionperiod+4)                  
#define node00_loc_addrPER_globcfg__signalvportro       (node00_loc_addrPER_globcfg__applyoutput+1)                   
#define node00_loc_addrPER_globcfg__toipaddr            (node00_loc_addrPER_globcfg__signalvportro+1)                   
#define node00_loc_addrPER_globcfg__withperiod          (node00_loc_addrPER_globcfg__toipaddr+4)

// globalconstants, button, led00, led01, timeoflife do not contain permanent NVs


// - definition of the hidden struct implementing the object ----------------------------------------------------------


extern const EOnetvarNode node00_loc_thenetvarnodes[];

extern eOnode00_loc_t node00_loc_vol;

// - declaration of extern hidden functions ---------------------------------------------------------------------------
// empty-section


#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



