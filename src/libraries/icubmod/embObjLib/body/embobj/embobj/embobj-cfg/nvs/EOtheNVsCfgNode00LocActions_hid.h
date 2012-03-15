
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOTHENVSCFGNODE00LOCACTIONS_HID_H_
#define _EOTHENVSCFGNODE00LOCACTIONS_HID_H_


/** @file       EOtheNVsCfgNode00LocActions_hid.h
    @brief      This header file implements hidden interface to the configuration of nvs for an example
    @author     marco.accame@iit.it
    @date       10/15/2009
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"


// - declaration of extern public interface ---------------------------------------------------------------------------
 
#include "EOtheNVsCfgNode00LocActions.h"


// - #define used with hidden struct ----------------------------------------------------------------------------------


// - typedef with hidden scope ----------------------------------------------------------------------------------------
// empty-section



// - definition of the hidden struct implementing the object ----------------------------------------------------------
// empty-section

// - declaration of extern hidden functions ---------------------------------------------------------------------------

// - declaration of extern public functions ---------------------------------------------------------------------------

extern void node00_loc_action_init_generic(void *p);
extern void node00_loc_action_update_generic(void *p);

extern void node00_loc_action_init_nvOBJ_root(void *p);
extern void node00_loc_action_update_nvOBJ_root(void *p);

extern void node00_loc_action_init_nvOBJ_vportRO__cfg(void *p);
extern void node00_loc_action_update_nvOBJ_vportRO__cfg(void *p);

extern void node00_loc_action_init_nvOBJ_vportRO__dat(void *p);

extern void node00_loc_action_update_nvOBJ_globalconfiguration__any(void *p);

extern void node00_loc_action_init_nvOBJ_button(void *p);
extern void node00_loc_action_update_nvOBJ_button__inputval(void *p);

extern void node00_loc_action_init_nvOBJ_led00(void *p);
extern void node00_loc_action_update_nvOBJ_led00__outputval(void *p);

extern void node00_loc_action_init_nvOBJ_led01(void *p);
extern void node00_loc_action_update_nvOBJ_led01__outputval(void *p);

extern void node00_loc_action_init_nvOBJ__timeoflife(void *p);
extern void node00_loc_action_update_nvOBJ__timeoflife(void *p);



#endif  // include guard

// - end-of-file (leave a blank line after)----------------------------------------------------------------------------



