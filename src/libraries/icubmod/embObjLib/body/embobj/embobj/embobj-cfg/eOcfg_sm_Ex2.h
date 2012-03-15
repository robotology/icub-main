
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOCFG_SM_EX2_H_
#define _EOCFG_SM_EX2_H_




/** @file       eOcfg_sm_Ex2.h
	@brief      This header file implements public interface to the configuration of the example state machine Ex2.
	@author     marco.accame@iit.it
	@date       09/01/2011
**/

/** @defgroup eo_theex2smconfig Configuration eOcfg_sm_Ex2m for EOsm
    ...
    The user should export a proper eOsmEvents<name>_t, and a proper smcfg_<name>_GetHandle() function.
    
    @{		
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOsm.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 

/**	@typedef    typedef enum eOsmEventsEx2_t
 	@brief 		The events to be used by a state machine configured with EOtheSMCfgEx2
 **/ 
typedef enum 
{
	eo_sm_ex2_evdummy      =  eo_sm_evNONE,       /**< the null event derived from eOsmBasicEvents_t */
	eo_sm_ex2_ev00 	    = (eOsmEvent_t) 0,	    /**< another event managed by the example state machine */
	eo_sm_ex2_ev01 	    = (eOsmEvent_t) 1,		/**< another event managed by the example state machine */
	eo_sm_ex2_ev02 	    = (eOsmEvent_t) 2,		/**< another event managed by the example state machine */
	eo_sm_ex2_ev03 	    = (eOsmEvent_t) 3,		/**< another event managed by the example state machine */
	eo_sm_ex2_ev04 	    = (eOsmEvent_t) 4,		/**< another event managed by the example state machine */
	eo_sm_ex2_ev05 	    = (eOsmEvent_t) 5,		/**< another event managed by the example state machine */
	eo_sm_ex2_ev06 	    = (eOsmEvent_t) 6,		/**< another event managed by the example state machine */
	eo_sm_ex2_ev07 	    = (eOsmEvent_t) 7,		/**< another event managed by the example state machine */
	eo_sm_ex2_ev08 	    = (eOsmEvent_t) 8		/**< another event managed by the example state machine */
} eOsmEventsEx2_t;


/** @typedef    typedef struct eOsmDynamicDataEx2_t
    @brief      eOsmDynamicDataEx2_t is the struct which contains the dynamic data structure used in the functions
                of this state machine. It will be responsibility of the state machine object to allocate
                RAM for this struct.
 **/ 
typedef struct 
{
    uint32_t    number_of_fired_transitions;
    uint32_t    number_of_crossed_states;
    uint32_t    data3;
} eOsmDynamicDataEx2_t;
    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------

 
/** @fn         extern const eOsm_cfg_t * eo_cfg_sm_ex2_Get(void) 
    @brief      Gets the pointer to the eOsm_cfg_t for a EOsm.
    @return     Pointer to the configuration. 
 **/
extern const eOsm_cfg_t * eo_cfg_sm_ex2_Get(void);




/** @}            
    end of group eo_theex2smconfig  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




