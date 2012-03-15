
// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _EOCFG_UMLSM_EX1_H_
#define _EOCFG_UMLSM_EX1_H_




/** @file       eOcfg_umlsm_Ex1.h
	@brief      This header file implements public interface to the configuration of the example state machine Ex1.
	@author     marco.accame@iit.it
	@date       09/01/2011
**/

/** @defgroup eo_cfgsmumlex1 Configuration for EOumlsm: Ex1
    see it.
    
    @{		
 **/



// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include "EOumlsm.h"



// - public #define  --------------------------------------------------------------------------------------------------
// empty-section
  

// - declaration of public user-defined types ------------------------------------------------------------------------- 

/**	@typedef    typedef enum eOumlsmEventsEx1_t
 	@brief 		The events to be used by a state machine configured with EOtheUMLsmCfgEx1
 **/ 
typedef enum 
{
	eo_umlsm_ex1_evdummy   =  eo_umlsm_evNONE,        /**< the null event derived from eOumlsmBasicEvents_t */
	eo_umlsm_ex1_ev00 	    = (eOumlsmEvent_t) 0,	    /**< another event managed by the example state machine */
	eo_umlsm_ex1_ev01 	    = (eOumlsmEvent_t) 1,		/**< another event managed by the example state machine */
	eo_umlsm_ex1_ev02 	    = (eOumlsmEvent_t) 2,		/**< another event managed by the example state machine */
	eo_umlsm_ex1_ev03 	    = (eOumlsmEvent_t) 3,		/**< another event managed by the example state machine */
	eo_umlsm_ex1_ev04 	    = (eOumlsmEvent_t) 4,		/**< another event managed by the example state machine */
	eo_umlsm_ex1_ev05 	    = (eOumlsmEvent_t) 5,		/**< another event managed by the example state machine */
	eo_umlsm_ex1_ev06 	    = (eOumlsmEvent_t) 6,		/**< another event managed by the example state machine */
	eo_umlsm_ex1_ev07 	    = (eOumlsmEvent_t) 7,		/**< another event managed by the example state machine */
	eo_umlsm_ex1_ev08 	    = (eOumlsmEvent_t) 8		/**< another event managed by the example state machine */
} eOumlsmEventsEx1_t;


/** @typedef    typedef struct eOumlsmDynamicDataEx1_t
    @brief      eOumlsmDynamicDataEx1_t is the struct which contains the dynamic data structure used in the 
                functions of this state machine. It will be responsibility of the state machine object to allocate
                RAM for this struct.
 **/ 
typedef struct 
{
    uint32_t    number_of_fired_transitions;
    uint32_t    number_of_attempt_to_guard_state1;
    uint32_t    data3;
} eOumlsmDynamicDataEx1_t;

    
// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------

 
/** @fn         extern eOumlsm_cfg_t* eo_cfg_umlsm_Ex1_Get(void) 
    @brief      Gets the handle of the EOVumlsmCfg-derived object.
    @return     Pointer to the required EOVumlsmCfg object. 
 **/
extern eOumlsm_cfg_t * eo_cfg_umlsm_Ex1_Get(void);




/** @}            
    end of group eo_cfgsmumlex1  
 **/

#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------




